#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Matrix = Eigen::Matrix4f;

bool finiteMatrix(const Matrix & matrix)
{
  return matrix.array().isFinite().all();
}

bool poseToMatrix(const geometry_msgs::msg::Pose & pose, Matrix & result)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const Eigen::Quaternionf quat(
    static_cast<float>(q.w), static_cast<float>(q.x),
    static_cast<float>(q.y), static_cast<float>(q.z));
  if (!std::isfinite(quat.norm()) || quat.norm() < 1.e-6f ||
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
  {
    return false;
  }
  result = Matrix::Identity();
  result.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
  result.block<3, 1>(0, 3) = Eigen::Vector3f(p.x, p.y, p.z);
  return finiteMatrix(result);
}

geometry_msgs::msg::Pose matrixToPose(const Matrix & matrix)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  return pose;
}

Cloud::Ptr downsample(const Cloud::ConstPtr & cloud, double leaf)
{
  auto output = std::make_shared<Cloud>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*output);
  return output;
}

double rotationAngle(const Matrix & rotation)
{
  const Eigen::Matrix3f relative = rotation.block<3, 3>(0, 0);
  const double cos_angle = std::clamp((static_cast<double>(relative.trace()) - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cos_angle);
}

struct StampedScan
{
  int64_t stamp_ns;
  Cloud::Ptr cloud;
};
}  // namespace

class SubmapLocalizer final : public rclcpp::Node
{
public:
  SubmapLocalizer()
  : rclcpp::Node("submap_localizer")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const std::string cloud_topic = declare_parameter<std::string>(
      "cloud_topic", "/cloud_registered");
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom_nav");
    const std::string init_topic = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    const std::string map_path = declare_parameter<std::string>("prior_map_path", "");
    submap_duration_sec_ = declare_parameter<double>("submap_duration_sec", 5.0);
    max_scans_ = declare_parameter<int>("max_scans", 60);
    scan_voxel_size_ = declare_parameter<double>("scan_voxel_size", 0.25);
    submap_voxel_size_ = declare_parameter<double>("submap_voxel_size", 0.30);
    map_voxel_size_ = declare_parameter<double>("prior_map_voxel_size", 0.30);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 30);
    min_submap_points_ = declare_parameter<int>("min_submap_points", 200);
    min_target_points_ = declare_parameter<int>("min_target_points", 200);
    registration_frequency_ = declare_parameter<double>("registration_frequency", 1.0);
    radius_xy_ = declare_parameter<double>("local_map_radius_xy", 20.0);
    half_height_ = declare_parameter<double>("local_map_half_height", 5.0);
    max_correspondence_ = declare_parameter<double>("gicp_max_correspondence_distance", 1.5);
    max_iterations_ = declare_parameter<int>("gicp_max_iterations", 40);
    trans_eps_ = declare_parameter<double>("gicp_transformation_epsilon", 0.0001);
    fitness_eps_ = declare_parameter<double>("gicp_euclidean_fitness_epsilon", 0.0001);
    max_fitness_ = declare_parameter<double>("max_fitness_score", 0.25);
    overlap_distance_ = declare_parameter<double>("overlap_distance", 0.60);
    min_overlap_ = declare_parameter<double>("min_overlap_ratio", 0.45);
    max_correction_t_ = declare_parameter<double>("max_correction_translation", 0.60);
    max_correction_r_ = declare_parameter<double>("max_correction_rotation_rad", 0.30);
    bootstrap_correction_t_ = declare_parameter<double>("bootstrap_max_correction_translation", 2.0);
    bootstrap_correction_r_ = declare_parameter<double>("bootstrap_max_correction_rotation_rad", 0.80);
    max_odom_age_sec_ = declare_parameter<double>("max_odom_age_sec", 1.0);
    max_alignment_age_sec_ = declare_parameter<double>("max_alignment_age_sec", 5.0);

    const std::vector<double> positive_values = {
      submap_duration_sec_, scan_voxel_size_, submap_voxel_size_, map_voxel_size_,
      registration_frequency_, radius_xy_, half_height_, max_correspondence_,
      trans_eps_, fitness_eps_, max_fitness_, overlap_distance_, max_correction_t_,
      max_correction_r_, bootstrap_correction_t_, bootstrap_correction_r_,
      max_odom_age_sec_, max_alignment_age_sec_};
    for (const double value : positive_values) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error("Numeric localization parameters must be finite and positive");
      }
    }
    if (map_path.empty() || map_frame_ == odom_frame_ || max_scans_ < 2 ||
      min_scan_points_ < 1 || min_submap_points_ < 1 || min_target_points_ < 1 ||
      max_iterations_ < 1 || min_overlap_ <= 0 || min_overlap_ > 1)
    {
      throw std::runtime_error("Missing prior_map_path or invalid localization configuration");
    }
    auto loaded_map = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *loaded_map) != 0 || loaded_map->empty()) {
      throw std::runtime_error("Cannot load 3D prior PCD: " + map_path);
    }
    // Remove non-finite map points before filtering; PCL's voxel filter expects finite positions.
    auto finite_map = std::make_shared<Cloud>();
    finite_map->reserve(loaded_map->size());
    for (const auto & point : loaded_map->points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        finite_map->push_back(point);
      }
    }
    prior_map_ = downsample(finite_map, map_voxel_size_);
    if (static_cast<int>(prior_map_->size()) < min_target_points_) {
      throw std::runtime_error("Prior map has insufficient finite points");
    }
    RCLCPP_INFO(get_logger(), "Loaded prior PCD: %zu points (voxelized) in frame '%s'",
      prior_map_->size(), map_frame_.c_str());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/localization/status", 10);
    submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/localization/submap", 1);

    // Separate callback group keeps scans/odometry flowing during potentially costly GICP.
    registration_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(30),
      std::bind(&SubmapLocalizer::onOdom, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&SubmapLocalizer::onCloud, this, std::placeholders::_1));
    init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      init_topic, rclcpp::QoS(10),
      std::bind(&SubmapLocalizer::onInitialPose, this, std::placeholders::_1));
    registration_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / registration_frequency_),
      std::bind(&SubmapLocalizer::registerSubmap, this), registration_group_);
    publishStatus("WAIT_INITIALPOSE");
    RCLCPP_WARN(get_logger(), "Waiting for /initialpose; TF map->odom will not publish until registration passes quality gates");
  }

private:
  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Expected odometry %s -> %s; got %s -> %s", odom_frame_.c_str(),
        base_frame_.c_str(), msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
      return;
    }
    Matrix odom_base;
    if (!poseToMatrix(msg->pose.pose, odom_base)) {
      return;
    }
    bool publish_tf = false;
    Matrix map_odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
      if (have_odom_ && stamp_ns < last_odom_stamp_ns_ - 100000000LL) {
        // A playback rewind or SLAM time reset invalidates all spatial history.
        scans_.clear();
        initialized_ = false;
        aligned_ = false;
        generation_++;
        RCLCPP_WARN(get_logger(), "Odometry timestamp reset: localization requires a new /initialpose");
      }
      last_odom_base_ = odom_base;
      last_odom_stamp_ns_ = stamp_ns;
      have_odom_ = true;
      if (aligned_) {
        const double age = (this->now().nanoseconds() - last_success_ns_) * 1.e-9;
        if (age >= 0.0 && age <= max_alignment_age_sec_) {
          map_odom = map_odom_;
          publish_tf = true;
        }
      }
    }
    if (!publish_tf) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    const auto pose = matrixToPose(map_odom);
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    geometry_msgs::msg::PoseStamped map_pose;
    map_pose.header.stamp = msg->header.stamp;
    map_pose.header.frame_id = map_frame_;
    map_pose.pose = matrixToPose(map_odom * odom_base);
    pose_pub_->publish(map_pose);
  }

  void onInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != map_frame_) {
      RCLCPP_ERROR(get_logger(), "Initial pose must be in '%s', got '%s'",
        map_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }
    Matrix map_base;
    if (!poseToMatrix(msg->pose.pose, map_base)) {
      RCLCPP_ERROR(get_logger(), "Invalid /initialpose quaternion or position");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double odom_age = have_odom_ ?
      (this->now().nanoseconds() - last_odom_stamp_ns_) * 1.e-9 :
      std::numeric_limits<double>::infinity();
    if (!have_odom_ || odom_age < -0.25 || odom_age > max_odom_age_sec_) {
      RCLCPP_WARN(get_logger(), "Rejecting /initialpose: fresh /odom_nav unavailable (age %.2fs)", odom_age);
      return;
    }
    map_odom_ = map_base * last_odom_base_.inverse();
    initialized_ = true;
    aligned_ = false;
    generation_++;
    RCLCPP_INFO(get_logger(), "Initial map->odom guess received; now validating against 3D prior map");
    publishStatus("INITIALPOSE_ACCEPTED_WAIT_REGISTRATION");
  }

  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Registered point cloud must be in %s (got %s). Supply a frame adapter before localization.",
        odom_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }
    auto raw = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *raw);
    auto clean = std::make_shared<Cloud>();
    clean->reserve(raw->size());
    for (const auto & point : raw->points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        clean->push_back(point);
      }
    }
    if (static_cast<int>(clean->size()) < min_scan_points_) {
      return;
    }
    auto filtered = downsample(clean, scan_voxel_size_);
    if (static_cast<int>(filtered->size()) < min_scan_points_) {
      return;
    }
    const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (stamp <= 0) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!scans_.empty() && stamp < scans_.back().stamp_ns) {
        // rosbag rewind or a non-monotonic scan stream invalidates the window.
        scans_.clear();
        initialized_ = false;
        aligned_ = false;
        generation_++;
        RCLCPP_WARN(get_logger(), "Point cloud timestamp reset; new /initialpose required");
      }
      if (!scans_.empty() && stamp == scans_.back().stamp_ns) {
        return;
      }
      scans_.push_back({stamp, filtered});
      const int64_t cutoff = stamp - static_cast<int64_t>(submap_duration_sec_ * 1.e9);
      while (!scans_.empty() && (scans_.front().stamp_ns < cutoff ||
        static_cast<int>(scans_.size()) > max_scans_))
      {
        scans_.pop_front();
      }
    }
  }

  Cloud::Ptr cropPriorMap(const Eigen::Vector3f & center) const
  {
    auto target = std::make_shared<Cloud>();
    target->reserve(prior_map_->size() / 5 + 1);
    for (const auto & point : prior_map_->points) {
      if (std::abs(point.x - center.x()) <= radius_xy_ &&
        std::abs(point.y - center.y()) <= radius_xy_ &&
        std::abs(point.z - center.z()) <= half_height_)
      {
        target->push_back(point);
      }
    }
    return target;
  }

  double overlapRatio(const Cloud::ConstPtr & aligned, const Cloud::ConstPtr & target) const
  {
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);
    std::vector<int> indices(1);
    std::vector<float> dist_sq(1);
    size_t matched = 0;
    const size_t stride = std::max<size_t>(1, aligned->size() / 5000);
    size_t checked = 0;
    const double threshold_sq = overlap_distance_ * overlap_distance_;
    for (size_t i = 0; i < aligned->size(); i += stride) {
      checked++;
      if (tree.nearestKSearch(aligned->points[i], 1, indices, dist_sq) > 0 &&
        dist_sq[0] <= threshold_sq)
      {
        matched++;
      }
    }
    return checked ? static_cast<double>(matched) / checked : 0.0;
  }

  void registerSubmap()
  {
    std::deque<StampedScan> scans;
    Matrix initial, odom_base;
    uint64_t generation;
    bool was_aligned;
    int64_t odom_stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!initialized_ || !have_odom_) {
        publishStatus("WAIT_INITIALPOSE_OR_ODOMETRY");
        return;
      }
      scans = scans_;
      initial = map_odom_;
      odom_base = last_odom_base_;
      was_aligned = aligned_;
      generation = generation_;
      odom_stamp = last_odom_stamp_ns_;
    }
    const double scan_age = scans.empty() ? std::numeric_limits<double>::infinity() :
      (this->now().nanoseconds() - scans.back().stamp_ns) * 1.e-9;
    const double odom_age = (this->now().nanoseconds() - odom_stamp) * 1.e-9;
    if (scans.size() < 2 || scan_age < -0.25 || scan_age > max_odom_age_sec_ ||
      odom_age < -0.25 || odom_age > max_odom_age_sec_ ||
      std::abs(scans.back().stamp_ns - odom_stamp) * 1.e-9 > max_odom_age_sec_)
    {
      publishStatus("WAIT_FRESH_SYNCHRONIZED_SCAN_ODOM");
      return;
    }
    auto combined = std::make_shared<Cloud>();
    for (const auto & scan : scans) {
      *combined += *scan.cloud;
    }
    auto source = downsample(combined, submap_voxel_size_);
    if (static_cast<int>(source->size()) < min_submap_points_) {
      publishStatus("TOO_FEW_SUBMAP_POINTS");
      return;
    }
    const Eigen::Vector3f current_map_position =
      (initial * odom_base).block<3, 1>(0, 3);
    auto target = cropPriorMap(current_map_position);
    if (static_cast<int>(target->size()) < min_target_points_) {
      publishStatus("TOO_FEW_PRIOR_MAP_POINTS_AROUND_GUESS");
      return;
    }

    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(*source, submap_msg);
    submap_msg.header.frame_id = odom_frame_;
    submap_msg.header.stamp = rclcpp::Time(scans.back().stamp_ns, RCL_ROS_TIME);
    submap_pub_->publish(submap_msg);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    gicp.setMaxCorrespondenceDistance(max_correspondence_);
    gicp.setMaximumIterations(max_iterations_);
    gicp.setTransformationEpsilon(trans_eps_);
    gicp.setEuclideanFitnessEpsilon(fitness_eps_);
    Cloud aligned;
    gicp.align(aligned, initial);
    const Matrix estimate = gicp.getFinalTransformation();
    if (!gicp.hasConverged() || !finiteMatrix(estimate)) {
      publishStatus("REJECT_NOT_CONVERGED");
      return;
    }
    const double fitness = gicp.getFitnessScore(max_correspondence_);
    const Cloud::ConstPtr aligned_ptr = std::make_shared<Cloud>(aligned);
    const double overlap = overlapRatio(aligned_ptr, target);
    const Matrix correction = estimate * initial.inverse();
    // Compare robot positions, not the map origin: rotation about a distant map origin
    // can exaggerate the translation part of (estimate * inverse(initial)).
    const double change_t = ((estimate * odom_base).block<3, 1>(0, 3) -
      (initial * odom_base).block<3, 1>(0, 3)).norm();
    const double change_r = rotationAngle(correction);
    const double limit_t = was_aligned ? max_correction_t_ : bootstrap_correction_t_;
    const double limit_r = was_aligned ? max_correction_r_ : bootstrap_correction_r_;
    std::ostringstream details;
    details << std::fixed << std::setprecision(3)
      << " fitness_m2=" << fitness << " overlap=" << overlap
      << " delta_translation_m=" << change_t << " delta_rotation_rad=" << change_r
      << " source_points=" << source->size() << " target_points=" << target->size();
    if (!std::isfinite(fitness) || fitness > max_fitness_ || overlap < min_overlap_ ||
      change_t > limit_t || change_r > limit_r)
    {
      publishStatus("REJECT_QUALITY" + details.str());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejected GICP correction: %s", details.str().c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != generation_) {
        publishStatus("REJECT_INITIALPOSE_CHANGED_DURING_REGISTRATION");
        return;
      }
      // Registration estimates map<-odom, not a modification of the SLAM odometry.
      map_odom_ = estimate;
      aligned_ = true;
      last_success_ns_ = this->now().nanoseconds();
    }
    publishStatus("ACCEPT" + details.str());
    RCLCPP_INFO(get_logger(), "Accepted 3D map registration: %s", details.str().c_str());
  }

  std::string map_frame_, odom_frame_, base_frame_;
  double submap_duration_sec_, scan_voxel_size_, submap_voxel_size_, map_voxel_size_;
  double registration_frequency_, radius_xy_, half_height_, max_correspondence_;
  double trans_eps_, fitness_eps_, max_fitness_, overlap_distance_, min_overlap_;
  double max_correction_t_, max_correction_r_, bootstrap_correction_t_, bootstrap_correction_r_;
  double max_odom_age_sec_, max_alignment_age_sec_;
  int max_scans_, min_scan_points_, min_submap_points_, min_target_points_, max_iterations_;
  Cloud::Ptr prior_map_;
  std::mutex mutex_;
  std::deque<StampedScan> scans_;
  Matrix map_odom_ = Matrix::Identity();
  Matrix last_odom_base_ = Matrix::Identity();
  int64_t last_odom_stamp_ns_ = 0;
  int64_t last_success_ns_ = 0;
  uint64_t generation_ = 0;
  bool have_odom_ = false;
  bool initialized_ = false;
  bool aligned_ = false;
  rclcpp::CallbackGroup::SharedPtr registration_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_pub_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SubmapLocalizer>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("submap_localizer"), "Startup/processing failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
