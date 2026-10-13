cmake_minimum_required(VERSION 3.8)
project(whill_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odom_to_base_link.py
  scripts/trajectory_waypoint_navigator.py
  scripts/twist_to_whill_joy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
