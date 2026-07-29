#!/usr/bin/env python3
"""FAST-LIO2 trajectory CSV -> Nav2 NavigateThroughPoses.

Default behavior is preview-only:
- Reads a FAST-LIO2 trajectory CSV.
- Removes stationary/noisy samples.
- Selects waypoints by distance and corner angle.
- Generates base_link headings from the path tangent.
- Checks waypoint clearance against a Nav2 map YAML/PGM.
- Publishes the selected poses as nav_msgs/Path on /trajectory_waypoints.

Navigation is started only when --execute is explicitly specified.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path as NavPath
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import yaml


REQUIRED_COLUMNS = (
    "p_w_b_x",
    "p_w_b_y",
    "q_w_b_x",
    "q_w_b_y",
    "q_w_b_z",
    "q_w_b_w",
)


@dataclass(frozen=True)
class Sample:
    source_index: int
    x: float
    y: float
    body_yaw: float


@dataclass(frozen=True)
class GridMap:
    yaml_path: Path
    image_path: Path
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    occupied_thresh: float
    free_thresh: float
    negate: bool
    pixels: bytes

    def world_to_image(self, x: float, y: float) -> Optional[tuple[int, int]]:
        """Convert map-frame coordinates to PGM row/column."""
        dx = x - self.origin_x
        dy = y - self.origin_y

        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)

        # R(-yaw) * [dx, dy]
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        map_x = math.floor(local_x / self.resolution)
        map_y = math.floor(local_y / self.resolution)

        if (
            map_x < 0
            or map_x >= self.width
            or map_y < 0
            or map_y >= self.height
        ):
            return None

        row = self.height - 1 - map_y
        col = map_x
        return row, col

    def cell_state(self, row: int, col: int) -> str:
        pixel = self.pixels[row * self.width + col]
        occupancy = (
            pixel / 255.0 if self.negate else (255 - pixel) / 255.0
        )

        if occupancy > self.occupied_thresh:
            return "occupied"
        if occupancy < self.free_thresh:
            return "free"
        return "unknown"

    def clearance_check(
        self,
        x: float,
        y: float,
        clearance: float,
        unknown_is_obstacle: bool,
    ) -> tuple[bool, str]:
        center = self.world_to_image(x, y)
        if center is None:
            return False, "地図範囲外"

        center_row, center_col = center
        radius_cells = max(0, math.ceil(clearance / self.resolution))
        radius_sq = radius_cells * radius_cells

        for dr in range(-radius_cells, radius_cells + 1):
            row = center_row + dr
            if row < 0 or row >= self.height:
                return False, "安全半径が地図範囲外"

            for dc in range(-radius_cells, radius_cells + 1):
                if dr * dr + dc * dc > radius_sq:
                    continue

                col = center_col + dc
                if col < 0 or col >= self.width:
                    return False, "安全半径が地図範囲外"

                state = self.cell_state(row, col)
                if state == "occupied":
                    return False, "障害物に近すぎる"
                if state == "unknown" and unknown_is_obstacle:
                    return False, "未知領域に近すぎる"

        return True, "free"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "FAST-LIO2軌跡CSVを間引いてNav2 NavigateThroughPosesへ入力します。"
            "既定ではプレビューのみで、--execute指定時だけ走行要求を送信します。"
        )
    )
    parser.add_argument(
        "--trajectory",
        type=Path,
        required=True,
        help="FAST-LIO2軌跡CSV",
    )
    parser.add_argument(
        "--map-yaml",
        type=Path,
        required=True,
        help="Nav2地図YAML。ウェイポイントの障害物チェックに使用",
    )
    parser.add_argument(
        "--frame-id",
        default="map",
        help="ウェイポイント座標系（既定値: map）",
    )
    parser.add_argument(
        "--start-index",
        type=int,
        default=0,
        help="CSVデータ行の開始番号（既定値: 0）",
    )
    parser.add_argument(
        "--end-index",
        type=int,
        default=-1,
        help="CSVデータ行の終了番号。-1は末尾（既定値: -1）",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="軌跡を逆順に走行する",
    )
    parser.add_argument(
        "--raw-min-step",
        type=float,
        default=0.05,
        help="静止ノイズ除去用の最低移動距離[m]（既定値: 0.05）",
    )
    parser.add_argument(
        "--waypoint-spacing",
        type=float,
        default=0.80,
        help="通常区間のウェイポイント間隔[m]（既定値: 0.80）",
    )
    parser.add_argument(
        "--corner-angle-deg",
        type=float,
        default=15.0,
        help="追加ウェイポイントを残す曲がり角閾値[deg]（既定値: 15）",
    )
    parser.add_argument(
        "--corner-min-spacing",
        type=float,
        default=0.25,
        help="曲がり角点同士の最低間隔[m]（既定値: 0.25）",
    )
    parser.add_argument(
        "--start-skip-distance",
        type=float,
        default=0.30,
        help="記録開始点からこの距離以内の目標を除外[m]（既定値: 0.30）",
    )
    parser.add_argument(
        "--yaw-source",
        choices=("tangent", "body"),
        default="tangent",
        help=(
            "姿勢生成方法。tangentは次点方向、bodyはFAST-LIO2姿勢を使用"
            "（既定値: tangent）"
        ),
    )
    parser.add_argument(
        "--body-to-base-yaw",
        type=float,
        default=math.pi,
        help=(
            "yaw-source=body時に加えるbody->base_link yaw[rad]"
            "（既定値: pi）"
        ),
    )
    parser.add_argument(
        "--clearance",
        type=float,
        default=0.45,
        help="各ウェイポイント周囲に必要な自由半径[m]（既定値: 0.45）",
    )
    parser.add_argument(
        "--allow-unknown",
        action="store_true",
        help="未知セルを通過候補として許可する",
    )
    parser.add_argument(
        "--unsafe-policy",
        choices=("error", "skip"),
        default="error",
        help="危険な候補点があった場合の処理（既定値: error）",
    )
    parser.add_argument(
        "--max-waypoints",
        type=int,
        default=200,
        help="許容する最大ウェイポイント数（既定値: 200）",
    )
    parser.add_argument(
        "--preview-topic",
        default="/trajectory_waypoints",
        help="プレビューPathのトピック（既定値: /trajectory_waypoints）",
    )
    parser.add_argument(
        "--plan-preview",
        action="store_true",
        help="Nav2 Plannerで経由経路を計算し、走行せず表示する",
    )
    parser.add_argument(
        "--planned-path-topic",
        default="/trajectory_planned_path",
        help="Planner計算結果のPathトピック（既定値: /trajectory_planned_path）",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="NavigateThroughPosesを実行する。未指定時はプレビューのみ",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1800.0,
        help="ナビゲーションを中止するまでの時間[s]（既定値: 1800）",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.start_index < 0:
        raise ValueError("--start-indexは0以上にしてください")
    if args.end_index != -1 and args.end_index <= args.start_index:
        raise ValueError("--end-indexは--start-indexより大きくしてください")
    for name in (
        "raw_min_step",
        "waypoint_spacing",
        "corner_min_spacing",
        "clearance",
        "timeout",
    ):
        if getattr(args, name) < 0.0:
            raise ValueError(f"--{name.replace('_', '-')}は0以上にしてください")
    if not (0.0 <= args.corner_angle_deg <= 180.0):
        raise ValueError("--corner-angle-degは0～180にしてください")
    if args.max_waypoints < 2:
        raise ValueError("--max-waypointsは2以上にしてください")


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError("ゼロノルムのクォータニオンです")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def distance(a: Sample, b: Sample) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def load_samples(
    path: Path,
    start_index: int,
    end_index: int,
    reverse: bool,
) -> list[Sample]:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"軌跡CSVが見つかりません: {path}")

    samples: list[Sample] = []
    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise RuntimeError("CSVヘッダを読み取れません")

        missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise RuntimeError(
                "CSVに必要な列がありません: " + ", ".join(missing)
            )

        for row_index, row in enumerate(reader):
            if row_index < start_index:
                continue
            if end_index != -1 and row_index >= end_index:
                break

            try:
                x = float(row["p_w_b_x"])
                y = float(row["p_w_b_y"])
                qx = float(row["q_w_b_x"])
                qy = float(row["q_w_b_y"])
                qz = float(row["q_w_b_z"])
                qw = float(row["q_w_b_w"])
            except (TypeError, ValueError) as error:
                raise RuntimeError(
                    f"CSVデータ行{row_index}を数値変換できません"
                ) from error

            values = (x, y, qx, qy, qz, qw)
            if not all(math.isfinite(value) for value in values):
                continue

            samples.append(
                Sample(
                    source_index=row_index,
                    x=x,
                    y=y,
                    body_yaw=quaternion_to_yaw(qx, qy, qz, qw),
                )
            )

    if reverse:
        samples.reverse()

    if len(samples) < 2:
        raise RuntimeError("指定区間に有効な軌跡点が2点以上ありません")

    return samples


def remove_stationary_noise(
    samples: Sequence[Sample],
    min_step: float,
) -> list[Sample]:
    if min_step <= 0.0:
        return list(samples)

    result = [samples[0]]
    for sample in samples[1:-1]:
        if distance(result[-1], sample) >= min_step:
            result.append(sample)

    if samples[-1] is not result[-1]:
        if distance(result[-1], samples[-1]) > 1.0e-9:
            result.append(samples[-1])
        else:
            result[-1] = samples[-1]

    if len(result) < 2:
        raise RuntimeError(
            "静止ノイズ除去後に2点未満になりました。--raw-min-stepを下げてください"
        )
    return result


def corner_angle(samples: Sequence[Sample], index: int) -> float:
    if index <= 0 or index >= len(samples) - 1:
        return 0.0

    prev = samples[index - 1]
    curr = samples[index]
    nxt = samples[index + 1]

    v1x = curr.x - prev.x
    v1y = curr.y - prev.y
    v2x = nxt.x - curr.x
    v2y = nxt.y - curr.y

    norm1 = math.hypot(v1x, v1y)
    norm2 = math.hypot(v2x, v2y)
    if norm1 < 1.0e-9 or norm2 < 1.0e-9:
        return 0.0

    dot = (v1x * v2x + v1y * v2y) / (norm1 * norm2)
    dot = max(-1.0, min(1.0, dot))
    return math.acos(dot)


def select_waypoints(
    samples: Sequence[Sample],
    spacing: float,
    corner_angle_rad: float,
    corner_min_spacing: float,
    start_skip_distance: float,
) -> list[Sample]:
    selected = [samples[0]]

    for index in range(1, len(samples) - 1):
        sample = samples[index]
        distance_from_last = distance(selected[-1], sample)

        keep_by_spacing = distance_from_last >= spacing
        keep_by_corner = (
            corner_angle(samples, index) >= corner_angle_rad
            and distance_from_last >= corner_min_spacing
        )

        if keep_by_spacing or keep_by_corner:
            selected.append(sample)

    if distance(selected[-1], samples[-1]) > 1.0e-9:
        selected.append(samples[-1])
    else:
        selected[-1] = samples[-1]

    # The first recorded pose is the route origin, not normally a goal.
    origin = selected[0]
    goals = [
        point
        for point in selected[1:]
        if distance(origin, point) >= start_skip_distance
    ]

    if not goals or distance(goals[-1], samples[-1]) > 1.0e-9:
        goals.append(samples[-1])

    # Remove accidental duplicates.
    deduplicated = [goals[0]]
    for point in goals[1:]:
        if distance(deduplicated[-1], point) > 1.0e-6:
            deduplicated.append(point)

    if len(deduplicated) < 2:
        raise RuntimeError(
            "ウェイポイントが2点未満です。区間を延ばすか間引き条件を緩めてください"
        )

    return deduplicated


def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    path = path.expanduser().resolve()

    with path.open("rb") as file:
        tokens: list[bytes] = []
        while len(tokens) < 4:
            line = file.readline()
            if not line:
                raise RuntimeError(f"PGMヘッダが不完全です: {path}")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())

        magic = tokens[0]
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])

        if magic == b"P5":
            if max_value > 255:
                raise RuntimeError("16-bit PGMには未対応です")
            pixels = file.read()
        elif magic == b"P2":
            remaining = b" ".join(tokens[4:]) + b" " + file.read()
            values = [int(token) for token in remaining.split()]
            pixels = bytes(values)
        else:
            raise RuntimeError(
                f"対応していないPGM形式です: {magic.decode(errors='replace')}"
            )

    expected = width * height
    if len(pixels) != expected:
        raise RuntimeError(
            f"PGM画素数不一致: expected={expected}, actual={len(pixels)}"
        )

    return width, height, max_value, pixels


def load_grid_map(yaml_path: Path) -> GridMap:
    yaml_path = yaml_path.expanduser().resolve()
    if not yaml_path.is_file():
        raise FileNotFoundError(f"地図YAMLが見つかりません: {yaml_path}")

    with yaml_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    required = (
        "image",
        "resolution",
        "origin",
        "negate",
        "occupied_thresh",
        "free_thresh",
    )
    missing = [name for name in required if name not in config]
    if missing:
        raise RuntimeError(
            "地図YAMLに必要な項目がありません: " + ", ".join(missing)
        )

    image_path = Path(str(config["image"])).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()

    width, height, max_value, pixels = read_pgm(image_path)
    if max_value != 255:
        # Convert the supported 8-bit range to Nav2's expected 0..255 scale.
        pixels = bytes(round(value * 255 / max_value) for value in pixels)

    origin = config["origin"]
    if not isinstance(origin, list) or len(origin) < 3:
        raise RuntimeError("地図YAMLのoriginは[x, y, yaw]にしてください")

    occupied_thresh = float(config["occupied_thresh"])
    free_thresh = float(config["free_thresh"])
    if not (0.0 <= free_thresh < occupied_thresh <= 1.0):
        raise RuntimeError("地図YAMLのfree/occupied閾値が不正です")

    return GridMap(
        yaml_path=yaml_path,
        image_path=image_path,
        width=width,
        height=height,
        resolution=float(config["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
        negate=bool(int(config["negate"])),
        pixels=pixels,
    )


def filter_unsafe_waypoints(
    waypoints: Sequence[Sample],
    grid_map: GridMap,
    clearance: float,
    unknown_is_obstacle: bool,
    policy: str,
) -> list[Sample]:
    safe: list[Sample] = []
    unsafe_messages: list[str] = []

    for output_index, waypoint in enumerate(waypoints):
        valid, reason = grid_map.clearance_check(
            waypoint.x,
            waypoint.y,
            clearance,
            unknown_is_obstacle,
        )
        if valid:
            safe.append(waypoint)
            continue

        message = (
            f"waypoint[{output_index}] CSV行={waypoint.source_index} "
            f"x={waypoint.x:.3f}, y={waypoint.y:.3f}: {reason}"
        )
        unsafe_messages.append(message)

    if unsafe_messages:
        print("\n危険判定された候補点:", file=sys.stderr)
        for message in unsafe_messages:
            print(f"  {message}", file=sys.stderr)

        if policy == "error":
            raise RuntimeError(
                "危険なウェイポイントがあります。地図・clearance・区間を確認してください"
            )

    if len(safe) < 2:
        raise RuntimeError("安全性フィルタ後のウェイポイントが2点未満です")

    return safe


def waypoint_yaw(
    waypoints: Sequence[Sample],
    index: int,
    yaw_source: str,
    body_to_base_yaw: float,
) -> float:
    if yaw_source == "body":
        return normalize_angle(
            waypoints[index].body_yaw + body_to_base_yaw
        )

    if index < len(waypoints) - 1:
        source = waypoints[index]
        target = waypoints[index + 1]
    else:
        source = waypoints[index - 1]
        target = waypoints[index]

    return math.atan2(target.y - source.y, target.x - source.x)


def make_pose(
    navigator: BasicNavigator,
    frame_id: str,
    x: float,
    y: float,
    yaw: float,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def create_poses(
    navigator: BasicNavigator,
    frame_id: str,
    waypoints: Sequence[Sample],
    yaw_source: str,
    body_to_base_yaw: float,
) -> list[PoseStamped]:
    return [
        make_pose(
            navigator,
            frame_id,
            waypoint.x,
            waypoint.y,
            waypoint_yaw(
                waypoints,
                index,
                yaw_source,
                body_to_base_yaw,
            ),
        )
        for index, waypoint in enumerate(waypoints)
    ]


def publish_preview(
    navigator: BasicNavigator,
    publisher,
    frame_id: str,
    poses: Sequence[PoseStamped],
) -> None:
    message = NavPath()
    message.header.frame_id = frame_id
    message.header.stamp = navigator.get_clock().now().to_msg()
    message.poses = list(poses)
    publisher.publish(message)


def print_summary(
    raw_samples: Sequence[Sample],
    cleaned_samples: Sequence[Sample],
    waypoints: Sequence[Sample],
    grid_map: GridMap,
    args: argparse.Namespace,
) -> None:
    route_length = sum(
        distance(waypoints[index - 1], waypoints[index])
        for index in range(1, len(waypoints))
    )

    print("\n=== Trajectory waypoint summary ===")
    print(f"CSV samples        : {len(raw_samples)}")
    print(f"After noise removal: {len(cleaned_samples)}")
    print(f"Waypoints          : {len(waypoints)}")
    print(f"Waypoint route     : {route_length:.2f} m")
    print(
        f"Map                : {grid_map.width} x {grid_map.height}, "
        f"{grid_map.resolution:.3f} m/cell"
    )
    print(f"Map image          : {grid_map.image_path}")
    print(f"Yaw source         : {args.yaw_source}")
    print(f"Clearance          : {args.clearance:.2f} m")
    print(
        f"Mode               : "
        f"{'EXECUTE NavigateThroughPoses' if args.execute else 'PREVIEW ONLY'}"
    )
    print("First/last waypoint:")
    print(
        f"  first CSV行={waypoints[0].source_index}: "
        f"({waypoints[0].x:.3f}, {waypoints[0].y:.3f})"
    )
    print(
        f"  last  CSV行={waypoints[-1].source_index}: "
        f"({waypoints[-1].x:.3f}, {waypoints[-1].y:.3f})"
    )


def main() -> int:
    args = parse_args()
    validate_args(args)

    raw_samples = load_samples(
        args.trajectory,
        args.start_index,
        args.end_index,
        args.reverse,
    )
    cleaned_samples = remove_stationary_noise(
        raw_samples,
        args.raw_min_step,
    )
    waypoints = select_waypoints(
        cleaned_samples,
        args.waypoint_spacing,
        math.radians(args.corner_angle_deg),
        args.corner_min_spacing,
        args.start_skip_distance,
    )

    grid_map = load_grid_map(args.map_yaml)
    waypoints = filter_unsafe_waypoints(
        waypoints,
        grid_map,
        args.clearance,
        not args.allow_unknown,
        args.unsafe_policy,
    )

    if len(waypoints) > args.max_waypoints:
        raise RuntimeError(
            f"ウェイポイント数{len(waypoints)}が上限{args.max_waypoints}を"
            "超えています。--waypoint-spacingを大きくしてください"
        )

    rclpy.init()
    navigator = BasicNavigator(node_name="trajectory_waypoint_navigator")

    preview_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    preview_publisher = navigator.create_publisher(
        NavPath,
        args.preview_topic,
        preview_qos,
    )
    planned_path_publisher = navigator.create_publisher(
        NavPath,
        args.planned_path_topic,
        preview_qos,
    )

    poses = create_poses(
        navigator,
        args.frame_id,
        waypoints,
        args.yaw_source,
        args.body_to_base_yaw,
    )
    publish_preview(
        navigator,
        preview_publisher,
        args.frame_id,
        poses,
    )
    print_summary(
        raw_samples,
        cleaned_samples,
        waypoints,
        grid_map,
        args,
    )
    print(f"Preview topic      : {args.preview_topic}")

    planned_path = None
    if args.plan_preview:
        print("Nav2 PlannerへComputePathThroughPosesを要求します...")
        ignored_start = PoseStamped()
        ignored_start.header.frame_id = args.frame_id
        ignored_start.header.stamp = navigator.get_clock().now().to_msg()
        ignored_start.pose.orientation.w = 1.0
        planned_path = navigator.getPathThroughPoses(
            ignored_start,
            poses,
            use_start=False,
        )
        if planned_path is None or len(planned_path.poses) == 0:
            raise RuntimeError("Nav2 Plannerが経由経路を生成できませんでした")
        planned_path.header.frame_id = args.frame_id
        planned_path.header.stamp = navigator.get_clock().now().to_msg()
        planned_path_publisher.publish(planned_path)
        print(
            f"Planned path       : {len(planned_path.poses)} poses, "
            f"topic={args.planned_path_topic}"
        )

    if not args.execute:
        print(
            "\nプレビューモードです。RVizでPathを追加し、"
            f"Topic={args.preview_topic}を確認してください。"
        )
        print("終了するにはCtrl+Cを押してください。")
        try:
            while rclpy.ok():
                publish_preview(
                    navigator,
                    preview_publisher,
                    args.frame_id,
                    poses,
                )
                if planned_path is not None:
                    planned_path.header.stamp = navigator.get_clock().now().to_msg()
                    planned_path_publisher.publish(planned_path)
                rclpy.spin_once(navigator, timeout_sec=1.0)
        except KeyboardInterrupt:
            pass
        finally:
            navigator.destroyNode()
            rclpy.shutdown()
        return 0

    print(
        "\n--executeが指定されました。"
        "WHILLの非常停止手段を確保した状態で実行します。"
    )

    accepted = navigator.goThroughPoses(poses)
    if not accepted:
        navigator.destroyNode()
        rclpy.shutdown()
        raise RuntimeError("NavigateThroughPoses goalが拒否されました")

    start_time = time.monotonic()
    last_feedback_print = 0.0

    try:
        while not navigator.isTaskComplete():
            publish_preview(
                navigator,
                preview_publisher,
                args.frame_id,
                poses,
            )

            elapsed = time.monotonic() - start_time
            if elapsed > args.timeout:
                print("タイムアウトのためナビゲーションをキャンセルします")
                navigator.cancelTask()
                break

            feedback = navigator.getFeedback()
            if feedback is not None and elapsed - last_feedback_print >= 2.0:
                remaining = getattr(
                    feedback,
                    "number_of_poses_remaining",
                    None,
                )
                if remaining is None:
                    print(f"navigation elapsed: {elapsed:.1f} s")
                else:
                    print(
                        f"navigation elapsed: {elapsed:.1f} s, "
                        f"remaining poses: {remaining}"
                    )
                last_feedback_print = elapsed
    except KeyboardInterrupt:
        print("\nCtrl+Cを受信したためナビゲーションをキャンセルします")
        navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("NavigateThroughPoses: SUCCEEDED")
        return_code = 0
    elif result == TaskResult.CANCELED:
        print("NavigateThroughPoses: CANCELED")
        return_code = 2
    elif result == TaskResult.FAILED:
        print("NavigateThroughPoses: FAILED")
        return_code = 1
    else:
        print("NavigateThroughPoses: UNKNOWN")
        return_code = 1

    navigator.destroyNode()
    rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
