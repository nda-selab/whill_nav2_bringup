"""Offline collision checks; no ROS nodes or motion commands are created.

Run after sourcing ROS: python3 -m unittest discover -s tests -v
"""

import contextlib
from dataclasses import replace
import importlib.util
import io
import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "trajectory_waypoint_navigator", ROOT / "scripts/trajectory_waypoint_navigator.py"
)
nav = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = nav
SPEC.loader.exec_module(nav)

FOOTPRINT = ((0.8, 0.3), (0.8, -0.3), (-0.3, -0.3), (-0.3, 0.3))


def make_map(cells=()):
    width = height = 100
    resolution = 0.05
    pixels = bytearray([255] * (width * height))
    for x, y, value in cells:
        col = math.floor(x / resolution)
        row = height - 1 - math.floor(y / resolution)
        pixels[row * width + col] = value
    return nav.GridMap(
        Path("test.yaml"), Path("test.pgm"), width, height, resolution,
        0.0, 0.0, 0.0, 0.65, 0.25, False, bytes(pixels),
    )


class FootprintCollisionTests(unittest.TestCase):
    def check_pose(self, grid, yaw=0.0, clearance=0.0, unknown=True, x=2.5, y=2.5):
        return grid.clearance_check(x, y, yaw, FOOTPRINT, clearance, unknown)[0]

    def test_front_overhang_beyond_old_radius_is_detected(self):
        grid = make_map([(3.2, 2.5, 0)])
        self.assertFalse(self.check_pose(grid))
        self.assertTrue(self.check_pose(grid, yaw=math.pi))

    def test_ninety_degree_heading_changes_collision(self):
        grid = make_map([(2.5, 3.2, 0)])
        self.assertTrue(self.check_pose(grid))
        self.assertFalse(self.check_pose(grid, yaw=math.pi / 2))

    def test_filled_interior_is_checked(self):
        self.assertFalse(self.check_pose(make_map([(2.5, 2.5, 0)])))

    def test_obstacle_cell_touching_front_edge_is_detected(self):
        # The cell center is outside the footprint, but its left edge touches it.
        self.assertFalse(self.check_pose(make_map([(3.325, 2.5, 0)])))

    def test_cell_touching_diagonal_corner_is_detected(self):
        # Rotated front-left vertex is (2.5 + 0.5/sqrt(2), 2.5 + 1.1/sqrt(2)).
        self.assertFalse(self.check_pose(
            make_map([(2.875, 3.275, 0)]), yaw=math.pi / 4,
        ))

    def test_extra_clearance_is_measured_from_body(self):
        grid = make_map([(3.5, 2.5, 0)])
        self.assertTrue(self.check_pose(grid))
        self.assertFalse(self.check_pose(grid, clearance=0.25))

    def test_space_beside_long_body_remains_usable(self):
        self.assertTrue(self.check_pose(make_map([(2.5, 3.0, 0)])))

    def test_unknown_policy(self):
        grid = make_map([(3.2, 2.5, 128)])
        self.assertFalse(self.check_pose(grid))
        self.assertTrue(self.check_pose(grid, unknown=False))

    def test_entire_body_and_margin_must_be_inside_map(self):
        grid = make_map()
        self.assertFalse(self.check_pose(grid, x=0.2))
        self.assertTrue(self.check_pose(grid, x=0.5))
        self.assertFalse(self.check_pose(grid, x=0.5, clearance=0.25))
        self.assertFalse(self.check_pose(grid, x=4.4))

    def test_rotated_translated_map_origin(self):
        angle = math.pi / 3
        grid = replace(
            make_map([(3.2, 2.5, 0)]), origin_x=10.0, origin_y=-3.0,
            origin_yaw=angle,
        )
        x = 10.0 + math.cos(angle) * 2.5 - math.sin(angle) * 2.5
        y = -3.0 + math.sin(angle) * 2.5 + math.cos(angle) * 2.5
        self.assertFalse(self.check_pose(grid, x=x, y=y, yaw=angle))
        self.assertTrue(self.check_pose(grid, x=x, y=y, yaw=angle + math.pi))

    def test_nonfinite_or_negative_margin_is_rejected(self):
        for margin in (math.nan, math.inf, -0.1):
            with self.subTest(margin=margin), self.assertRaises(ValueError):
                self.check_pose(make_map(), clearance=margin)

    def test_skip_rechecks_tangent_headings(self):
        points = [
            nav.Sample(0, 2.0, 2.0, 0.0), nav.Sample(1, 3.0, 2.0, 0.0),
            nav.Sample(2, 2.0, 3.0, 0.0), nav.Sample(3, 2.0, 4.0, 0.0),
        ]
        grid = make_map([(3.0, 2.0, 0), (2.0, 2.5, 0)])
        with contextlib.redirect_stderr(io.StringIO()):
            safe = nav.filter_unsafe_waypoints(
                points, grid, FOOTPRINT, 0.0, True, "skip", "tangent", 0.0,
            )
        # Removing point 1 turns point 0 north, into the second obstacle.
        self.assertEqual([p.source_index for p in safe], [2, 3])

    def test_body_heading_offset_is_checked(self):
        points = [nav.Sample(0, 2.5, 2.5, 0.0), nav.Sample(1, 2.5, 3.5, 0.0)]
        grid = make_map([(3.2, 2.5, 0)])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(RuntimeError):
            nav.filter_unsafe_waypoints(
                points, grid, FOOTPRINT, 0.0, True, "error", "body", 0.0,
            )
        self.assertEqual(nav.filter_unsafe_waypoints(
            points, grid, FOOTPRINT, 0.0, True, "error", "body", math.pi,
        ), points)


class FootprintConfigTests(unittest.TestCase):
    def load_config(self, points, padding=None):
        params = {"footprint": points}
        if padding is not None:
            params["footprint_padding"] = padding
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.yaml"
            path.write_text(yaml.safe_dump({
                "local_costmap": {"local_costmap": {"ros__parameters": params}}
            }))
            return nav.load_footprint(path)

    def test_string_footprint_and_humble_default_padding(self):
        footprint = self.load_config(str([list(p) for p in FOOTPRINT]))
        self.assertEqual(footprint[0], (0.81, 0.31))
        self.assertEqual(footprint[2], (-0.31, -0.31))

    def test_list_footprint_and_explicit_padding(self):
        self.assertEqual(self.load_config(FOOTPRINT, padding=0.0), FOOTPRINT)
        self.assertEqual(self.load_config(FOOTPRINT, padding=0.1)[0], (0.9, 0.4))

    def test_clockwise_and_counterclockwise_are_supported(self):
        self.assertEqual(nav.validate_footprint(FOOTPRINT), FOOTPRINT)
        self.assertEqual(nav.validate_footprint(FOOTPRINT[::-1]), FOOTPRINT[::-1])

    def test_malformed_or_nonconvex_footprints_are_rejected(self):
        invalid = [
            [], [[0, 0], [1, 0]], [[0, 0], [1, 0], [2, 0]],
            [[0, 0], [1, 0], [1, 1], [0.5, 0.2], [0, 1]],
            [[0, 0], [1, 0], [math.nan, 1]], [[0, 0], [1, 0], [0, 0]],
            [[0, 0, 1], [1, 0], [0, 1]], "invalid",
        ]
        for points in invalid:
            with self.subTest(points=points), self.assertRaises(ValueError):
                self.load_config(points)

    def test_invalid_padding_is_rejected(self):
        for padding in (-0.01, math.nan, math.inf):
            with self.subTest(padding=padding), self.assertRaises(ValueError):
                self.load_config(FOOTPRINT, padding)

    def test_repository_footprint_can_be_loaded(self):
        footprint = nav.load_footprint(ROOT / "config/nav2_params.yaml")
        self.assertEqual(len(footprint), 4)
        self.assertGreater(max(x for x, _ in footprint), 0.8)


if __name__ == "__main__":
    unittest.main()
