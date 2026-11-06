import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

from extrinsics import audit, read_transform

ROOT = Path(__file__).resolve().parents[1]


def fixture(name):
    return json.loads((ROOT / "examples" / f"{name}.json").read_text(encoding="utf-8"))


class FrameTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for got, want in zip(actual, expected, strict=True):
            self.assertAlmostEqual(got, want, places=9)

    def test_parallel_camera_known_origin_and_pixel(self):
        result = audit(fixture("parallel"))
        self.assertVector(result["camera_center_world_m"], (.2, -.1, .5))
        self.assertVector(result["world_origin_camera_m"], (-.2, .1, -.5))
        self.assertVector(result["points"]["axis"]["pixel"], (320, 240))
        self.assertVector(result["points"]["right_down"]["pixel"], (440, 330))

    def test_side_view_against_independent_world_axes(self):
        # Optical right is world -Z; down is world +Y; forward is world +X.
        result = audit(fixture("side_view"))
        self.assertVector(result["camera_center_world_m"], (1, .25, 2))
        self.assertVector(result["world_origin_camera_m"], (2, -.25, -1))
        self.assertVector(result["points"]["right_down"]["camera_m"], (.5, .2, 2))
        self.assertVector(result["points"]["right_down"]["pixel"], (520, 330))

    def test_same_side_view_as_explicit_world_to_camera(self):
        scene = fixture("side_view")
        opposite = dict(scene, direction="world_to_camera", transform=[
            [0, 0, -1, 2], [0, 1, 0, -.25], [1, 0, 0, -1], [0, 0, 0, 1],
        ])
        self.assertEqual(audit(scene), audit(opposite))

    def test_mislabeled_direction_passes_rotation_check_but_fails_fixture(self):
        scene = fixture("side_view")
        wrong = audit(dict(scene, direction="world_to_camera"))
        self.assertEqual(wrong["points"]["axis"]["state"], "behind_camera")
        self.assertVector(wrong["camera_center_world_m"], (2, -.25, -1))

    def test_zero_behind_and_half_open_bounds(self):
        p = audit(fixture("depth_and_bounds"))["points"]
        self.assertEqual([entry["state"] for entry in p.values()], [
            "zero_depth", "behind_camera", "outside_frame", "inside_frame", "inside_frame",
        ])
        self.assertIsNone(p["on_plane"]["pixel"])
        self.assertIsNone(p["behind"]["pixel"])
        self.assertVector(p["right_edge"]["pixel"], (640, 240))
        self.assertVector(p["left_edge"]["pixel"], (0, 240))
        self.assertVector(p["inside"]["pixel"], (620, 240))

    def test_pinhole_has_no_metric_depth_from_one_pixel(self):
        scene = fixture("parallel")
        scene["points"]["twice_depth"] = [.8, .3, 4.5]
        p = audit(scene)["points"]
        self.assertVector(p["twice_depth"]["pixel"], (440, 330))
        self.assertAlmostEqual(p["twice_depth"]["camera_m"][2], 4.0)

    def test_reflection_rejected_even_with_orthogonal_rows(self):
        with self.assertRaisesRegex(ValueError, "determinant"):
            audit(fixture("reflection"))

    def test_reject_bad_transform_metadata_and_values(self):
        base = fixture("parallel")
        for row, col, val, message in (
            (0, 0, 2, "orthonormal"), (3, 3, 2, "last row"),
            (0, 3, math.inf, "finite"), (0, 0, True, "finite"),
        ):
            with self.subTest(row=row, col=col, value=val):
                scene = copy.deepcopy(base)
                scene["transform"][row][col] = val
                with self.assertRaisesRegex(ValueError, message):
                    audit(scene)
        with self.assertRaisesRegex(ValueError, "direction"):
            read_transform(base["transform"], "unknown")
        with self.assertRaisesRegex(ValueError, "focal lengths"):
            audit(dict(base, intrinsics_px=[0, 900, 320, 240]))

    def test_cli_output_and_invalid_exit_status(self):
        def run(name):
            return subprocess.run([sys.executable, str(ROOT / "extrinsics.py"),
                                   str(ROOT / "examples" / f"{name}.json")],
                                  cwd=ROOT, capture_output=True, text=True, check=False)
        good, bad = run("side_view"), run("reflection")
        self.assertEqual(good.returncode, 0, good.stderr)
        self.assertEqual(json.loads(good.stdout)["points"]["right_down"]["pixel"], [520, 330])
        self.assertEqual(bad.returncode, 2)
        self.assertEqual(bad.stdout, "")
        self.assertIn("determinant", bad.stderr)
