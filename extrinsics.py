"""Audit a known rigid camera transform with synthetic or surveyed 3D points."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

ROTATION_TOL = 1e-8


def number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{label} must be a finite number")
    return out


def vector(values, size, label):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f"{label} must have {size} values")
    return tuple(number(x, label) for x in values)


def dot(a, b):
    result = math.fsum(x * y for x, y in zip(a, b, strict=True))
    return number(result, "calculated value")


def transpose(rows):
    return tuple(zip(*rows, strict=True))


def matvec(rows, v):
    return tuple(dot(row, v) for row in rows)


def read_transform(matrix, direction):
    """Return (R_cw, t_cw) regardless of input direction."""
    if direction not in ("world_to_camera", "camera_to_world"):
        raise ValueError("direction must be world_to_camera or camera_to_world")
    if not isinstance(matrix, (list, tuple)) or len(matrix) != 4:
        raise ValueError("transform must be a 4 by 4 matrix")
    rows = tuple(vector(row, 4, "transform row") for row in matrix)
    if rows[3] != (0.0, 0.0, 0.0, 1.0):
        raise ValueError("last row must be [0, 0, 0, 1]")
    r = tuple(row[:3] for row in rows[:3])
    t = tuple(row[3] for row in rows[:3])
    for i, a in enumerate(r):
        for j, b in enumerate(r):
            if abs(dot(a, b) - float(i == j)) > ROTATION_TOL:
                raise ValueError("rotation rows must be orthonormal; check scale or rounding")
    a, b, c = r
    cross = (b[1]*c[2]-b[2]*c[1], b[2]*c[0]-b[0]*c[2], b[0]*c[1]-b[1]*c[0])
    if abs(dot(a, cross) - 1.0) > ROTATION_TOL:
        raise ValueError("rotation determinant must be +1; check for a reflected axis")
    if direction == "camera_to_world":
        r = transpose(r)
        t = tuple(-x for x in matvec(r, t))
    return r, t


def audit(scene):
    if not isinstance(scene, dict):
        raise ValueError("scene must be a JSON object")
    r, t = read_transform(scene["transform"], scene["direction"])
    fx, fy, cx, cy = vector(scene["intrinsics_px"], 4, "intrinsics_px")
    if fx <= 0 or fy <= 0:
        raise ValueError("focal lengths must be positive")
    width, height = scene["image_size_px"]
    if any(type(v) is not int or v <= 0 for v in (width, height)):
        raise ValueError("image_size_px must contain two positive integers")
    if not isinstance(scene["points"], dict) or not scene["points"]:
        raise ValueError("points must be a nonempty object of named 3D points")
    center = tuple(-x for x in matvec(transpose(r), t))
    results = {}
    for name, world in scene["points"].items():
        rx, ry, rz = matvec(r, vector(world, 3, f"point {name!r}"))
        x, y, z = vector((rx+t[0], ry+t[1], rz+t[2]), 3, "camera point")
        # Zero depth cannot be projected. The check is exact for these fixtures.
        if z <= 0:
            state, pixel = ("behind_camera" if z < 0 else "zero_depth"), None
        else:
            u, v = vector((fx*(x/z)+cx, fy*(y/z)+cy), 2, "projected pixel")
            pixel = (u, v)
            state = "inside_frame" if 0 <= u < width and 0 <= v < height else "outside_frame"
        results[name] = {"camera_m": (x, y, z), "pixel": pixel, "state": state}
    return {"camera_center_world_m": center, "world_origin_camera_m": t, "points": results}


def for_display(item):
    # Display rounding does not affect bounds tests or returned audit values.
    if isinstance(item, float):
        return round(item, 6) or 0.0
    if isinstance(item, dict):
        return {key: for_display(value) for key, value in item.items()}
    if isinstance(item, tuple):
        return [for_display(x) for x in item]
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path, help="scene JSON; positions and translation are in metres")
    args = parser.parse_args()
    try:
        result = audit(json.loads(args.scene.read_text(encoding="utf-8")))
    except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
        parser.error(str(exc))
    print(json.dumps(for_display(result), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
