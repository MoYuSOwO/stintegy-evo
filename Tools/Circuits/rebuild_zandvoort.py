#!/usr/bin/env python3
"""Rebuild Zandvoort's track.csv from a traced centreline and a manual surface.

The centreline is a periodic smoothing spline through the current centreline
(the traced route), sampled at one metre.  Grade and banking are read from
surface.json and are deliberately authored, not inferred from curvature:

  grade.controlPoints   (s, z) knots of a periodic cubic spline
  bank.sections         start/end/peakDegrees/direction/blendMeters

Run with any Python that has numpy, scipy and matplotlib (the TUM bake venv does):
  python Tools/Circuits/rebuild_zandvoort.py
"""

from __future__ import annotations

import json
import math
import tempfile
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, splprep, splev
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
CIRCUIT = ROOT / "Circuits" / "zandvoort"
TRACK = CIRCUIT / "track.csv"
PROFILE = CIRCUIT / "surface.json"


def read_track(path: Path) -> list[list[float]]:
    rows: list[list[float]] = []
    with path.open() as handle:
        for line in handle:
            if line.startswith("#") or line.startswith("x_m"):
                continue
            parts = line.strip().split(",")
            if len(parts) < 8:
                continue
            rows.append([float(part) for part in parts])
    return rows


def smoothstep(t: float) -> float:
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def closed_section_weight(s: float, start: float, end: float, blend: float, lap: float) -> float:
    """1 inside [start, end], easing to 0 over blend at either end, wrapped."""
    blend = max(blend, 1e-3)
    # Work in a frame where start is 0, then wrap the offset.
    offset = (s - (start - blend)) % lap
    span = (end - start) + 2.0 * blend
    if offset >= span:
        return 0.0
    return min(
        smoothstep(offset / blend),
        smoothstep((span - offset) / blend),
    )


def centreline_points(
    x: np.ndarray,
    y: np.ndarray,
    length: int,
    spacing: float,
    smoothing: float,
) -> tuple[np.ndarray, np.ndarray]:
    if smoothing <= 0.0:
        return x.copy(), y.copy()

    points = np.column_stack([x, y])
    segment = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    loop_length = float(segment.sum())
    count = max(64, int(round(loop_length / spacing)))
    parameter = np.linspace(0.0, loop_length, count, endpoint=False)
    cumulative = np.concatenate(([0.0], np.cumsum(segment)))
    controls = np.empty((count, 2))
    for index, target in enumerate(parameter):
        segment_index = int(np.clip(np.searchsorted(cumulative, target, side="right") - 1, 0, length - 1))
        fraction = (target - cumulative[segment_index]) / segment[segment_index]
        controls[index] = points[segment_index] + (
            points[(segment_index + 1) % length] - points[segment_index]
        ) * fraction

    spline, _ = splprep(
        [controls[:, 0], controls[:, 1]],
        u=np.arange(count) / count,
        s=smoothing,
        per=1,
        k=3,
    )
    dense_count = max(4096, length * 4)
    dense_u = np.linspace(0.0, 1.0, dense_count, endpoint=True)
    dense_x, dense_y = splev(dense_u, spline)
    dense_x[-1] = dense_x[0]
    dense_y[-1] = dense_y[0]
    dense = np.column_stack([dense_x, dense_y])
    dense_length = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))))
    if dense_length[-1] <= 1e-6:
        raise SystemExit("smoothed centreline has no length")

    target_s = np.arange(length, dtype=float)
    dense_s = dense_length / dense_length[-1] * length
    new_x = np.interp(target_s, dense_s, dense_x)
    new_y = np.interp(target_s, dense_s, dense_y)
    new_x -= new_x[0]
    new_y -= new_y[0]
    return new_x, new_y


def main() -> None:
    profile = json.loads(PROFILE.read_text())
    rows = read_track(TRACK)
    length = len(rows)
    if length < 100:
        raise SystemExit(f"{TRACK} has too few rows: {length}")

    x = np.array([row[0] for row in rows])
    y = np.array([row[1] for row in rows])
    z_old = np.array([row[2] for row in rows])
    width = np.array([row[3] for row in rows])
    runoff_left = np.array([row[4] for row in rows])
    runoff_right = np.array([row[5] for row in rows])
    bank_old = np.degrees(np.arctan([row[6] for row in rows]))
    crown = np.array([row[7] for row in rows])

    points = np.column_stack([x, y])
    spacing = float(profile["centerline"]["resampleSpacingMeters"])
    smoothing = float(profile["centerline"]["smoothing"])
    new_x, new_y = centreline_points(x, y, length, spacing, smoothing)
    target_s = np.arange(length, dtype=float)

    # Manual grade: a periodic cubic through the authored (s, z) knots.
    knots = sorted(profile["grade"]["controlPoints"], key=lambda knot: knot["s"])
    knot_s = [float(knot["s"]) for knot in knots]
    knot_z = [float(knot["z"]) for knot in knots]
    if knot_s[0] > 0.0:
        knot_s.insert(0, 0.0)
        knot_z.insert(0, knot_z[0])
    knot_s.append(float(length))
    knot_z.append(knot_z[0])
    grade_spline = CubicSpline(np.asarray(knot_s), np.asarray(knot_z), bc_type="periodic")
    new_z = grade_spline(target_s)

    # Manual banking: base crossfall plus smooth, fixed-direction bank sections.
    base_slope = float(profile["bank"].get("baseSlope", 0.0))
    new_bank_slope = np.full(length, base_slope, dtype=float)
    for section in profile["bank"]["sections"]:
        peak = math.tan(math.radians(float(section["peakDegrees"]))) * float(section["direction"])
        start = float(section["start"])
        end = float(section["end"])
        blend = float(section["blendMeters"])
        for index, s in enumerate(target_s):
            new_bank_slope[index] += peak * closed_section_weight(s, start, end, blend, length)

    # Report the things that matter for continuity.
    tree = cKDTree(np.column_stack([new_x, new_y]))
    distance, _ = tree.query(points)
    grade = np.gradient(new_z)
    bank_degrees = np.degrees(np.arctan(new_bank_slope))
    print(f"length={length} m")
    print(f"centreline: max deviation={distance.max():.3f} m, mean={distance.mean():.3f} m")
    print(f"grade: max={np.max(np.abs(grade)) * 100:.2f}%")
    print(f"bank: max={np.max(np.abs(bank_degrees)):.2f} deg, max rate={np.max(np.abs(np.diff(bank_degrees))):.3f} deg/m")

    header = (
        f"# zandvoort: {length} m, one row a metre; "
        "traced centreline + manual grade/bank from surface.json\n"
        "x_m,y_m,z_m,width_m,runoff_left_m,runoff_right_m,bank_slope,crown\n"
    )
    with TRACK.open("w") as handle:
        handle.write(header)
        for index in range(length):
            handle.write(
                f"{new_x[index]:.5f},{new_y[index]:.5f},{new_z[index]:.4f},"
                f"{width[index]:.4f},{runoff_left[index]:.3f},{runoff_right[index]:.3f},"
                f"{new_bank_slope[index]:.6f},{crown[index]:.7f}\n"
            )

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))
    axes[0, 0].plot(x, y, "--", lw=1.2, color="#0072B2", label="current (traced reference)")
    axes[0, 0].plot(new_x, new_y, "-", lw=1.2, color="#F5793A", label="s=50 smoothed")
    axes[0, 0].set_aspect("equal")
    axes[0, 0].legend()
    axes[0, 0].set_title(f"Zandvoort centreline: max deviation {distance.max():.2f} m")
    axes[0, 1].plot(target_s, z_old, "--", lw=1.0, color="#0072B2", label="current z")
    axes[0, 1].plot(target_s, new_z, "-", lw=1.4, color="#F5793A", label="manual grade")
    axes[0, 1].set_ylabel("z (m)")
    axes[0, 1].legend()
    axes[0, 1].set_title("grade: manual periodic cubic")
    axes[1, 0].plot(target_s, np.gradient(new_z) * 100.0, lw=0.8, color="#009E73")
    axes[1, 0].set_xlabel("s (m)")
    axes[1, 0].set_ylabel("grade (%)")
    axes[1, 0].set_title("grade profile")
    axes[1, 1].plot(target_s, bank_old, "--", lw=1.0, color="#0072B2", label="current bank")
    axes[1, 1].plot(target_s, bank_degrees, "-", lw=1.4, color="#F5793A", label="manual bank")
    axes[1, 1].set_xlabel("s (m)")
    axes[1, 1].set_ylabel("bank (deg)")
    axes[1, 1].legend()
    axes[1, 1].set_title("bank: 19 deg / 18 deg with 80/90 m blends")
    for axis in axes.flat:
        axis.grid(alpha=0.25)
    fig.tight_layout()
    preview = Path(tempfile.gettempdir()) / "zandvoort-rebuild.png"
    fig.savefig(preview, dpi=130)
    print("wrote", TRACK)
    print(f"preview {preview}")


if __name__ == "__main__":
    main()
