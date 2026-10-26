#!/usr/bin/env python3
"""Offline lane bake by minimum curvature, with TUM's trajectory_planning_helpers.

The lanes are drawn on the circuit's own centreline and widths (Circuits/<id>/track.csv),
each as the line that bends least within its part of the road: the free lane on the
whole road, the left and right lanes each on their own half, the middle lane within
a few metres of the centre. Speeds are not baked: the driver plans them from the car's
envelope and the learner corrects them, so a lane here is only where to drive.

Against TUM's 3D minimum-time bake (bake_lanes_3d.py) on Silverstone, learned, the
free lane was 0.4 s a lap slower, followed more closely, and straight on the
straights, where the minimum-time line drifts from side to side because there it
costs no time; it bakes in a minute or two a lane rather than half an hour. What it cannot
see is grip: a corner banked more steeply on the outside still gets the inside line.

    python bake_lanes_mincurv.py silverstone spa --out Drivers/Bake/lanes

Needs numpy, scipy, trajectory-planning-helpers and quadprog (on macOS, build
quadprog from source: pip install --no-binary quadprog quadprog).
"""

from __future__ import annotations

import argparse
import csv
import os

import numpy as np
import trajectory_planning_helpers as tph
from scipy.interpolate import CubicSpline

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))


def centreline(track_id: str):
    path = os.path.join(REPO, "Circuits", track_id, "track.csv")
    rows = [r for r in csv.reader(line for line in open(path) if not line.startswith("#"))][1:]
    xy = np.array([[float(r[0]), float(r[1])] for r in rows])
    width = np.array([float(r[3]) for r in rows])
    tangent = np.gradient(xy, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    left = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    return xy, width, left


def solve(ref_xy, sign, lo, hi, car_width, step):
    """The least-bending line with the car's centre between lo and hi metres
    left of the centreline at each reference point."""
    # The solver's offset runs along its own normal; where that points right,
    # left of the centreline is its negative.
    a_lo = np.where(sign > 0, lo, -hi)
    a_hi = np.where(sign > 0, hi, -lo)
    reftrack = np.column_stack([ref_xy, a_hi + car_width / 2, car_width / 2 - a_lo])
    cx, cy, a, normals = tph.calc_splines.calc_splines(path=np.vstack([reftrack[:, :2], reftrack[0, :2]]))
    lengths = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=cx, coeffs_y=cy)
    psi, kappa, dkappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=cx, coeffs_y=cy, ind_spls=np.arange(len(reftrack)), t_spls=np.zeros(len(reftrack)), calc_dcurv=True)
    # TUM's iterative form: the problem is linearised about the reference,
    # so it is solved again about each answer until the curvature agrees.
    alpha, ref, normals_out = tph.iqp_handler.iqp_handler(
        reftrack=reftrack, normvectors=normals, A=a, spline_len=lengths, psi=psi, kappa=kappa, dkappa=dkappa,
        kappa_bound=0.25, w_veh=car_width, print_debug=False, plot_debug=False,
        stepsize_interp=step, iters_min=5, curv_error_allowed=0.01)[:3]
    return ref[:, :2] + normals_out * alpha[:, None]


def resample(points, spacing=2.0):
    loop = np.vstack([points, points[0]])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(loop, axis=0).T))])
    spline = CubicSpline(s, loop, bc_type="periodic")
    return spline(np.linspace(0.0, s[-1], int(s[-1] / spacing), endpoint=False))


def bake(track_id: str, args: argparse.Namespace) -> None:
    xy, width, left = centreline(track_id)
    every = max(1, int(round(args.step)))
    idx = np.arange(0, len(xy), every)
    ref_xy = xy[idx]
    half = width[idx] / 2
    _, _, _, normals = tph.calc_splines.calc_splines(path=np.vstack([ref_xy, ref_xy[0]]))
    sign = np.sign(np.einsum("ij,ij->i", normals, left[idx]))

    edge = half - args.width / 2 - args.safety
    bounds = {
        "free": (-edge, edge),
        "left": (np.full_like(half, args.avoid), edge),
        "right": (-edge, np.full_like(half, -args.avoid)),
        "middle": (np.maximum(-edge, -args.middle), np.minimum(edge, args.middle)),
    }
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, f"{track_id}.lanes3d.csv")
    with open(path, "w") as out:
        out.write("lane,s_m,x_m,y_m\n")
        for lane in args.lanes:
            lo, hi = bounds[lane]
            points = resample(solve(ref_xy, sign, lo, hi, args.width + 2 * args.safety, args.step))
            s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(points, axis=0).T))])
            for si, (x, y) in zip(s, points):
                out.write(f"{lane},{si:.3f},{x:.4f},{y:.4f}\n")
            print(f"{track_id} {lane}: {s[-1]:.0f} m", flush=True)
    print(f"{track_id} -> {path}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("tracks", nargs="+")
    parser.add_argument("--out", default=os.path.join(REPO, "Drivers", "Bake", "lanes"))
    parser.add_argument("--lanes", nargs="+", default=["free", "left", "right", "middle"])
    parser.add_argument("--step", type=float, default=3.0, help="spacing of the reference points, m")
    # Centre of the car to the edge: half a two-metre FIA car plus a tenth,
    # so the body stays off the white line, as the 3D bake keeps it.
    parser.add_argument("--width", type=float, default=2.0)
    parser.add_argument("--safety", type=float, default=0.1)
    parser.add_argument("--avoid", type=float, default=0.5, help="a side lane's inner edge, metres past the centre on its own side")
    parser.add_argument("--middle", type=float, default=2.5, help="the middle lane keeps the car's centre this close to the road's, m (USR's)")
    args = parser.parse_args()
    for track_id in args.tracks:
        bake(track_id, args)


if __name__ == "__main__":
    main()
