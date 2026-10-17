#!/usr/bin/env python3
"""Offline 3D lane bake for the three-lane driver, with TUM's planner.

Nothing here runs inside the game. The C# exporter (ExportTracks) writes each
circuit's two edges in 3D and this car's grip against speed and apparent
gravity; this script turns them into the bake's 3D track, smooths it, and
solves the time-optimal line three times: on the whole road, and on the road
with a car's width taken off either side. The lines come back as plain x, y, z
points, which the driver projects onto its own track when it imports them.

    python bake_lanes_3d.py --raw <dir with exports> --out <dir> silverstone spa

Needs casadi (IPOPT ships with it), numpy, scipy, pandas, pyyaml.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import pandas as pd

os.environ.setdefault("MPLBACKEND", "Agg")

BAKE_DIR = os.path.dirname(os.path.abspath(__file__))
TUM_ROOT = os.path.join(BAKE_DIR, "tumrt")
sys.path.insert(0, os.path.join(TUM_ROOT, "src"))
sys.path.insert(0, os.path.join(TUM_ROOT, "global_racing_line"))

import gen_global_racing_line as ggr  # noqa: E402
from track3D import Track3D  # noqa: E402

# Smoothing weights as TUM ships them for its own tracks.
SMOOTHING = {
    "w_c": 1e0, "w_l": 1e0, "w_r": 1e0,
    "w_theta": 1e7, "w_mu": 1e5, "w_phi": 1e4,
    "w_nl": 1e-2, "w_nr": 1e-2,
    # Ours, on top of TUM's: the heading stiffness eases with the measured
    # curvature -- a 160 m corner takes half of it, a 9 m hairpin a three
    # hundredth -- and the result is refused if either smoothed edge leaves
    # the measured one by more than max_edge_dev. TUM's constant stiffness,
    # tuned on Las Vegas and Bathurst, moved Monaco's hairpin edges by two
    # metres; this holds them to 0.2 m with the straights as smooth as before.
    # The same limit as a hard constraint on the solve (edge_constraint) was
    # tried and fought the smoothness for over half an hour; it is not needed.
    # Half a metre: a few tenths off the measured edge is well inside what
    # the driver notices; holding the solve to a quarter cost convergence.
    "kappa_ref": 1.0 / 160.0, "kappa_window_m": 10.0,
    "max_edge_dev": 0.5, "edge_constraint": False,
}

# How much slower than the free lane a side lane may be and still be a
# line rather than a solver that never left its starting guess.
SIDE_LANE_MAX_SLOWDOWN = 1.10

SOLVER = {
    # Side lanes, with a car's width taken off one edge, converge more slowly
    # than the free lane; 5000 was not always enough for them.
    "ipopt.max_iter": 20000,
    "ipopt.mu_strategy": "adaptive",
    # TUM ships limited-memory. Side lanes, which run along a moved edge for
    # long stretches, circled for over an hour with it; exact second
    # derivatives cost more per step and take far fewer of them.
    "ipopt.hessian_approximation": "exact",
    "ipopt.line_search_method": "cg-penalty",
    # A lap is tens of seconds and a station's line is metres: once every
    # constraint holds and the lap time has stopped moving, the last digits
    # of stationarity buy nothing. Diagnosed on Daytona's right lane: after
    # 1500 iterations the bounds were met exactly and the worst constraint
    # was off by 1.5e-6, yet the solver was still chasing the default
    # dual tolerance. Stop there and keep the line.
    "ipopt.acceptable_tol": 1e-3,
    "ipopt.acceptable_dual_inf_tol": 1e-1,
    "ipopt.acceptable_compl_inf_tol": 1e-3,
    "ipopt.acceptable_constr_viol_tol": 1e-4,
    "ipopt.acceptable_obj_change_tol": 1e-6,
    "ipopt.acceptable_iter": 10,
    "ipopt.constr_viol_tol": 1e-4,
    # One line per iteration, so a slow lane shows whether it is closing in.
    "ipopt.print_level": 5,
    "ipopt.print_frequency_iter": 50,
    "print_time": 0,
}


def write_gg(csv_path: str, out_dir: str, exponent: float) -> None:
    """The exporter's table as the arrays TUM's GGManager loads."""
    table = pd.read_csv(csv_path)
    v_list = np.sort(table["v_mps"].unique())
    g_list = np.sort(table["g_mps2"].unique())
    shape = (v_list.size, g_list.size)
    grid = table.set_index(["v_mps", "g_mps2"])
    ax_max = np.zeros(shape)
    ax_min = np.zeros(shape)
    ay_max = np.zeros(shape)
    for i, v in enumerate(v_list):
        for j, g in enumerate(g_list):
            row = grid.loc[(v, g)]
            ax_max[i, j] = row["ax_max"]
            ax_min[i, j] = row["ax_min"]
            ay_max[i, j] = row["ay_max"]

    # The polar form is loaded even when only the diamond is used; give it
    # the same ellipse.
    alpha_list = np.linspace(-np.pi, np.pi, 250)
    rho = np.zeros(shape + (alpha_list.size,))
    for k, alpha in enumerate(alpha_list):
        s, c = np.sin(alpha), np.cos(alpha)
        # Past its top speed the car gains nothing: the polar form needs a
        # radius, so it is given a sliver there.
        ax_lim = np.where(s >= 0.0, np.maximum(ax_max, 0.1), -ax_min)
        rho[:, :, k] = 1.0 / np.sqrt((s / ax_lim) ** 2 + (c / ay_max) ** 2)

    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, "v_list.npy"), v_list)
    np.save(os.path.join(out_dir, "g_list.npy"), g_list)
    np.save(os.path.join(out_dir, "alpha_list.npy"), alpha_list)
    np.save(os.path.join(out_dir, "rho.npy"), rho)
    np.save(os.path.join(out_dir, "gg_exponent.npy"), np.full(shape, exponent))
    np.save(os.path.join(out_dir, "ax_min.npy"), ax_min)
    np.save(os.path.join(out_dir, "ax_max.npy"), ax_max)
    np.save(os.path.join(out_dir, "ay_max.npy"), ay_max)


def bake(track_id: str, args: argparse.Namespace) -> None:
    work = os.path.join(args.out, "work")
    three_d = os.path.join(work, "3d", f"{track_id}.csv")
    smoothed_dir = os.path.join(work, "smoothed")
    os.makedirs(os.path.dirname(three_d), exist_ok=True)
    os.makedirs(smoothed_dir, exist_ok=True)
    os.makedirs(os.path.join(work, "lines"), exist_ok=True)

    started = time.time()
    Track3D().generate_3d_from_3d_track_bounds(
        path=os.path.join(args.raw, f"{track_id}_bounds_3d.csv"),
        out_path=three_d,
        ignore_banking=False,
        visualize=False,
    )
    smoothed = os.path.join(smoothed_dir, f"{track_id}.csv")
    Track3D().smooth_track(
        out_path=smoothed, weights=SMOOTHING, step_size=args.step, visualize=False, in_path=three_d
    )
    base = pd.read_csv(smoothed)

    ggr.track_path = smoothed_dir
    ggr.gg_diagram_path = os.path.join(args.out, "gg")
    rows = []
    free_lap = None
    solved = {}
    refused = []
    for lane in args.lanes:
        frame = base.copy()
        # The bake's n is positive to the left of travel. Our "left" lane is
        # our positive offset, which is the bake's right, and it leaves its
        # room on the other side: the bake's left.
        if lane == "left":
            frame["w_tr_left_m"] = frame["w_tr_left_m"] - args.side_room
        elif lane == "right":
            frame["w_tr_right_m"] = frame["w_tr_right_m"] + args.side_room
        name = f"{track_id}_{lane}.csv"
        frame.to_csv(os.path.join(smoothed_dir, name), index=False)

        line_path = os.path.join(work, "lines", name)
        if args.reuse and os.path.exists(line_path):
            line = pd.read_csv(line_path, index_col=0)
            print(f"{track_id} {lane}: reusing {line_path}", flush=True)
        else:
            try:
                start = None
                if args.warm and solved:
                    start = solved.get("free") if lane != "free" and "free" in solved else next(iter(solved.values()))
                line = solve(name, line_path, args, init_line=start)
            except RuntimeError as why:
                # One lane that will not bake is not a reason to lose the
                # others; it is tried again below from a lane that did.
                print(f"{track_id} {lane}: refused, {why}", flush=True)
                refused.append((lane, name, line_path))
                continue
        solved[lane] = line
        rows += lane_rows(track_id, lane, line, smoothed_dir, name, started)

    # A lane that would not bake from a cold start is tried again from a
    # lane that did: free first, which is feasible for either side once
    # clipped, else whichever side baked.
    for lane, name, line_path in refused:
        start = solved.get("free") if lane != "free" else None
        if start is None:
            start = next((solved[k] for k in ("free", "left", "right") if k in solved and k != lane), None)
        if start is None:
            print(f"{track_id} {lane}: no baked lane to start from; left to the driver's own bake", flush=True)
            continue
        try:
            line = solve(name, line_path, args, init_line=start)
        except RuntimeError as why:
            print(f"{track_id} {lane}: refused again from a baked start, {why}", flush=True)
            continue
        solved[lane] = line
        rows += lane_rows(track_id, lane, line, smoothed_dir, name, started)

    free_lap = float(solved["free"]["laptime"].iloc[0]) if "free" in solved else None
    kept = []
    for row in rows:
        lane, lap = row[0], row[7]
        if lane != "free" and free_lap is not None and lap > free_lap * SIDE_LANE_MAX_SLOWDOWN:
            continue
        kept.append(row)
    for lane in ("left", "right"):
        if lane in solved and free_lap is not None and float(solved[lane]["laptime"].iloc[0]) > free_lap * SIDE_LANE_MAX_SLOWDOWN:
            # A side lane gives up a strip of road, not a tenth of the lap.
            # One this much slower never got away from its starting guess
            # (Monaco's left lane once sat at the guessed 30 m/s for the
            # whole lap), so it is refused, and its line is removed so that
            # a --reuse run does not pick it up.
            print(f"{track_id} {lane}: refused, {float(solved[lane]['laptime'].iloc[0]):.2f}s against the free lane's {free_lap:.2f}s", flush=True)
            os.remove(os.path.join(work, "lines", f"{track_id}_{lane}.csv"))
    rows = kept

    out = pd.DataFrame(rows, columns=["lane", "s_m", "x_m", "y_m", "z_m", "n_bake_m", "v_bake_mps", "bake_lap_s"])
    out_path = os.path.join(args.out, f"{track_id}.lanes3d.csv")
    out.to_csv(out_path, index=False, float_format="%.6f")
    print(f"{track_id}: wrote {out_path}", flush=True)


def lane_rows(track_id, lane, line, smoothed_dir, name, started):
    lap = float(line["laptime"].iloc[0])
    track = Track3D(path=os.path.join(smoothed_dir, name))
    xyz = track.sn2cartesian(line["s_opt"].to_numpy(), line["n_opt"].to_numpy())
    print(f"{track_id} {lane}: bake lap {lap:.2f}s  n [{line['n_opt'].min():.2f}, {line['n_opt'].max():.2f}]"
          f"  v [{line['v_opt'].min():.1f}, {line['v_opt'].max():.1f}]  ({time.time() - started:.0f}s)", flush=True)
    return [(lane, line["s_opt"].iloc[k], xyz[k, 0], xyz[k, 1], xyz[k, 2],
             line["n_opt"].iloc[k], line["v_opt"].iloc[k], lap) for k in range(len(line))]



def solve(name: str, line_path: str, args: argparse.Namespace, init_line=None) -> pd.DataFrame:
    options = dict(SOLVER)
    options["ipopt.max_wall_time"] = float(args.max_minutes) * 60.0
    if args.line_search:
        options["ipopt.line_search_method"] = args.line_search
    return ggr.calc_global_raceline(
            track_name=name,
            vehicle_params={"total_width": args.width, "h": 0.32},
            gg_mode="diamond",
            gg_margin=0.0,
            safety_distance=args.safety,
            w_T=1.0,
            w_jx=1e-2,
            w_jy=1e-2,
            w_dOmega_z=0.0,
            RK4_steps=1,
            V_guess=args.v_guess,
            neglect_w_omega_x=True,
            neglect_w_omega_y=True,
            neglect_euler=True,
            neglect_centrifugal=True,
            neglect_w_dot=False,
            neglect_V_omega=False,
            sol_opt=options,
            out_path=line_path,
            init_line=init_line,
            # The friction circle is an ellipse of exponent two; say it the
            # smooth way (see gen_global_racing_line).
            smooth_ellipse=args.exponent == 2.0,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("tracks", nargs="+")
    parser.add_argument("--raw", required=True, help="directory the C# exporter wrote to")
    parser.add_argument("--out", required=True)
    parser.add_argument("--lanes", nargs="+", default=["free", "left", "right"])
    parser.add_argument("--step", type=float, default=2.0, help="station spacing of the smoothed track, m")
    # Centre of the car to the edge: half the track width of the wheels, so
    # the wheels can use the line, as the driver's own bake does.
    parser.add_argument("--width", type=float, default=1.7)
    parser.add_argument("--safety", type=float, default=0.0)
    parser.add_argument("--side-room", type=float, default=3.0)
    parser.add_argument("--reuse", action="store_true", help="keep lines already baked in the work directory")
    parser.add_argument("--max-minutes", type=float, default=30.0, help="give up on a lane after this long")
    parser.add_argument("--exponent", type=float, default=2.0, help="friction ellipse; the physics combines as a circle")
    # A lane that sits at a nearly feasible point for the whole time budget
    # (Spa's free lane, from 30 m/s) can be started differently.
    parser.add_argument("--warm", action="store_true", help="start each lane from one already baked, when there is one")
    parser.add_argument("--v-guess", type=float, default=30.0, help="speed the solve starts from, m/s")
    parser.add_argument("--line-search", default="", help="IPOPT line search; empty keeps cg-penalty, 'filter' is IPOPT's default")
    args = parser.parse_args()

    write_gg(os.path.join(args.raw, "stintegy_gg.csv"), os.path.join(args.out, "gg"), args.exponent)
    for track_id in args.tracks:
        bake(track_id, args)


if __name__ == "__main__":
    main()
