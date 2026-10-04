#!/usr/bin/env python3
"""Bake closed 3D racing lines with TUMRT/online_3D_racing_line_planning."""

from __future__ import annotations

import os
import sys

import numpy as np
import pandas as pd

os.environ.setdefault("MPLBACKEND", "Agg")

BAKE_DIR = os.path.dirname(os.path.abspath(__file__))
TUM_ROOT = os.path.join(BAKE_DIR, "tumrt")
RAW_DIR = os.path.join(BAKE_DIR, "raw")
WORK_DIR = os.path.join(BAKE_DIR, "work")
OUT_DIR = os.path.join(BAKE_DIR, "..", "Tracks")

sys.path.insert(0, os.path.join(TUM_ROOT, "src"))
sys.path.insert(0, os.path.join(TUM_ROOT, "global_racing_line"))

from gen_global_racing_line import calc_global_raceline  # noqa: E402
from track3D import Track3D  # noqa: E402


def lerp_env(v: float, a12: float, a40: float, a70: float) -> float:
    if v <= 12.0:
        return a12
    if v <= 40.0:
        return a12 + (a40 - a12) * (v * v - 144.0) / (1600.0 - 144.0)
    if v >= 70.0:
        return a70
    return a40 + (a70 - a40) * (v * v - 1600.0) / (4900.0 - 1600.0)


def synth_gg(path: str) -> None:
    os.makedirs(path, exist_ok=True)
    v_list = np.linspace(10.0, 90.0, 17)
    g_list = np.linspace(0.5 * 9.81, 3.5 * 9.81, 13)
    alpha_list = np.linspace(-np.pi, np.pi, 250)
    n_v, n_g = v_list.size, g_list.size
    ay_max = np.zeros((n_v, n_g))
    ax_min = np.zeros((n_v, n_g))
    ax_max = np.zeros((n_v, n_g))
    gg_exponent = np.full((n_v, n_g), 1.6)
    rho = np.zeros((n_v, n_g, alpha_list.size))
    for i, v in enumerate(v_list):
        ay_flat = lerp_env(v, 15.6, 23.3, 40.8)
        ax_drive = lerp_env(v, 9.2, 11.9, 6.8)
        ax_brake = lerp_env(v, 17.3, 25.9, 40.0)
        for j, g in enumerate(g_list):
            scale = g / 9.81
            ay = ay_flat * scale
            axn = ax_brake * scale
            axp = ax_drive
            ay_max[i, j] = ay
            ax_min[i, j] = -axn
            ax_max[i, j] = axp
            for k, alpha in enumerate(alpha_list):
                c, s = np.cos(alpha), np.sin(alpha)
                if abs(c) < 1e-9:
                    rho[i, j, k] = ay
                    continue
                # diamond: |ax/ax_lim|^p + |ay/ay_lim|^p <= 1
                ax_lim = axp if s >= 0.0 else axn
                t = abs(np.tan(alpha))
                ay_d = ax_lim / ((t ** 1.6 + (ax_lim / max(ay, 1e-3)) ** 1.6) ** (1.0 / 1.6))
                ax_d = ay_d * t
                if s > 0.0:
                    cap = axp / max(abs(s), 1e-6)
                    r = min(np.hypot(ax_d, ay_d), cap)
                else:
                    r = np.hypot(ax_d, ay_d)
                rho[i, j, k] = r
    np.save(os.path.join(path, "v_list.npy"), v_list)
    np.save(os.path.join(path, "g_list.npy"), g_list)
    np.save(os.path.join(path, "alpha_list.npy"), alpha_list)
    np.save(os.path.join(path, "rho.npy"), rho)
    np.save(os.path.join(path, "gg_exponent.npy"), gg_exponent)
    np.save(os.path.join(path, "ax_min.npy"), ax_min)
    np.save(os.path.join(path, "ax_max.npy"), ax_max)
    np.save(os.path.join(path, "ay_max.npy"), ay_max)


def kappa_from_xy(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    n = x.size
    kappa = np.zeros(n)
    chord = 3
    for i in range(n):
        i0 = (i - chord) % n
        i1 = i
        i2 = (i + chord) % n
        ux, uy = x[i1] - x[i0], y[i1] - y[i0]
        vx, vy = x[i2] - x[i1], y[i2] - y[i1]
        a = np.hypot(ux, uy)
        b = np.hypot(vx, vy)
        c = np.hypot(x[i2] - x[i0], y[i2] - y[i0])
        if min(a, b, c) < 1e-3:
            continue
        twice = ux * vy - uy * vx
        kappa[i] = 2.0 * twice / (a * b * c)
    return kappa


def bake_track(track_id: str) -> None:
    raw = os.path.join(RAW_DIR, f"{track_id}_centerline_banking.csv")
    three_d_dir = os.path.join(WORK_DIR, "3d_track_data")
    smooth_dir = os.path.join(WORK_DIR, "smoothed_track_data")
    line_dir = os.path.join(WORK_DIR, "global_racing_lines")
    os.makedirs(three_d_dir, exist_ok=True)
    os.makedirs(smooth_dir, exist_ok=True)
    os.makedirs(line_dir, exist_ok=True)
    three_d_path = os.path.join(three_d_dir, f"{track_id}_3d.csv")
    handler = Track3D()
    handler.generate_3d_from_2d_reference_line(
        path=raw,
        out_path=three_d_path,
        ignore_banking=False,
        visualize=False,
    )
    smoothed = os.path.join(smooth_dir, f"{track_id}_3d.csv")
    pd.read_csv(three_d_path).to_csv(smoothed, index=False)
    vehicle = {
        "m": 820.0,
        "h": 0.32,
        "total_width": 1.9,
        "v_max": 90.0,
    }
    sol_opts = {
        "ipopt.max_iter": 3000,
        "ipopt.hessian_approximation": "limited-memory",
        "ipopt.mu_strategy": "adaptive",
        "ipopt.acceptable_iter": 15,
        "ipopt.acceptable_tol": 1e-3,
        "ipopt.acceptable_dual_inf_tol": 1e-2,
        "ipopt.constr_viol_tol": 1e-4,
        "ipopt.print_level": 3,
    }
    raceline_csv = os.path.join(line_dir, f"{track_id}_timeoptimal.csv")
    raceline = calc_global_raceline(
        track_name=f"{track_id}_3d.csv",
        vehicle_params=vehicle,
        gg_mode="diamond",
        gg_margin=0.0,
        safety_distance=0.25,
        w_T=1.0,
        w_jx=1e-2,
        w_jy=1e-2,
        w_dOmega_z=0.0,
        RK4_steps=1,
        V_guess=22.0,
        neglect_w_omega_x=True,
        neglect_w_omega_y=True,
        neglect_euler=True,
        neglect_centrifugal=True,
        neglect_w_dot=False,
        neglect_V_omega=False,
        sol_opt=sol_opts,
        out_path=raceline_csv,
    )
    track = Track3D(path=smoothed)
    normal = track.get_normal_vector_numpy(theta=track.theta, mu=track.mu, phi=track.phi)
    n_opt = raceline["n_opt"].to_numpy()
    v_opt = raceline["v_opt"].to_numpy()
    x = track.x + normal[0] * n_opt
    y = track.y + normal[1] * n_opt
    # TUM n is left-positive. Our TrackD is right-positive.
    offset = -n_opt
    kappa = kappa_from_xy(x, y)
    out = pd.DataFrame(
        {
            "s_m": raceline["s_opt"].to_numpy(),
            "offset_m": offset,
            "kappa_radpm": kappa,
            "v_mps": v_opt,
            "x_m": x,
            "y_m": y,
        }
    )
    os.makedirs(OUT_DIR, exist_ok=True)
    dest = os.path.join(OUT_DIR, f"{track_id}.line.csv")
    out.to_csv(dest, index=False, float_format="%.6f")
    lap = float(raceline["laptime"].iloc[0])
    print(f"{track_id}: baked {dest}  TUM lap {lap:.2f}s  n in [{offset.min():.2f},{offset.max():.2f}] v {v_opt.min():.1f}-{v_opt.max():.1f}")


def main() -> None:
    gg_path = os.path.join(WORK_DIR, "gg_diagrams", "stintegy", "velocity_frame")
    synth_gg(gg_path)
    # Point the TUM solver at our work dir by matching its expected relatives.
    # calc_global_raceline uses paths next to gen_global_racing_line.py.
    # Patch those globals after import.
    import gen_global_racing_line as ggr

    ggr.track_path = os.path.join(WORK_DIR, "smoothed_track_data")
    ggr.gg_diagram_path = gg_path
    tracks = sys.argv[1:] or ["simple", "silverstone"]
    for track_id in tracks:
        bake_track(track_id)


if __name__ == "__main__":
    main()
