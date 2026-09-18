"""Where the lap's speed is: corner minimums, and straights against the car.

Certification's straight-line and cornering readings from one nominal run.
Speed is binned by position round the lap from the race distance the
protocol already reports; the corners are named from the curvature the
cars actually drove (signed, averaged, then made absolute, so a straight
driven with corrections reads as a straight). For every corner the slowest
bin is its minimum speed; for every straight the fastest bin is its top
speed, set beside the flat-road terminal speed the host computes for the
same power rung. A long straight that tops out well short of that figure is
one the car lifted on or never had the length to finish.

    python3 lap_shape.py checkpoints/evalparent2h-1225000.pt --modes 3,3
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess

import numpy as np

from host_env import DEFAULT_HOST_PROJECT, HostEnv
from sac import SacAgent, SacConfig
from spin_forensics import _speed, _yaw_rate, build_corner_map, CORNER_BIN_METRES
from train import STEP_SECONDS, TRACKS, assert_observation_layout


def terminal_speeds() -> dict[str, float]:
    """The host's flat-road top speed for each power rung, as JSON."""
    output = subprocess.run(
        ["dotnet", "run", "-c", "Release", "--project", DEFAULT_HOST_PROJECT,
         "--", "--terminal-speed"],
        check=True, capture_output=True, text=True,
    ).stdout
    line = next(l for l in output.splitlines() if l.startswith("{"))
    return json.loads(line)


def measure(policy: str, track: str, lanes: int, seconds: float, seed: int,
            modes: tuple[int, int]) -> dict:
    lap_metres, _ = TRACKS[track]
    bins = max(1, int(round(lap_metres / CORNER_BIN_METRES)))
    metres = np.zeros(bins)
    time = np.zeros(bins)
    curvature_sum = np.zeros(bins)
    curvature_count = np.zeros(bins)
    with HostEnv(batch=lanes, seed_base=seed, solo=True, track=track,
                 episode_seconds=seconds + 60.0, ego_modes=modes) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
        agent.load(policy)
        obs = env.reset()
        assert_observation_layout(obs)
        previous: np.ndarray | None = None
        for _ in range(int(round(seconds / STEP_SECONDS))):
            action = agent.act(obs, deterministic=True)
            obs, _, done, _, _, race, final_obs, _ = env.step(action)
            race = np.asarray(race, dtype=np.float64)
            speed = _speed(final_obs)
            yaw = _yaw_rate(final_obs)
            if previous is not None:
                for lane in range(lanes):
                    moved = race[lane] - previous[lane]
                    if done[lane] or moved <= 0.0:
                        continue
                    mid = (previous[lane] + moved * 0.5) % lap_metres
                    b = min(int(mid / CORNER_BIN_METRES), bins - 1)
                    metres[b] += moved
                    time[b] += STEP_SECONDS
                    if speed[lane] > 5.0:
                        curvature_sum[b] += yaw[lane] / speed[lane]
                        curvature_count[b] += 1.0
            previous = race.copy()
    binned = np.where(time > 0, metres / np.maximum(time, 1e-9), np.nan)
    return {
        "bin_metres": CORNER_BIN_METRES,
        "speed": binned,
        "corners": build_corner_map(curvature_sum, curvature_count, lap_metres),
        "lap_metres": lap_metres,
    }


def span_bins(start_m: float, end_m: float, bin_metres: float, bins: int) -> list[int]:
    first = int(start_m / bin_metres) % bins
    last = int(end_m / bin_metres) % bins
    out, b = [], first
    while True:
        out.append(b)
        if b == last:
            return out
        b = (b + 1) % bins


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("policy")
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--lanes", type=int, default=12)
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--seed-base", type=int, default=900_001)
    parser.add_argument("--modes", default="3,3")
    parser.add_argument("--min-straight", type=float, default=200.0,
                        help="shortest straight to print; all are kept in --json")
    parser.add_argument("--json", default=None)
    args = parser.parse_args()
    tyre, power = (int(x) for x in args.modes.split(","))

    top = terminal_speeds()
    # On a flat straight the tyre rung never binds -- the host reports 3,5
    # and 5,5 as the same speed -- so the power rung alone picks the figure.
    theory = top[f"3,{power}"]
    reference = top.get("5,5")
    r = measure(args.policy, args.track, args.lanes, args.seconds, args.seed_base,
                (tyre, power))
    speed, corners, bm = r["speed"], r["corners"], r["bin_metres"]
    bins = speed.size

    rows_corners = []
    for c in corners:
        idx = span_bins(c["start_m"], c["end_m"], bm, bins)
        values = [(speed[b], b) for b in idx if math.isfinite(speed[b])]
        if not values:
            continue
        minimum, at = min(values)
        rows_corners.append({
            "corner": c["index"], "start_m": c["start_m"], "end_m": c["end_m"],
            "min_speed_mps": float(minimum), "min_at_m": at * bm,
            "peak_curvature": c["peak_curvature"],
        })

    rows_straights = []
    for i, c in enumerate(corners):
        following = corners[(i + 1) % len(corners)]
        idx = span_bins(c["end_m"], following["start_m"], bm, bins)
        length = len(idx) * bm
        finite = [b for b in idx if math.isfinite(speed[b])]
        if len(finite) < 2:
            continue
        # The fastest bin rather than the last: the last is usually already
        # in the braking zone for the corner that follows.
        top_bin = max(finite, key=lambda b: speed[b])
        rows_straights.append({
            "after_corner": c["index"], "length_m": length,
            "top_at_m": top_bin * bm, "top_speed_mps": float(speed[top_bin]),
            "terminal_mps": theory,
            "share_of_terminal": float(speed[top_bin] / theory) if theory else None,
        })

    print(f"{args.policy} · {args.track} · 档位 {tyre}/{power} · "
          f"{args.lanes} lane × {args.seconds:.0f} s · seed {args.seed_base}")
    print(f"平路理论末速：本档 {theory:.2f} m/s（{theory * 3.6:.0f} km/h）"
          f" · 5/5 副列 {reference:.2f} m/s")
    print(f"\n弯速分布（{len(rows_corners)} 个弯）：")
    for row in rows_corners:
        print(f"  T{row['corner']:<2} {row['start_m']:6.0f}–{row['end_m']:6.0f} m  "
              f"最低 {row['min_speed_mps']:5.1f} m/s（{row['min_speed_mps'] * 3.6:5.0f} km/h）"
              f" @ {row['min_at_m']:6.0f} m")
    print(f"\n直道极速（≥{args.min_straight:.0f} m 的直道）：")
    for row in sorted(rows_straights, key=lambda x: -x["length_m"]):
        if row["length_m"] < args.min_straight:
            continue
        print(f"  T{row['after_corner']:<2} 之后 {row['length_m']:5.0f} m 直道  "
              f"极速 {row['top_speed_mps']:5.1f} m/s @ {row['top_at_m']:6.0f} m"
              f" = 理论 {row['share_of_terminal'] * 100:5.1f}%")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump({"policy": args.policy, "track": args.track, "modes": [tyre, power],
                       "terminal_speeds": top, "corners": rows_corners,
                       "straights": rows_straights}, handle, indent=1)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
