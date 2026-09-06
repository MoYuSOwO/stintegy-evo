"""Where does a lap lose its time.

Runs a checkpoint on one circuit, streams the per-step race distance the
protocol already reports, and bins speed by position on the lap. Two
checkpoints compared this way turn an aggregate gap into a map: which
hundred metres of the circuit the slower one gives its seconds away on.

Also reads the tyre allowance and drive ceiling straight out of the
observation, because evaluation lanes draw their strategy modes from the
seed and nobody has ever checked what the evaluation is actually driving
in - a lane stuck in Protect/Save would carry a built-in handicap against
the analytic baseline and it would look exactly like a slow policy.
"""

from __future__ import annotations

import math
import sys

import numpy as np

from host_env import HostEnv
from sac import SacAgent, SacConfig
from train import STEP_SECONDS
ROAD_OFFSET = 219
CEILING = ROAD_OFFSET + 7
ALLOWANCE = ROAD_OFFSET + 8

TIRE_MODES = {
    0.955: "Protect", 0.966: "Light", 0.977: "Normal",
    0.9885: "Push", 1.0: "Attack",
}


def nearest_mode(value: float) -> str:
    return min(TIRE_MODES.items(), key=lambda kv: abs(kv[0] - value))[1]


def run(checkpoint: str, track: str, lap_metres: float, steps: int,
        batch: int, seed_base: int, bin_metres: float) -> dict:
    with HostEnv(batch=batch, seed_base=seed_base, solo=True, track=track,
                 episode_seconds=steps * STEP_SECONDS + 60.0) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
        agent.load(checkpoint)
        obs = env.reset()
        allowances = obs[:, ALLOWANCE].copy()

        bins = int(math.ceil(lap_metres / bin_metres))
        time_in_bin = np.zeros(bins)
        metres_in_bin = np.zeros(bins)
        ceiling_in_bin = np.zeros(bins)
        best_lap = math.inf
        laps = 0
        crossed: list[float | None] = [None] * batch
        previous = None
        for step in range(steps):
            action = agent.act(obs, deterministic=True)
            obs, _, done, _, _, race, _ = env.step(action)
            now = (step + 1) * STEP_SECONDS
            if previous is not None:
                for lane in range(batch):
                    if done[lane] or race[lane] <= previous[lane]:
                        crossed[lane] = None
                        continue
                    moved = race[lane] - previous[lane]
                    # The step's distance lands in the bin of its midpoint;
                    # at half a second of travel per bin that is exact
                    # enough for a map.
                    mid = (previous[lane] + moved * 0.5) % lap_metres
                    b = min(int(mid / bin_metres), bins - 1)
                    time_in_bin[b] += STEP_SECONDS
                    metres_in_bin[b] += moved
                    ceiling_in_bin[b] += obs[lane, CEILING] * STEP_SECONDS
                    before = math.floor(previous[lane] / lap_metres)
                    after = math.floor(race[lane] / lap_metres)
                    for line in range(before + 1, after + 1):
                        share = (line * lap_metres - previous[lane]) / moved
                        at = now - STEP_SECONDS + share * STEP_SECONDS
                        if crossed[lane] is not None:
                            laps += 1
                            best_lap = min(best_lap, at - crossed[lane])
                        crossed[lane] = at
            previous = race.copy()
            if done.any():
                previous = None

    speed = np.where(time_in_bin > 0, metres_in_bin / np.maximum(time_in_bin, 1e-9), np.nan)
    ceiling = np.where(time_in_bin > 0, ceiling_in_bin / np.maximum(time_in_bin, 1e-9), np.nan)
    return {
        "speed": speed, "best_lap": best_lap, "laps": laps,
        "allowances": allowances, "ceiling": ceiling,
    }


def main() -> int:
    track = "silverstone"
    lap_metres = 5891.0
    bin_metres = 100.0
    # Ten minutes of watching, in whatever number of steps the rate makes
    # that: several laps in each lane on any circuit here.
    steps = int(round(600.0 / STEP_SECONDS))
    seed_base = 900001                 # the evaluation's own seeds
    a_path, b_path = sys.argv[1], sys.argv[2]

    a = run(a_path, track, lap_metres, steps, 2, seed_base, bin_metres)
    b = run(b_path, track, lap_metres, steps, 2, seed_base, bin_metres)

    for name, r in ((a_path, a), (b_path, b)):
        modes = ", ".join(nearest_mode(float(v)) for v in r["allowances"])
        print(f"{name}: best lap {r['best_lap']:.3f}s over {r['laps']} laps"
              f"  eval-lane tyre modes: [{modes}]")

    # Seconds each checkpoint spends in each bin, and where they differ.
    ta = bin_metres / a["speed"]
    tb = bin_metres / b["speed"]
    diff = tb - ta                     # positive: b slower here
    total = np.nansum(diff)
    print(f"\nper-lap time difference ({b_path} minus {a_path}): "
          f"{total:+.2f}s over {np.count_nonzero(~np.isnan(diff))} bins")

    order = np.argsort(np.nan_to_num(diff, nan=0.0))[::-1]
    print("\nwhere the slower one bleeds (top 12 hundred-metre bins):")
    print("   at metres   loss    speeds (fast vs slow)   drive ceiling")
    for b_i in order[:12]:
        if math.isnan(diff[b_i]):
            continue
        print(f"  {b_i * bin_metres:7.0f}   {diff[b_i]:+.3f}s   "
              f"{a['speed'][b_i]:5.1f} vs {b['speed'][b_i]:5.1f} m/s   "
              f"{a['ceiling'][b_i]:.2f} vs {b['ceiling'][b_i]:.2f}")

    spread = np.nanstd(diff)
    print(f"\nconcentration: std of per-bin loss {spread:.3f}s; "
          f"top-10 bins hold {np.nansum(np.sort(np.nan_to_num(diff))[-10:]) / total * 100 if total else 0:.0f}% of the total")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
