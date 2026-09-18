"""Gate 1 of the budget re-core (PLAN_zh.md §7): does energy trade against
lap time along a convex curve, or a straight one?

Two curves from the same checkpoint on the same circuit and seeds:

1. The power ladder. Tyre rung fixed at Normal, power rungs 1..5. This is
   the mechanism the re-core wants to replace, and PLAN predicts it trades
   nearly linearly, because a cap only bites where the car is already
   power-limited.
2. Lift and coast. A baseline run records where each lap's braking zones
   begin. Then the same policy drives again with its throttle held at zero
   for the last D metres before every braking zone, for a ladder of D.
   Energy that would have gone into speed the brakes are about to take
   away is not spent. This is the behaviour the re-core is meant to ask
   a driver for, and the curve gate 1 is about.

Every point is the standard evaluation (clean laps only for time), with
energy per lap as the pack share times the pack's megajoules.
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np

PACK_MEGAJOULES = 1100.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--python-dir", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--lanes", type=int, default=12)
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--seed-base", type=int, default=900_001)
    parser.add_argument("--lifts", type=float, nargs="+",
                        default=[0, 25, 50, 100, 150, 200, 300])
    parser.add_argument("--json", required=True)
    parser.add_argument("--per-zone", type=float, default=None,
                        help="instead of the ladders: lift this far before one zone at a time")
    args = parser.parse_args()

    sys.path.insert(0, args.python_dir)
    import host_env
    import train
    from sac import SacAgent, SacConfig

    lap_m = train.TRACKS[args.track][0]
    bin_m = 5.0
    bins = int(np.ceil(lap_m / bin_m))

    with host_env.HostEnv(batch=2, seed_base=1, solo=True, track=args.track,
                          episode_seconds=60) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
    agent.load(args.checkpoint)

    # 1. Where braking zones begin: an accelerate-or-coast step followed by
    # a real brake command at speed, binned round the lap.
    onsets = np.zeros(bins)
    with host_env.HostEnv(batch=args.lanes, seed_base=args.seed_base, solo=True,
                          track=args.track, episode_seconds=args.seconds + 60,
                          ego_modes=(3, 3)) as env:
        obs = env.reset()
        previous = np.zeros(args.lanes)
        race = None
        for _ in range(int(round(args.seconds / train.STEP_SECONDS))):
            action = agent.act(obs, deterministic=True)
            speed = obs[:, train.EGO_SPEED] * train.SPEED_SCALE
            if race is not None:
                for lane in range(args.lanes):
                    if previous[lane] >= 0.0 and action[lane, 1] < -0.3 and speed[lane] > 20.0:
                        onsets[int((race[lane] % lap_m) / bin_m) % bins] += 1
            previous = action[:, 1].copy()
            out = env.step(action)
            obs, race = out[0], np.asarray(out[5], dtype=np.float64)
    laps_seen = max(onsets.sum(), 1.0)
    # A zone is a bin where most laps start braking; neighbouring bins are
    # merged into the first of the run.
    # Onsets scatter over a few bins and a policy dabs the brake more than
    # once on the way into a corner, so bins with a real share of the
    # onsets are merged into one zone per 150 m, starting at the earliest.
    threshold = 0.25 * onsets.max()
    marked = [b for b in range(bins) if onsets[b] >= threshold]
    zones_m = []
    for b in marked:
        metres = b * bin_m
        if not zones_m or metres - zones_m[-1] > 150.0:
            zones_m.append(metres)
    if len(zones_m) > 1 and zones_m[0] + lap_m - zones_m[-1] <= 150.0:
        zones_m.pop()
    print(f"braking zones at {zones_m} m ({len(zones_m)} per lap)", flush=True)

    # 2. The patches: HostEnv.step records each lane's race distance, and
    # the agent clamps throttle when a zone is within D metres ahead.
    state = {"race": None, "lift": 0.0, "zones": zones_m}
    original_step = host_env.HostEnv.step
    original_reset = host_env.HostEnv.reset

    def step(self, actions):
        out = original_step(self, actions)
        state["race"] = np.asarray(out[5], dtype=np.float64)
        return out

    def reset(self, *a, **k):
        state["race"] = None
        return original_reset(self, *a, **k)

    host_env.HostEnv.step = step
    host_env.HostEnv.reset = reset
    act = agent.act

    def lifting(obs, deterministic=False):
        action = act(obs, deterministic=deterministic)
        if state["lift"] > 0 and state["race"] is not None:
            s = state["race"] % lap_m
            for lane in range(action.shape[0]):
                for z in state["zones"]:
                    ahead = (z - s[lane]) % lap_m
                    if 0.0 < ahead <= state["lift"]:
                        action[lane, 1] = min(action[lane, 1], 0.0)
                        break
        return action

    agent.act = lifting

    def point(modes, lift):
        state["lift"] = lift
        r = train.evaluate(agent, args.lanes, args.seed_base, True, args.track,
                           args.seconds, modes=modes)
        cleans = r["clean_lap_times"]
        row = {
            "modes": list(modes), "lift_m": lift, "laps": r["laps"],
            "clean_laps": r["clean_laps"], "spins": r["spins"],
            "mean_clean": float(np.mean(cleans)) if cleans else None,
            "fastest_clean": r["lap"],
            "energy_mj_per_lap": r["charge_per_lap"] * PACK_MEGAJOULES,
            "charge_per_lap": r["charge_per_lap"],
        }
        print(json.dumps(row), flush=True)
        return row

    if args.per_zone is not None:
        rows = []
        for z in zones_m:
            state["zones"] = [z]
            row = point((3, 3), args.per_zone)
            row["zone_m"] = z
            rows.append(row)
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump({"track": args.track, "per_zone_lift_m": args.per_zone,
                       "braking_zones_m": zones_m, "zones": rows}, handle, indent=1)
        return 0

    ladder = [point((3, power), 0.0) for power in (1, 2, 3, 4, 5)]
    lifts = [point((3, 3), d) for d in args.lifts]
    with open(args.json, "w", encoding="utf-8") as handle:
        json.dump({"track": args.track, "lanes": args.lanes, "seconds": args.seconds,
                   "seed_base": args.seed_base, "checkpoint": args.checkpoint,
                   "braking_zones_m": zones_m, "ladder": ladder, "lift": lifts},
                  handle, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
