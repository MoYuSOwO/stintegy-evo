"""The same checkpoint, the same seeds, the same Python, two worlds.

Run once with the world-v2 checkout's Training/python and once with
world-v3's. The Python stacks are byte-identical (world-v3 carried them
over unchanged), so everything that differs between the two readings is
the host: the physics, the contact resolvers, and the adapter that now
builds the observation from the frozen frame.

    python probe.py --python-dir <checkout>/Training/python --label v2 \\
        --checkpoint <path> --json v2.json

Per circuit it records the standard evaluation (laps, spins, clean laps,
fastest and mean clean lap, off-course and four-wheel readings, energy
and wear per lap) and, because the wall is the thing that changed most,
a second deterministic pass over the same seeds measuring wall contact:
seconds against a barrier per lap, the share of steps that touched one,
and the speed carried while touching.
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys

import numpy as np


def wall_pass(train, host_env, sac, agent, track, lanes, seconds, seed, modes):
    comp = host_env.COMPONENT_NAMES.index("wall")
    wall_seconds = np.zeros(lanes)
    touching_steps = 0
    steps = 0
    touch_speed = []
    distance = np.zeros(lanes)
    with host_env.HostEnv(batch=lanes, seed_base=seed, solo=True, track=track,
                          episode_seconds=seconds + 60.0, ego_modes=modes) as env:
        obs = env.reset()
        last = None
        for _ in range(int(round(seconds / train.STEP_SECONDS))):
            action = agent.act(obs, deterministic=True)
            out = env.step(action)
            obs, components, race = out[0], out[4], np.asarray(out[5], dtype=np.float64)
            speed = out[6][:, train.EGO_SPEED] * train.SPEED_SCALE
            penalty = -np.asarray(components[comp], dtype=np.float64)
            secs = penalty / (train.WALL_RATE * np.maximum(speed ** 2, 1e-6))
            secs = np.clip(secs, 0.0, train.STEP_SECONDS)
            wall_seconds += secs
            touching = secs > 0
            touching_steps += int(touching.sum())
            steps += lanes
            touch_speed.extend(speed[touching].tolist())
            if last is not None:
                distance += np.maximum(race - last, 0.0)
            last = race
    lap_m = train.TRACKS[track][0]
    laps_driven = distance.sum() / lap_m
    return {
        "wall_seconds_total": float(wall_seconds.sum()),
        "wall_seconds_per_lap": float(wall_seconds.sum() / max(laps_driven, 1e-9)),
        "touching_step_share": touching_steps / max(steps, 1),
        "mean_speed_while_touching": float(np.mean(touch_speed)) if touch_speed else 0.0,
        "laps_driven_by_distance": float(laps_driven),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--python-dir", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--tracks", nargs="+", default=["silverstone", "monaco", "singapore"])
    parser.add_argument("--lanes", type=int, default=12)
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--seed-base", type=int, default=900_001)
    parser.add_argument("--modes", default="3,3")
    parser.add_argument("--json", required=True)
    parser.add_argument("--zero-channel", type=int, action="append", default=[],
                        help="ablation: hold this observation index at zero")
    args = parser.parse_args()
    modes = tuple(int(x) for x in args.modes.split(","))

    sys.path.insert(0, args.python_dir)
    train = importlib.import_module("train")
    host_env = importlib.import_module("host_env")
    sac = importlib.import_module("sac")

    with host_env.HostEnv(batch=2, seed_base=1, solo=True, track="silverstone",
                          episode_seconds=60) as env:
        agent = sac.SacAgent(env.obs_size, env.action_size, sac.SacConfig())
        print(f"[{args.label}] host {host_env.DEFAULT_HOST_PROJECT}", flush=True)
    agent.load(args.checkpoint)
    if args.zero_channel:
        act = agent.act

        def masked(obs, deterministic=False):
            obs = obs.copy()
            obs[:, args.zero_channel] = 0.0
            return act(obs, deterministic=deterministic)

        agent.act = masked

    results = {"label": args.label, "zeroed_channels": args.zero_channel, "python_dir": args.python_dir,
               "checkpoint": args.checkpoint, "lanes": args.lanes,
               "seconds": args.seconds, "seed_base": args.seed_base,
               "modes": list(modes), "tracks": {}}
    for track in args.tracks:
        r = train.evaluate(agent, args.lanes, args.seed_base, True, track,
                           args.seconds, modes=modes)
        cleans = r["clean_lap_times"]
        row = {
            "laps": r["laps"], "spins": r["spins"], "clean_laps": r["clean_laps"],
            "clean_share": r["clean_share"], "fastest_clean": r["lap"],
            "mean_clean": float(np.mean(cleans)) if cleans else None,
            "off_per_lap": r["off_per_lap"],
            "four_wheel_clean_share": r["four_wheel_clean_share"],
            "charge_per_lap": r["charge_per_lap"], "wear_per_lap": r["wear_per_lap"],
            "stalls": r["stalls"], "wall_reward_mean": r["wall"],
        }
        row.update(wall_pass(train, host_env, sac, agent, track, args.lanes,
                             args.seconds, args.seed_base, modes))
        results["tracks"][track] = row
        print(f"[{args.label}] {track}: " + json.dumps(row), flush=True)
    with open(args.json, "w", encoding="utf-8") as handle:
        json.dump(results, handle, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
