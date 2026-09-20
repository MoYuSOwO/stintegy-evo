"""What a second car costs, measured rather than assumed.

Drives the host with fixed actions and reports lane-steps a second, solo
and in a duel, on the same machine in the same run. No learner is involved:
what is being measured is the simulation and the pipe, which is where a
duel's extra work is -- a second car to step, a second observation to
build, and both of them across the wire.

    python3 Training/python/duel_probe.py --batch 16 --steps 400

Quote the numbers with what else was running. A laptop with a bake on it
has no idle CPU, and both arms here are measured under whatever that is.
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np

from host_env import HostEnv


def measure(batch: int, steps: int, duel: bool, track: str) -> dict[str, float]:
    with HostEnv(
        batch=batch,
        seed_base=17,
        solo=not duel,
        duel=duel,
        track=track,
        episode_seconds=240.0,
        randomise_episode_start=True,
        hidden_curriculum=True,
    ) as env:
        action = np.tile([0.05, 0.6], (batch, 1)).astype(np.float32)
        partner = np.tile([0.0, 0.4], (batch, 1)).astype(np.float32)
        env.reset()
        # A few steps of warm-up so the JIT and the first-touch page faults
        # are not charged to the measurement.
        for _ in range(20):
            env.step(action, partner if duel else None)
        started = time.perf_counter()
        for _ in range(steps):
            env.step(action, partner if duel else None)
        elapsed = time.perf_counter() - started
    return {
        "lane_steps_per_second": batch * steps / elapsed,
        "car_steps_per_second": batch * steps * (2 if duel else 1) / elapsed,
        "seconds": elapsed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--json", default=None)
    args = parser.parse_args()

    solo = measure(args.batch, args.steps, duel=False, track=args.track)
    duel = measure(args.batch, args.steps, duel=True, track=args.track)
    ratio = duel["lane_steps_per_second"] / solo["lane_steps_per_second"]
    print(
        f"batch {args.batch}  steps {args.steps}  track {args.track}\n"
        f"  solo  {solo['lane_steps_per_second']:8.0f} lane-steps/s"
        f"  ({solo['car_steps_per_second']:8.0f} car-steps/s)\n"
        f"  duel  {duel['lane_steps_per_second']:8.0f} lane-steps/s"
        f"  ({duel['car_steps_per_second']:8.0f} car-steps/s)\n"
        f"  a duel lane costs {1 / ratio:.2f}x a solo lane "
        f"({ratio * 100:.0f}% of the rate)"
    )
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(
                {"solo": solo, "duel": duel, "ratio": ratio,
                 "batch": args.batch, "steps": args.steps, "track": args.track},
                handle,
                indent=2,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
