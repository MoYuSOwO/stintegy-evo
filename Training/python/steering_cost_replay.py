"""What the steering costs actually charge, replayed through the host.

The coefficients were fitted offline, against traces of the acted steering
read back from three checkpoints that already exist. Offline fitting can be
wrong in ways arithmetic cannot catch: a window implemented as sliding where
it was measured as non-overlapping is out by a factor of two, and a quantity
billed per second where it was measured per decision is out by fifteen. So
before anything is trained on the price list, the host is asked what it would
charge, and the answer is compared with what the fit predicted.

    python3 Training/python/steering_cost_replay.py \\
        --weaving  checkpoints/evalparent5c-500000.pt \\
        --clean    checkpoints/evalparent5a-75000.pt \\
        --settled  checkpoints/evalparent6a-275000.pt

Nothing here trains anything, and nothing here fits anything. If a reading
misses, the implementation and the fit disagree, and the answer is to find out
which is wrong — not to move a coefficient until the number lands.
"""

from __future__ import annotations

import argparse

import numpy as np
import torch

from host_env import COMPONENT_NAMES, HostEnv
from sac import SacAgent, SacConfig
from train import EVALUATION_MODES, STEP_SECONDS

PROGRESS = COMPONENT_NAMES.index("own_progress")
DETOUR = COMPONENT_NAMES.index("steering_detour")
TRAVEL = COMPONENT_NAMES.index("steering_travel")

# What the offline fit says each reference checkpoint should pay, as a share
# of the progress it earns. Written down before the replay runs.
EXPECTED = {"weaving": 14.0, "settled": 5.4, "clean": 1.0}
TOLERANCE = 0.10


def drive(
    checkpoint: str,
    seconds: float,
    detour: float,
    travel: float,
    track: str,
    batch: int,
    seed: int,
    delta_actions: bool,
) -> dict[str, float]:
    """One checkpoint, driven with the costs on, and what they charged."""
    torch.manual_seed(seed)
    np.random.seed(seed)
    with HostEnv(
        batch=batch,
        seed_base=seed,
        solo=True,
        track=track,
        episode_seconds=seconds + 60.0,
        ego_modes=EVALUATION_MODES,
        delta_actions=delta_actions,
        steering_detour_cost=detour,
        steering_travel_cost=travel,
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig(device="cpu"))
        agent.load(checkpoint)
        obs = env.reset()
        totals = np.zeros(len(COMPONENT_NAMES))
        worst_step = 0.0
        steps = int(round(seconds / STEP_SECONDS))
        for _ in range(steps):
            action = agent.act(obs, deterministic=True)
            obs, _r, _d, _reason, components, _race, _final, _spins = env.step(
                action
            )
            totals += components.mean(axis=1)
            worst_step = min(
                worst_step,
                float((components[DETOUR] + components[TRAVEL]).min()),
            )
    progress = totals[PROGRESS]
    steering = totals[DETOUR] + totals[TRAVEL]
    return {
        "progress": float(progress),
        "detour": float(totals[DETOUR]),
        "travel": float(totals[TRAVEL]),
        "steering": float(steering),
        "share": float(-steering / progress * 100.0) if progress > 0 else float("inf"),
        "worst_step": worst_step,
        "seconds": seconds,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weaving", required=True, help="a checkpoint in the weaving form")
    parser.add_argument("--clean", required=True, help="a checkpoint that drives cleanly")
    parser.add_argument("--settled", required=True, help="parent6a's last checkpoint")
    parser.add_argument("--detour", type=float, default=0.1183)
    parser.add_argument("--travel", type=float, default=0.0182)
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--seed", type=int, default=900_000)
    parser.add_argument(
        "--absolute-actions", action="store_true",
        help="the checkpoints were baked before the incremental contract",
    )
    args = parser.parse_args()

    print(
        f"detour {args.detour}/rad (sliding, two decisions)   "
        f"travel {args.travel}/rad\n"
        f"{args.seconds:.0f}s per checkpoint, {args.batch} lanes, "
        f"deterministic\n"
    )

    ok = True
    for name in ("weaving", "settled", "clean"):
        reading = drive(
            getattr(args, name),
            args.seconds,
            args.detour,
            args.travel,
            args.track,
            args.batch,
            args.seed,
            not args.absolute_actions,
        )
        expected = EXPECTED[name]
        low, high = expected * (1 - TOLERANCE), expected * (1 + TOLERANCE)
        hit = low <= reading["share"] <= high
        ok &= hit
        print(
            f"{name:<8} {getattr(args, name).split('/')[-1]}\n"
            f"   progress earned   {reading['progress']:+9.3f}\n"
            f"   detour charged    {reading['detour']:+9.4f}"
            f"   ({reading['detour'] / args.seconds:+.5f} per lane second)\n"
            f"   travel charged    {reading['travel']:+9.4f}"
            f"   ({reading['travel'] / args.seconds:+.5f} per lane second)\n"
            f"   share of progress {reading['share']:8.2f}%"
            f"   expected {expected:.1f}%  "
            f"{'HIT' if hit else 'MISS'}\n"
            f"   dearest decision  {reading['worst_step']:+9.5f}\n"
        )

    print(
        "replay matches the fit" if ok else
        "REPLAY DISAGREES WITH THE FIT -- window or units, not the coefficient"
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
