"""How much the wheel is being shaken, on the straights.

The first screen for the delta-action pilot. Every driver this world has
baked comes out weaving at about ten reversals a second — parent1 at 9.98,
parent2h at 10.61, the whole third and fourth generations at the same
figure — and the pilot's question is whether making the action an increment
takes that away without taking the driver with it.

    python3 Training/python/jitter_probe.py \\
        --checkpoint checkpoints/evalparent5a-450000.pt --delta-actions

What is counted is the steering command the car is actually given, read back
out of the observation's ego block, not the raw action: under the pilot the
action is an increment, and counting its reversals would be counting how
often the policy changes its mind rather than how much the wheel moves.

Only the straights count. In a corner a wheel that never moves is not a
steady hand, it is a car going off, so the reading is taken where the car
is not cornering (lateral acceleration under a threshold) and the corners
are reported separately for context.

This is a screen, not a verdict: the gate is the user watching it drive.
"""

from __future__ import annotations

import argparse

import numpy as np
import torch

from host_env import HostEnv
from sac import SacAgent, SacConfig
from train import (
    EVALUATION_MODES, EGO_SPEED, OBSERVATION_SIZE, STEP_SECONDS,
)

# The ego block, from DirectDriveObservation: speed, longitudinal, lateral,
# yaw rate, sideslip, sin/cos heading error, lateral offset, the two edge
# distances, then the last steering command and the last pedal.
EGO_LATERAL_ACCEL = EGO_SPEED + 2
EGO_COMMAND = EGO_SPEED + 10
ACCELERATION_SCALE = 20.0

# What counts as a straight: a car pulling less than this sideways is not
# cornering, whatever the road is doing.
STRAIGHT_LATERAL = 2.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=300.0)
    parser.add_argument("--seed", type=int, default=900_000)
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--delta-actions", action="store_true",
        help="the checkpoint was baked on the incremental contract",
    )
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    config = SacConfig(device=args.device)

    with HostEnv(
        batch=args.batch,
        seed_base=args.seed,
        solo=True,
        track=args.track,
        episode_seconds=args.seconds + 60.0,
        ego_modes=EVALUATION_MODES,
        delta_actions=args.delta_actions,
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, config)
        restored = agent.load(args.checkpoint)
        obs = env.reset()
        assert env.obs_size == OBSERVATION_SIZE, env.obs_size

        previous = obs[:, EGO_COMMAND].copy()
        direction = np.zeros(args.batch)
        straight_reversals = 0
        straight_samples = 0
        corner_reversals = 0
        corner_samples = 0
        swing = 0.0
        spins = 0

        for _ in range(int(round(args.seconds / STEP_SECONDS))):
            action = agent.act(obs, deterministic=True)
            obs, _reward, done, _reason, _components, _race, _final, lane_spins = (
                env.step(action)
            )
            spins += int(lane_spins.sum())

            command = obs[:, EGO_COMMAND]
            delta = command - previous
            lateral = np.abs(obs[:, EGO_LATERAL_ACCEL]) * ACCELERATION_SCALE
            straight = lateral < STRAIGHT_LATERAL
            moved = np.abs(delta) > 1e-4
            now = np.sign(delta)
            reversed_ = moved & (direction != 0) & (now != direction)

            straight_reversals += int(np.count_nonzero(reversed_ & straight))
            corner_reversals += int(np.count_nonzero(reversed_ & ~straight))
            straight_samples += int(np.count_nonzero(straight))
            corner_samples += int(np.count_nonzero(~straight))
            swing += float(np.abs(delta).mean())

            direction = np.where(moved, now, direction)
            # A lane that re-seeded starts a fresh wheel; its first delta is
            # not a movement anybody made.
            direction = np.where(done, 0, direction)
            previous = np.where(done, command, command)

    straight_seconds = straight_samples * STEP_SECONDS
    corner_seconds = corner_samples * STEP_SECONDS
    straight_rate = straight_reversals / max(straight_seconds, 1e-6)
    corner_rate = corner_reversals / max(corner_seconds, 1e-6)
    print(
        f"{args.checkpoint} (step {restored['step']}, "
        f"{'delta' if args.delta_actions else 'absolute'} actions)\n"
        f"  straights: {straight_rate:.2f} reversals/s "
        f"over {straight_seconds:.0f} lane-seconds\n"
        f"  corners:   {corner_rate:.2f} reversals/s "
        f"over {corner_seconds:.0f} lane-seconds\n"
        f"  mean command move per decision {swing / max(straight_samples + corner_samples, 1):.4f}\n"
        f"  spins {spins}"
    )
    # The pre-registered screen: under three a second on the straights
    # passes to the viewer, five to ten is the family's own figure and
    # fails, and the band between them asks for a longer look.
    verdict = (
        "PASS to the viewer" if straight_rate < 3.0
        else "FAIL — this is the family's own weave" if straight_rate >= 5.0
        else "INCONCLUSIVE — watch it longer"
    )
    print(f"  screen: {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
