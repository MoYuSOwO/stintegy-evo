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
        amplitudes: list[float] = []
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

            if np.any(reversed_ & straight):
                # The size of the wheel movement a reversal turns around:
                # a reversal of a thousandth is a policy holding a line,
                # and counting it beside one of a fiftieth was the reason
                # the rate alone was never a verdict.
                amplitudes.extend(
                    np.abs(delta[reversed_ & straight]).tolist()
                )
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
    # Two conditions, because either alone can be passed by a car nobody
    # wants to watch. A rate under three a second with a swing of a
    # fiftieth of full lock is a car visibly snaking; a rate of ten with a
    # swing of a thousandth is a car holding its line and dithering in the
    # last digit. What the eye objects to is the two together, so both are
    # named and the stricter one decides.
    #
    # The amplitude gate is the median movement a reversal turns around,
    # against 0.02 of the normalised command. The family's weaving form
    # sits at 0.0124 and climbing, which is why the gate is where it is.
    swing = float(np.median(amplitudes)) if amplitudes else 0.0
    print(f"  median swing at a reversal {swing:.5f} (gate 0.02)")
    calm_enough = straight_rate < 3.0
    small_enough = swing < 0.02
    if calm_enough and small_enough:
        verdict = "PASS to the viewer"
    elif straight_rate >= 5.0 and not small_enough:
        verdict = "FAIL — this is the family's own weave"
    elif not small_enough:
        verdict = "FAIL — the swing is what the eye objects to"
    else:
        verdict = "INCONCLUSIVE — watch it longer"
    print(f"  screen: {verdict}")
    print(
        "  (a rate this probe likes can still be a slow weave: run "
        "spectrum_probe.py against a baseline before the viewer)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
