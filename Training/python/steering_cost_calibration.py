"""What the steering costs would charge, before anything is trained on them.

The two coefficients are ours: Sophy names the quantities (arXiv
2511.02094, appendix F) and publishes no numbers. So they are fitted to
three readings taken on checkpoints that already exist, and the readings
are the deliverable rather than the coefficients:

1. **The weave, priced.** A policy in the family's weaving form should pay
   a few percent of what it earns — enough to be worth stopping, not
   enough to drown the lap.
2. **A clean lap, priced.** A policy that drives properly should barely
   notice: the whole lap's steering bill under a third of a percent.
3. **A catch, priced.** One correction should cost a fraction of what
   losing the car costs, or the policy learns not to catch it.

    python3 Training/python/steering_cost_calibration.py \\
        --weaver checkpoints/evalparent5c-500000.pt \\
        --clean checkpoints/evalparent5a-75000.pt

Nothing here trains anything. Each checkpoint drives with the costs
switched on, and what is reported is what the bill would have been.
"""

from __future__ import annotations

import argparse

import numpy as np
import torch

from host_env import COMPONENT_NAMES, HostEnv
from sac import SacAgent, SacConfig
from train import EVALUATION_MODES, STEP_SECONDS

PROGRESS = COMPONENT_NAMES.index("own_progress")
REVERSAL = COMPONENT_NAMES.index("steering_reversal")
CHANGE = COMPONENT_NAMES.index("steering_change")
RETIREMENT = COMPONENT_NAMES.index("retirement")


def drive(
    checkpoint: str,
    seconds: float,
    reversal: float,
    change: float,
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
        steering_reversal_cost=reversal,
        steering_change_cost=change,
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig(device="cpu"))
        agent.load(checkpoint)
        obs = env.reset()
        totals = np.zeros(len(COMPONENT_NAMES))
        spins = 0
        worst_step_reversal = 0.0
        for _ in range(int(round(seconds / STEP_SECONDS))):
            action = agent.act(obs, deterministic=True)
            obs, _r, _d, _reason, components, _race, _final, lane_spins = env.step(
                action
            )
            totals += components.mean(axis=1)
            spins += int(lane_spins.sum())
            worst_step_reversal = min(
                worst_step_reversal, float(components[REVERSAL].min())
            )
    progress = totals[PROGRESS]
    steering = totals[REVERSAL] + totals[CHANGE]
    return {
        "progress": float(progress),
        "reversal": float(totals[REVERSAL]),
        "change": float(totals[CHANGE]),
        "steering": float(steering),
        "share": float(-steering / progress) if progress > 0 else float("inf"),
        "worst_step": worst_step_reversal,
        "spins": float(spins),
        "retirement": float(totals[RETIREMENT]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weaver", required=True, help="a checkpoint in the weaving form")
    parser.add_argument("--clean", required=True, help="a checkpoint that drives cleanly")
    parser.add_argument("--reversal", type=float, default=0.35)
    parser.add_argument("--change", type=float, default=0.03)
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
        f"reversal {args.reversal}/s   change {args.change}/s   "
        f"{args.seconds:.0f}s per checkpoint, {args.batch} lanes\n"
    )
    readings = {}
    for name, checkpoint in (("weaving", args.weaver), ("clean", args.clean)):
        reading = drive(
            checkpoint,
            args.seconds,
            args.reversal,
            args.change,
            args.track,
            args.batch,
            args.seed,
            not args.absolute_actions,
        )
        readings[name] = reading
        print(
            f"{name:<8} {checkpoint.split('/')[-1]}\n"
            f"   progress earned   {reading['progress']:+9.3f}\n"
            f"   reversal charged  {reading['reversal']:+9.3f}\n"
            f"   change charged    {reading['change']:+9.3f}\n"
            f"   steering as a share of progress  {reading['share'] * 100:6.2f}%\n"
            f"   dearest single decision          {reading['worst_step']:+9.5f}\n"
            f"   spins {reading['spins']:.0f}\n"
        )

    # The third reading: one catch against what losing the car costs.
    #
    # A spin is not charged directly. What it takes is the ground the car
    # would have covered -- the progress reward is masked for as long as
    # the car is spinning -- so the honest yardstick is the progress
    # foregone over a spin, which on this circuit runs about two seconds.
    # The retirement penalty is quoted beside it as the reward's only
    # explicit price for losing a car, but it is the softer of the two
    # tests and the progress figure is the one that has to pass.
    catch = abs(readings["weaving"]["worst_step"])
    per_second = readings["weaving"]["progress"] / args.seconds
    spin_progress = per_second * 2.0
    retirement = 30.0
    print(
        f"dearest catch {catch:.5f}\n"
        f"   against two seconds of progress ({spin_progress:.2f}): "
        f"{catch / spin_progress * 100:.2f}%\n"
        f"   against the retirement penalty ({retirement:.0f}): "
        f"{catch / retirement * 100:.3f}%"
    )
    print("\ntargets: weaving 2-5%, clean under 0.3%, a catch under 1% of a spin")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
