"""Training loop for the direct-drive racing policy.

Stage one of the plan's curriculum: a single car learning to drive. The
graduation question is whether the learned policy covers more ground than
the analytic baseline, so every evaluation reports the ratio against the
coach-passthrough reference measured on the same tracks — including the
held-out one the policy never trains on, which is what makes the
track-agnostic claim testable rather than asserted.

Usage:
    python3 Training/python/train.py --solo --steps 300000
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from host_env import COMPONENT_NAMES, TERMINAL_NAMES, HostEnv
from nstep import NStepBatcher
from sac import SacAgent, SacConfig


def evaluate(
    agent: SacAgent,
    batch: int,
    seed_base: int,
    solo: bool,
    track: str | None,
    steps: int,
) -> dict[str, float]:
    """Run the deterministic policy and report per-lane statistics."""
    with HostEnv(
        batch=batch, seed_base=seed_base, solo=solo, track=track
    ) as env:
        obs = env.reset()
        rewards = np.zeros(batch, dtype=np.float64)
        distance = np.zeros(batch, dtype=np.float64)
        excess = np.zeros(batch, dtype=np.float64)
        reasons: dict[str, int] = {}
        for _ in range(steps):
            action = agent.act(obs, deterministic=True)
            obs, reward, done, reason, components = env.step(action)
            rewards += reward
            # Own-progress reward is a fixed rate per meter, so it doubles
            # as a distance meter without a second channel.
            distance += components[COMPONENT_NAMES.index("own_progress")]
            excess += components[COMPONENT_NAMES.index("mode_excess")]
            for lane in np.flatnonzero(done):
                name = TERMINAL_NAMES[reason[lane]]
                reasons[name] = reasons.get(name, 0) + 1
    stalls = reasons.get("stalled", 0)
    return {
        "eval_reward": float(rewards.mean()),
        "eval_progress": float(distance.mean()),
        "eval_mode_excess": float(excess.mean()),
        "eval_stalls": float(stalls),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=300_000)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--solo", action="store_true")
    parser.add_argument("--track", default=None)
    parser.add_argument("--eval-every", type=int, default=10_000)
    parser.add_argument("--eval-steps", type=int, default=900)
    parser.add_argument("--eval-batch", type=int, default=8)
    parser.add_argument("--held-out-track", default="sepang")
    parser.add_argument("--log-every", type=int, default=1_000)
    parser.add_argument(
        "--checkpoint-dir",
        default=str(Path(__file__).resolve().parent / "checkpoints"),
    )
    parser.add_argument("--resume", default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument("--updates-per-step", type=int, default=None)
    parser.add_argument(
        "--quantiles", type=int, default=None,
        help="0 keeps the scalar critic; a positive count makes it distributional",
    )
    parser.add_argument(
        "--no-critic-layer-norm", action="store_true",
        help="turn off critic layer norm, which is on by default",
    )
    parser.add_argument("--hidden", default=None, help='e.g. "256,256"')
    parser.add_argument("--tag", default="", help="suffix for checkpoint files")
    args = parser.parse_args()

    checkpoint_dir = Path(args.checkpoint_dir)
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    overrides: dict[str, object] = {}
    if args.device:
        overrides["device"] = args.device
    if args.updates_per_step is not None:
        overrides["updates_per_step"] = args.updates_per_step
    if args.quantiles is not None:
        overrides["quantiles"] = args.quantiles
    if args.no_critic_layer_norm:
        overrides["critic_layer_norm"] = False
    if args.hidden:
        overrides["hidden"] = tuple(
            int(part) for part in args.hidden.split(",")
        )
    config = SacConfig(**overrides)
    print(
        f"device: {config.device} hidden={config.hidden} "
        f"quantiles={config.quantiles} layer_norm={config.critic_layer_norm} "
        f"updates/step={config.updates_per_step}"
    )
    with HostEnv(
        batch=args.batch,
        seed_base=args.seed,
        solo=args.solo,
        track=args.track,
        quiet=True,
    ) as env:
        print(
            f"env: obs={env.obs_size} action={env.action_size} "
            f"lanes={env.batch} solo={args.solo}"
        )
        agent = SacAgent(env.obs_size, env.action_size, config)
        if args.resume:
            agent.load(args.resume)
            print(f"resumed from {args.resume}")

        obs = env.reset()
        batcher = NStepBatcher(env.batch, config.n_step, config.gamma)
        window_reward = 0.0
        window_components = np.zeros(len(COMPONENT_NAMES))
        window_terminals: dict[str, int] = {}
        best_progress = -np.inf
        started = time.time()

        for step in range(1, args.steps + 1):
            transitions = step * env.batch
            if transitions < config.start_steps:
                action = np.random.uniform(
                    -1.0, 1.0, size=(env.batch, env.action_size)
                ).astype(np.float32)
            else:
                action = agent.act(obs)

            next_obs, reward, done, reason, components = env.step(action)
            # `next_obs` is already the post-reset observation on finished
            # lanes, so the terminal flag must stop the bootstrap there.
            ready = batcher.add(
                obs, action, reward, next_obs, done.astype(np.float32)
            )
            if ready is not None:
                agent.buffer.add_batch(*ready)
            obs = next_obs

            window_reward += float(reward.mean())
            window_components += components.mean(axis=1)
            for lane in np.flatnonzero(done):
                name = TERMINAL_NAMES[reason[lane]]
                window_terminals[name] = window_terminals.get(name, 0) + 1

            if (
                transitions >= config.start_steps
                and agent.buffer.size >= config.batch_size
            ):
                for _ in range(config.updates_per_step):
                    stats = agent.update()
            else:
                stats = {}

            if step % args.log_every == 0:
                elapsed = time.time() - started
                pieces = " ".join(
                    f"{name}={value / args.log_every:+.4f}"
                    for name, value in zip(COMPONENT_NAMES, window_components)
                    if abs(value) > 1e-9
                )
                print(
                    f"step {step:>7} "
                    f"transitions {transitions:>9} "
                    f"reward {window_reward / args.log_every:+.4f} "
                    f"alpha {stats.get('alpha', float('nan')):.3f} "
                    f"q {stats.get('q_mean', float('nan')):+.2f} "
                    # The critic's own fit, which is the first thing to look
                    # at before blaming capacity: a network too small to
                    # represent its target shows up here as a loss that
                    # stops falling while performance stops improving.
                    f"closs {stats.get('critic_loss', float('nan')):.3f} "
                    f"{step * env.batch / max(elapsed, 1e-6):.0f} tps"
                )
                print(f"          {pieces}")
                print(f"          terminals {window_terminals or 'none'}")
                window_reward = 0.0
                window_components[:] = 0.0
                window_terminals = {}

            if step % args.eval_every == 0:
                train_stats = evaluate(
                    agent,
                    args.eval_batch,
                    args.seed + 500_000,
                    args.solo,
                    args.track,
                    args.eval_steps,
                )
                held_out = evaluate(
                    agent,
                    args.eval_batch,
                    args.seed + 900_000,
                    args.solo,
                    args.held_out_track,
                    args.eval_steps,
                )
                print(
                    f"  eval  train progress {train_stats['eval_progress']:+.2f} "
                    f"stalls {train_stats['eval_stalls']:.0f} "
                    f"mode {train_stats['eval_mode_excess']:+.3f} | "
                    f"held-out({args.held_out_track}) progress "
                    f"{held_out['eval_progress']:+.2f} "
                    f"stalls {held_out['eval_stalls']:.0f}"
                )
                agent.save(str(checkpoint_dir / f"latest{args.tag}.pt"))
                if train_stats["eval_progress"] > best_progress:
                    best_progress = train_stats["eval_progress"]
                    agent.save(str(checkpoint_dir / f"best{args.tag}.pt"))
                    print(f"  saved best (progress {best_progress:+.2f})")

    print("training finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
