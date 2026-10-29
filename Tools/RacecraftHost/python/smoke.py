"""Drive the host on the racing line and at random for a while: does it
talk, are the numbers sane, and how many quarter seconds a second does it run."""

from __future__ import annotations

import argparse
import time

import numpy as np

from racecraft_env import COMPONENTS, RacecraftEnv


def racing_line(env: RacecraftEnv) -> np.ndarray:
    action = np.zeros((env.batch, env.action_size), dtype=np.float32)
    # Brake factor 1: the last number maps -1..1 onto 0.85..1.03.
    action[:, -1] = 2 * (1 - 0.85) / (1.03 - 0.85) - 1
    return action


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--steps", type=int, default=480)
    args = parser.parse_args()
    rng = np.random.default_rng(0)
    with RacecraftEnv(batch=args.batch) as env:
        print(f"obs {env.obs_size} action {env.action_size} batch {env.batch}")
        for mode in ("line", "random"):
            obs = env.reset(seed=1)
            assert np.isfinite(obs).all()
            totals = np.zeros(env.components)
            reward_sum, episodes = 0.0, 0
            start = time.perf_counter()
            for _ in range(args.steps):
                action = racing_line(env) if mode == "line" else rng.uniform(-1, 1, (env.batch, env.action_size)).astype(np.float32)
                obs, final, rewards, ended, out_of_time, parts = env.step(action)
                assert np.isfinite(obs).all() and np.isfinite(rewards).all()
                totals += parts.sum(axis=0)
                reward_sum += rewards.sum()
                episodes += int(ended.sum())
            elapsed = time.perf_counter() - start
            steps = args.steps * env.batch
            print(f"{mode}: {steps / elapsed:.0f} decisions/s ({steps * 0.25 / elapsed:.0f}x real time), "
                  f"{episodes} duels ended, mean reward/decision {reward_sum / steps:+.4f}, "
                  + ", ".join(f"{n} {v:.1f}" for n, v in zip(COMPONENTS, totals)))


if __name__ == "__main__":
    main()
