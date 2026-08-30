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


# Constants the harness shares with the host. Kept here rather than
# rediscovered, because every one of them has been got wrong once: the
# progress rate is what turns a reward back into metres, the step is what
# turns steps back into seconds, and the off-course rate is what turns its
# penalty back into the seconds spent beside the road.
STEP_SECONDS = 0.1
OWN_PROGRESS_RATE = 0.02
OFF_COURSE_RATE = 1e-3
SPEED_SCALE = 100.0
# Speed is the first slot of the ego block: geometry 198, tyres 17, mode 1,
# aero 3, road and limits 13.
EGO_SPEED = 232


# The harness may know the track; the policy may not. Lap lengths in metres
# and the analytic driver's own flying lap over the same circuits, so the
# log reports the gap in the unit a lap is actually measured in.
TRACKS: dict[str, tuple[float, float, bool]] = {
    # name: (lap metres, analytic flying lap seconds, in the training set)
    "silverstone":    (5891.0, 101.974, True),
    "shanghai":       (5451.0, 103.729, True),
    "zandvoort":      (4259.0,  83.241, True),
    "simple-right":   (1804.0,  36.160, True),
    "simple-left":    (1804.0,  36.160, True),
    "banked-sweeper": (4946.0,  67.348, True),
    "sepang":         (5543.0, 105.714, False),
    "monaco":         (3337.0,  80.857, False),
    "daytona":        (4016.0,  54.572, False),
    "speedway":       (8512.0, 110.216, False),
}

# Ninety seconds does not reach the end of a lap of Silverstone, so the old
# evaluation was reporting a standing start and part of a lap. Four minutes
# with the first minute thrown away is a flying lap on every circuit here,
# which is what a lap time means.
EVAL_WARMUP_STEPS = 600


def evaluate(
    agent: SacAgent,
    batch: int,
    seed_base: int,
    solo: bool,
    track: str,
    steps: int,
) -> dict[str, float]:
    """Flying-lap time on one circuit, and what it cost to get it."""
    lap_metres, analytic, _ = TRACKS[track]
    with HostEnv(
        batch=batch, seed_base=seed_base, solo=solo, track=track
    ) as env:
        obs = env.reset()
        distance = np.zeros(batch, dtype=np.float64)
        off_course = np.zeros(batch, dtype=np.float64)
        wall = np.zeros(batch, dtype=np.float64)
        excess = np.zeros(batch, dtype=np.float64)
        speed_squared = 0.0
        reasons: dict[str, int] = {}
        for step in range(steps):
            action = agent.act(obs, deterministic=True)
            obs, reward, done, reason, components = env.step(action)
            for lane in np.flatnonzero(done):
                name = TERMINAL_NAMES[reason[lane]]
                reasons[name] = reasons.get(name, 0) + 1
            if step < EVAL_WARMUP_STEPS:
                continue
            distance += components[COMPONENT_NAMES.index("own_progress")]
            off_course += components[COMPONENT_NAMES.index("off_course")]
            wall += components[COMPONENT_NAMES.index("wall")]
            excess += components[COMPONENT_NAMES.index("mode_excess")]
            speed_squared += float(
                ((obs[:, EGO_SPEED] * SPEED_SCALE) ** 2).mean()
            )

    flying = (steps - EVAL_WARMUP_STEPS) * STEP_SECONDS
    metres = distance.mean() / OWN_PROGRESS_RATE
    lap = lap_metres * flying / max(metres, 1.0)
    # The off-course penalty is a rate times speed squared times seconds, so
    # the seconds come back out of it. Progress is masked off course, which
    # is why a lap time alone can flatter a car that spends the lap beside
    # the road rather than on it.
    mean_speed_squared = speed_squared / (steps - EVAL_WARMUP_STEPS)
    off_seconds = (
        -off_course.mean() / (OFF_COURSE_RATE * mean_speed_squared)
        if mean_speed_squared > 1.0
        else 0.0
    )
    return {
        "lap": lap,
        "analytic": analytic,
        "gap": lap - analytic,
        "off_seconds": off_seconds,
        "wall": float(wall.mean()),
        "mode_excess": float(excess.mean()),
        "stalls": float(reasons.get("stalled", 0)),
    }


def report(agent, args, seed_base: int) -> dict[str, dict[str, float]]:
    """Every circuit, trained and held out, as flying lap times."""
    out: dict[str, dict[str, float]] = {}
    for name in TRACKS:
        out[name] = evaluate(
            agent, args.eval_batch, seed_base, args.solo, name,
            args.eval_steps,
        )
    return out


def lap_string(seconds: float) -> str:
    return f"{int(seconds // 60)}:{seconds % 60:06.3f}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=300_000)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--solo", action="store_true")
    parser.add_argument("--track", default=None)
    parser.add_argument("--eval-every", type=int, default=25_000)
    parser.add_argument("--eval-steps", type=int, default=2_400)
    parser.add_argument("--eval-batch", type=int, default=4)
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
        best_gap = -np.inf
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
                laps = report(agent, args, args.seed + 900_000)
                trained = [n for n, (_, _, t) in TRACKS.items() if t]
                held = [n for n, (_, _, t) in TRACKS.items() if not t]
                print(f"  eval at step {step}    飞驰圈")
                for group, names in (("训练", trained), ("保留", held)):
                    for name in names:
                        r = laps[name]
                        flags = ""
                        if r["off_seconds"] > 1.0:
                            flags += f"  出界 {r['off_seconds']:.0f}s"
                        if r["wall"] < -1.0:
                            flags += f"  撞墙 {r['wall']:.0f}"
                        if r["stalls"] > 0:
                            flags += f"  退赛 {r['stalls']:.0f}"
                        print(
                            f"    {group} {name:<15}"
                            f"{lap_string(r['lap']):>10}"
                            f"  解析 {lap_string(r['analytic']):>9}"
                            f"  {r['gap']:+7.2f}s{flags}"
                        )
                # The mean gap over the trained circuits is what a best
                # checkpoint is chosen on: one number, in seconds a lap,
                # and lower is better.
                mean_gap = sum(laps[n]["gap"] for n in trained) / len(trained)
                held_gap = sum(laps[n]["gap"] for n in held) / len(held)
                print(
                    f"    平均差  训练 {mean_gap:+.2f}s   保留 {held_gap:+.2f}s"
                )
                agent.save(str(checkpoint_dir / f"latest{args.tag}.pt"))
                if -mean_gap > best_gap:
                    best_gap = -mean_gap
                    agent.save(str(checkpoint_dir / f"best{args.tag}.pt"))
                    print(f"    saved best (训练平均差 {mean_gap:+.2f}s)")

    print("training finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
