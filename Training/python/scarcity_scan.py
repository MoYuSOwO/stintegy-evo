"""Where energy becomes a decision, and where it is only a number.

The five power settings currently differ by under three per cent in what
they consume, and the store is generous enough that finishing the
distance is never in question. That is not a strategy game: there is one
right answer -- run Attack -- and a choice with one right answer is not a
choice. The energy on board has to be scarce enough that going flat out
does not reach the end.

How scarce is a calibration, and the yardstick comes from outside:

    Formula 2      no energy management at all, flat out throughout
    Formula 1 2026 deployment has to be budgeted, but budgeted rather
                   than survived
    Formula E      about twice the store gets used at full effort -- half
                   the race has to be found by driving differently

Under 1.0 there is no decision, because Attack always finishes. Near 2.0
saving beats pace and the racing disappears into economy running. The
band this project wants is the middle: **1.2 to 1.4 times the store**,
where a lap is still worth driving hard and the sums still have to be
done.

The scan does not need re-simulation per cell. Energy per lap at full
effort barely depends on how much store the car started with -- the pack
only intrudes once it is nearly empty, and a car that runs out was never
in the band being looked for. So one measurement of joules per lap makes
the whole capacity-by-distance matrix arithmetic.

    python3 scarcity_scan.py --policy checkpoints/keep/<a best>.pt
"""

from __future__ import annotations

import argparse

import numpy as np

from host_env import HostEnv
from train import STEP_SECONDS, TRACKS

# Where the store's remaining fraction sits in the observation: the
# tyre-and-battery block is four wheels of four channels, then the store.
TYRE_BLOCK = 198
STORE = TYRE_BLOCK + 16

# Attack on both ladders. Consumption is being measured, so the car has to
# be asked for everything it has; a scan run at Normal would report the
# scarcity of a car nobody is driving flat out.
FULL_EFFORT_MODES = (5, 5)

SEED_BASE = 900_001

# What the store channel measures is a *fraction* of whatever pack is
# fitted, so turning it into joules needs that pack's size — and this
# script cannot read a C# constant. It is therefore an argument with a
# default, printed loudly, rather than a constant quietly baked in: the
# first version hard-coded 1470 MJ, the pack was changed to 1100
# underneath it, and it went on reporting joules a third too high while
# every measured fraction it printed was correct.
DEFAULT_CAPACITY_MJ = 1100.0
CAPACITY_SCALES = (1.34, 1.15, 1.0, 0.9, 0.8, 0.7, 0.55, 0.4)
RACE_LAPS = (10, 15, 20, 25, 30, 40, 52)

TARGET_LOW, TARGET_HIGH = 1.2, 1.4


def measure_energy_per_lap(
    policy: str, track: str, lanes: int, seconds: float, capacity_j: float
) -> dict[str, float]:
    """Joules spent per lap with the car driven as hard as it will go.

    Measured as store consumed over distance covered, across every lane,
    rather than lane by lane: a lane that re-seeds mid-lap contributes its
    metres and its joules honestly, and neither number needs a completed
    lap to be meaningful.
    """
    from sac import SacAgent, SacConfig

    lap_metres, _ = TRACKS[track]
    with HostEnv(
        batch=lanes,
        seed_base=SEED_BASE,
        solo=True,
        track=track,
        episode_seconds=seconds + 60.0,
        ego_modes=FULL_EFFORT_MODES,
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
        agent.load(policy)
        obs = env.reset()
        steps = int(round(seconds / STEP_SECONDS))

        previous_store = obs[:, STORE].copy()
        previous_race: np.ndarray | None = None
        spent_fraction = 0.0
        metres = 0.0
        floor_steps = 0

        for _ in range(steps):
            action = agent.act(obs, deterministic=True)
            obs, _, done, _, _, race, final_obs, _ = env.step(action)

            store = final_obs[:, STORE]
            # Regeneration means the store goes up as well as down; both
            # directions are real and the net is what the race pays.
            drop = previous_store - store
            spent_fraction += float(drop[~done].sum())
            floor_steps += int((store < 0.2).sum())

            if previous_race is not None:
                advanced = race - previous_race
                metres += float(advanced[(advanced > 0.0) & ~done].sum())
            previous_race = race.copy()
            previous_store = obs[:, STORE].copy()

    laps = metres / lap_metres
    return {
        "laps": laps,
        "fraction_per_lap": spent_fraction / laps if laps > 0 else float("nan"),
        "joules_per_lap": (
            spent_fraction / laps * capacity_j if laps > 0 else float("nan")
        ),
        "steps_under_20pc": float(floor_steps),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", required=True)
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--lanes", type=int, default=12)
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument(
        "--capacity-mj", type=float, default=DEFAULT_CAPACITY_MJ,
        help="the pack actually fitted; must match ElectricPowertrain",
    )
    args = parser.parse_args()

    if args.track not in TRACKS:
        print(f"unknown circuit: {args.track}")
        return 2

    print(
        f"稀缺扫描 · {args.track} · {args.lanes} 车道 × {args.seconds:.0f} 模拟秒"
        f" · 档位 {FULL_EFFORT_MODES[0]}/{FULL_EFFORT_MODES[1]}（全力）",
        flush=True,
    )
    capacity_j = args.capacity_mj * 1e6
    print(f"  假定装车容量 {args.capacity_mj:.0f} MJ（须与 ElectricPowertrain 一致）")
    m = measure_energy_per_lap(
        args.policy, args.track, args.lanes, args.seconds, capacity_j
    )
    print(
        f"\n实测：{m['laps']:.1f} 圈  每圈耗 {m['fraction_per_lap'] * 100:.2f}% "
        f"存量 = {m['joules_per_lap'] / 1e6:.1f} MJ/圈"
        f"   低于 20% SoC 的步数 {m['steps_under_20pc']:.0f}"
    )
    print(
        f"  现行容量 {args.capacity_mj:.0f} MJ 下，全力可跑 "
        f"{1.0 / m['fraction_per_lap']:.1f} 圈"
    )

    print("\n全力消耗 ÷ 存量（目标带 1.2~1.4 标 ★，1.0 以下无决定）")
    header = "  容量 MJ  " + "".join(f"{n:>7d}圈" for n in RACE_LAPS)
    print(header)
    for scale in CAPACITY_SCALES:
        capacity = capacity_j * scale
        cells = []
        for laps in RACE_LAPS:
            ratio = laps * m["joules_per_lap"] / capacity
            mark = "★" if TARGET_LOW <= ratio <= TARGET_HIGH else " "
            cells.append(f"{ratio:>7.2f}{mark}")
        print(f"  {capacity / 1e6:>7.0f}  " + "".join(cells))

    print(
        "\n读法：★ 的格子里，全力跑完要花 1.2~1.4 倍的存量——省下来的那\n"
        "两三成必须靠开法找回来，而找回来的手段（刹车区前松电门）要付\n"
        "一点圈速。这就是凸取舍存在的地方。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
