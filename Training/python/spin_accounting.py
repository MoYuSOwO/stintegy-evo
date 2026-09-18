"""Whether the spins happen while the driver is disobeying the pit wall.

A tyre mode is a contract about how much of the friction circle the driver
may spend: at 3/3 that is 97.7% of it, and the environment charges a
penalty per second for whatever is taken beyond. The proposition this tool
exists to settle is that the spins are a by-product of breaking that
contract rather than a risk of keeping it.

Two things make that claim easy to confirm by accident, and both are
guarded here.

The first is the slide itself. A car that is already sliding is spending
the whole friction circle by definition, so the second before a spin is
declared is nearly all spin. Reading the contract there measures the
consequence and calls it the cause. So the run-up is cut at the onset of
the slide -- the first moment the sideslip angle leaves anything a corner
would produce -- and the contract is read in the second before *that*.

The second is exposure. A driver who spends a third of its lap fractionally
over the allowance will be over it during most of anything, spins included.
So the same window is measured twice: once before each loss, and once on
every ordinary second of the session, ordinary meaning that no slide is in
it. What matters is the difference between the two columns.

Everything is read from the observation the policy itself receives -- the
allowance, each axle's friction-circle use, sideslip, tyre temperatures and
wear -- so nothing here needs a host-side probe, and the excess computed is
the same number the reward's mode_excess term is built on.

    python3 spin_accounting.py checkpoints/evalparent2f-1075000.pt
    python3 spin_accounting.py checkpoints/a.pt checkpoints/b.pt \\
        --lanes 12 --seconds 600 --json out.json
"""

from __future__ import annotations

import argparse
import json
from collections import deque

import numpy as np

from host_env import TERMINAL_NAMES, HostEnv
from spin_forensics import (
    CORNER_BIN_METRES,
    ROAD_BLOCK,
    SEED_BASE,
    TEMPERATURE_SCALE,
    TYRE_BLOCK,
    _consumables,
    _road,
    _sideslip,
    _speed,
    _yaw_rate,
    build_corner_map,
    locate,
    wheel_surfaces,
)
from train import (
    COMPONENT_NAMES,
    DECISION_HZ as DECISION_RATE,
    EVALUATION_MODES,
    STEP_SECONDS,
    TRACKS,
    assert_observation_layout,
)

# The road-and-limits block, past the four wall distances and the three
# slopes: the drive ceiling, the pit wall's allowance, then what each axle
# is spending. The last two are the two halves of the max() the mode_excess
# penalty is built on, so an excess computed here is the number the reward
# charged for.
DRIVE_CEILING = ROAD_BLOCK + 7
GRIP_ALLOWANCE = ROAD_BLOCK + 8
FRONT_USE = ROAD_BLOCK + 9
REAR_USE = ROAD_BLOCK + 10

# Rear tyre surface temperatures, which is where this era's heat trouble
# lived. The tyre block is four wheels of (surface, core, wear, load).
REAR_LEFT_SURFACE = TYRE_BLOCK + 8
REAR_RIGHT_SURFACE = TYRE_BLOCK + 12

# How much history to keep per lane. The judgement needs a second of
# contract reading that ends before the slide starts, and a slide can take
# most of a second to be declared, so four is room for both.
WINDOW_SECONDS = 4.0

# The second that gets judged, and the same second the exposure column is
# built from.
JUDGED_SECONDS = 1.0

# What counts as no longer driving. The spin verdict itself is 0.61 rad
# held for a quarter second -- far too late to be a cause -- and ordinary
# cornering in this car lives well below this figure, which the session's
# own percentiles are reported against. A quarter radian is fourteen
# degrees of the car pointing somewhere other than where it is going: not
# a corner, and not yet a verdict.
ONSET_SIDESLIP_RADIANS = 0.25

# An excess has to be worth a name. A float's worth of overshoot is not
# disobedience; a percent of the friction circle is a decision.
MATERIAL_EXCESS = 0.01


def _use(obs: np.ndarray) -> tuple[np.ndarray, ...]:
    """Allowance, each axle's spending, the harder-worked one, the excess.

    Which axle is over matters as much as that one is: the rear over its
    allowance on the way out of a corner is a different mistake from the
    front over it on the way in, and the penalty only ever saw the larger
    of the two.
    """
    allowance = obs[:, GRIP_ALLOWANCE]
    front = obs[:, FRONT_USE]
    rear = obs[:, REAR_USE]
    use = np.maximum(front, rear)
    return allowance, front, rear, use, use - allowance


def _rear_surface(obs: np.ndarray) -> np.ndarray:
    return np.maximum(
        obs[:, REAR_LEFT_SURFACE], obs[:, REAR_RIGHT_SURFACE]
    ) * TEMPERATURE_SCALE


def _sliding(window: list[dict]) -> bool:
    return any(
        abs(item["sideslip"]) >= ONSET_SIDESLIP_RADIANS for item in window
    )


def _axle(step: dict | None) -> str:
    if step is None:
        return "?"
    front = step["front_use"] - step["allowance"]
    rear = step["rear_use"] - step["allowance"]
    if front > 0.0 and rear > 0.0:
        return "both"
    if rear > 0.0:
        return "rear"
    if front > 0.0:
        return "front"
    return "none"


def _judge(window: list[dict], judged_steps: int) -> dict:
    """Read the contract in the second before the car stopped driving.

    The window is the whole history. The slide's onset is the first step in
    it where the sideslip leaves ordinary cornering; everything from there
    on is the loss rather than its run-up, and is reported separately so
    that the two are never added together.
    """
    onset = next(
        (
            index
            for index, item in enumerate(window)
            if abs(item["sideslip"]) >= ONSET_SIDESLIP_RADIANS
        ),
        len(window),
    )
    judged = window[max(0, onset - judged_steps):onset]
    during = window[onset:]
    peak = max(judged, key=lambda item: item["excess"], default=None)
    return {
        "judged_steps": len(judged),
        "onset_found": onset < len(window),
        "peak_excess": peak["excess"] if peak else float("nan"),
        "peak_use": peak["use"] if peak else float("nan"),
        "allowance": peak["allowance"] if peak else float("nan"),
        "axle_at_peak": _axle(peak),
        "accel_cmd_at_peak": peak["accel_cmd"] if peak else float("nan"),
        "curvature_cmd_at_peak": (
            peak["curvature_cmd"] if peak else float("nan")
        ),
        "speed_at_peak": peak["speed"] if peak else float("nan"),
        "wear_at_peak": peak["wear"] if peak else float("nan"),
        "rear_surface_at_peak": (
            peak["rear_surface_c"] if peak else float("nan")
        ),
        "sideslip_at_peak": peak["sideslip"] if peak else float("nan"),
        # For contrast only: what the sliding itself reads. A car that is
        # already sideways spends the whole circle, and quoting that as
        # evidence of disobedience is the mistake this split exists to
        # prevent.
        "peak_excess_during_slide": max(
            (item["excess"] for item in during), default=float("nan")
        ),
    }


def probe(
    policy: str,
    track: str,
    lanes: int,
    seconds: float,
    modes: tuple[int, int],
    seed_base: int,
) -> dict:
    from sac import SacAgent, SacConfig

    lap_metres, _ = TRACKS[track]
    bins = max(1, int(round(lap_metres / CORNER_BIN_METRES)))
    curvature_sum = np.zeros(bins)
    curvature_count = np.zeros(bins)
    window_steps = max(1, int(round(WINDOW_SECONDS / STEP_SECONDS)))
    judged_steps = max(1, int(round(JUDGED_SECONDS / STEP_SECONDS)))
    events: list[dict] = []

    # The exposure column: every ordinary second the session offered, an
    # ordinary second being one with no slide in it, by exactly the test
    # the run-up windows are cut with.
    steps_seen = 0
    steps_over = 0
    steps_over_material = 0
    windows_seen = 0
    windows_over = 0
    windows_over_material = 0
    windows_sliding = 0
    excess_sum = 0.0
    use_sum = 0.0
    sideslip_samples: list[float] = []
    # What a step of disobedience buys, measured where the confound lives.
    # Excess happens where the car is working hardest, and the reward for a
    # step is the ground it covered, so comparing over-the-allowance steps
    # with the session's average would credit disobedience with the
    # difference between a corner and a straight. These are binned by
    # station instead, and only bins that hold both kinds are compared, so
    # the comparison is between two passes through the same ten metres.
    progress_over = np.zeros(bins)
    progress_over_count = np.zeros(bins)
    excess_over_sum = np.zeros(bins)
    progress_clean = np.zeros(bins)
    progress_clean_count = np.zeros(bins)
    # The same question asked of whole laps. A step of excess buys speed
    # the car keeps for the seconds after it, and a per-step comparison
    # cannot see that: it credits the purchase only where it was made. A
    # lap that spent more of itself over the allowance and covered more
    # ground per step has been paid for the carrying as well.
    lap_ledger: dict[tuple[int, int], dict[str, float]] = {}
    corner_steps = np.zeros(bins)
    distance = 0.0

    with HostEnv(
        batch=lanes,
        seed_base=seed_base,
        solo=True,
        track=track,
        episode_seconds=seconds + 60.0,
        ego_modes=modes,
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
        agent.load(policy)
        obs = env.reset()
        assert_observation_layout(obs)
        history = [deque(maxlen=window_steps) for _ in range(lanes)]
        steps = int(round(seconds / STEP_SECONDS))
        previous_race: np.ndarray | None = None

        for step_index in range(steps):
            action = agent.act(obs, deterministic=True)
            allowance, front_use, rear_use, use, excess = _use(obs)
            speed = _speed(obs)
            slip = _sideslip(obs)
            wear, core, store = _consumables(obs)
            rear_surface = _rear_surface(obs)
            half_width, offset = _road(obs)
            for lane in range(lanes):
                history[lane].append(
                    {
                        "allowance": float(allowance[lane]),
                        "use": float(use[lane]),
                        "front_use": float(front_use[lane]),
                        "rear_use": float(rear_use[lane]),
                        "excess": float(excess[lane]),
                        "speed": float(speed[lane]),
                        "sideslip": float(slip[lane]),
                        "wear": float(wear[lane]),
                        "core_c": float(core[lane]),
                        "rear_surface_c": float(rear_surface[lane]),
                        "store": float(store[lane]),
                        "surfaces": wheel_surfaces(
                            float(offset[lane]), float(half_width[lane])
                        ),
                        "curvature_cmd": float(action[lane, 0]),
                        "accel_cmd": float(action[lane, 1]),
                    }
                )
                steps_seen += 1
                excess_sum += float(excess[lane])
                use_sum += float(use[lane])
                sideslip_samples.append(abs(float(slip[lane])))
                if excess[lane] > 0.0:
                    steps_over += 1
                if excess[lane] > MATERIAL_EXCESS:
                    steps_over_material += 1
                recent = list(history[lane])[-judged_steps:]
                if len(recent) == judged_steps:
                    if _sliding(recent):
                        windows_sliding += 1
                    else:
                        windows_seen += 1
                        peak = max(item["excess"] for item in recent)
                        if peak > 0.0:
                            windows_over += 1
                        if peak > MATERIAL_EXCESS:
                            windows_over_material += 1

            station_before = (
                np.mod(previous_race, lap_metres)
                if previous_race is not None
                else None
            )
            obs, _, done, reason, components, race, final_obs, spins = (
                env.step(action)
            )
            progress = components[COMPONENT_NAMES.index("own_progress")]
            if station_before is not None:
                where = np.clip(
                    (station_before / CORNER_BIN_METRES).astype(int),
                    0,
                    bins - 1,
                )
                for lane in range(lanes):
                    # A step whose progress was masked -- off course or
                    # spinning -- is not a step of driving, and averaging
                    # its zero into either column would price the mask
                    # rather than the excess.
                    if progress[lane] <= 0.0 or done[lane]:
                        continue
                    lap = lap_ledger.setdefault(
                        (lane, int(previous_race[lane] // lap_metres)),
                        {"progress": 0.0, "excess": 0.0, "steps": 0.0},
                    )
                    lap["progress"] += float(progress[lane])
                    lap["excess"] += max(0.0, float(excess[lane]))
                    lap["steps"] += 1.0
                    if excess[lane] > 0.0:
                        progress_over[where[lane]] += float(progress[lane])
                        progress_over_count[where[lane]] += 1.0
                        excess_over_sum[where[lane]] += float(excess[lane])
                    else:
                        progress_clean[where[lane]] += float(progress[lane])
                        progress_clean_count[where[lane]] += 1.0

            station = np.mod(race, lap_metres)
            race = np.asarray(race, dtype=float)
            if previous_race is not None:
                step_distance = race - previous_race
                distance += float(np.sum(step_distance[step_distance >= 0.0]))
            previous_race = race.copy()

            end_speed = _speed(final_obs)
            end_yaw = _yaw_rate(final_obs)
            moving = end_speed > 5.0
            # Signed: build_corner_map averages before it drops the sign,
            # so that a straight driven with corrections averages to zero
            # rather than to a corner.
            path_curvature = np.divide(
                end_yaw,
                np.maximum(end_speed, 1.0),
                out=np.zeros_like(end_yaw),
                where=moving,
            )
            idx = np.clip(
                (station / CORNER_BIN_METRES).astype(int), 0, bins - 1
            )
            for lane in range(lanes):
                if moving[lane]:
                    curvature_sum[idx[lane]] += path_curvature[lane]
                    curvature_count[idx[lane]] += 1.0
                    corner_steps[idx[lane]] += 1.0

            for lane in range(lanes):
                stalled = bool(done[lane]) and (
                    TERMINAL_NAMES[reason[lane]] == "stalled"
                )
                if not spins[lane] and not stalled:
                    continue
                window = list(history[lane])
                call = _judge(window, judged_steps)
                last = window[-1] if window else {}
                events.append(
                    {
                        "kind": "退赛" if stalled else "旋转",
                        "lane": lane,
                        "t_seconds": step_index * STEP_SECONDS,
                        "station_m": float(station[lane]),
                        "window_full": len(window) == window_steps,
                        "speed_kph": float(last.get("speed", float("nan")))
                        * 3.6,
                        "wear": float(last.get("wear", float("nan"))),
                        "core_c": float(last.get("core_c", float("nan"))),
                        "rear_surface_c": float(
                            last.get("rear_surface_c", float("nan"))
                        ),
                        "store": float(last.get("store", float("nan"))),
                        "surfaces": last.get("surfaces", "?"),
                        **call,
                        "window": window,
                    }
                )
                if done[lane]:
                    history[lane].clear()

    # Laps long enough to be laps: a lane's first and last are partial, and
    # a short one is a fragment of a session rather than a lap of driving.
    full = [
        entry
        for entry in lap_ledger.values()
        if entry["steps"] >= 0.7 * lap_metres / 60.0 * DECISION_RATE
    ]
    if len(full) >= 8:
        lap_excess = np.array([e["excess"] / e["steps"] for e in full])
        lap_progress = np.array([e["progress"] / e["steps"] for e in full])
        lap_slope, lap_intercept = np.polyfit(lap_excess, lap_progress, 1)
        lap_correlation = float(np.corrcoef(lap_excess, lap_progress)[0, 1])
    else:
        lap_slope = lap_intercept = lap_correlation = float("nan")

    both = (progress_over_count > 0) & (progress_clean_count > 0)
    weight = progress_over_count[both]
    delta = (
        progress_over[both] / progress_over_count[both]
        - progress_clean[both] / progress_clean_count[both]
    )
    mean_excess_over = excess_over_sum[both] / progress_over_count[both]
    extra_progress = float(np.sum(weight * delta))
    total_excess = float(np.sum(weight * mean_excess_over))
    margin = extra_progress / total_excess if total_excess > 0 else float("nan")

    corners = build_corner_map(curvature_sum, curvature_count, lap_metres)
    for event in events:
        event["corner"] = locate(event["station_m"], corners)
    laps = distance / lap_metres
    sideslip = np.asarray(sideslip_samples)
    return {
        "policy": policy,
        "track": track,
        "lanes": lanes,
        "seconds": seconds,
        "modes": list(modes),
        "seed_base": seed_base,
        "laps": laps,
        "material_excess": MATERIAL_EXCESS,
        "judged_seconds": JUDGED_SECONDS,
        "onset_sideslip": ONSET_SIDESLIP_RADIANS,
        "exposure": {
            "steps": steps_seen,
            "steps_over": steps_over,
            "steps_over_material": steps_over_material,
            "windows": windows_seen,
            "windows_over": windows_over,
            "windows_over_material": windows_over_material,
            "windows_sliding": windows_sliding,
            "mean_excess": excess_sum / max(1, steps_seen),
            "mean_use": use_sum / max(1, steps_seen),
            # Where the onset threshold sits relative to ordinary driving,
            # so a reader can see it is not cutting into cornering.
            "sideslip_p99": float(np.percentile(sideslip, 99)),
            "sideslip_p999": float(np.percentile(sideslip, 99.9)),
        },
        "margin": {
            "profit_per_unit_excess_per_step": margin,
            "lap_slope": float(lap_slope),
            "lap_correlation": lap_correlation,
            "laps_compared": len(full),
            "lap_excess_mean": float(np.mean(lap_excess)) if len(full) >= 8 else float("nan"),
            "lap_excess_p90": float(np.percentile(lap_excess, 90)) if len(full) >= 8 else float("nan"),
            "bins_compared": int(both.sum()),
            "steps_over": float(weight.sum()),
            "mean_excess_where_over": (
                total_excess / float(weight.sum()) if weight.sum() else float("nan")
            ),
            "mean_progress_clean": float(
                np.sum(progress_clean[both]) / np.sum(progress_clean_count[both])
            ) if both.any() else float("nan"),
        },
        "corners": corners,
        "corner_steps": corner_steps.tolist(),
        "events": events,
    }


def verdict(result: dict) -> dict:
    """The two columns, and what their difference is worth."""
    spins = [
        event
        for event in result["events"]
        if event["kind"] == "旋转" and event["judged_steps"] > 0
    ]
    exposure = result["exposure"]
    base_any = exposure["windows_over"] / max(1, exposure["windows"])
    base_material = exposure["windows_over_material"] / max(
        1, exposure["windows"]
    )
    over_any = sum(1 for event in spins if event["peak_excess"] > 0.0)
    over_material = sum(
        1
        for event in spins
        if event["peak_excess"] > result["material_excess"]
    )
    count = len(spins)
    share_any = over_any / count if count else float("nan")
    share_material = over_material / count if count else float("nan")
    return {
        "spins_judged": count,
        "spins_without_run_up": len(
            [
                event
                for event in result["events"]
                if event["kind"] == "旋转" and event["judged_steps"] == 0
            ]
        ),
        "over_any": over_any,
        "over_material": over_material,
        "share_over_any": share_any,
        "share_over_material": share_material,
        "base_over_any": base_any,
        "base_over_material": base_material,
        "lift_any": share_any / base_any if base_any > 0 else float("inf"),
        "lift_material": (
            share_material / base_material
            if base_material > 0
            else float("inf")
        ),
        "expected_over_any": base_any * count,
        "expected_over_material": base_material * count,
    }


def summarise(result: dict) -> None:
    spins = [event for event in result["events"] if event["kind"] == "旋转"]
    retirements = [
        event for event in result["events"] if event["kind"] == "退赛"
    ]
    call = verdict(result)
    exposure = result["exposure"]
    print(f"\n{result['policy']}")
    print(
        f"  {result['lanes']} lane × {result['seconds']:.0f} s ·"
        f" 档位 {result['modes'][0]}/{result['modes'][1]} ·"
        f" seed {result['seed_base']} · {result['laps']:.1f} 圈"
    )
    print(
        f"  旋转 {len(spins)}"
        f"（{len(spins) / max(result['laps'], 1e-9) * 100:.1f}/百圈）"
        f"  退赛 {len(retirements)}"
    )
    print(
        f"  合同：平均使用率 {exposure['mean_use']:.3f}，"
        f"平均超约 {exposure['mean_excess']:+.4f}；"
        f"逐步超约 {exposure['steps_over'] / max(1, exposure['steps']):.1%}"
        f"（实质 "
        f"{exposure['steps_over_material'] / max(1, exposure['steps']):.1%}）"
    )
    print(
        f"  失控起点定义：|侧滑| ≥ {result['onset_sideslip']:.2f} rad"
        f"（本场普通驾驶 p99 {exposure['sideslip_p99']:.3f}，"
        f"p99.9 {exposure['sideslip_p999']:.3f} rad）"
    )
    print(
        f"  基线：无打滑的 {result['judged_seconds']:.0f} 秒共 "
        f"{exposure['windows']} 个，{call['base_over_any']:.1%} 含超约"
        f"（实质 {call['base_over_material']:.1%}）"
    )
    print(
        f"  失控起点前 {result['judged_seconds']:.0f} 秒："
        f"{call['spins_judged']} 跤中 {call['over_any']} 跤含超约"
        f"（{call['share_over_any']:.1%}，实质 "
        f"{call['share_over_material']:.1%}），"
        f"倍率 {call['lift_any']:.2f}×"
        f"（实质 {call['lift_material']:.2f}×）"
    )
    if call["spins_judged"]:
        print(
            f"  若与超约无关，预计 {call['expected_over_any']:.1f} 跤，"
            f"实得 {call['over_any']}"
        )
    if call["spins_without_run_up"]:
        print(
            f"  另有 {call['spins_without_run_up']} 跤没有可判的起点前窗口，未计入"
        )

    margin = result["margin"]
    print(
        f"  圈级口径：每圈平均超约每高 1 单位，每步推进 "
        f"{margin['lap_slope']:+.4f}（{margin['laps_compared']} 圈，"
        f"相关 {margin['lap_correlation']:+.2f}，"
        f"圈均超约 {margin['lap_excess_mean']:.4f}，p90 {margin['lap_excess_p90']:.4f}）"
    )
    print(
        f"  步级口径：每单位 e 每步 {margin['profit_per_unit_excess_per_step']:+.4f} "
        f"（同桩号对照，{margin['bins_compared']} 格，"
        f"{margin['steps_over']:.0f} 个超约步，"
        f"平均 e {margin['mean_excess_where_over']:.4f}，"
        f"守约步平均推进 {margin['mean_progress_clean']:+.4f}/步）"
    )

    axles: dict[str, int] = {}
    for event in spins:
        axles[event["axle_at_peak"]] = axles.get(event["axle_at_peak"], 0) + 1
    if axles:
        print(
            "  起点前峰值超约在哪根轴：" + "，".join(
                f"{name} {count}"
                for name, count in sorted(axles.items(), key=lambda x: -x[1])
            )
        )

    pockets: dict[str, dict] = {}
    for event in spins:
        pocket = pockets.setdefault(
            event["corner"],
            {"count": 0, "wear": [], "rear_c": [], "excess": []},
        )
        pocket["count"] += 1
        pocket["wear"].append(event["wear"])
        pocket["rear_c"].append(event["rear_surface_c"])
        pocket["excess"].append(event["peak_excess"])
    if pockets:
        print("  旋转聚集（弯角 × 胎况）：")
        for name, pocket in sorted(
            pockets.items(), key=lambda item: -item[1]["count"]
        ):
            print(
                f"    {name:<20} {pocket['count']:>3} 跤 "
                f"胎耗中位 {np.median(pocket['wear']):.0%} "
                f"后胎面中位 {np.median(pocket['rear_c']):.0f}℃ "
                f"起点前峰值超约中位 {np.nanmedian(pocket['excess']):+.3f}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("policies", nargs="+")
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--lanes", type=int, default=12)
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--seed-base", type=int, default=SEED_BASE)
    parser.add_argument("--json", default=None)
    args = parser.parse_args()

    results = []
    for policy in args.policies:
        result = probe(
            policy,
            args.track,
            args.lanes,
            args.seconds,
            EVALUATION_MODES,
            args.seed_base,
        )
        summarise(result)
        results.append(result)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(results, handle, ensure_ascii=False, indent=1)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
