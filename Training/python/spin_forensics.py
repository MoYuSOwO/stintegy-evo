"""Where the car is lost, and what the driver had just asked for.

An evaluation reports that a policy spun six times and retired four. Six
and four are not a diagnosis: a driver who loses it at the same corner
every time has a hole in one skill, and a driver who loses it uniformly
round the lap has a general one. These are opposite problems and the
scoreboard spells them identically.

So this records the landing point of every spin and every retirement --
the station round the lap, the speed it triggered at, and the second of
commands that led into it -- and then bins them by corner. The corner map
is not imported from anywhere; it is measured from the same session, out
of the path curvature the cars themselves drove.

    python3 spin_forensics.py checkpoints/bestarmA.pt
    python3 spin_forensics.py checkpoints/bestarmA.pt --track silverstone \\
        --lanes 12 --seconds 600 --json forensics.json

Commands are reported as fractions of the car's own limits, because that
is the space the policy acts in: the two action channels are normalised
against the curvature and brake ceilings the car reported that step.
"Asked for 0.98" means it asked for ninety-eight per cent of what the car
said it had, whatever that was worth at the speed it was doing.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import deque

import numpy as np

from host_env import TERMINAL_NAMES, HostEnv
from train import EGO_SPEED, EVALUATION_MODES, STEP_SECONDS, TRACKS

# Ego block layout: speed, longitudinal accel, lateral accel, yaw rate,
# sideslip. The scales are the observation writer's own
# (Core/Drivers/Learned/DirectDriveObservation.cs).
EGO_YAW_RATE = EGO_SPEED + 3
EGO_SIDESLIP = EGO_SPEED + 4
SPEED_SCALE = 100.0
YAW_RATE_SCALE = 2.0
SIDESLIP_SCALE = 0.5

# How much of the run-up to keep. One second is the window the order asks
# for; it is also about the horizon over which a slip angle builds, so a
# command that caused the loss should be inside it and a command that
# merely followed it should not.
LEAD_SECONDS = 1.0

# Corner map resolution. Ten metres is fine enough to separate a chicane's
# two halves and coarse enough that a single lane's wobble does not invent
# a corner.
CORNER_BIN_METRES = 10.0

# What counts as cornering, in path curvature (1/m). 0.004 is a radius of
# 250 m -- flat out in a fast car, but geometrically a corner, and the
# point of the map is to name places rather than to judge them.
CORNER_CURVATURE = 0.004

# A corner has to be at least this long to be one, and two corners closer
# than this are the same corner. Below it the map fills up with the
# curvature ripple of a straight, which is exactly the artefact the
# centreline cleaning order exists to remove.
CORNER_MIN_METRES = 30.0

SEED_BASE = 900_001


def _speed(obs: np.ndarray) -> np.ndarray:
    return obs[:, EGO_SPEED] * SPEED_SCALE


def _yaw_rate(obs: np.ndarray) -> np.ndarray:
    return obs[:, EGO_YAW_RATE] * YAW_RATE_SCALE


def _sideslip(obs: np.ndarray) -> np.ndarray:
    return obs[:, EGO_SIDESLIP] * SIDESLIP_SCALE


def build_corner_map(
    curvature_sum: np.ndarray, curvature_count: np.ndarray, lap_metres: float
) -> list[dict[str, float]]:
    """Name the corners from what the cars drove.

    Path curvature is yaw rate over speed -- the curvature of the line
    actually taken, not of the centreline. That is the right quantity
    here even though it is not the road: an event is being attributed to
    a place, and the place a driver is in is the one its own line is in.
    """
    bins = curvature_sum.size
    mean = np.divide(
        curvature_sum,
        np.maximum(curvature_count, 1.0),
        out=np.zeros(bins),
        where=curvature_count > 0,
    )
    cornering = mean >= CORNER_CURVATURE
    if not cornering.any():
        return []

    # Walk the loop from a bin that is not cornering, so a corner sitting
    # across the start line is one corner rather than two.
    start = int(np.argmin(cornering))
    if cornering[start]:
        return [
            {
                "index": 1,
                "start_m": 0.0,
                "end_m": lap_metres,
                "peak_curvature": float(mean.max()),
            }
        ]

    corners: list[dict[str, float]] = []
    run_start: int | None = None
    for offset in range(bins + 1):
        i = (start + offset) % bins
        inside = bool(cornering[i]) and offset < bins
        if inside and run_start is None:
            run_start = offset
        elif not inside and run_start is not None:
            length = (offset - run_start) * CORNER_BIN_METRES
            if length >= CORNER_MIN_METRES:
                span = [
                    mean[(start + k) % bins] for k in range(run_start, offset)
                ]
                corners.append(
                    {
                        "index": len(corners) + 1,
                        "start_m": ((start + run_start) % bins)
                        * CORNER_BIN_METRES,
                        "end_m": ((start + offset) % bins) * CORNER_BIN_METRES,
                        "peak_curvature": float(max(span)),
                    }
                )
            run_start = None
    for i, corner in enumerate(corners, start=1):
        corner["index"] = i
    return corners


def locate(station: float, corners: list[dict[str, float]]) -> str:
    for corner in corners:
        start, end = corner["start_m"], corner["end_m"]
        inside = (
            start <= station < end
            if start < end
            else station >= start or station < end
        )
        if inside:
            return f"T{corner['index']:d}"
    return "直道"


def probe(
    policy: str,
    track: str,
    lanes: int,
    seconds: float,
    modes: tuple[int, int],
) -> dict:
    from sac import SacAgent, SacConfig

    lap_metres, _ = TRACKS[track]
    bins = max(1, int(round(lap_metres / CORNER_BIN_METRES)))
    curvature_sum = np.zeros(bins)
    curvature_count = np.zeros(bins)
    lead_steps = max(1, int(round(LEAD_SECONDS / STEP_SECONDS)))
    events: list[dict] = []

    with HostEnv(
        batch=lanes,
        seed_base=SEED_BASE,
        solo=True,
        track=track,
        episode_seconds=seconds + 60.0,
        ego_modes=modes,
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
        agent.load(policy)
        obs = env.reset()
        # One rolling second of history per lane. A spin is read off the
        # step it began on, so the run-up has to already be in hand when
        # it happens -- there is no going back for it.
        history = [deque(maxlen=lead_steps) for _ in range(lanes)]
        steps = int(round(seconds / STEP_SECONDS))

        for _ in range(steps):
            action = agent.act(obs, deterministic=True)
            speed = _speed(obs)
            slip = _sideslip(obs)
            for lane in range(lanes):
                history[lane].append(
                    {
                        "curvature_cmd": float(action[lane, 0]),
                        "accel_cmd": float(action[lane, 1]),
                        "speed": float(speed[lane]),
                        "sideslip_deg": float(math.degrees(slip[lane])),
                    }
                )
            obs, _, done, reason, _, race, final_obs, spins = env.step(action)

            station = np.mod(race, lap_metres)
            end_speed = _speed(final_obs)
            end_yaw = _yaw_rate(final_obs)
            moving = end_speed > 5.0
            path_curvature = np.abs(
                np.divide(
                    end_yaw,
                    np.maximum(end_speed, 1.0),
                    out=np.zeros_like(end_yaw),
                    where=moving,
                )
            )
            idx = np.clip((station / CORNER_BIN_METRES).astype(int), 0, bins - 1)
            for lane in range(lanes):
                if moving[lane]:
                    curvature_sum[idx[lane]] += path_curvature[lane]
                    curvature_count[idx[lane]] += 1.0

            for lane in range(lanes):
                stalled = bool(done[lane]) and (
                    TERMINAL_NAMES[reason[lane]] == "stalled"
                )
                if not spins[lane] and not stalled:
                    continue
                lead = list(history[lane])
                events.append(
                    {
                        "kind": "退赛" if stalled else "旋转",
                        "lane": lane,
                        "station_m": float(station[lane]),
                        "speed_kph": (
                            float(lead[-1]["speed"] * 3.6)
                            if lead
                            else float("nan")
                        ),
                        "sideslip_deg": (
                            float(lead[-1]["sideslip_deg"])
                            if lead
                            else float("nan")
                        ),
                        "lead": lead,
                    }
                )
                if done[lane]:
                    # The lane re-seeded; its next second belongs to a new
                    # episode and must not be read as the run-up to
                    # anything that happened in the old one.
                    history[lane].clear()

    corners = build_corner_map(curvature_sum, curvature_count, lap_metres)
    for event in events:
        event["corner"] = locate(event["station_m"], corners)
    return {
        "policy": policy,
        "track": track,
        "lanes": lanes,
        "seconds": seconds,
        "corners": corners,
        "events": events,
    }


def summarise(result: dict) -> None:
    events = result["events"]
    corners = result["corners"]
    spins = [e for e in events if e["kind"] == "旋转"]
    retires = [e for e in events if e["kind"] == "退赛"]
    print(
        f"事故化验 · {result['policy']} · {result['track']} · "
        f"{result['lanes']} 车道 × {result['seconds']:.0f} 秒"
    )
    print(
        f"弯 {len(corners)} 个（自本场测量，非赛道数据）  "
        f"旋转 {len(spins)}  退赛 {len(retires)}\n"
    )
    if not events:
        print("无事故。")
        return

    print("落点直方图")
    print(f"  {'位置':<8} {'桩号':>12}  {'旋转':>4}  {'退赛':>4}")
    places = [f"T{c['index']:d}" for c in corners] + ["直道"]
    for place in places:
        here = [e for e in events if e["corner"] == place]
        if not here:
            continue
        if place == "直道":
            span = "—"
        else:
            corner = corners[int(place[1:]) - 1]
            span = f"{corner['start_m']:.0f}~{corner['end_m']:.0f}"
        print(
            f"  {place:<8} {span:>12}  "
            f"{sum(1 for e in here if e['kind'] == '旋转'):>4}  "
            f"{sum(1 for e in here if e['kind'] == '退赛'):>4}"
        )

    print("\n逐次事故（前 1 秒指令取该秒极值，占车自身上限的比例）")
    print(
        f"  {'类型':<5} {'位置':<6} {'桩号':>8} {'时速':>7} {'侧滑':>7}"
        f"  {'曲率峰':>7} {'刹车峰':>7} {'曲率末':>7}"
    )
    for event in sorted(events, key=lambda e: e["station_m"]):
        lead = event["lead"]
        if lead:
            curvature_peak = max(abs(s["curvature_cmd"]) for s in lead)
            brake_peak = -min(min(s["accel_cmd"] for s in lead), 0.0)
            curvature_last = lead[-1]["curvature_cmd"]
        else:
            curvature_peak = brake_peak = curvature_last = float("nan")
        print(
            f"  {event['kind']:<5} {event['corner']:<6} "
            f"{event['station_m']:8.0f} {event['speed_kph']:7.1f} "
            f"{event['sideslip_deg']:7.1f}"
            f"  {curvature_peak:7.2f} {brake_peak:7.2f} {curvature_last:7.2f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("policy")
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--lanes", type=int, default=12)
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--json", default=None)
    args = parser.parse_args()

    if args.track not in TRACKS:
        print(f"unknown circuit: {args.track}")
        return 2

    result = probe(
        args.policy, args.track, args.lanes, args.seconds, EVALUATION_MODES
    )
    summarise(result)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        print(f"\n明细写入 {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
