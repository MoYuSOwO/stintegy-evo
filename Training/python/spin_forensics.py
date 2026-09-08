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

# The road-and-limits block sits immediately before the ego block. Its
# first four slots are the distance from the car's centre to each wall and
# the width of each buffer, all divided by the same scale -- which is
# enough to recover the half-width of the road and where the car is
# across it, without adding a channel to the protocol.
ROAD_BLOCK = EGO_SPEED - 13
BUFFER_SCALE = 20.0

# Where the wheels are, relative to the car's centre. Only the track width
# matters here: the wheels' fore-and-aft positions move them across the
# road only through the car's yaw relative to the track, which this cannot
# see. At ten degrees of yaw that is about a quarter of a metre on the
# front axle -- enough to blur which side of a boundary a wheel is on in a
# genuinely marginal case, not enough to move a histogram.
HALF_TRACK_METRES = 0.8

# The tyre-and-battery block: four wheels of (surface temp, core temp,
# wear, load), then the primary store. Reading wear and the store at the
# moment of the loss is what turns "it happens late in the session" from a
# correlation into a mechanism -- or refutes it. The block sits at the
# front of the observation, right after the geometry.
TYRE_BLOCK = 198
TEMPERATURE_SCALE = 150.0

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


def _consumables(obs: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Worst tyre wear, hottest core, and what is left in the store."""
    wear = np.max(obs[:, [TYRE_BLOCK + 2 + 4 * i for i in range(4)]], axis=1)
    core = np.max(
        obs[:, [TYRE_BLOCK + 1 + 4 * i for i in range(4)]], axis=1
    ) * TEMPERATURE_SCALE
    store = obs[:, TYRE_BLOCK + 16]
    return wear, core, store


def _road(obs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Half the road's width, and how far the car is from its middle.

    Positive offset is to the right of travel, matching the simulation's
    own convention: its normal is the mathematical right, so d grows to
    the right.
    """
    to_left_wall = obs[:, ROAD_BLOCK] * BUFFER_SCALE
    to_right_wall = obs[:, ROAD_BLOCK + 1] * BUFFER_SCALE
    left_buffer = obs[:, ROAD_BLOCK + 2] * BUFFER_SCALE
    right_buffer = obs[:, ROAD_BLOCK + 3] * BUFFER_SCALE
    to_left_line = to_left_wall - left_buffer
    to_right_line = to_right_wall - right_buffer
    half_width = 0.5 * (to_left_line + to_right_line)
    offset = 0.5 * (to_right_line - to_left_line)
    return half_width, offset


def surface_of(offset: float, half_width: float) -> str:
    """Which of the edge grammar's three surfaces a wheel is on.

    Tarmac to the white line, kerb for the first 0.6 m past it, grass
    beyond that -- the same three the physics prices, named rather than
    numbered so a histogram reads as a sentence.
    """
    past = abs(offset) - half_width
    if past <= 0.0:
        return "柏油"
    return "路肩" if past <= 0.6 else "草"


def wheel_surfaces(offset: float, half_width: float) -> str:
    left = surface_of(offset - HALF_TRACK_METRES, half_width)
    right = surface_of(offset + HALF_TRACK_METRES, half_width)
    return left if left == right else f"{left}/{right}"

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

        for step_index in range(steps):
            action = agent.act(obs, deterministic=True)
            speed = _speed(obs)
            slip = _sideslip(obs)
            half_width, offset = _road(obs)
            wear, core, store = _consumables(obs)
            for lane in range(lanes):
                history[lane].append(
                    {
                        "curvature_cmd": float(action[lane, 0]),
                        "accel_cmd": float(action[lane, 1]),
                        "speed": float(speed[lane]),
                        "sideslip_deg": float(math.degrees(slip[lane])),
                        "offset_m": float(offset[lane]),
                        "half_width_m": float(half_width[lane]),
                        "surfaces": wheel_surfaces(
                            float(offset[lane]), float(half_width[lane])
                        ),
                        "wear": float(wear[lane]),
                        "core_c": float(core[lane]),
                        "store": float(store[lane]),
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
                        # When in the session, not just where on the lap.
                        # A driver that is clean on fresh tyres and loses
                        # it on worn ones and a driver that is simply loose
                        # produce the same total; only the clock separates
                        # them.
                        "t_seconds": step_index * STEP_SECONDS,
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
                        "surfaces": lead[-1]["surfaces"] if lead else "?",
                        "wear": lead[-1]["wear"] if lead else float("nan"),
                        "core_c": lead[-1]["core_c"] if lead else float("nan"),
                        "store": lead[-1]["store"] if lead else float("nan"),
                        "offset_m": (
                            float(lead[-1]["offset_m"]) if lead else float("nan")
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

    seconds = result["seconds"]
    print("\n何时出的事（前后半场）与哪条车道")
    half = seconds / 2.0
    early = sum(1 for e in events if e.get("t_seconds", 0.0) < half)
    print(f"  前半场 {early}   后半场 {len(events) - early}   "
          f"（会话 {seconds:.0f} 秒，胎是越跑越旧的）")
    lanes: dict[int, int] = {}
    for e in events:
        lanes[e["lane"]] = lanes.get(e["lane"], 0) + 1
    print("  逐车道：" + "  ".join(
        f"{lane}:{n}" for lane, n in sorted(lanes.items())
    ))

    worn = [e["wear"] for e in events if e.get("wear") == e.get("wear")]
    if worn:
        cores = [e["core_c"] for e in events]
        stores = [e["store"] for e in events]
        print("\n触发瞬间的耗材状态（最坏那条胎 / 主储能）")
        print(
            f"  胎耗  最小 {min(worn) * 100:.0f}%  中位 "
            f"{sorted(worn)[len(worn) // 2] * 100:.0f}%  最大 "
            f"{max(worn) * 100:.0f}%"
        )
        print(
            f"  核心温 最小 {min(cores):.0f}°C  中位 "
            f"{sorted(cores)[len(cores) // 2]:.0f}°C  最大 {max(cores):.0f}°C"
        )
        print(
            f"  余量  最小 {min(stores) * 100:.0f}%  中位 "
            f"{sorted(stores)[len(stores) // 2] * 100:.0f}%  最大 "
            f"{max(stores) * 100:.0f}%"
        )

    print("\n触发瞬间四轮所在表面")
    tally: dict[str, list[int]] = {}
    for event in events:
        slot = tally.setdefault(event.get("surfaces", "?"), [0, 0])
        slot[0 if event["kind"] == "旋转" else 1] += 1
    for surfaces, (spun, retired) in sorted(
        tally.items(), key=lambda kv: -sum(kv[1])
    ):
        print(f"  {surfaces:<10}  旋转 {spun:>3}  退赛 {retired:>3}")

    print("\n逐次事故（前 1 秒指令取该秒极值，占车自身上限的比例）")
    print(
        f"  {'类型':<5} {'位置':<6} {'桩号':>8} {'时速':>7} {'侧滑':>7}"
        f"  {'左右轮':<10} {'横位':>6}"
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
            f"  {event.get('surfaces', '?'):<10} {event.get('offset_m', 0.0):6.2f}"
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
