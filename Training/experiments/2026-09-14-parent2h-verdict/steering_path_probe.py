"""Where the tyre was on its curve when a reflex-governed car still spun.

The reflex caps the pedals, so under it use <= max(allowance, s): any step
still over the allowance is a step whose slip angle alone is over it. This
probe asks the follow-up the order needs -- for each remaining spin, how
far along its curve the axle that was over had been driven.

The rear slip angle is not in the observation, but everything it is built
from is: body sideslip, yaw rate and speed, with the rear axle 1.457 m
behind the centre of mass (3.1 m wheelbase x 0.47 front load share, from
CarConfig), exactly as CarPhysics.RearSlipAngle computes it. The front
cannot be rebuilt the same way, because its slip angle needs the steer
angle and the observation carries only the steering *command*; for the
front, the probe reports the share of the curve the axle was using, which
is s itself once the pedals are capped, and says which side of the peak
it cannot tell.

One-off analysis for the parent2h verdict, kept with its data. It reads
the observation the policy receives and writes nothing but its output.

    python3 steering_path_probe.py checkpoints/evalparent2h-1250000.pt
"""
import json
import math
import sys
from collections import deque

TREE = "/Users/jayhuang/Code/stintegy-evo/.worktrees/parent2h/Training/python"
sys.path.insert(0, TREE)

import numpy as np

from host_env import TERMINAL_NAMES, HostEnv
from spin_accounting import FRONT_USE, GRIP_ALLOWANCE, REAR_USE
from spin_forensics import SEED_BASE, _sideslip, _speed, _yaw_rate
from train import EVALUATION_MODES, STEP_SECONDS, TRACKS, assert_observation_layout

REAR_ARM_METRES = 3.1 * 0.47
MINIMUM_SLIP_SPEED = 3.0
PEAK_SLIP_RADIANS = 0.13962634
ONSET_SIDESLIP = 0.25
WINDOW_STEPS = int(round(4.0 / STEP_SECONDS))
JUDGED_STEPS = int(round(1.0 / STEP_SECONDS))


def main(policy: str, lanes: int = 12, seconds: float = 600.0) -> int:
    from sac import SacAgent, SacConfig

    lap = TRACKS["silverstone"][0]
    events = []
    over_steps = 0
    over_rear_past_peak = 0
    over_rear_steps = 0
    steps_seen = 0
    with HostEnv(batch=lanes, seed_base=SEED_BASE, solo=True, track="silverstone",
                 episode_seconds=seconds + 60.0, ego_modes=EVALUATION_MODES) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
        agent.load(policy)
        obs = env.reset()
        assert_observation_layout(obs)
        history = [deque(maxlen=WINDOW_STEPS) for _ in range(lanes)]
        for step in range(int(round(seconds / STEP_SECONDS))):
            action = agent.act(obs, deterministic=True)
            allowance = obs[:, GRIP_ALLOWANCE]
            front = obs[:, FRONT_USE]
            rear = obs[:, REAR_USE]
            speed = _speed(obs)
            slip = _sideslip(obs)
            yaw = _yaw_rate(obs)
            rear_alpha = -slip + REAR_ARM_METRES * yaw / np.maximum(speed, MINIMUM_SLIP_SPEED)
            for lane in range(lanes):
                item = {
                    "allowance": float(allowance[lane]),
                    "front_use": float(front[lane]),
                    "rear_use": float(rear[lane]),
                    "excess": float(max(front[lane], rear[lane]) - allowance[lane]),
                    "rear_alpha_over_peak": float(abs(rear_alpha[lane]) / PEAK_SLIP_RADIANS),
                    "sideslip": float(slip[lane]),
                    "speed": float(speed[lane]),
                    "accel_cmd": float(action[lane, 1]),
                    "curvature_cmd": float(action[lane, 0]),
                }
                history[lane].append(item)
                steps_seen += 1
                if item["excess"] > 0.0:
                    over_steps += 1
                    if rear[lane] - allowance[lane] > 0.0:
                        over_rear_steps += 1
                        if item["rear_alpha_over_peak"] >= 1.0:
                            over_rear_past_peak += 1
            obs, _, done, reason, _, race, _, spins = env.step(action)
            station = np.mod(race, lap)
            for lane in range(lanes):
                stalled = bool(done[lane]) and TERMINAL_NAMES[reason[lane]] == "stalled"
                if not spins[lane] and not stalled:
                    continue
                window = list(history[lane])
                onset = next((i for i, it in enumerate(window) if abs(it["sideslip"]) >= ONSET_SIDESLIP), len(window))
                judged = window[max(0, onset - JUDGED_STEPS):onset]
                peak = max(judged, key=lambda it: it["excess"], default=None)
                events.append({
                    "kind": "retirement" if stalled else "spin",
                    "lane": lane,
                    "t_seconds": step * STEP_SECONDS,
                    "station_m": float(station[lane]),
                    "judged_steps": len(judged),
                    "peak_excess": peak["excess"] if peak else float("nan"),
                    "mode_excess_billed": bool(peak and peak["excess"] > 0.0),
                    "front_over": bool(peak and peak["front_use"] > peak["allowance"]),
                    "rear_over": bool(peak and peak["rear_use"] > peak["allowance"]),
                    "front_share_of_curve": peak["front_use"] if peak else float("nan"),
                    "rear_share_of_curve": peak["rear_use"] if peak else float("nan"),
                    "rear_alpha_over_peak": peak["rear_alpha_over_peak"] if peak else float("nan"),
                    "max_rear_alpha_over_peak_in_judged_second": max((it["rear_alpha_over_peak"] for it in judged), default=float("nan")),
                    "accel_cmd_at_peak": peak["accel_cmd"] if peak else float("nan"),
                    "speed_at_peak": peak["speed"] if peak else float("nan"),
                    "window": window,
                })
                if done[lane]:
                    history[lane].clear()
    out = {
        "policy": policy,
        "lanes": lanes,
        "seconds": seconds,
        "seed_base": SEED_BASE,
        "rear_arm_metres": REAR_ARM_METRES,
        "exposure": {
            "steps": steps_seen,
            "over_steps": over_steps,
            "over_steps_rear": over_rear_steps,
            "over_steps_rear_past_peak": over_rear_past_peak,
        },
        "events": events,
    }
    json.dump(out, open("steering-path.json", "w"), ensure_ascii=False, indent=1)
    spins = [e for e in events if e["kind"] == "spin"]
    print(f"{policy}: {len(spins)} spins, {sum(1 for e in events if e['kind']=='retirement')} retirements")
    print(f"exposure: {over_steps/steps_seen:.1%} of steps over; rear over in {over_rear_steps} of them, "
          f"rear past its peak slip angle in {over_rear_past_peak}")
    for e in spins:
        print(f"  t={e['t_seconds']:6.1f}s station={e['station_m']:6.0f}m billed={e['mode_excess_billed']} "
              f"front_over={e['front_over']} rear_over={e['rear_over']} "
              f"front_s={e['front_share_of_curve']:.3f} rear_s={e['rear_share_of_curve']:.3f} "
              f"rear_alpha/peak={e['rear_alpha_over_peak']:.2f} (max {e['max_rear_alpha_over_peak_in_judged_second']:.2f}) "
              f"accel_cmd={e['accel_cmd_at_peak']:+.2f} speed={e['speed_at_peak']:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
