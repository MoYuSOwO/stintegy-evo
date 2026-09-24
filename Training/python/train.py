"""Training loop for the direct-drive racing policy.

Stage one of the plan's curriculum: a single car learning to drive.

The question used to be whether the learned policy beat the analytic
driver, and every lap was quoted as a gap to it. That comparison has been
retired along with the driver: a rule-based reference is only a yardstick
while it is the better driver, and once it is not, quoting against it says
more about the yardstick than about the car. Laps are now quoted
absolutely, against the reference band a real car of this class runs on
the same length of road.

Usage:
    python3 Training/python/train.py --solo --steps 300000
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np
import torch

from host_env import (
    COMPONENT_NAMES, DEFAULT_DECISION_HZ, TERMINAL_NAMES, HostEnv,
)
from nstep import NStepBatcher
from opponent import FrozenOpponent, draw_from_pool
from sac import SacAgent, SacConfig


# Constants the harness shares with the host. Kept here rather than
# rediscovered, because every one of them has been got wrong once: the
# progress rate is what turns a reward back into metres, the step is what
# turns steps back into seconds, and the off-course rate is what turns its
# penalty back into the seconds spent beside the road.
DECISION_HZ = DEFAULT_DECISION_HZ
STEP_SECONDS = 1.0 / DECISION_HZ
OWN_PROGRESS_RATE = 0.02
OFF_COURSE_RATE = 1e-3
WALL_RATE = 5e-3
SPEED_SCALE = 100.0
# Where the blocks are, from
# Training/StintegyEVO.TrainingHost/Adapter/DirectDriveObservation.cs.
# The world-v3 contract, 457 channels: geometry 198, tyres and battery 17,
# mode 1, aero 3, road and limits 13, resources and budget 5, ego 14,
# opponents 6x16, then the previous frame of ego and opponents (110).
#
# History worth keeping: in the 480-channel generation the resource slots
# 4x5 and vehicle descriptors 8 sat in front of ego.
# The resource slots and the descriptors were inserted in front of ego when
# the observation went from 452 to 480, and this constant was not moved with
# them. It spent that era pointing at a resource slot's presence flag, which
# is 1.0 whenever the slot is filled -- so every reader of it saw a car
# travelling at a steady 100 m/s. Nothing threw, because a plausible number
# is exactly what a wrong offset returns.
OBSERVATION_SIZE = 457
ROAD_AND_LIMITS = 219
RESOURCE_BLOCK = 232
RESOURCE_SLOT_REMAINING = (RESOURCE_BLOCK, RESOURCE_BLOCK + 2)
RESOURCE_SLOT_CAPACITY = (RESOURCE_BLOCK + 1, RESOURCE_BLOCK + 3)
BUDGET_DEVIATION = RESOURCE_BLOCK + 4
EGO_SPEED = 237
EGO_HEADING_SIN = EGO_SPEED + 5
EGO_HEADING_COS = EGO_SPEED + 6
# Ego block after speed: long, lat, yaw, sideslip, sin, cos, offset,
# two edge distances, last steering command. Same indices jitter_probe
# uses, so the eval screen and the standalone probe count one wheel.
EGO_LATERAL_ACCEL = EGO_SPEED + 2
EGO_COMMAND = EGO_SPEED + 10
STEER_ACCEL_SCALE = 20.0
STRAIGHT_LATERAL = 2.0
TIMEOUT_REASON = TERMINAL_NAMES.index("timeout")


# The harness may know the track; the policy may not. Lap lengths in metres,
# and whether the circuit is one the policy trains on.
#
# The middle column used to be the analytic driver's own flying lap, and
# every gap in every report was measured against it. It is gone. The
# rule-based driver was a yardstick for exactly as long as it was the
# quicker driver, and the moment the learned one passed it the number
# stopped meaning "how good is this policy" and started meaning "how good
# is that script". Laps are absolute now.
TRACKS: dict[str, tuple[float, bool]] = {
    # name: (lap metres, in the training set)
    "silverstone":    (5891.0, True),
    "shanghai":       (5451.0, True),
    "zandvoort":      (4259.0, True),
    "simple-right":   (1804.0, True),
    "simple-left":    (1804.0, True),
    "banked-sweeper": (4946.0, True),
    "sepang":         (5543.0, False),
    "monaco":         (3337.0, False),
    "daytona":        (4016.0, False),
    "speedway":       (8512.0, False),
    # The second coverage round. Baku trains; the other three examine.
    "baku":           (6003.0, True),
    "spa":            (7004.0, False),
    "monza":          (5793.0, False),
    "interlagos":     (4309.0, False),
    "singapore":      (4928.0, True),
    "portimao":       (4653.0, True),
    "flat-sweeper":   (4946.0, True),
}

# Four hundred seconds is two flying laps of the slowest circuit here at the
# pace the policy currently drives it, and more of the quicker ones. Ninety
# seconds did not reach the end of one lap of Silverstone.


# What a real car of this class does over this length of road, in seconds.
#
# The evaluation runs eighty percent charge, Normal tyres, Normal power and
# warm rubber, and that is a race configuration - so it is scored against a
# race lap and not a qualifying one. A single band, with the qualifying
# figure kept only as an alarm line.
#
# The circuit is Silverstone-*style*: the length is the real 5891 m and the
# character is the real one, but it is not a corner-by-corner replica. A
# second of soft edge on each side keeps this a yardstick rather than a
# vernier.
#
# Only circuits whose model geometry has been checked against the real one
# belong here. A band nobody has verified is a target nobody can fail.
REFERENCE_BANDS: dict[str, tuple[float, float, float]] = {
    # name: (race band low, race band high, qualifying band low)
    "silverstone": (102.0, 105.0, 98.0),
}

# The tolerance the "style" in the circuit's name buys.
BAND_SOFT_EDGE_SECONDS = 1.0


def band_note(track: str, seconds: float) -> str:
    """Where a lap sits against the reference band, when there is one.

    Graduation is "no slower than the top of the race band". Being quicker
    than the band is not a failure - it is the point - but there is a line
    past which it stops being a compliment: a race configuration that beats
    the bottom of the *qualifying* band is not a fast policy, it is a
    suspicion that the car is too generous, and it is read the way a
    leakage alarm is read rather than as a record.
    """
    band = REFERENCE_BANDS.get(track)
    if band is None or not math.isfinite(seconds):
        return ""
    race_low, race_high, qualifying_low = band
    edge = BAND_SOFT_EDGE_SECONDS
    if seconds < qualifying_low - edge:
        return "  ⚠物理过宽嫌疑"
    if seconds < race_low - edge:
        return "  超正赛带"
    if seconds <= race_high + edge:
        return "  正赛带"
    return "  慢于正赛带"


def meets_reference_band(track: str, seconds: float) -> bool:
    """Whether a lap clears the graduation line: no slower than the top of
    the race band, and not so fast that the car itself is in question."""
    band = REFERENCE_BANDS.get(track)
    if band is None or not math.isfinite(seconds):
        return False
    race_low, race_high, qualifying_low = band
    edge = BAND_SOFT_EDGE_SECONDS
    return qualifying_low - edge <= seconds <= race_high + edge


EVALUATION_MODES = (3, 3)
# Tyres, then the primary store: the tyre block is four wheels of surface
# temperature, core temperature, wear and load, and the store's remaining
# fraction is the slot after them.
TIRE_BLOCK = 198
WEAR_SLOTS = (TIRE_BLOCK + 2, TIRE_BLOCK + 6, TIRE_BLOCK + 10, TIRE_BLOCK + 14)
PRIMARY_STORE = TIRE_BLOCK + 16

# Per-channel magnitude bounds for the self-check (freeze design section 4).
# Every channel is O(1) and held to 3, except where its range is set by
# construction or by physics the smoke measured, each named here: the four
# tyre loads (a share of static corner load; downforce at speed on banking
# reaches 4.5), the sideslip (up to pi over 0.5 when a car slides
# backwards), and the yaw rate (a crawling car in the kinematic regime lays
# its heading onto its travel at up to 17 rad/s, 8.5 scaled). The previous
# frame's copies of the ego channels take the same bounds.
PREVIOUS_FRAME = 347
OBSERVATION_BOUNDS = np.full(OBSERVATION_SIZE, 3.0)
for _load in (TIRE_BLOCK + 3, TIRE_BLOCK + 7, TIRE_BLOCK + 11, TIRE_BLOCK + 15):
    OBSERVATION_BOUNDS[_load] = 6.0
for _ego in (EGO_SPEED, PREVIOUS_FRAME):
    OBSERVATION_BOUNDS[_ego + 3] = 10.0            # yaw rate / 2
    OBSERVATION_BOUNDS[_ego + 4] = 2.0 * np.pi / 0.5 + 1e-3  # sideslip / 0.5


def assert_observation_layout(obs: np.ndarray) -> None:
    """Check the observation is laid out the way this file thinks it is.

    A stale offset does not raise; it returns a plausible number from the
    wrong channel, and every figure computed downstream stays plausible.
    So four things are asked that only the right layout can answer
    (freeze design, section 4):

    - the width is the contract's, 457;
    - two ego slots are the sine and cosine of the same angle, which no
      other pair in the protocol is;
    - absence is written on capacity: a resource slot whose capacity is
      zero must read zero remaining as well, and every present slot has a
      capacity above zero;
    - every channel is within its bound (OBSERVATION_BOUNDS: 3, with the
      named exceptions), since nothing is statistically normalised.
    """
    if obs.shape[-1] != OBSERVATION_SIZE:
        raise AssertionError(
            f"observation has {obs.shape[-1]} channels, the contract has "
            f"{OBSERVATION_SIZE}"
        )
    sin = obs[:, EGO_HEADING_SIN]
    cos = obs[:, EGO_HEADING_COS]
    error = np.max(np.abs(sin * sin + cos * cos - 1.0))
    if not error < 1e-3:
        raise AssertionError(
            f"the ego block is not at {EGO_SPEED}: sin^2 + cos^2 is off by "
            f"{error:.3f} at the slots that should hold a heading error. "
            "Re-read the offsets in DirectDriveObservation.cs."
        )
    for remaining, capacity in zip(RESOURCE_SLOT_REMAINING, RESOURCE_SLOT_CAPACITY):
        absent = obs[:, capacity] == 0.0
        if np.any(obs[absent, remaining] != 0.0):
            raise AssertionError(
                f"resource slot at {remaining} reads charge with zero capacity: "
                "absence must be written on the capacity"
            )
    if not np.all(obs[:, RESOURCE_SLOT_CAPACITY[0]] > 0.0):
        raise AssertionError(
            "the primary store reads zero capacity; the car has a battery"
        )
    if not np.all(np.isfinite(obs)):
        raise AssertionError("the observation has a non-finite channel")
    excess = np.max(np.abs(obs), axis=0) - OBSERVATION_BOUNDS
    if np.any(excess > 0.0):
        worst = int(np.argmax(excess))
        raise AssertionError(
            f"channel {worst} reads {np.max(np.abs(obs[:, worst])):.3g}, over its "
            f"bound {OBSERVATION_BOUNDS[worst]:.3g}"
        )


def evaluate(
    agent: SacAgent,
    batch: int,
    seed_base: int,
    solo: bool,
    track: str,
    seconds: float,
    modes: tuple[int, int] = EVALUATION_MODES,
    opponent: "FrozenOpponent | None" = None,
    delta_actions: bool = False,
) -> dict[str, float]:
    """What the policy did on this circuit, and whether it was allowed to.

    A lap is timed the way a lap is timed: by watching the car cross the
    line. The host reports each car's along-track race distance, which runs
    continuously through the start line and does not care whether the car
    is on the road, so a crossing is that distance passing a multiple of
    the lap length. The crossing moment is interpolated inside the step and
    the first lap is thrown away because it begins from a standstill.

    What is new here is that a lap now has to be legal to count. The
    previous version put every completed lap into one list regardless of
    what it cost, took the minimum over every lane, and reported that as
    the policy's lap time - so the headline number could be, and on this
    project was, a lap driven partly beside the road by whichever of two
    lanes had drawn the most permissive tyre mode. Under the fixed
    instruction used here, the early checkpoint's nineteen complete laps
    contained no clean one at all and the late checkpoint's sixteen
    contained one. A measurement that cannot tell those apart from a
    genuine flying lap cannot be used to declare anything graduated.

    So: a lap is clean when nothing was charged against it for leaving the
    road or touching a barrier, and the headline lap time is the best clean
    one. Beside it is the *charged* lap - the lap time with the seconds it
    spent beyond the line and against a barrier added back at a second per
    second - and that, not the clean lap, is what a checkpoint is ranked on.

    The reason is that a clean lap is not always available to rank. At the
    ten decisions a second the learned driver is held to, holding a line
    is hard enough that a whole evaluation can go by without one circuit
    producing a single clean lap from any driver at all. A ranking
    criterion that is undefined in that case selects nothing, and a
    project that cannot select a checkpoint cannot train. Charging the
    excursions back keeps the ranking defined wherever laps are completed,
    monotone in how far outside the lines the policy went, and impossible
    to win by cutting - a corner cut pays for itself and then some.

    Everything the definition throws away is reported beside it rather
    than silently dropped - how many laps were completed, how many were
    clean, what the dirty ones cost, whether the pit wall's instruction was
    obeyed, and what was left of the tyres and the store. The clean
    definition is this environment's penalty accounting, not a scrutineer.
    """
    lap_metres, _ = TRACKS[track]
    clean_laps: list[float] = []
    dirty_laps: list[float] = []
    charged_laps: list[float] = []
    off_per_lap: list[float] = []
    lanes_with_clean = 0
    # The duel columns, and only the minimum of them: who was in front when
    # each episode ended, by how much, and how often the cars touched. A
    # lap time is still a lap time with another car on the road, so
    # everything above this is measured exactly as it is solo.
    duel_wins = 0
    duel_episodes = 0
    duel_gaps: list[float] = []
    duel_contacts = 0
    with HostEnv(
        batch=batch,
        seed_base=seed_base,
        solo=solo and opponent is None,
        duel=opponent is not None,
        track=track,
        episode_seconds=seconds + 60.0,
        ego_modes=modes,
        # A policy trained on increments has to be measured on them; the
        # same numbers under the other contract would be a different
        # driver's.
        delta_actions=delta_actions,
    ) as env:
        # A budget in seconds, spent at whatever the rate is. Steps used
        # to be the budget, which meant every change of rate silently
        # changed how long an evaluation watched for.
        steps = int(round(seconds / STEP_SECONDS))
        obs = env.reset()
        assert_observation_layout(obs)
        off_course = np.zeros(batch, dtype=np.float64)
        wall = np.zeros(batch, dtype=np.float64)
        excess = np.zeros(batch, dtype=np.float64)
        # The same three, but only since this lane last crossed the line,
        # and in seconds rather than in penalty: a penalty is a rate times
        # the square of a speed times a duration, so dividing it by the
        # first two gives back the duration it was charged for. Seconds are
        # what a lap can be charged in.
        lap_off = np.zeros(batch, dtype=np.float64)
        lap_wall = np.zeros(batch, dtype=np.float64)
        lap_excess = np.zeros(batch, dtype=np.float64)
        # The race's ruler beside the trainer's: seconds this lap with all
        # four wheels over the white line. Graduation reads it; the reward
        # never sees it.
        lap_four_wheels = np.zeros(batch, dtype=np.float64)
        four_wheel_laps: list[float] = []
        # What the session spent, step by step, so that a lane re-seeding
        # onto a fresh pack and new tyres does not read as a refill. Only
        # decreases in charge and increases in wear count.
        charge_used = 0.0
        wear_used = 0.0
        metres_driven = 0.0
        last_charge = obs[:, PRIMARY_STORE].astype(np.float64)
        last_wear = np.mean(
            [obs[:, w] for w in WEAR_SLOTS], axis=0
        ).astype(np.float64)
        last_race: np.ndarray | None = None
        lane_clean = np.zeros(batch, dtype=np.int64)
        speed_squared = 0.0
        stalls = 0
        # Spins are counted for the whole session rather than charged to a
        # lap. They are not a lap's penalty accounting - a spin is the car
        # being taken off the driver - and graduation asks for both things
        # separately: a clean lap, and a session with none of these in it.
        spin_events = 0
        previous: list[float | None] = [None] * batch
        crossed: list[float | None] = [None] * batch
        prev_command = None
        command_dir = np.zeros(batch)
        straight_reversals = 0
        straight_samples = 0
        reversal_amplitudes: list[float] = []
        command_moves = 0.0
        command_samples = 0
        for step in range(steps):
            action = agent.act(obs, deterministic=True)
            if opponent is None:
                obs, reward, done, reason, components, race, final_obs, spins = (
                    env.step(action)
                )
            else:
                partner = opponent.act(env.opponent_obs)
                obs, reward, done, reason, components, race, final_obs, spins = (
                    env.step(action, partner)
                )
                duel_contacts += int(
                    np.count_nonzero(
                        components[COMPONENT_NAMES.index("contact")] < 0.0
                    )
                )
                # The lead the step ended on, which for a finished lane is
                # the result: the host reads it after the cars have moved
                # and before anything re-seeds, so it is the gap at the
                # flag rather than the fresh episode's starting gap.
                lead = env.lead_metres
                for lane in np.flatnonzero(done):
                    duel_episodes += 1
                    # Positive while the partner is ahead, so the ego's own
                    # advantage is its negation.
                    gap = -float(lead[lane])
                    duel_gaps.append(gap)
                    if gap > 0.0:
                        duel_wins += 1
            lap_four_wheels += env.four_wheels_off
            end_charge = final_obs[:, PRIMARY_STORE].astype(np.float64)
            end_wear = np.mean(
                [final_obs[:, w] for w in WEAR_SLOTS], axis=0
            ).astype(np.float64)
            # final_obs is the state each lane's step ended on, before any
            # re-seed, so a lane that finished still paid for this step;
            # the next step is measured from the fresh observation instead.
            charge_used += float(np.sum(np.maximum(last_charge - end_charge, 0.0)))
            wear_used += float(np.sum(np.maximum(end_wear - last_wear, 0.0)))
            race_now = np.asarray(race, dtype=np.float64)
            if last_race is not None:
                moved = race_now - last_race
                metres_driven += float(np.sum(moved[moved > 0.0]))
            last_race = race_now
            last_charge = obs[:, PRIMARY_STORE].astype(np.float64)
            last_wear = np.mean(
                [obs[:, w] for w in WEAR_SLOTS], axis=0
            ).astype(np.float64)
            now = (step + 1) * STEP_SECONDS
            spin_events += int(spins.sum())
            command = obs[:, EGO_COMMAND]
            if prev_command is None:
                prev_command = command.copy()
            else:
                delta = command - prev_command
                lateral = np.abs(obs[:, EGO_LATERAL_ACCEL]) * STEER_ACCEL_SCALE
                straight = lateral < STRAIGHT_LATERAL
                moved = np.abs(delta) > 1e-4
                now_dir = np.sign(delta)
                reversed_ = moved & (command_dir != 0) & (now_dir != command_dir)
                reversal_amplitudes.extend(
                    np.abs(delta[reversed_ & straight]).tolist()
                )
                straight_reversals += int(np.count_nonzero(reversed_ & straight))
                straight_samples += int(np.count_nonzero(straight))
                command_moves += float(np.abs(delta).mean())
                command_samples += 1
                command_dir = np.where(moved, now_dir, command_dir)
                command_dir = np.where(done, 0, command_dir)
                prev_command = np.where(done, command, command)
            step_off = components[COMPONENT_NAMES.index("off_course")]
            step_wall = components[COMPONENT_NAMES.index("wall")]
            step_excess = components[COMPONENT_NAMES.index("mode_excess")]
            off_course += step_off
            wall += step_wall
            excess += step_excess
            v_squared = np.maximum(
                (obs[:, EGO_SPEED] * SPEED_SCALE) ** 2, 1e-6
            )
            lap_off += -step_off / (OFF_COURSE_RATE * v_squared)
            lap_wall += -step_wall / (WALL_RATE * v_squared)
            lap_excess += -step_excess
            speed_squared += float(v_squared.mean())
            for lane in range(batch):
                # A lane that ended its episode restarts the clock for
                # itself alone. This used to blank one shared variable, so
                # any lane finishing made every other lane skip its own
                # line check for that step - a silently dropped lap in
                # every batch that ever saw a terminal.
                if done[lane]:
                    if TERMINAL_NAMES[reason[lane]] == "stalled":
                        stalls += 1
                    crossed[lane] = None
                    previous[lane] = None
                    lap_off[lane] = 0.0
                    lap_wall[lane] = 0.0
                    lap_excess[lane] = 0.0
                    lap_four_wheels[lane] = 0.0
                    continue
                before_distance = previous[lane]
                previous[lane] = float(race[lane])
                if before_distance is None or race[lane] <= before_distance:
                    continue
                before = math.floor(before_distance / lap_metres)
                after = math.floor(race[lane] / lap_metres)
                for line in range(before + 1, after + 1):
                    share = (line * lap_metres - before_distance) / (
                        race[lane] - before_distance
                    )
                    at = now - STEP_SECONDS + share * STEP_SECONDS
                    if crossed[lane] is not None:
                        lap = at - crossed[lane]
                        charged_laps.append(
                            lap + lap_off[lane] + lap_wall[lane]
                        )
                        off_per_lap.append(lap_off[lane])
                        four_wheel_laps.append(lap_four_wheels[lane])
                        if lap_off[lane] < 1e-6 and lap_wall[lane] < 1e-6:
                            clean_laps.append(lap)
                            if lane_clean[lane] == 0:
                                lanes_with_clean += 1
                            lane_clean[lane] += 1
                        else:
                            dirty_laps.append(lap)
                    crossed[lane] = at
                    lap_off[lane] = 0.0
                    lap_wall[lane] = 0.0
                    lap_excess[lane] = 0.0
                    lap_four_wheels[lane] = 0.0

    mean_speed_squared = speed_squared / steps
    off_seconds = (
        -off_course.mean() / (OFF_COURSE_RATE * mean_speed_squared)
        if mean_speed_squared > 1.0
        else 0.0
    )
    completed = len(clean_laps) + len(dirty_laps)
    best = min(clean_laps) if clean_laps else float("inf")
    charged = min(charged_laps) if charged_laps else float("inf")
    return {
        "lap": best,
        "charged_lap": charged,
        "off_per_lap": float(np.median(off_per_lap)) if off_per_lap else 0.0,
        # Every lap's charge, not just the middle one. The median says how
        # much a typical lap costs; the shape says what kind of driver this
        # is - a clean one with an occasional big mistake looks nothing like
        # one that clips every corner, and they need different fixing.
        "off_each_lap": list(off_per_lap),
        # Track limits by the race's ruler: a lap is clean on it when no
        # moment of it had all four wheels over the line.
        "four_wheels_off_each_lap": list(four_wheel_laps),
        # Consumption per lap, as a share of the pack and of a tyre's life
        # (the mean of four wheels). Accumulated per step, so re-seeded
        # lanes are not read as refills.
        "charge_per_lap": (
            charge_used / (metres_driven / lap_metres)
            if metres_driven > 0.0 else 0.0
        ),
        "wear_per_lap": (
            wear_used / (metres_driven / lap_metres)
            if metres_driven > 0.0 else 0.0
        ),
        "four_wheel_clean_laps": float(
            sum(1 for seconds in four_wheel_laps if seconds < 1e-6)
        ),
        "four_wheel_clean_share": (
            sum(1 for seconds in four_wheel_laps if seconds < 1e-6)
            / len(four_wheel_laps)
            if four_wheel_laps
            else 0.0
        ),
        "clean_lap_times": list(clean_laps),
        "laps": float(completed),
        "clean_laps": float(len(clean_laps)),
        "clean_share": len(clean_laps) / completed if completed else 0.0,
        "lanes_with_clean": float(lanes_with_clean),
        "lanes": float(batch),
        "best_dirty": min(dirty_laps) if dirty_laps else float("inf"),
        "off_seconds": off_seconds,
        "wall": float(wall.mean()),
        "mode_excess": float(excess.mean()),
        "stalls": float(stalls),
        "spins": float(spin_events),
        "spins_per_lap": spin_events / completed if completed else 0.0,
        # Same two numbers jitter_probe prints: reversals a second on the
        # straights, and the median wheel move a reversal turns around.
        "straight_reversals_per_s": (
            straight_reversals / max(straight_samples * STEP_SECONDS, 1e-6)
        ),
        "reversal_swing": (
            float(np.median(reversal_amplitudes)) if reversal_amplitudes else 0.0
        ),
        "mean_command_move": (
            command_moves / max(command_samples, 1)
        ),
        # The duel's own three, zero in a solo evaluation.
        "duel_episodes": float(duel_episodes),
        "duel_wins": float(duel_wins),
        "duel_win_share": duel_wins / duel_episodes if duel_episodes else 0.0,
        "duel_gap": float(np.mean(duel_gaps)) if duel_gaps else 0.0,
        "duel_contacts": float(duel_contacts),
        "tyre_wear": float(np.mean([obs[:, w] for w in WEAR_SLOTS])),
        "store": float(obs[:, PRIMARY_STORE].mean()),
        "modes": modes,
    }



def per_lap(events: float, laps: float) -> str:
    """An event count in the unit the event happens in.

    Six spins per twenty-five thousand steps is a number nobody can hold.
    One spin every seven laps is a driver you can picture, and it is also
    the unit graduation is written in -- zero spins in an evaluation
    session -- so the training log and the verdict stop being quoted in
    different currencies.
    """
    if laps < 0.5:
        return "圈数不足"
    if events <= 0:
        return f"≈0/{laps:.0f} 圈"
    per = laps / events
    return f"≈1/{per:.0f} 圈" if per >= 1.0 else f"≈{events / laps:.1f}/圈"


def progress_lines(
    laps: dict[str, dict[str, float]], names: list[str]
) -> tuple[float, float, float, float, float] | None:
    """Clean-lap mean, spins, off-course, straight reversal rate, swing.

    Lower is better on every line. Any one beating its record is progress.
    Charged (dirty) mean is not a line. None if a trained circuit
    completed no lap.
    """
    if not names or any(laps[n]["laps"] <= 0 for n in names):
        return None
    spins = sum(laps[n]["spins"] for n in names)
    off = sum(laps[n]["off_per_lap"] for n in names) / len(names)
    paces: list[float] = []
    rates: list[float] = []
    swings: list[float] = []
    for name in names:
        times = laps[name].get("clean_lap_times") or []
        paces.append(float(np.mean(times)) if times else math.inf)
        rates.append(float(laps[name].get("straight_reversals_per_s", math.inf)))
        swings.append(float(laps[name].get("reversal_swing", math.inf)))
    return (
        sum(paces) / len(paces),
        spins,
        off,
        sum(rates) / len(rates),
        sum(swings) / len(swings),
    )


def report(
    agent, args, seed_base: int, names: list[str], opponent=None
) -> dict[str, dict[str, float]]:
    """The requested circuits as flying lap times.

    With a sparring partner the same circuits are driven wheel-to-wheel:
    the lap columns mean what they always did, and the duel's own three
    (who led at the flag, by how much, how often they touched) come back
    beside them.
    """
    out: dict[str, dict[str, float]] = {}
    for name in names:
        out[name] = evaluate(
            agent, args.eval_batch, seed_base, args.solo, name,
            args.eval_seconds, opponent=opponent,
            delta_actions=args.delta_actions,
        )
    return out


def eval_split(track: str | None) -> tuple[list[str], list[str]]:
    """Which circuits an evaluation drives, and which of them count.

    Joint training reads the whole table. A specialist reads its own
    circuit plus two sentinels - simple-right as a health check and sepang
    as a transfer probe - because evaluating seventeen circuits every
    twenty-five thousand steps costs more wall clock than the training
    between evaluations, for fifteen answers nobody is asking about. Only
    the first list feeds the best-checkpoint choice, so a specialist is
    judged on its own circuit alone.
    """
    if track:
        sentinels = [
            n for n in ("simple-right", "sepang") if n != track
        ]
        return [track], sentinels
    trained = [n for n, (_, _, t) in TRACKS.items() if t]
    held = [n for n, (_, _, t) in TRACKS.items() if not t]
    return trained, held


def gap_string(seconds: float) -> str:
    return "     -- " if not math.isfinite(seconds) else f"{seconds:+7.2f}s"


def lap_string(seconds: float) -> str:
    if not math.isfinite(seconds):
        return "     --  "
    return f"{int(seconds // 60)}:{seconds % 60:06.3f}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=300_000)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--seed", type=int, default=0)
    # Pins the entropy coefficient instead of letting the tuner chase a
    # target entropy. The tuner is under suspicion: it settles at a value
    # that makes the entropy term a third of the objective, which is not
    # the same objective the evaluation scores.
    parser.add_argument("--fixed-alpha", type=float, default=None)
    parser.add_argument("--solo", action="store_true")
    # The delta-action pilot. The first action stops being the curvature to
    # hold and becomes how far to move it this decision; the host carries
    # the command between decisions and reports it back in the ego block,
    # so the policy can see the wheel it is holding. Nothing else moves --
    # not the physics, not the reward, not the curriculum, not the 457
    # channels -- and with the flag off the run is the old one to the bit.
    parser.add_argument(
        "--delta-actions", action="store_true",
        help="action[0] is an increment to the steering command, not the command",
    )
    # Wheel-to-wheel. The partner is a frozen checkpoint driving the second
    # car through the same interface, and only the ego learns: the reward,
    # the terminals and the replay buffer are the ego's alone.
    parser.add_argument(
        "--duel", action="store_true",
        help="put a frozen checkpoint in the other car",
    )
    parser.add_argument(
        "--opponent-checkpoint", default=None,
        help="who to spar against; required with --duel unless --opponent-pool",
    )
    parser.add_argument(
        "--opponent-pool", type=int, default=0,
        help="STUB: draw the partner from this tag's latest K eval checkpoints",
    )
    parser.add_argument("--track", default=None)
    parser.add_argument("--eval-every", type=int, default=25_000)
    # Four hundred seconds of watching, whatever the decision rate turns
    # that into in steps.
    # Six hundred, not the four hundred this was.
    #
    # Four hundred could not see the cliff it was supposed to be watching
    # for. Both certified arms fell apart late in a session -- every one of
    # fifty-seven spins above twenty-five per cent tyre wear -- and a
    # four-hundred-second evaluation stopped before the tyres got there, so
    # it reported zero spins for a driver that certification found spinning
    # every other lap. A best checkpoint chosen on a window that cannot see
    # the failure is chosen on a road it was never asked to drive.
    #
    # This makes every evaluation recorded before it incomparable, which is
    # the price and is worth paying once. Figures are quoted with their
    # session length from here on.
    parser.add_argument("--eval-seconds", type=float, default=600.0)
    parser.add_argument("--eval-batch", type=int, default=2)
    parser.add_argument("--episode-seconds", type=float, default=240.0)
    # On by default, because the alternative is what produced a policy that
    # had never met a worn tyre. The flag exists to turn it off for a
    # controlled comparison, not because off is a reasonable way to bake.
    parser.add_argument(
        "--fixed-episode-start",
        action="store_true",
        help="start every episode on fresh warm tyres and 80%% charge",
    )
    # On by default for the same reason: the freeze design's hidden
    # curriculum (per-episode limiter strength, perception noise) is part of
    # what a world-v3 bake is. Off only for a controlled comparison.
    parser.add_argument(
        "--no-hidden-curriculum",
        action="store_true",
        help="train with the limiter at full strength and clean perception",
    )
    # The two steering costs, per second, for a controlled comparison or a
    # recalibration. Left alone they are the host's own numbers.
    parser.add_argument("--steering-detour-cost", type=float, default=None)
    parser.add_argument("--steering-travel-cost", type=float, default=None)
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
    # The recipe, as flags. Every threshold here is relative to something
    # the run measured itself; none of them is a figure borrowed from a
    # paper written about another problem.
    parser.add_argument(
        "--alpha-rebound-ratio", type=float, default=2.0,
        help="freeze alpha once it climbs this many times above its own floor",
    )
    parser.add_argument(
        "--alpha-rebound-windows", type=int, default=3,
        help="consecutive log windows above that ratio before freezing",
    )
    parser.add_argument(
        "--alpha-freeze-cap", type=int, default=75_000,
        help="freeze alpha at its floor after this many steps regardless",
    )
    parser.add_argument(
        "--stop-after-stale", type=int, default=0,
        help="unused: legs stop at --steps, not on a quiet evaluation",
    )
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
    if args.fixed_alpha is not None:
        overrides["fixed_alpha"] = args.fixed_alpha
    if args.hidden:
        overrides["hidden"] = tuple(
            int(part) for part in args.hidden.split(",")
        )
    # The seed reached the environment and the evaluation but never the
    # learner: torch and numpy started wherever the interpreter left them,
    # so the network's initialisation, the exploration noise and every
    # minibatch draw were unrepeatable. Two runs of the same command were
    # never the same experiment, which is fatal to any comparison that
    # needs more than one seed to mean anything.
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    config = SacConfig(**overrides)
    print(
        f"device: {config.device} hidden={config.hidden} "
        f"quantiles={config.quantiles} layer_norm={config.critic_layer_norm} "
        f"updates/step={config.updates_per_step} "
        f"alpha={'auto' if config.fixed_alpha is None else config.fixed_alpha} "
        f"gamma={config.gamma} n_step={config.n_step} seed={args.seed}"
    )
    opponent_path = args.opponent_checkpoint
    if args.duel and opponent_path is None and args.opponent_pool > 0:
        drawn = draw_from_pool(checkpoint_dir, args.tag, args.opponent_pool)
        opponent_path = str(drawn) if drawn else None
    if args.duel and opponent_path is None:
        parser.error(
            "--duel needs --opponent-checkpoint (or a pool with checkpoints in it)"
        )
    if args.duel and args.solo:
        parser.error("--solo and --duel ask for different cars on the road")

    with HostEnv(
        batch=args.batch,
        seed_base=args.seed,
        solo=args.solo,
        duel=args.duel,
        track=args.track,
        # Four minutes, not the host's default sixty seconds. Sixty-second
        # episodes meant no tyre ever got more than a minute old in
        # training, and a four-hundred-second evaluation then drove the
        # policy through tyre states it had never once observed — measured
        # as a car crawling at eleven metres a second on half-worn tyres.
        # Four minutes is also most of a lap of the longest circuit here,
        # so a lap is something training actually contains.
        episode_seconds=args.episode_seconds,
        randomise_episode_start=not args.fixed_episode_start,
        hidden_curriculum=not args.no_hidden_curriculum,
        delta_actions=args.delta_actions,
        steering_detour_cost=args.steering_detour_cost,
        steering_travel_cost=args.steering_travel_cost,
        quiet=True,
    ) as env:
        print(
            f"env: obs={env.obs_size} action={env.action_size} "
            f"lanes={env.batch} solo={args.solo} "
            f"actions={'delta' if args.delta_actions else 'absolute'}"
        )
        agent = SacAgent(env.obs_size, env.action_size, config)
        sparring = None
        if args.duel:
            sparring = FrozenOpponent(
                env.obs_size, env.action_size, config, opponent_path
            )
            print(f"sparring against {sparring.describe()}")
        resumed_step = 0
        run_kind = "fresh"
        if args.resume:
            restored = agent.load(args.resume)
            # A resume that restores the weights and nothing else is a warm
            # start: the optimizer moments are empty, the random streams
            # begin again, and the step counter goes back to one, which puts
            # the run back inside its window of uniformly random actions.
            # Both are legitimate; conflating them is not, and the log has
            # to say which happened because nothing downstream can tell.
            full = restored["optimizers"] and restored["rng"]
            run_kind = "continued" if full else "warm-start"
            resumed_step = restored["step"] if full else 0
            print(
                f"{run_kind} from {args.resume} "
                f"(format {restored['format']}, step {restored['step']}, "
                f"optimizers {'yes' if restored['optimizers'] else 'no'}, "
                f"rng {'yes' if restored['rng'] else 'no'}, "
                f"replay no)"
            )


        # Travels with every checkpoint this run writes, so that whatever
        # loads one later -- the exporter, the viewer, a duel's frozen
        # partner -- reads the contract off the file instead of being told.
        action_semantics = "delta" if args.delta_actions else "absolute"

        obs = env.reset()
        batcher = NStepBatcher(env.batch, config.n_step, config.gamma)
        window_reward = 0.0
        window_components = np.zeros(len(COMPONENT_NAMES))
        window_terminals: dict[str, int] = {}
        # Spins per thousand steps is the curve this batch of physics was
        # built to be read against: it should start high on a policy that
        # learned to live in the old free corner, and go to zero.
        window_spins = 0
        window_metres = 0.0
        previous_race = None
        window_lap_metres, _unused_trained = TRACKS[args.track] \
            if args.track else (TRACKS["silverstone"][0], None)
        # Five records, each lower-is-better. A new best is any evaluation
        # that beats at least one of them. The leg itself stops at --steps.
        best_clean_pace = math.inf
        best_spins = math.inf
        best_off = math.inf
        best_reversal_rate = math.inf
        best_swing = math.inf
        # The alpha valley detector. The tuner is allowed to discover what
        # this problem's entropy is worth; when it starts climbing back out
        # of the floor it found, the floor is what gets kept. Every part of
        # the judgement is relative - a historical minimum and a multiple of
        # it - so nothing here is a constant borrowed from another problem.
        alpha_floor = math.inf
        alpha_rebound_windows = 0
        alpha_frozen = args.fixed_alpha is not None
        started = time.time()

        for step in range(resumed_step + 1, resumed_step + args.steps + 1):
            transitions = step * env.batch
            if transitions < config.start_steps:
                action = np.random.uniform(
                    -1.0, 1.0, size=(env.batch, env.action_size)
                ).astype(np.float32)
            else:
                action = agent.act(obs)

            if sparring is None:
                next_obs, reward, done, reason, components, race, final_obs, spins = (
                    env.step(action)
                )
            else:
                # The partner answers the frame from its own seat, frozen
                # and deterministic, before the ego's action is sent: one
                # decision each, both held over the same interval.
                partner_action = sparring.act(env.opponent_obs)
                next_obs, reward, done, reason, components, race, final_obs, spins = (
                    env.step(action, partner_action)
                )
            # Distance covered this step, per lane, so the window's events
            # can be quoted per lap. Lanes that just re-seeded are skipped
            # rather than differenced: their race distance restarts, and a
            # negative delta is a new episode rather than a car reversing.
            if previous_race is not None:
                advanced = race - previous_race
                window_metres += float(
                    advanced[(advanced > 0.0) & ~done].sum()
                )
            previous_race = race.copy()
            # An episode ending and the future being worth nothing are two
            # different facts. Stalling is a real ending; a timeout is the
            # clock running out on a race that was still going, so the
            # learner bootstraps across it — from `final_obs`, the frame the
            # clock stopped at, because `next_obs` on a finished lane is
            # already the fresh episode's first frame.
            terminal = done & (reason != TIMEOUT_REASON)
            ready = batcher.add(
                obs, action, reward, final_obs,
                done.astype(np.float32),
                terminal.astype(np.float32),
            )
            if ready is not None:
                agent.buffer.add_batch(*ready)
            obs = next_obs

            window_reward += float(reward.mean())
            window_spins += int(spins.sum())
            window_components += components.mean(axis=1)
            for lane in np.flatnonzero(done):
                name = TERMINAL_NAMES[reason[lane]]
                window_terminals[name] = window_terminals.get(name, 0) + 1

            # Gated on what is in the buffer, not on how many steps have
            # been counted. They are the same number for a run that starts
            # from nothing and wildly different for one that resumes: the
            # step counter carries over, so a warm start would find this
            # gate already open and begin two updates a step against a
            # buffer holding a few hundred nearly identical transitions.
            # That does not continue a good checkpoint, it grinds it up.
            if agent.buffer.size >= config.start_steps:
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
                    # Exploration health, which the steering costs are in a
                    # position to suppress: the policy's entropy and the
                    # spread it still has on the wheel. A leg that stops
                    # weaving because it stopped exploring has not learned
                    # anything, and these two are how that is told apart from
                    # a leg that learned to hold a line.
                    f"H {stats.get('entropy', float('nan')):+.2f} "
                    f"σ {stats.get('sigma_steer', float('nan')):.3f} "
                    # The critic's own fit, which is the first thing to look
                    # at before blaming capacity: a network too small to
                    # represent its target shows up here as a loss that
                    # stops falling while performance stops improving.
                    f"closs {stats.get('critic_loss', float('nan')):.3f} "
                    # This run's transitions over this run's clock. The
                    # step counter carries the whole lineage after a
                    # resume, and dividing that by the time since this
                    # process started reported a continuation at a
                    # million transitions a second.
                    f"{(step - resumed_step) * env.batch / max(elapsed, 1e-6):.0f}"
                    f" tps"
                )
                window_laps = window_metres / window_lap_metres
                stalls = window_terminals.get("stalled", 0)
                print(
                    f"          spins {window_spins} "
                    f"({window_spins / args.log_every:.3f}/step, "
                    f"{per_lap(window_spins, window_laps)})"
                    f"  退赛 {stalls} ({per_lap(stalls, window_laps)})"
                    f"  圈 {window_laps:.0f}"
                )
                print(f"          {pieces}")
                print(f"          terminals {window_terminals or 'none'}")

                observed = stats.get("alpha")
                if not alpha_frozen and observed is not None and observed > 0:
                    alpha_floor = min(alpha_floor, float(observed))
                    if observed > alpha_floor * args.alpha_rebound_ratio:
                        alpha_rebound_windows += 1
                    else:
                        alpha_rebound_windows = 0
                    forced = step - resumed_step >= args.alpha_freeze_cap
                    if (
                        alpha_rebound_windows >= args.alpha_rebound_windows
                        or forced
                    ):
                        agent.freeze_alpha(alpha_floor)
                        alpha_frozen = True
                        print(
                            f"          alpha frozen at {alpha_floor:.4f} "
                            + (
                                "(step cap)"
                                if forced
                                else f"(rebounded above "
                                     f"{args.alpha_rebound_ratio:g}x for "
                                     f"{alpha_rebound_windows} windows)"
                            )
                        )
                window_reward = 0.0
                window_components[:] = 0.0
                window_terminals = {}
                window_spins = 0
                window_metres = 0.0

            if step % args.eval_every == 0:
                trained, held = eval_split(args.track)
                laps = report(
                    agent, args, args.seed + 900_000, trained + held,
                    opponent=sparring,
                )
                tyre, power = EVALUATION_MODES
                print(
                    f"  eval at step {step}    干净飞驰圈"
                    f"  (档位 轮胎{tyre}/动力{power}, {args.eval_batch} lane)"
                )
                groups = (
                    ("专家", trained), ("哨兵", held)
                ) if args.track else (("训练", trained), ("保留", held))
                for group, names in groups:
                    for name in names:
                        r = laps[name]
                        flags = ""
                        if r["laps"] < 1.0:
                            flags += "  未完成一圈"
                        if r["wall"] < -1.0:
                            flags += f"  撞墙 {r['wall']:.0f}"
                        if r["mode_excess"] < -0.02:
                            flags += f"  抗命 {r['mode_excess']:.2f}"
                        if r["stalls"] > 0:
                            flags += (
                                f"  退赛 {r['stalls']:.0f} "
                                f"({per_lap(r['stalls'], r['laps'])})"
                            )
                        if r["spins"] > 0:
                            flags += (
                                f"  旋转 {r['spins']:.0f} "
                                f"({per_lap(r['spins'], r['laps'])})"
                            )
                        print(
                            f"    {group} {name:<15}"
                            f"  干净 {lap_string(r['lap']):>9}"
                            f"  计罚 {lap_string(r['charged_lap']):>9}"
                            f"  {r['clean_laps']:.0f}/{r['laps']:.0f} 干净"
                            f"  出界 {r['off_per_lap']:.1f}s/圈"
                            f"  胎耗 {r['tyre_wear'] * 100:.0f}%"
                            f"  余量 {r['store'] * 100:.0f}%"
                            f"{band_note(name, r['lap'])}{flags}"
                        )
                        if r["duel_episodes"] > 0:
                            print(
                                f"      对局 {r['duel_wins']:.0f}/"
                                f"{r['duel_episodes']:.0f} 胜"
                                f"  差距 {r['duel_gap']:+.1f}m"
                                f"  接触 {r['duel_contacts']:.0f} 步"
                            )
                # The mean charged lap over the circuits that count is what
                # a best checkpoint is chosen on: one number, in seconds a
                # lap, and lower is better. Absolute now rather than a gap
                # to a script - the ranking is unchanged, since subtracting
                # a per-circuit constant never reordered anything, but the
                # number now means what it says.
                def mean_lap_of(names):
                    # A circuit the policy cannot lap counts as four minutes
                    # against it, not as silence. Excluding those let a
                    # checkpoint set a best mean in the same evaluation
                    # where a circuit stopped completing laps at all.
                    return sum(
                        min(laps[n]["charged_lap"], 240.0)
                        if math.isfinite(laps[n]["charged_lap"]) else 240.0
                        for n in names
                    ) / len(names)
                mean_lap = mean_lap_of(trained)
                held_lap = mean_lap_of(held)
                left, right = ("专家", "哨兵") if args.track else ("训练", "保留")
                lines = progress_lines(laps, trained)
                spins_total = sum(laps[n]["spins"] for n in trained)
                completed = sum(laps[n]["laps"] for n in trained)
                clean_total = sum(laps[n]["clean_laps"] for n in trained)
                print(
                    f"    计罚平均圈  {left} {lap_string(mean_lap)}"
                    f"   {right} {lap_string(held_lap)}"
                )
                stalls_total = sum(laps[n]["stalls"] for n in trained)
                if lines is not None:
                    clean_pace, _, off_mean, reversal_rate, reversal_swing = lines
                else:
                    clean_pace = off_mean = reversal_rate = reversal_swing = math.inf
                print(
                    f"    干净口径    旋转 {spins_total:.0f} "
                    f"({per_lap(spins_total, completed)})"
                    f"  退赛 {stalls_total:.0f} "
                    f"({per_lap(stalls_total, completed)})"
                    f"  干净 {clean_total:.0f}/{completed:.0f}"
                    f"  出界 {off_mean:.1f}s/圈"
                    f"  干净均速 {lap_string(clean_pace)}"
                )
                print(
                    f"    手          直道翻转 {reversal_rate:.2f}/秒"
                    f"  摆幅 {reversal_swing:.5f} (门 0.015)"
                )
                agent.save(
                    str(checkpoint_dir / f"latest{args.tag}.pt"),
                    step,
                    action_semantics=action_semantics,
                )
                # And one that nothing overwrites. A checkpoint is twenty
                # eight megabytes and an evaluation is twenty minutes of
                # compute, so keeping every one of them is free and losing
                # one is not: "best" tracks a single ranking criterion, and
                # the run whose graduation asked for a clean lap, zero spins
                # and a lap time in the band spent three evaluations
                # watching the checkpoint that had all three get overwritten
                # by one that was a hundredth of a second quicker on the
                # criterion that only counts two of them.
                agent.save(
                    str(checkpoint_dir / f"eval{args.tag}-{step}.pt"),
                    step,
                    action_semantics=action_semantics,
                )
                records: list[str] = []
                if lines is not None:
                    clean_pace, spins_line, off_line, reversal_rate, reversal_swing = (
                        lines
                    )
                    if clean_pace < best_clean_pace:
                        records.append("干净圈")
                        best_clean_pace = clean_pace
                    if spins_line < best_spins:
                        records.append("旋转")
                        best_spins = spins_line
                    if off_line < best_off:
                        records.append("出界")
                        best_off = off_line
                    if reversal_rate < best_reversal_rate:
                        records.append("翻转")
                        best_reversal_rate = reversal_rate
                    if reversal_swing < best_swing:
                        records.append("摆幅")
                        best_swing = reversal_swing
                if records:
                    agent.save(
                        str(checkpoint_dir / f"best{args.tag}.pt"),
                        step,
                        action_semantics=action_semantics,
                    )
                    print(
                        f"    saved best (旋转 {spins_total:.0f}, 出界 "
                        f"{off_mean:.1f}s/圈, 干净均速 {lap_string(clean_pace)}, "
                        f"翻转 {reversal_rate:.2f}/秒, 摆幅 {reversal_swing:.5f}"
                        f")  新纪录: {'、'.join(records)}"
                    )

    print("training finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
