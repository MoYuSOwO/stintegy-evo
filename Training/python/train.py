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
# Speed is the first slot of the ego block: geometry 198, tyres 17, mode 1,
# aero 3, road and limits 13.
EGO_SPEED = 232
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


def evaluate(
    agent: SacAgent,
    batch: int,
    seed_base: int,
    solo: bool,
    track: str,
    seconds: float,
    modes: tuple[int, int] = EVALUATION_MODES,
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
    with HostEnv(
        batch=batch,
        seed_base=seed_base,
        solo=solo,
        track=track,
        episode_seconds=seconds + 60.0,
        ego_modes=modes,
    ) as env:
        # A budget in seconds, spent at whatever the rate is. Steps used
        # to be the budget, which meant every change of rate silently
        # changed how long an evaluation watched for.
        steps = int(round(seconds / STEP_SECONDS))
        obs = env.reset()
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
        for step in range(steps):
            action = agent.act(obs, deterministic=True)
            obs, reward, done, reason, components, race, _, spins = env.step(action)
            now = (step + 1) * STEP_SECONDS
            spin_events += int(spins.sum())
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


def clean_criterion_key(
    laps: dict[str, dict[str, float]], names: list[str]
) -> tuple[int, int, float]:
    """How good a checkpoint is under the criterion graduation actually
    uses, as a tuple that sorts larger-is-better.

    Lexicographic, and the order is the argument. A checkpoint that never
    loses the car beats one that is quicker and does; among those, one that
    laps cleanly most of the time beats one that manages it occasionally,
    which beats one that never does; and only inside a tier does pace
    decide. The charged lap - the ranking this project has used since
    clean laps stopped being reliably available - survives as the tiebreak
    inside the tier where there is no clean lap to compare, which is the
    one place it was ever the only defined answer.

    The alternative is what the slip-angle batch's own gate 2 did: rank on
    charged lap alone, and watch a checkpoint with a clean lap, no spins
    and a time inside the band get overwritten by one two hundredths of a
    second quicker that had none of those things.
    """
    spins = sum(laps[n]["spins"] for n in names)
    completed = sum(laps[n]["laps"] for n in names)
    clean = sum(laps[n]["clean_laps"] for n in names)
    share = clean / completed if completed else 0.0
    tier = 2 if share > 0.5 else (1 if clean > 0 else 0)

    def circuit_pace(name: str) -> float:
        times = laps[name].get("clean_lap_times") or []
        if tier > 0 and times:
            return float(np.mean(times))
        charged = laps[name]["charged_lap"]
        return charged if math.isfinite(charged) else 240.0

    pace = sum(circuit_pace(n) for n in names) / len(names)
    return (1 if spins == 0 else 0, tier, -pace)


def report(
    agent, args, seed_base: int, names: list[str]
) -> dict[str, dict[str, float]]:
    """The requested circuits as flying lap times."""
    out: dict[str, dict[str, float]] = {}
    for name in names:
        out[name] = evaluate(
            agent, args.eval_batch, seed_base, args.solo, name,
            args.eval_seconds,
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
        "--stop-after-stale", type=int, default=3,
        help="stop once this many evaluations improve neither line",
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
    with HostEnv(
        batch=args.batch,
        seed_base=args.seed,
        solo=args.solo,
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
        quiet=True,
    ) as env:
        print(
            f"env: obs={env.obs_size} action={env.action_size} "
            f"lanes={env.batch} solo={args.solo}"
        )
        agent = SacAgent(env.obs_size, env.action_size, config)
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
        # C3's key, and the charged mean beside it. Both are tracked
        # because the stopping rule is deliberately the looser of the two:
        # a run is only stagnant when neither the criterion that decides
        # graduation nor the one that decides pace has moved.
        best_key: tuple[int, int, float] | None = None
        best_charged = math.inf
        stale_evaluations = 0
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

            next_obs, reward, done, reason, components, race, final_obs, spins = (
                env.step(action)
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
                    # The critic's own fit, which is the first thing to look
                    # at before blaming capacity: a network too small to
                    # represent its target shows up here as a loss that
                    # stops falling while performance stops improving.
                    f"closs {stats.get('critic_loss', float('nan')):.3f} "
                    f"{step * env.batch / max(elapsed, 1e-6):.0f} tps"
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
                    agent, args, args.seed + 900_000, trained + held
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
                key = clean_criterion_key(laps, trained)
                spins_total = sum(laps[n]["spins"] for n in trained)
                completed = sum(laps[n]["laps"] for n in trained)
                clean_total = sum(laps[n]["clean_laps"] for n in trained)
                print(
                    f"    计罚平均圈  {left} {lap_string(mean_lap)}"
                    f"   {right} {lap_string(held_lap)}"
                )
                stalls_total = sum(laps[n]["stalls"] for n in trained)
                print(
                    f"    干净口径    旋转 {spins_total:.0f} "
                    f"({per_lap(spins_total, completed)})"
                    f"  退赛 {stalls_total:.0f} "
                    f"({per_lap(stalls_total, completed)})"
                    f"  干净 {clean_total:.0f}/{completed:.0f}"
                    f"  档位 {('无', '有', '过半')[key[1]]}"
                    f"  均速 {lap_string(-key[2])}"
                )
                agent.save(
                    str(checkpoint_dir / f"latest{args.tag}.pt"), step
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
                    str(checkpoint_dir / f"eval{args.tag}-{step}.pt"), step
                )
                # A checkpoint that completes nothing is not a best
                # checkpoint, however flattering its mean happens to be.
                laps_everywhere = all(laps[n]["laps"] > 0 for n in trained)
                improved_criterion = (
                    laps_everywhere and (best_key is None or key > best_key)
                )
                improved_pace = (
                    laps_everywhere
                    and math.isfinite(mean_lap)
                    and mean_lap < best_charged
                )
                if improved_criterion:
                    best_key = key
                    agent.save(
                        str(checkpoint_dir / f"best{args.tag}.pt"), step
                    )
                    print(
                        f"    saved best (旋转 {spins_total:.0f}, 干净档 "
                        f"{('无', '有', '过半')[key[1]]}, 均速 "
                        f"{lap_string(-key[2])})"
                    )
                if improved_pace:
                    best_charged = mean_lap

                # Stagnant only when neither line has moved. The strict
                # criterion can sit still for a long time while the car is
                # still finding pace, and pace can plateau while the car is
                # still learning to keep it clean; stopping on either alone
                # throws away the half of the run that was still working.
                if improved_criterion or improved_pace:
                    stale_evaluations = 0
                else:
                    stale_evaluations += 1
                    print(
                        f"    无进步 {stale_evaluations}/"
                        f"{args.stop_after_stale} 评"
                    )
                    if stale_evaluations >= args.stop_after_stale:
                        print(
                            f"training stopped at step {step}: neither the "
                            f"clean criterion nor the charged mean improved "
                            f"for {stale_evaluations} evaluations"
                        )
                        break

    print("training finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
