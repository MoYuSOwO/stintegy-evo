"""What a decision rate costs, circuit by circuit.

Ten decisions a second was inherited from the Gran Turismo paper and has
never been checked here. It matters more than a hyperparameter usually
does, because the command this driver sends is a curvature - a target the
car tracks through a yaw response of about a hundred and fifty
milliseconds - so a hundred-millisecond command period sits inside the
plant's own time constant. The analytic driver, held to that rate, runs
several seconds a lap outside the white lines on circuits it laps
perfectly cleanly at its own.

This drives the analytic stack round every circuit at a ladder of rates
and reports what each one buys: the best clean lap, the best charged lap,
how many seconds a lap were spent outside the lines, and how many laps
were clean at all. The analytic driver is the instrument rather than the
subject - it is the same driver at every rate, so whatever changes is the
rate.

Two rates are set independently and both are swept together here. The
agent step is how long a control is held for; the analytic driver's own
decision period is how often it corrects. They are set to the same value
at every rung, which is what makes the ladder a ladder: at each rung
everything on the circuit decides that often and no oftener.

    python3 frequency_sweep.py                    # every circuit
    python3 frequency_sweep.py silverstone monza  # just these

Writes frequency_sweep.json beside itself. Prints as it goes, because a
scan that only speaks at the end cannot be told from a scan that died.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from host_env import COMPONENT_NAMES, HostEnv
from train import (
    EGO_SPEED,
    EVALUATION_MODES,
    OFF_COURSE_RATE,
    SPEED_SCALE,
    TRACKS,
    WALL_RATE,
)

# The rungs the ladder was first climbed on. A run can ask for others:
#   python3 frequency_sweep.py --rates 15,60
RATES = (10.0, 15.0, 20.0, 30.0, 60.0)
SEED_BASE = 900_001
LANES = 2
# Simulated seconds per lane, not steps: a step is a different amount of
# time at every rung, and the point is to give each rung the same race.
SIM_SECONDS = 600.0


def measure(
    track: str,
    lap_metres: float,
    hz: float,
    modes: tuple[int, int],
) -> dict[str, float]:
    """One circuit at one rate, through the evaluation's own lap timer."""
    step_seconds = 1.0 / hz
    steps = int(round(SIM_SECONDS * hz))
    clean: list[float] = []
    dirty: list[float] = []
    charged: list[float] = []
    off_each: list[float] = []
    spin_events = 0

    started = time.perf_counter()
    with HostEnv(
        batch=LANES,
        seed_base=SEED_BASE,
        solo=True,
        track=track,
        episode_seconds=SIM_SECONDS + 60.0,
        ego_modes=modes,
        ego_analytic=True,
        analytic_hz=hz,
        decision_hz=hz,
    ) as env:
        obs = env.reset()
        # The analytic driver is at the wheel; these go nowhere and exist
        # only because the protocol expects an action every step.
        idle = np.zeros((LANES, env.action_size), dtype=np.float32)
        off = np.zeros(LANES, dtype=np.float64)
        wall = np.zeros(LANES, dtype=np.float64)
        crossed: list[float | None] = [None] * LANES
        previous: list[float | None] = [None] * LANES
        for step in range(steps):
            obs, _, done, _, components, race, _, spins = env.step(idle)
            spin_events += int(spins.sum())
            now = (step + 1) * step_seconds
            # A penalty is a rate times a squared speed times a duration,
            # so dividing by the first two gives the duration back.
            v_squared = np.maximum(
                (obs[:, EGO_SPEED] * SPEED_SCALE) ** 2, 1e-6
            )
            off += -components[COMPONENT_NAMES.index("off_course")] / (
                OFF_COURSE_RATE * v_squared
            )
            wall += -components[COMPONENT_NAMES.index("wall")] / (
                WALL_RATE * v_squared
            )
            for lane in range(LANES):
                if done[lane]:
                    crossed[lane] = None
                    previous[lane] = None
                    off[lane] = 0.0
                    wall[lane] = 0.0
                    continue
                before_distance = previous[lane]
                previous[lane] = float(race[lane])
                if before_distance is None or race[lane] <= before_distance:
                    continue
                first = math.floor(before_distance / lap_metres)
                last = math.floor(race[lane] / lap_metres)
                for line in range(first + 1, last + 1):
                    share = (line * lap_metres - before_distance) / (
                        race[lane] - before_distance
                    )
                    at = now - step_seconds + share * step_seconds
                    if crossed[lane] is not None:
                        lap = at - crossed[lane]
                        charged.append(lap + off[lane] + wall[lane])
                        off_each.append(off[lane])
                        if off[lane] < 1e-6 and wall[lane] < 1e-6:
                            clean.append(lap)
                        else:
                            dirty.append(lap)
                    crossed[lane] = at
                    off[lane] = 0.0
                    wall[lane] = 0.0

    return {
        "hz": hz,
        "clean_lap": min(clean) if clean else float("inf"),
        "charged_lap": min(charged) if charged else float("inf"),
        "fastest_lap": min(clean + dirty) if (clean or dirty) else float("inf"),
        "off_per_lap": float(np.median(off_each)) if off_each else 0.0,
        # The analytic driver is the calibration of the limit zone: it has
        # never gone past five degrees of sideslip, so any spin here is the
        # verdict line set too close to the road rather than a driver who
        # deserved one.
        "spins": spin_events,
        "clean_laps": len(clean),
        "laps": len(clean) + len(dirty),
        "sim_seconds": SIM_SECONDS,
        "lanes": LANES,
        "wall_seconds": round(time.perf_counter() - started, 2),
    }


def clock(value: float) -> str:
    return (
        f"{int(value // 60)}:{value % 60:06.3f}"
        if math.isfinite(value) else "    --   "
    )


def main() -> int:
    global RATES
    argv = sys.argv[1:]
    if "--rates" in argv:
        at = argv.index("--rates")
        RATES = tuple(float(r) for r in argv[at + 1].split(","))
        del argv[at:at + 2]
    wanted = argv or list(TRACKS)
    unknown = [name for name in wanted if name not in TRACKS]
    if unknown:
        print(f"unknown circuit(s): {', '.join(unknown)}", flush=True)
        return 2

    tyre, power = EVALUATION_MODES
    print(
        f"frequency sweep · analytic driver · tyre {tyre} / power {power} · "
        f"{LANES} lanes × {SIM_SECONDS:.0f} simulated seconds per rung",
        flush=True,
    )
    print(
        f"rates: {', '.join(f'{r:.0f}' for r in RATES)} Hz · "
        f"seed base {SEED_BASE} · clean = no off-course and no wall charge",
        flush=True,
    )

    out: dict[str, dict[str, dict[str, float]]] = {}
    for name in wanted:
        lap_metres, _ = TRACKS[name]
        out[name] = {}
        print(f"\n{name}", flush=True)
        print(
            f"    {'Hz':>4}  {'clean':>9}  {'charged':>9}  {'fastest':>9}"
            f"  {'off s/lap':>9}  {'clean/laps':>10}  {'spins':>5}"
            f"  {'wall s':>7}",
            flush=True,
        )
        for hz in RATES:
            result = measure(name, lap_metres, hz, EVALUATION_MODES)
            out[name][f"{hz:.0f}"] = result
            print(
                f"    {hz:4.0f}  {clock(result['clean_lap']):>9}"
                f"  {clock(result['charged_lap']):>9}"
                f"  {clock(result['fastest_lap']):>9}"
                f"  {result['off_per_lap']:9.2f}"
                f"  {result['clean_laps']:4d}/{result['laps']:<5d}"
                f"  {result['spins']:5d}"
                f"  {result['wall_seconds']:7.1f}",
                flush=True,
            )
            destination = Path(__file__).with_name("frequency_sweep.json")
            destination.write_text(
                json.dumps(
                    {
                        "modes": list(EVALUATION_MODES),
                        "seed_base": SEED_BASE,
                        "lanes": LANES,
                        "sim_seconds": SIM_SECONDS,
                        "rates_hz": list(RATES),
                        "driver": "analytic (ReferenceLineDriver)",
                        "definition": (
                            "clean = a lap charged nothing for leaving the "
                            "road or touching a barrier; charged = lap time "
                            "plus those seconds added back"
                        ),
                        "note": (
                            "the agent step and the analytic driver's own "
                            "decision period are set to the same rate at "
                            "every rung, so a rung is one rate for "
                            "everything on the circuit"
                        ),
                        "tracks": out,
                    },
                    indent=2,
                )
                + "\n"
            )
    print("\nwritten to frequency_sweep.json", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
