"""What the analytic driver does, measured the way the learner is measured.

Every gap this project has quoted was against a constant in a table. Where
those constants came from is no longer recoverable: not the tyre mode, not
the car, not the decision rate, and certainly not a requirement that the
lap be legal. Comparing a learned policy's best clean lap against them is
comparing two different measurements and calling the difference progress.

So this drives the analytic stack round the same circuits through the same
host, the same ten decisions a second, the same pit-wall instruction and
the same timing loop as `train.py` uses for a policy - and reports the same
clean flying lap. The output is meant to replace the numbers in TRACKS.

    python3 analytic_baseline.py                 # every circuit
    python3 analytic_baseline.py silverstone     # just this one
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

from host_env import COMPONENT_NAMES, TERMINAL_NAMES, HostEnv
from train import (
    EGO_SPEED, EVALUATION_MODES, OFF_COURSE_RATE, SPEED_SCALE,
    STEP_SECONDS, TRACKS, WALL_RATE,
)


def measure(
    track: str,
    lap_metres: float,
    batch: int,
    seed_base: int,
    steps: int,
    modes: tuple[int, int],
    analytic_hz: float,
) -> dict[str, float]:
    """One circuit. Same lap timer and same clean test as an evaluation."""
    clean: list[float] = []
    dirty: list[float] = []
    charged: list[float] = []
    off_each: list[float] = []
    with HostEnv(
        batch=batch,
        seed_base=seed_base,
        solo=True,
        track=track,
        episode_seconds=steps * STEP_SECONDS + 60.0,
        ego_modes=modes,
        ego_analytic=True,
        analytic_hz=analytic_hz,
    ) as env:
        obs = env.reset()
        # The analytic driver is at the wheel; these go nowhere and exist
        # only because the protocol expects an action every step.
        idle = np.zeros((batch, env.action_size), dtype=np.float32)
        off = np.zeros(batch, dtype=np.float64)
        wall = np.zeros(batch, dtype=np.float64)
        crossed: list[float | None] = [None] * batch
        previous: list[float | None] = [None] * batch
        for step in range(steps):
            obs, _, done, reason, components, race, _ = env.step(idle)
            now = (step + 1) * STEP_SECONDS
            v_squared = np.maximum(
                (obs[:, EGO_SPEED] * SPEED_SCALE) ** 2, 1e-6
            )
            off += -components[COMPONENT_NAMES.index("off_course")] / (
                OFF_COURSE_RATE * v_squared
            )
            wall += -components[COMPONENT_NAMES.index("wall")] / (
                WALL_RATE * v_squared
            )
            for lane in range(batch):
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
                    at = now - STEP_SECONDS + share * STEP_SECONDS
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
        "clean_lap": min(clean) if clean else float("inf"),
        "charged_lap": min(charged) if charged else float("inf"),
        "fastest_lap": min(clean + dirty) if (clean or dirty) else float("inf"),
        "off_per_lap": float(np.median(off_each)) if off_each else 0.0,
        "clean_laps": len(clean),
        "dirty_laps": len(dirty),
    }


def main() -> int:
    wanted = sys.argv[1:] or list(TRACKS)
    unknown = [name for name in wanted if name not in TRACKS]
    if unknown:
        print(f"unknown circuit(s): {', '.join(unknown)}")
        return 2

    tyre, power = EVALUATION_MODES
    print(
        f"analytic baseline · tyre {tyre} / power {power} · "
        f"clean flying lap, two decision rates"
    )
    print(
        "  native is the analytic stack as it is designed and shipped; "
        "matched is it held to the learner's ten hertz."
    )
    out: dict[str, dict[str, dict[str, float]]] = {}
    for name in wanted:
        lap_metres, table_value, _ = TRACKS[name]
        # Long enough for several laps of the slowest circuit here.
        native = measure(
            name, lap_metres, 2, 900_001, 6_000, EVALUATION_MODES, 60.0
        )
        matched = measure(
            name, lap_metres, 2, 900_001, 6_000, EVALUATION_MODES, 10.0
        )
        out[name] = {"native_60hz": native, "matched_10hz": matched}

        def clock(value: float) -> str:
            return (
                f"{int(value // 60)}:{value % 60:06.3f}"
                if math.isfinite(value) else "     --  "
            )

        print(
            f"  {name:<16}"
            f"原生 {clock(native['clean_lap']):>9}"
            f" ({native['clean_laps']}/"
            f"{native['clean_laps'] + native['dirty_laps']} 干净)"
            f"   10Hz 计罚 {clock(matched['charged_lap']):>9}"
            f"  出界 {matched['off_per_lap']:5.2f}s/圈"
            f"   表中 {table_value:8.3f}"
        )

    destination = Path(__file__).with_name("analytic_baseline.json")
    destination.write_text(
        json.dumps(
            {
                "modes": list(EVALUATION_MODES),
                "definition": "best lap with no off-course and no wall charge",
                "note": (
                    "native_60hz is the analytic driver at its own decision "
                    "rate, which is what the constants in TRACKS were taken "
                    "at and which laps every circuit clean. matched_10hz is "
                    "the same driver held to the learned driver's rate, "
                    "which is the like-for-like reference."
                ),
                "tracks": out,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"written to {destination.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
