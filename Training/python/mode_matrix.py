"""The whole instruction grid: every tyre rung against every power rung.

A player's command space is all twenty-five cells, so certification reads
all twenty-five. Each cell is a nominal evaluation at that instruction, and
four maps come out of it -- clean lap, energy per lap, tyre wear per lap,
and disobedience -- plus a check of the one property the strategy game is
built on: that each rung is a real lever. Holding one axis still, a higher
rung should be quicker and should cost more of what it spends. A row or a
column that runs the other way means the instruction has stopped meaning
what it says, and is reported rather than smoothed over.

Heatmaps are written as plain SVG so that nothing beyond the training
environment is needed to draw them.

    python3 mode_matrix.py checkpoints/evalparent2h-1225000.pt --out matrix/
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from host_env import HostEnv
from sac import SacAgent, SacConfig
from train import evaluate

RUNGS = (1, 2, 3, 4, 5)
TYRE_NAMES = ("Protect", "Light", "Normal", "Push", "Attack")
POWER_NAMES = ("Save", "Eco", "Normal", "Push", "Attack")
# The pack the car races with. Charge per lap is reported by evaluate() as
# a share of the pack; megajoules are what a strategist reads.
PACK_MEGAJOULES = 1100.0


def clock(seconds: float) -> str:
    if not math.isfinite(seconds):
        return "--"
    return f"{int(seconds // 60)}:{seconds % 60:06.3f}"


def run_grid(policy: str, track: str, lanes: int, seconds: float, seed: int) -> list[dict]:
    with HostEnv(batch=2, seed_base=1, solo=True, track=track, episode_seconds=60) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
    agent.load(policy)

    cells = []
    for tyre in RUNGS:
        for power in RUNGS:
            r = evaluate(agent, lanes, seed, True, track, seconds, modes=(tyre, power))
            cleans = r["clean_lap_times"]
            cell = {
                "tyre": tyre,
                "power": power,
                "laps": r["laps"],
                "fastest_clean": r["lap"],
                "mean_clean": sum(cleans) / len(cleans) if cleans else float("inf"),
                "clean_share": r["clean_share"],
                "spins": r["spins"],
                "retirements": r["stalls"],
                "energy_mj_per_lap": r["charge_per_lap"] * PACK_MEGAJOULES,
                "wear_percent_per_lap": r["wear_per_lap"] * 100.0,
                "disobedience": r["mode_excess"],
                "four_wheel_clean_share": r["four_wheel_clean_share"],
            }
            cells.append(cell)
            print(
                f"  {TYRE_NAMES[tyre - 1]:>7}/{POWER_NAMES[power - 1]:<7} "
                f"laps {cell['laps']:3.0f}  clean {cell['clean_share'] * 100:5.1f}%  "
                f"fastest {clock(cell['fastest_clean'])}  mean {clock(cell['mean_clean'])}  "
                f"{cell['energy_mj_per_lap']:5.1f} MJ/lap  "
                f"{cell['wear_percent_per_lap']:4.2f} %wear/lap  "
                f"disobedience {cell['disobedience']:+.3f}  spins {cell['spins']:.0f}",
                flush=True,
            )
    return cells


def cell_at(cells: list[dict], tyre: int, power: int) -> dict:
    return next(c for c in cells if c["tyre"] == tyre and c["power"] == power)


def monotonicity(cells: list[dict]) -> list[str]:
    """Rows and columns where a higher rung does not do what it says.

    Along the tyre axis a higher rung should be no slower and wear no less;
    along the power axis a higher rung should be no slower and spend no less
    energy. Pace is read on the mean clean lap, which a single lucky lap
    cannot move. A small tolerance absorbs sampling noise without hiding a
    reversal of any size worth reading.
    """
    pace_tolerance = 0.05      # seconds
    spend_tolerance = 0.02     # relative
    breaks = []
    for power in RUNGS:
        for tyre in RUNGS[:-1]:
            lower, upper = cell_at(cells, tyre, power), cell_at(cells, tyre + 1, power)
            if upper["mean_clean"] > lower["mean_clean"] + pace_tolerance:
                breaks.append(
                    f"tyre {tyre}->{tyre + 1} at power {power}: slower "
                    f"({clock(lower['mean_clean'])} -> {clock(upper['mean_clean'])})"
                )
            if upper["wear_percent_per_lap"] < lower["wear_percent_per_lap"] * (1 - spend_tolerance):
                breaks.append(
                    f"tyre {tyre}->{tyre + 1} at power {power}: wears less "
                    f"({lower['wear_percent_per_lap']:.3f} -> {upper['wear_percent_per_lap']:.3f} %/lap)"
                )
    for tyre in RUNGS:
        for power in RUNGS[:-1]:
            lower, upper = cell_at(cells, tyre, power), cell_at(cells, tyre, power + 1)
            if upper["mean_clean"] > lower["mean_clean"] + pace_tolerance:
                breaks.append(
                    f"power {power}->{power + 1} at tyre {tyre}: slower "
                    f"({clock(lower['mean_clean'])} -> {clock(upper['mean_clean'])})"
                )
            if upper["energy_mj_per_lap"] < lower["energy_mj_per_lap"] * (1 - spend_tolerance):
                breaks.append(
                    f"power {power}->{power + 1} at tyre {tyre}: spends less "
                    f"({lower['energy_mj_per_lap']:.1f} -> {upper['energy_mj_per_lap']:.1f} MJ/lap)"
                )
    return breaks


def heatmap_svg(cells: list[dict], key: str, title: str, fmt, lower_is_better: bool) -> str:
    """A five-by-five grid, tyre rungs down and power rungs across."""
    size, pad_left, pad_top = 88, 96, 64
    width, height = pad_left + size * 5 + 16, pad_top + size * 5 + 40
    values = [c[key] for c in cells if math.isfinite(c[key])]
    low, high = (min(values), max(values)) if values else (0.0, 1.0)
    span = (high - low) or 1.0

    def colour(value: float) -> str:
        if not math.isfinite(value):
            return "#cccccc"
        t = (value - low) / span
        if lower_is_better:
            t = 1.0 - t
        # Pale to deep blue: a sequential ramp that stays legible in print.
        r = int(236 - 190 * t)
        g = int(242 - 130 * t)
        b = int(250 - 60 * t)
        return f"rgb({r},{g},{b})"

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'font-family="Helvetica, Arial, sans-serif" font-size="12">',
        f'<rect width="{width}" height="{height}" fill="white"/>',
        f'<text x="{pad_left}" y="24" font-size="15" font-weight="bold">{title}</text>',
        f'<text x="{pad_left}" y="44" fill="#555">power rung across, tyre rung down</text>',
    ]
    for i, name in enumerate(POWER_NAMES):
        parts.append(
            f'<text x="{pad_left + size * i + size / 2}" y="{pad_top - 6}" '
            f'text-anchor="middle" fill="#333">{i + 1} {name}</text>'
        )
    for j, name in enumerate(TYRE_NAMES):
        parts.append(
            f'<text x="{pad_left - 8}" y="{pad_top + size * j + size / 2 + 4}" '
            f'text-anchor="end" fill="#333">{j + 1} {name}</text>'
        )
    for c in cells:
        x = pad_left + size * (c["power"] - 1)
        y = pad_top + size * (c["tyre"] - 1)
        value = c[key]
        fill = colour(value)
        text_colour = "white" if fill != "#cccccc" and (
            ((value - low) / span) if not lower_is_better else 1 - (value - low) / span
        ) > 0.6 else "#111"
        parts.append(
            f'<rect x="{x}" y="{y}" width="{size - 2}" height="{size - 2}" fill="{fill}"/>'
        )
        parts.append(
            f'<text x="{x + size / 2}" y="{y + size / 2 + 4}" text-anchor="middle" '
            f'fill="{text_colour}">{fmt(value)}</text>'
        )
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("policy")
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--lanes", type=int, default=6)
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--seed-base", type=int, default=900_001)
    parser.add_argument("--out", default="mode-matrix")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    print(
        f"档位矩阵 · {args.track} · {args.lanes} 车道 × {args.seconds:.0f} 模拟秒 · "
        f"种子基 {args.seed_base} · 25 格",
        flush=True,
    )
    cells = run_grid(args.policy, args.track, args.lanes, args.seconds, args.seed_base)
    breaks = monotonicity(cells)

    maps = (
        ("mean_clean", "Mean clean lap", clock, True),
        ("energy_mj_per_lap", "Energy per lap (MJ)", lambda v: f"{v:.1f}", True),
        ("wear_percent_per_lap", "Tyre wear per lap (%)", lambda v: f"{v:.2f}", True),
        ("disobedience", "Disobedience (mode_excess)", lambda v: f"{v:+.3f}", False),
    )
    for key, title, fmt, lower_is_better in maps:
        (out / f"{key}.svg").write_text(
            heatmap_svg(cells, key, title, fmt, lower_is_better), encoding="utf-8"
        )

    attack = cell_at(cells, 5, 5)
    protect = cell_at(cells, 1, 1)
    spread = protect["mean_clean"] - attack["mean_clean"]
    summary = {
        "policy": args.policy,
        "track": args.track,
        "lanes": args.lanes,
        "seconds": args.seconds,
        "seed_base": args.seed_base,
        "cells": cells,
        "monotonicity_breaks": breaks,
        "strategy_spread_seconds": spread,
        "qualifying_pace": attack["fastest_clean"],
    }
    (out / "matrix.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")

    print(f"\nProtect/Save → Attack/Attack 干净均速差：{spread:+.3f} 秒（策略空间宽度）")
    print(f"5/5 最快干净圈（排位配速读数）：{clock(attack['fastest_clean'])}")
    if breaks:
        print(f"单调性破坏 {len(breaks)} 处（软门槛，上报裁决）：")
        for line in breaks:
            print(f"  {line}")
    else:
        print("单调性：25 格逐行逐列均成立")
    print(f"\nwrote {out}/matrix.json and four heatmaps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
