"""Large-sample certification of a checkpoint.

The per-evaluation numbers a training run prints are read off fifteen or
eighteen laps, and the quantities that decide graduation - how often a lap
comes out clean, how often the car is lost - are rates. A rate from
eighteen samples swings from nothing to most of them on luck alone, which
is exactly what this campaign's own logs show. This runs one much larger
sample per checkpoint and reports the shape as well as the middle.

    python3 certify.py checkpoints/slip-eval-500000.pt [more.pt ...]
"""

from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np

from host_env import HostEnv
from sac import SacAgent, SacConfig
from train import (
    EVALUATION_MODES,
    REFERENCE_BANDS,
    band_note,
    evaluate,
    meets_reference_band,
    per_lap,
)

LANES = 12
# Six hundred seconds is part of the criterion, not a convenience.
#
# Both certified arms fell apart late in a session: every one of
# fifty-seven spins across two drivers who shared no ancestry happened
# above twenty-five per cent tyre wear, and a shorter session stops before
# the tyres get there. A verdict taken on a window that ends before the
# failure is a verdict about the window.
SECONDS = 600.0
SEED_BASE = 900_001


def clock(value: float) -> str:
    return (
        f"{int(value // 60)}:{value % 60:06.3f}"
        if math.isfinite(value) else "    --   "
    )


def certify(path: str, track: str, seed_base: int = SEED_BASE) -> dict:
    with HostEnv(
        batch=2, seed_base=1, solo=True, track=track, episode_seconds=60
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
    agent.load(path)
    return evaluate(agent, LANES, seed_base, True, track, SECONDS)


# The race steward's ruler asks for 99% of laps with no moment of all four
# wheels over the line. It is only meaningful over a hundred laps or more,
# where one infraction is exactly the line; on a single 57-lap session one
# infraction is already 98.2%, so a session reports the rate and the pooled
# seeds decide.
FOUR_WHEEL_CLEAN_REQUIRED = 0.99
FOUR_WHEEL_MINIMUM_LAPS = 100


def band_leg(track: str, lap: float) -> tuple[bool, str]:
    """Whether the lap clears the pace floor, and how to say so.

    A circuit with no reference band has no floor to clear. Reporting it as
    having missed one would fail every generalist session for a line it was
    never given, so it is marked not applicable instead.
    """
    if track not in REFERENCE_BANDS:
        return True, "无参考带（该项不适用）"
    if meets_reference_band(track, lap):
        return True, "落带"
    return False, "未落带"


def report(path: str, track: str, seed_base: int, r: dict) -> None:
    """One session, printed the way certification has always printed it."""
    laps = r["laps"]
    clean = r["clean_laps"]
    share = clean / laps if laps else 0.0
    off = np.asarray(r["off_each_lap"]) if r["off_each_lap"] else np.zeros(0)
    cleans = np.asarray(r["clean_lap_times"]) if r["clean_lap_times"] else np.zeros(0)

    print(f"\n{path}  种子基 {seed_base}")
    print(
        f"  圈数 {laps:.0f}  干净 {clean:.0f}  干净率 {share * 100:.1f}%  "
        f"旋转 {r['spins']:.0f} ({per_lap(r['spins'], laps)})  "
        f"退赛 {r['stalls']:.0f} ({per_lap(r['stalls'], laps)})"
    )
    print(
        f"  最快干净圈 {clock(r['lap'])}"
        f"{band_note(track, r['lap'])}"
        f"   干净圈均速 {clock(float(cleans.mean())) if cleans.size else '    --   '}"
        f"   计罚圈 {clock(r['charged_lap'])}"
    )
    if off.size:
        shape = (
            f"  出界秒数/圈：0 秒 {float((off < 1e-6).mean()) * 100:.0f}%"
            f"  中位 {np.median(off):.3f}"
            f"  p90 {np.percentile(off, 90):.3f}"
            f"  最大 {off.max():.3f}"
            f"  总计 {off.sum():.1f}s"
        )
        print(shape)
        # Shape, stated rather than left to be inferred: a driver who
        # clips every corner and one who is clean but occasionally
        # throws it away have the same median and need different work.
        dirty = off[off > 1e-6]
        if dirty.size:
            concentrated = dirty.max() > 0.5 * off.sum()
            print(
                "  形态：" + (
                    "偶发大错（单圈占全部出界过半）"
                    if concentrated
                    else "圈圈小蹭（出界分散在多圈）"
                )
            )
    four = r.get("four_wheels_off_each_lap", [])
    if four:
        four_clean = r["four_wheel_clean_share"]
        print(
            f"  四轮尺：干净率 {four_clean * 100:.1f}%"
            f"（{r['four_wheel_clean_laps']:.0f}/{len(four)} 圈，"
            f"四轮全出合计 {float(np.sum(four)):.2f}s）"
        )
    # Retirement joins the criterion as its own leg. A car stuck on
    # the grass is not a slow lap, it is no lap, and averaging it into
    # a clean-lap rate quietly forgives it.
    band_ok, band_label = band_leg(track, r["lap"])
    graduated = (
        r["spins"] == 0
        and r["stalls"] == 0
        and share > 0.5
        and band_ok
    )
    print(
        "  本场口径：" + (
            f"✅ 四项齐（零旋转 + 零退赛 + 干净率过半 + {band_label}）"
            f" · {SECONDS:.0f} 秒会话；四轮尺待合并圈数判定"
            if graduated
            else "❌ " + " / ".join(
                x for x in (
                    f"旋转 {r['spins']:.0f}" if r["spins"] else None,
                    f"退赛 {r['stalls']:.0f}" if r["stalls"] else None,
                    f"干净率 {share * 100:.0f}%" if share <= 0.5 else None,
                    band_label if not band_ok else None,
                ) if x
            )
        )
    )


def pooled(path: str, track: str, sessions: list[dict]) -> None:
    """All of one checkpoint's seeds together, judged on every leg.

    This is where the graduation verdict lives: the four-wheel ruler is
    stated over a hundred laps, and the rates beside it are steadier for
    being read on the same sample.
    """
    laps = sum(r["laps"] for r in sessions)
    clean = sum(r["clean_laps"] for r in sessions)
    spins = sum(r["spins"] for r in sessions)
    stalls = sum(r["stalls"] for r in sessions)
    four = [s for r in sessions for s in r.get("four_wheels_off_each_lap", [])]
    four_clean = (
        sum(1 for s in four if s < 1e-6) / len(four) if four else 0.0
    )
    best = min(r["lap"] for r in sessions)
    share = clean / laps if laps else 0.0
    band_ok, band_label = band_leg(track, best)
    enough = len(four) >= FOUR_WHEEL_MINIMUM_LAPS
    legs = [
        (spins == 0, f"旋转 {spins:.0f}"),
        (stalls == 0, f"退赛 {stalls:.0f}"),
        (share > 0.5, f"干净率 {share * 100:.1f}%"),
        (band_ok, band_label),
        (
            enough and four_clean >= FOUR_WHEEL_CLEAN_REQUIRED,
            f"四轮尺 {four_clean * 100:.1f}%"
            + ("" if enough else f"（仅 {len(four)} 圈，不足 {FOUR_WHEEL_MINIMUM_LAPS}）"),
        ),
    ]
    print(
        f"\n{path}  合并 {len(sessions)} 种子 · {laps:.0f} 圈"
        f" · 旋转 {per_lap(spins, laps)} · 最快干净圈 {clock(best)}"
    )
    print(
        "  毕业口径：" + (
            "✅ 五项齐（零旋转 + 零退赛 + 干净率过半 + "
            f"{band_label} + 四轮尺 ≥{FOUR_WHEEL_CLEAN_REQUIRED:.0%}）"
            if all(ok for ok, _ in legs)
            else "❌ " + " / ".join(label for ok, label in legs if not ok)
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoints", nargs="+")
    parser.add_argument("--track", default="silverstone")
    # A rate read off one seed is a rate read off one draw of the weather
    # and the starting tyres. Certification reports a number; how many
    # sessions it came from belongs beside it.
    parser.add_argument(
        "--seed-base",
        type=int,
        nargs="+",
        default=[SEED_BASE],
        help="one session per seed, each reported on its own",
    )
    parser.add_argument(
        "--json",
        default=None,
        help="write every session's raw result here, for ranking later",
    )
    args = parser.parse_args()

    print(
        f"大样本认证 · {args.track} · {LANES} 车道 × {SECONDS:.0f} 模拟秒 · "
        f"档位 {EVALUATION_MODES[0]}/{EVALUATION_MODES[1]} · 种子基 "
        f"{'、'.join(str(seed) for seed in args.seed_base)}",
        flush=True,
    )
    records: list[dict] = []
    for path in args.checkpoints:
        sessions: list[dict] = []
        for seed_base in args.seed_base:
            r = certify(path, args.track, seed_base)
            report(path, args.track, seed_base, r)
            sessions.append(r)
            records.append(
                {
                    "checkpoint": path,
                    "seed_base": seed_base,
                    "track": args.track,
                    "lanes": LANES,
                    "seconds": SECONDS,
                    **r,
                }
            )
        pooled(path, args.track, sessions)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(records, handle, ensure_ascii=False, indent=1)
        print(f"\nwrote {args.json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
