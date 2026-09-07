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
import math
import sys

import numpy as np

from host_env import HostEnv
from sac import SacAgent, SacConfig
from train import EVALUATION_MODES, band_note, evaluate, meets_reference_band

LANES = 12
SECONDS = 600.0
SEED_BASE = 900_001


def clock(value: float) -> str:
    return (
        f"{int(value // 60)}:{value % 60:06.3f}"
        if math.isfinite(value) else "    --   "
    )


def certify(path: str, track: str) -> dict:
    with HostEnv(
        batch=2, seed_base=1, solo=True, track=track, episode_seconds=60
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig())
    agent.load(path)
    return evaluate(agent, LANES, SEED_BASE, True, track, SECONDS)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoints", nargs="+")
    parser.add_argument("--track", default="silverstone")
    args = parser.parse_args()

    print(
        f"大样本认证 · {args.track} · {LANES} 车道 × {SECONDS:.0f} 模拟秒 · "
        f"档位 {EVALUATION_MODES[0]}/{EVALUATION_MODES[1]} · 种子基 {SEED_BASE}",
        flush=True,
    )
    for path in args.checkpoints:
        r = certify(path, args.track)
        laps = r["laps"]
        clean = r["clean_laps"]
        share = clean / laps if laps else 0.0
        off = np.asarray(r["off_each_lap"]) if r["off_each_lap"] else np.zeros(0)
        cleans = np.asarray(r["clean_lap_times"]) if r["clean_lap_times"] else np.zeros(0)

        print(f"\n{path}")
        print(
            f"  圈数 {laps:.0f}  干净 {clean:.0f}  干净率 {share * 100:.1f}%  "
            f"旋转 {r['spins']:.0f}  退赛 {r['stalls']:.0f}"
        )
        print(
            f"  最快干净圈 {clock(r['lap'])}"
            f"{band_note(args.track, r['lap'])}"
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
        graduated = (
            r["spins"] == 0
            and share > 0.5
            and meets_reference_band(args.track, r["lap"])
        )
        print(
            "  毕业口径：" + (
                "✅ 三项齐（零旋转 + 干净率过半 + 落带）"
                if graduated
                else "❌ " + " / ".join(
                    x for x in (
                        f"旋转 {r['spins']:.0f}" if r["spins"] else None,
                        f"干净率 {share * 100:.0f}%" if share <= 0.5 else None,
                        "未落带" if not meets_reference_band(args.track, r["lap"]) else None,
                    ) if x
                )
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
