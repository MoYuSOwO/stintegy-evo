"""The comparison table the two-arm bake exists to produce.

Arm A relays from the incumbent expert; arm B bakes one from scratch on the
same physics. The question is whether starting again is worth what it
costs, and the answer is a table rather than an opinion: lap time, clean
rate, spins, the shape of the off-course seconds, where the entropy
coefficient settled, and how many steps it took from nothing.

    python3 d_table.py
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def clock(seconds: float) -> str:
    return (
        f"{int(seconds // 60)}:{seconds % 60:06.3f}"
        if math.isfinite(seconds) else "   --    "
    )


def read_arm(log: Path) -> dict:
    if not log.exists():
        return {"missing": True}
    text = log.read_text()
    steps = [int(m) for m in re.findall(r"^step +(\d+)", text, re.M)]
    resumed = re.search(r"format 2, step (\d+)", text)
    start = int(resumed.group(1)) if resumed else 0
    frozen = re.search(r"alpha frozen at ([0-9.]+) \((.*?)\)", text)
    # Two lines per evaluation. The per-circuit line carries the lap
    # times, the clean count and the off-course seconds; the criterion
    # line carries the spin and retirement totals in the form the verdict
    # is written in. Reading the totals off the criterion line rather than
    # off the per-circuit flags is what makes this survive the day the
    # flags gained their per-lap parentheses -- which it did not, the
    # first time, and quietly reported three spins as none.
    blocks = re.split(r"eval at step (\d+)", text)[1:]
    rows = []
    for step, body in zip(blocks[0::2], blocks[1::2]):
        circuit = re.search(
            r"专家 \S+ +干净 +(\S+) +计罚 +(\S+) +(\d+)/(\d+) 干净"
            r" +出界 ([0-9.]+)s/圈",
            body,
        )
        if circuit is None:
            continue
        criterion = re.search(
            r"干净口径 +旋转 (\d+)(?: \([^)]*\))?"
            r"(?: +退赛 (\d+)(?: \([^)]*\))?)?",
            body,
        )
        clean, charged, cl, laps, off = circuit.groups()
        rows.append(
            {
                "step": int(step),
                "clean": clean,
                "charged": charged,
                "clean_laps": int(cl),
                "laps": int(laps),
                "off": float(off),
                "spins": int(criterion.group(1)) if criterion else 0,
                # Retirement only joined the criterion line partway
                # through; an older log simply does not say.
                "stalls": (
                    int(criterion.group(2))
                    if criterion and criterion.group(2) is not None
                    else None
                ),
            }
        )
    return {
        "missing": False,
        "start": start,
        "last": steps[-1] if steps else start,
        "own_steps": (steps[-1] if steps else start) - start,
        "alpha_floor": float(frozen.group(1)) if frozen else None,
        "alpha_reason": frozen.group(2) if frozen else None,
        "evals": rows,
        "stopped": "training stopped" in text,
        "finished": "training finished" in text,
    }


def main() -> int:
    arms = {
        "臂A 接力": read_arm(HERE / "run-armA-relay.log"),
        "臂B 从零": read_arm(HERE / "run-armB-scratch.log"),
    }
    print("# D 对照表 · 双臂各一炉\n")
    for name, arm in arms.items():
        print(f"## {name}")
        if arm["missing"]:
            print("  （尚未起跑）\n")
            continue
        state = (
            "停训判据触发" if arm["stopped"]
            else "跑满步数上限" if arm["finished"]
            else "进行中"
        )
        print(f"  本炉步数 {arm['own_steps']}（{arm['start']} → {arm['last']}）  {state}")
        if arm["alpha_floor"] is not None:
            print(
                f"  α 谷底 {arm['alpha_floor']:.4f}  触发路径 {arm['alpha_reason']}"
            )
        else:
            print("  α 固定（接力口径）或尚未冻结")
        if arm["evals"]:
            print(
                "\n  |    步 |   干净圈 |   计罚圈 |      干净率 | 出界 s/圈 "
                "| 旋转 | 退赛 |"
            )
            print(
                "  |------:|----------|----------|-------------|-----------"
                "|------|------|"
            )
            for r in arm["evals"]:
                share = r["clean_laps"] / r["laps"] * 100 if r["laps"] else 0.0
                stalls = "  —" if r["stalls"] is None else f"{r['stalls']:>3}"
                print(
                    f"  | {r['step'] // 1000:>4}k | {r['clean']:>8} | "
                    f"{r['charged']:>8} | {r['clean_laps']:>2}/{r['laps']:<2} "
                    f"({share:>3.0f}%) | {r['off']:>9.1f} | {r['spins']:>4} "
                    f"| {stalls} |"
                )
        print()
    print(
        "大样本认证（12 车道 × 600 秒）另跑 certify.py，出界分布形态与最终\n"
        "毕业判定以那份为准——15~18 圈的率不足以分辨 5% 和 47%。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
