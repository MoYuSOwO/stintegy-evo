"""The parent chain's gate, as a script rather than a pair of human eyes.

A leg ends at a wide stop. The registered criterion (see
Training/experiments/2026-09-20-parent3f/manifest.md) says the chain carries
on if the leg broke any of three lines, and holds -- reporting first -- if it
broke none. Reading the log to decide that is mechanical, and while it waited
on a session waking up, the furnace once stood cold for four and a half hours
(2026-09-20-parent3i/manifest.md). So the watchdog decides with this, and a
human reads the reasoning afterwards.

    python chain_gate.py run-parent3i.log run-parent3*.log

The first argument is the leg being judged; the rest are every leg of the
chain, whose evaluations before this one set the records to beat. Prints a
verdict line (BREAK or FLAT) and its reasons, and exits 0 for BREAK, 1 for
FLAT, 2 if the log cannot be read as a finished leg.
"""

import re
import sys

# The zero-spin line's threshold is the chain best it has to beat, fixed when
# the criterion was registered: parent3c's 1:49.725.
ZERO_SPIN_THRESHOLD = 109.725

EVAL = re.compile(r"eval at step (\d+)")
CHARGED = re.compile(r"计罚平均圈\s+专家\s+(\S+)")
CLEAN = re.compile(r"干净口径\s+旋转 (\d+).*?干净 (\d+)/(\d+).*?均速\s+(\S+)")


def seconds(lap: str) -> float | None:
    """1:42.899 -> 102.899, and '--' (no clean lap) -> None."""
    if ":" not in lap:
        return None
    minutes, rest = lap.split(":", 1)
    return int(minutes) * 60 + float(rest)


def evaluations(path: str) -> list[dict]:
    """Every evaluation in a log, as (step, charged, spins, clean rate, mean)."""
    found: list[dict] = []
    current: dict = {}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            if step := EVAL.search(line):
                current = {"step": int(step.group(1))}
                continue
            if not current:
                continue
            if charged := CHARGED.search(line):
                current["charged"] = seconds(charged.group(1))
                continue
            if clean := CLEAN.search(line):
                laps = int(clean.group(3))
                current["spins"] = int(clean.group(1))
                current["clean"] = int(clean.group(2)) / laps if laps else 0.0
                current["clean_text"] = f"{clean.group(2)}/{clean.group(3)}"
                current["mean"] = seconds(clean.group(4))
                found.append(current)
                current = {}
    return found


def records(logs: list[str], upto_step: int) -> tuple[float, float]:
    """The best charged pace and clean rate anywhere in the chain before this leg."""
    pace = float("inf")
    clean = 0.0
    for log in logs:
        for evaluation in evaluations(log):
            if evaluation["step"] >= upto_step:
                continue
            if (charged := evaluation.get("charged")) is not None:
                pace = min(pace, charged)
            clean = max(clean, evaluation["clean"])
    return pace, clean


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("usage: chain_gate.py <leg log> [<every chain log> ...]")
        return 2
    leg, chain = argv[1], argv[2:] or [argv[1]]

    finished = False
    with open(leg, encoding="utf-8") as handle:
        for line in handle:
            if "training stopped at step" in line or "training finished" in line:
                finished = True
    evals = evaluations(leg)
    if not evals:
        print(f"FLAT  {leg}: no evaluations in the log")
        return 2
    if not finished:
        print(f"FLAT  {leg}: the leg has not finished")
        return 2

    first = min(evaluation["step"] for evaluation in evals)
    pace_record, clean_record = records(chain, first)

    reasons: list[str] = []
    best_pace = min(
        (e for e in evals if e.get("charged") is not None),
        key=lambda e: e["charged"],
        default=None,
    )
    if best_pace and best_pace["charged"] < pace_record:
        reasons.append(
            f"pace {best_pace['charged']:.3f}s at {best_pace['step']} "
            f"beats {pace_record:.3f}s"
        )
    best_clean = max(evals, key=lambda e: e["clean"])
    if best_clean["clean"] > clean_record:
        reasons.append(
            f"clean {best_clean['clean_text']} at {best_clean['step']} "
            f"beats {clean_record:.3f}"
        )
    for evaluation in evals:
        mean = evaluation.get("mean")
        if evaluation["spins"] == 0 and mean is not None and mean < ZERO_SPIN_THRESHOLD:
            reasons.append(
                f"spin-free {mean:.3f}s at {evaluation['step']} "
                f"beats {ZERO_SPIN_THRESHOLD:.3f}s"
            )
            break

    verdict = "BREAK" if reasons else "FLAT"
    print(f"{verdict}  {leg}  ({len(evals)} evaluations)")
    print(f"  records to beat: pace {pace_record:.3f}s, clean {clean_record:.3f}")
    for reason in reasons:
        print(f"  broke: {reason}")
    if not reasons:
        print(
            f"  best: pace {best_pace['charged']:.3f}s, "
            f"clean {best_clean['clean_text']}, "
            f"fewest spins {min(e['spins'] for e in evals)}"
        )
    return 0 if reasons else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
