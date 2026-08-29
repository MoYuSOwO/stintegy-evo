"""Which (lanes, updates-per-step) buys the most learning per wall minute.

Throughput is not the thing to maximise. A run that produces thirty
transitions for every gradient step is throwing twenty-nine of them away,
and the environment being fast only means it throws them away faster. What
matters is how much the policy improves per minute of wall clock, so that
is what this measures: every configuration gets the same minutes and is
judged on where the reward ended up.
"""
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
STEP = re.compile(r"step\s+(\d+).*reward\s+(-?\d+\.\d+).*?(\d+)\s+tps")

CONFIGS = [
    (64, 2),    # what the baseline run used
    (32, 8),
    (16, 16),
    (32, 16),
    (16, 32),
]


def run(lanes: int, updates: int, seconds: float) -> dict:
    log = HERE / f"sweep-{lanes}x{updates}.log"
    with log.open("w") as sink:
        process = subprocess.Popen(
            [
                sys.executable, "-u", "train.py", "--solo",
                "--batch", str(lanes),
                "--updates-per-step", str(updates),
                "--steps", "1000000",
                "--log-every", "200",
                "--eval-every", "100000000",
                "--seed", "1",
                "--tag", f"sweep{lanes}x{updates}",
            ],
            cwd=HERE, stdout=sink, stderr=subprocess.STDOUT,
        )
        deadline = time.time() + seconds
        while time.time() < deadline and process.poll() is None:
            time.sleep(2)
        process.terminate()
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            process.kill()

    rows = STEP.findall(log.read_text())
    if not rows:
        return {"lanes": lanes, "updates": updates, "rows": 0}
    steps = int(rows[-1][0])
    tail = [float(r[1]) for r in rows[-3:]]
    return {
        "lanes": lanes,
        "updates": updates,
        "rows": len(rows),
        "steps": steps,
        "transitions": steps * lanes,
        "gradient_updates": steps * updates,
        "reward": sum(tail) / len(tail),
        "tps": int(rows[-1][2]),
    }


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    print(f"each configuration gets {seconds:.0f} s\n")
    print(f"{'lanes':>6} {'upd/step':>9} {'steps':>8} {'transitions':>12} "
          f"{'updates':>9} {'tps':>7} {'reward':>9}")
    results = []
    for lanes, updates in CONFIGS:
        r = run(lanes, updates, seconds)
        results.append(r)
        if not r["rows"]:
            print(f"{lanes:>6} {updates:>9}   no output")
            continue
        print(f"{lanes:>6} {updates:>9} {r['steps']:>8} "
              f"{r['transitions']:>12} {r['gradient_updates']:>9} "
              f"{r['tps']:>7} {r['reward']:>9.4f}")

    best = max((r for r in results if r.get("rows")),
               key=lambda r: r["reward"], default=None)
    if best:
        print(f"\nfurthest along after {seconds:.0f} s: "
              f"{best['lanes']} lanes x {best['updates']} updates, "
              f"reward {best['reward']:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
