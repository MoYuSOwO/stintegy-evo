"""The same sessions, read with the stale ego offset, for comparison.

EGO_SPEED pointed at 232 for the whole 480-dimension era: a resource
slot's presence flag, which is 1.0 whenever the slot is filled, so every
reader saw a car at a steady 100 m/s. Off-course and wall penalties are
converted from penalty units into seconds by dividing by that speed
squared, so both were understated -- and the charged lap, which is what
selection and the stopping rule rank on, carries them.

This reruns the ranking's top names with the old constant restored, on
the same seed, so the two accounts can be put side by side. It patches
the module attribute rather than the file: the layout self-check reads
its own constants and still passes, which is the point -- the guard
catches a wrong offset, and this is deliberately asking for one.

    python3 old_account.py checkpoints/a.pt [more.pt ...]
"""
import json
import sys

sys.path.insert(0, "/Users/jayhuang/Code/stintegy-evo/.worktrees/spin-accounting/Training/python")

import numpy as np
import train

STALE_EGO_SPEED = 232


def main() -> int:
    from certify import LANES, SECONDS, SEED_BASE, certify

    out = []
    for path in sys.argv[1:]:
        train.EGO_SPEED = STALE_EGO_SPEED
        stale = certify(path, "silverstone", SEED_BASE)
        train.EGO_SPEED = 260
        fresh = certify(path, "silverstone", SEED_BASE)
        row = {
            "checkpoint": path.split("/")[-1],
            "stale_charged": stale["charged_lap"],
            "fresh_charged": fresh["charged_lap"],
            "stale_off_median": float(np.median(stale["off_each_lap"])),
            "fresh_off_median": float(np.median(fresh["off_each_lap"])),
            "stale_off_mean": float(np.mean(stale["off_each_lap"])),
            "fresh_off_mean": float(np.mean(fresh["off_each_lap"])),
            "stale_clean_share": stale["clean_share"],
            "fresh_clean_share": fresh["clean_share"],
            "stale_best": stale["lap"],
            "fresh_best": fresh["lap"],
            "stale_spins": stale["spins"],
            "fresh_spins": fresh["spins"],
        }
        out.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    json.dump(out, open("old-vs-new-account.json", "w"), ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
