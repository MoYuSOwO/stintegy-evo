"""How far sideways a driver actually goes, and for how long.

The limit-zone constants — where the axles start fading, where the referee
declares a car lost — are only defensible against a measurement of what the
drivers already do. This reads the sideslip channel out of the observation
vector for a whole session and reports the distribution: percentiles, the
peak, and how much of the time is spent past each candidate line.

    python3 probe_limit_zone.py silverstone --rates 15,60
    python3 probe_limit_zone.py silverstone --rates 15 --policy checkpoints/bestalpha002.pt

With no ``--policy`` the analytic driver is at the wheel, which is the
calibration that matters: a line the rule-based driver crosses in normal
running is a line drawn across the road rather than around the limit.
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from host_env import HostEnv
from train import EGO_SPEED, EVALUATION_MODES, TRACKS

# The ego block runs speed, longitudinal, lateral, yaw rate, sideslip.
EGO_SIDESLIP = EGO_SPEED + 4
SIDESLIP_SCALE = 0.5
SEED_BASE = 900_001
LANES = 2
SIM_SECONDS = 600.0

# The lines this probe exists to argue about, in degrees.
WATCHED_DEGREES = (4.0, 5.0, 8.0, 10.0, 12.0)

# What counts as being up against the model's ceiling. Slightly under the
# limit itself because the angle makes a round trip through a float32
# observation scaled by a half, and an equality test on that is a test of
# rounding.
CEILING_DEGREES = 9.9


def measure(
    track: str,
    hz: float,
    seconds: float,
    policy: str | None,
    warmup: float = 0.0,
) -> dict[str, float]:
    """``warmup`` seconds at the start of the session are driven but not
    counted: an episode begins at a random point on the circuit at a speed
    nobody chose, and the seconds a driver spends settling onto its line are
    not the seconds this probe is about."""
    steps = int(round(seconds * hz))
    skip = int(round(warmup * hz))
    agent = None
    with HostEnv(
        batch=LANES,
        seed_base=SEED_BASE,
        solo=True,
        track=track,
        episode_seconds=seconds + 60.0,
        ego_modes=EVALUATION_MODES,
        ego_analytic=policy is None,
        analytic_hz=hz if policy is None else None,
        decision_hz=hz,
    ) as env:
        if policy is not None:
            from sac import SacAgent, SacConfig

            agent = SacAgent(env.obs_size, env.action_size, SacConfig())
            agent.load(policy)
        obs = env.reset()
        idle = np.zeros((LANES, env.action_size), dtype=np.float32)
        samples: list[float] = []
        spin_events = 0
        # How long each visit to the ceiling lasts, per lane. The occupancy
        # is not the question - a driver who touches the limit and catches
        # it has done nothing wrong, and the whole verdict rule turns on
        # telling that apart from a driver who sits there.
        holding = np.zeros(LANES)
        holds: list[float] = []
        for step in range(steps):
            action = idle if agent is None else agent.act(obs, deterministic=True)
            obs, _, _, _, _, _, _, spins = env.step(action)
            if step < skip:
                continue
            spin_events += int(spins.sum())
            degrees = np.degrees(np.abs(obs[:, EGO_SIDESLIP] * SIDESLIP_SCALE))
            samples.extend(degrees.tolist())
            at_ceiling = degrees >= CEILING_DEGREES
            for lane in range(LANES):
                if at_ceiling[lane]:
                    holding[lane] += 1.0 / hz
                elif holding[lane] > 0.0:
                    holds.append(holding[lane])
                    holding[lane] = 0.0
        for lane in range(LANES):
            if holding[lane] > 0.0:
                holds.append(holding[lane])

    degrees = np.asarray(samples)
    out = {
        "hz": hz,
        "samples": float(degrees.size),
        "spins": float(spin_events),
        "mean": float(degrees.mean()),
        "p50": float(np.percentile(degrees, 50)),
        "p95": float(np.percentile(degrees, 95)),
        "p99": float(np.percentile(degrees, 99)),
        "p999": float(np.percentile(degrees, 99.9)),
        "max": float(degrees.max()),
    }
    for line in WATCHED_DEGREES:
        out[f"over{line:g}"] = float((degrees >= line).mean())
    runs = np.asarray(holds) if holds else np.zeros(0)
    out["holds"] = float(runs.size)
    out["hold_p50"] = float(np.percentile(runs, 50)) if runs.size else 0.0
    out["hold_p95"] = float(np.percentile(runs, 95)) if runs.size else 0.0
    out["hold_max"] = float(runs.max()) if runs.size else 0.0
    out["hold_over_250ms"] = float((runs > 0.25).sum()) if runs.size else 0.0
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("tracks", nargs="*", default=["silverstone"])
    parser.add_argument("--rates", default="15")
    parser.add_argument("--seconds", type=float, default=SIM_SECONDS)
    parser.add_argument("--policy", default=None)
    parser.add_argument("--warmup", type=float, default=15.0)
    args = parser.parse_args()

    rates = [float(r) for r in args.rates.split(",")]
    tracks = args.tracks or ["silverstone"]
    unknown = [t for t in tracks if t not in TRACKS]
    if unknown:
        print(f"unknown circuit(s): {', '.join(unknown)}")
        return 2

    who = args.policy or "analytic"
    print(
        f"sideslip probe · {who} · {LANES} lanes × "
        f"{args.seconds:.0f} simulated seconds · degrees",
        flush=True,
    )
    header = (
        f"    {'Hz':>4}  {'mean':>6}  {'p95':>6}  {'p99':>6}  {'max':>6}"
        f"  {'spins':>5}  {'holds':>5}  {'h.p50':>6}  {'h.p95':>6}"
        f"  {'h.max':>6}  {'>250ms':>6}  "
        + "  ".join(f"≥{line:g}°" for line in WATCHED_DEGREES)
    )
    for name in tracks:
        print(f"\n{name}", flush=True)
        print(header, flush=True)
        for hz in rates:
            r = measure(name, hz, args.seconds, args.policy, args.warmup)
            shares = "  ".join(
                f"{r[f'over{line:g}'] * 100:4.1f}%" for line in WATCHED_DEGREES
            )
            print(
                f"    {hz:4.0f}  {r['mean']:6.2f}  {r['p95']:6.2f}"
                f"  {r['p99']:6.2f}  {r['max']:6.2f}  {r['spins']:5.0f}"
                f"  {r['holds']:5.0f}  {r['hold_p50']:6.3f}"
                f"  {r['hold_p95']:6.3f}  {r['hold_max']:6.3f}"
                f"  {r['hold_over_250ms']:6.0f}  {shares}",
                flush=True,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
