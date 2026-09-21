"""Where in the frequency band a driver's steering lives.

The detour cost is measured over two decisions, so it bites hardest at the
top of the band: a wheel sawn at twelve hertz opposes itself every decision
and pays every decision. An oscillation slower than the window slips
between its teeth — turn one way for three decisions, back for three, and
no two neighbouring moves ever oppose. That is a slower weave, and it looks
worse rather than better: a car visibly snaking down a straight instead of
shimmering. The travel tax is the floor under it; this probe is how we find
out whether the floor held.

So the frequency is watched as well as the rate. Two spectra are taken,
because they answer different questions:

* **command** — what the policy asked for, decision by decision. This is
  where an escape shows first.
* **acted** — what the front wheels did, which is the command through the
  rack's own rate limit. This is what a viewer sees.

The verdict is comparative: against a baseline checkpoint, no new peak
below 5 Hz. A probe that only counted reversals would call the escape a
success.

    python3 Training/python/spectrum_probe.py \\
        --checkpoint checkpoints/evalparent6a-400000.pt \\
        --baseline checkpoints/evalparent5c-500000.pt
"""

from __future__ import annotations

import argparse

import numpy as np
import torch

from host_env import HostEnv
from sac import SacAgent, SacConfig
from train import EGO_SPEED, EVALUATION_MODES, STEP_SECONDS

EGO_COMMAND = EGO_SPEED + 10
# Below this the "oscillation" is the circuit: a lap of Silverstone has
# corners a few seconds long, and a car going round them is not weaving.
LOWEST_BAND_HZ = 0.5
# The band a slow weave would escape into.
SLOW_BAND_HZ = 5.0


def trace(
    checkpoint: str,
    seconds: float,
    track: str,
    batch: int,
    seed: int,
    delta_actions: bool,
    detour: float | None,
    travel: float | None,
) -> tuple[np.ndarray, np.ndarray]:
    """The steering command and the acted steering, per lane, per decision."""
    torch.manual_seed(seed)
    np.random.seed(seed)
    with HostEnv(
        batch=batch,
        seed_base=seed,
        solo=True,
        track=track,
        episode_seconds=seconds + 60.0,
        ego_modes=EVALUATION_MODES,
        delta_actions=delta_actions,
        steering_detour_cost=detour,
        steering_travel_cost=travel,
    ) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig(device="cpu"))
        agent.load(checkpoint)
        obs = env.reset()
        commands: list[np.ndarray] = []
        acted: list[np.ndarray] = []
        for _ in range(int(round(seconds / STEP_SECONDS))):
            action = agent.act(obs, deterministic=True)
            commands.append(obs[:, EGO_COMMAND].copy())
            obs, *_rest = env.step(action)
            # The front wheels themselves, reported by the host beside the
            # other scoreboard readings: the command through the rack's
            # rate limit, which is the steering a viewer watches.
            acted.append(env.steer_angle.copy())
    return np.asarray(commands), np.asarray(acted)


def spectrum(trace: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Mean single-sided amplitude spectrum over the lanes, in Hz."""
    steps, lanes = trace.shape
    centred = trace - trace.mean(axis=0, keepdims=True)
    window = np.hanning(steps)[:, None]
    spectra = np.abs(np.fft.rfft(centred * window, axis=0)) / steps * 2.0
    frequencies = np.fft.rfftfreq(steps, d=STEP_SECONDS)
    return frequencies, spectra.mean(axis=1)


def peaks(frequencies: np.ndarray, amplitude: np.ndarray, count: int = 4):
    band = frequencies >= LOWEST_BAND_HZ
    order = np.argsort(amplitude[band])[::-1][:count]
    return [(float(frequencies[band][i]), float(amplitude[band][i])) for i in order]


def report(name: str, frequencies: np.ndarray, amplitude: np.ndarray) -> None:
    print(f"  {name}")
    for hz, size in peaks(frequencies, amplitude):
        print(f"    {hz:5.2f} Hz   amplitude {size:.5f}")
    slow = (frequencies >= LOWEST_BAND_HZ) & (frequencies < SLOW_BAND_HZ)
    fast = frequencies >= SLOW_BAND_HZ
    print(
        f"    energy under {SLOW_BAND_HZ:.0f} Hz "
        f"{amplitude[slow].sum():.4f}, above {amplitude[fast].sum():.4f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--baseline", default=None,
                        help="the checkpoint this one has to be no worse than")
    parser.add_argument("--track", default="silverstone")
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--seed", type=int, default=900_000)
    parser.add_argument("--detour", type=float, default=None)
    parser.add_argument("--travel", type=float, default=None)
    parser.add_argument("--absolute-actions", action="store_true")
    args = parser.parse_args()

    runs = [("checkpoint", args.checkpoint)]
    if args.baseline:
        runs.append(("baseline", args.baseline))

    measured = {}
    for label, path in runs:
        commands, acted = trace(
            path, args.seconds, args.track, args.batch, args.seed,
            not args.absolute_actions, args.detour, args.travel,
        )
        print(f"{label}: {path.split('/')[-1]}")
        measured[label] = {}
        for name, data in (("command", commands), ("acted", acted)):
            frequencies, amplitude = spectrum(data)
            report(name, frequencies, amplitude)
            measured[label][name] = (frequencies, amplitude)
        print()

    if "baseline" not in measured:
        return 0

    # The verdict: nothing new below five hertz. "New" means the band's
    # energy grew -- a policy is allowed to keep the baseline's slow
    # content, which is mostly the circuit itself.
    worst = 0.0
    for name in ("command", "acted"):
        frequencies, amplitude = measured["checkpoint"][name]
        _, before = measured["baseline"][name]
        slow = (frequencies >= LOWEST_BAND_HZ) & (frequencies < SLOW_BAND_HZ)
        grew = amplitude[slow].sum() / max(before[slow].sum(), 1e-9)
        worst = max(worst, grew)
        print(
            f"{name}: slow-band energy {grew:.2f}x the baseline's "
            f"({amplitude[slow].sum():.4f} against {before[slow].sum():.4f})"
        )
    print(
        "\nverdict: "
        + ("PASS -- no new slow weave" if worst <= 1.2 else
           "FAIL -- the oscillation moved down the band")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
