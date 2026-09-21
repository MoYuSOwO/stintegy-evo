"""How far the car actually wanders sideways on a straight, in centimetres.

The jitter screen counts reversals and measures them in normalised command
units, which says nothing about what a viewer sees. What a viewer sees is the
car moving across the road. Lateral acceleration is an observation, so the
displacement that produced it can be recovered per frequency:

    displacement(f) = acceleration(f) / (2*pi*f)^2

Summed over the band above the track's own content, that is the width of the
weave itself, separated from the line the driver is actually taking.
"""

from __future__ import annotations

import argparse
import sys

sys.path.insert(
    0, "/Users/jayhuang/Code/stintegy-evo/.worktrees/steering-detour/Training/python"
)

import numpy as np
import torch

from host_env import HostEnv
from sac import SacAgent, SacConfig
from train import EVALUATION_MODES, EGO_SPEED, STEP_SECONDS

EGO_LATERAL_ACCEL = EGO_SPEED + 2
ACCELERATION_SCALE = 20.0
STRAIGHT_LATERAL = 2.0          # m/s^2, the jitter screen's own definition
WEAVE_BAND_HZ = 1.5             # above the track's 0.5-0.6 Hz content
MIN_RUN = 48                    # decisions, ~3.2 s of straight


def trace(checkpoint, seconds, track, batch, seed):
    torch.manual_seed(seed)
    np.random.seed(seed)
    with HostEnv(batch=batch, seed_base=seed, solo=True, track=track,
                 episode_seconds=seconds + 60.0, ego_modes=EVALUATION_MODES,
                 delta_actions=True) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig(device="cpu"))
        agent.load(checkpoint)
        obs = env.reset()
        rows = []
        for _ in range(int(round(seconds / STEP_SECONDS))):
            action = agent.act(obs, deterministic=True)
            rows.append(obs[:, EGO_LATERAL_ACCEL].copy() * ACCELERATION_SCALE)
            obs, *_rest = env.step(action)
    return np.asarray(rows)


def wobble(accel: np.ndarray) -> tuple[float, float, int]:
    """RMS sideways displacement in the weave band, in metres."""
    rate = 1.0 / STEP_SECONDS
    energies, peaks, runs = [], [], 0
    for lane in range(accel.shape[1]):
        series = accel[:, lane]
        straight = np.abs(series) < STRAIGHT_LATERAL
        start = None
        for i in range(len(series) + 1):
            inside = i < len(series) and straight[i]
            if inside and start is None:
                start = i
            elif not inside and start is not None:
                if i - start >= MIN_RUN:
                    segment = series[start:i]
                    segment = segment - segment.mean()
                    window = np.hanning(len(segment))
                    spectrum = np.fft.rfft(segment * window)
                    freq = np.fft.rfftfreq(len(segment), 1.0 / rate)
                    # Hann halves the amplitude; two-sided to one-sided doubles.
                    amp = np.abs(spectrum) * 4.0 / len(segment)
                    band = freq >= WEAVE_BAND_HZ
                    disp = np.zeros_like(amp)
                    disp[band] = amp[band] / (2 * np.pi * freq[band]) ** 2
                    energies.append(float(np.sum(disp**2) / 2))
                    peaks.append(float(disp.max()))
                    runs += 1
                start = None
    if not energies:
        return float("nan"), float("nan"), 0
    return float(np.sqrt(np.mean(energies))), float(np.mean(peaks)), runs


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", action="append", required=True)
    p.add_argument("--track", default="silverstone")
    p.add_argument("--batch", type=int, default=4)
    p.add_argument("--seconds", type=float, default=240.0)
    p.add_argument("--seed", type=int, default=900_000)
    args = p.parse_args()

    print(f"直道判据 |横向加速度| < {STRAIGHT_LATERAL} m/s^2,"
          f"蛇行频带 >= {WEAVE_BAND_HZ} Hz\n")
    print(f"{'检查点':<22}{'RMS 横移':>12}{'峰值分量':>12}{'直道段数':>10}")
    for entry in args.checkpoint:
        label, path = entry.split("=", 1)
        accel = trace(path, args.seconds, args.track, args.batch, args.seed)
        rms, peak, runs = wobble(accel)
        print(f"{label:<22}{rms*100:9.2f} cm{peak*100:9.2f} cm{runs:10d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
