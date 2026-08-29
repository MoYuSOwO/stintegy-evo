"""Where the update time goes, and whether Metal would take it.

The learner costs about twenty times what collecting a transition costs,
so it is the whole budget. This times an update and an action on both
devices, at the network we have and at the one Sony used, because the
answer is allowed to depend on the size.
"""
import time

import numpy as np
import torch

from sac import SacAgent, SacConfig

OBS, ACTION = 303, 2


def bench(device: str, hidden: tuple[int, ...], quantiles: int,
          lanes: int, repeats: int = 30) -> tuple[float, float]:
    config = SacConfig(device=device, hidden=hidden, quantiles=quantiles)
    agent = SacAgent(OBS, ACTION, config)
    rng = np.random.default_rng(0)
    count = config.batch_size + 64
    agent.buffer.add_batch(
        rng.standard_normal((count, OBS), dtype=np.float32),
        rng.standard_normal((count, ACTION), dtype=np.float32),
        np.zeros(count, dtype=np.float32),
        rng.standard_normal((count, OBS), dtype=np.float32),
        np.zeros(count, dtype=np.float32),
    )
    obs = rng.standard_normal((lanes, OBS), dtype=np.float32)

    for _ in range(5):
        agent.update()
        agent.act(obs)
    if device == "mps":
        torch.mps.synchronize()

    start = time.perf_counter()
    for _ in range(repeats):
        agent.update()
    if device == "mps":
        torch.mps.synchronize()
    update_ms = (time.perf_counter() - start) / repeats * 1000

    start = time.perf_counter()
    for _ in range(repeats):
        agent.act(obs)
    if device == "mps":
        torch.mps.synchronize()
    act_ms = (time.perf_counter() - start) / repeats * 1000
    return update_ms, act_ms


def main() -> int:
    devices = ["cpu"]
    if torch.backends.mps.is_available():
        devices.append("mps")
    print(f"devices: {devices}   torch {torch.__version__}\n")

    shapes = [
        ("ours       512,512,256", (512, 512, 256), 0),
        ("ours + QR  512,512,256", (512, 512, 256), 32),
        ("Sony    2048x4        ", (2048, 2048, 2048, 2048), 0),
        ("Sony+QR 2048x4        ", (2048, 2048, 2048, 2048), 32),
    ]
    print(f"{'network':<24} {'device':>6} {'update ms':>10} "
          f"{'act(64) ms':>11} {'ms per env step':>16}")
    for name, hidden, quantiles in shapes:
        for device in devices:
            update_ms, act_ms = bench(device, hidden, quantiles, 64)
            # what one environment step actually costs the learner:
            # two updates plus one action for the whole batch.
            per_step = 2 * update_ms + act_ms
            print(f"{name:<24} {device:>6} {update_ms:>10.2f} "
                  f"{act_ms:>11.3f} {per_step:>16.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
