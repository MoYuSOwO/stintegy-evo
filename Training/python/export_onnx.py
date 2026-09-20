"""The actor's mean head, in a format the game can run.

A checkpoint is a torch file, and the game is a .NET process: nothing in it
can load one. ONNX is how a network crosses that line, and it is the same
crossing a shipped build would make, so this is a format conversion rather
than a tool that extracts results.

What is exported is exactly the deterministic policy the scoreboard
measures: the mean head, squashed by tanh, with the log-standard-deviation
half of the network's output dropped. 457 channels in, 2 out, one row at a
time.

    python3 Training/python/export_onnx.py \\
        --checkpoint checkpoints/evalparent3l-2650000.pt \\
        --out Assets/Drivers/parent3l.onnx
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import torch
from torch import nn

from sac import SacAgent, SacConfig
from train import OBSERVATION_SIZE


class DeterministicActor(nn.Module):
    """tanh(mean), and nothing else.

    The trained actor emits a mean and a log standard deviation and samples
    between them; a deployed driver takes the mean. Wrapping it this way
    keeps the exported graph honest about which of the two the game runs.
    """

    def __init__(self, actor: nn.Module, action_size: int) -> None:
        super().__init__()
        self.net = actor.net
        self.action_size = action_size

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        # Sliced rather than chunked: chunk exports as a Split node whose
        # attributes belong to a later opset than the one asked for, and
        # the runtime refuses the graph. The half taken is the same half.
        head = self.net(observation)
        return torch.tanh(head[..., : self.action_size])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--action-size", type=int, default=2)
    args = parser.parse_args()

    checkpoint = Path(args.checkpoint).resolve()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    config = SacConfig(device="cpu")
    agent = SacAgent(OBSERVATION_SIZE, args.action_size, config)
    restored = agent.load(str(checkpoint))
    policy = DeterministicActor(agent.actor, args.action_size).eval()

    example = torch.zeros(1, OBSERVATION_SIZE)
    torch.onnx.export(
        policy,
        example,
        str(out),
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes={"observation": {0: "batch"}, "action": {0: "batch"}},
        opset_version=17,
        # The classic exporter, which writes one self-contained file. The
        # dynamo path splits the weights into a sidecar, and a driver that
        # is two files is a driver somebody will ship half of.
        dynamo=False,
    )

    # What the game is running, written beside it: a network with no
    # provenance is a driver nobody can trace back to a bake.
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    card = {
        "checkpoint": checkpoint.name,
        "checkpoint_sha256": digest,
        "checkpoint_step": int(restored["step"]),
        "observation_size": OBSERVATION_SIZE,
        "action_size": args.action_size,
        "head": "tanh(mean), deterministic",
        "onnx_sha256": hashlib.sha256(out.read_bytes()).hexdigest(),
    }
    out.with_suffix(".json").write_text(json.dumps(card, indent=2))

    # A last check against the torch policy it came from, on random
    # observations: if these disagree, the game is driving a different
    # network from the one the bake graded.
    import numpy as np
    import onnxruntime

    session = onnxruntime.InferenceSession(str(out))
    sample = np.random.RandomState(0).randn(8, OBSERVATION_SIZE).astype(np.float32)
    with torch.no_grad():
        expected = policy(torch.from_numpy(sample)).numpy()
    got = session.run(["action"], {"observation": sample})[0]
    largest = float(np.max(np.abs(expected - got)))
    print(
        f"{out}  step {card['checkpoint_step']}  "
        f"largest disagreement with torch {largest:.2e}"
    )
    if largest > 1e-5:
        print("the exported network does not match the checkpoint")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
