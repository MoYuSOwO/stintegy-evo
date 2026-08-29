"""Pipeline smoke test for the direct-drive training host.

Launches the C# host as a subprocess, speaks the binary stdio protocol,
runs a few batched episodes with random actions, and prints observation,
reward, and terminal statistics. This script is deliberately not a trainer:
it exists to prove every link of the chain — protocol framing, batched
reset/step, observation layout, reward components — before any learning
code is written on top.

Usage:
    python3 Training/python/smoke_client.py [--batch 4] [--steps 300]
"""

from __future__ import annotations

import argparse
import math
import random
import struct
import subprocess
import sys
from pathlib import Path

MAGIC = 0x53544556
VERSION = 1
HEADER = struct.Struct("<IHHi")

KIND_HELLO = 1
KIND_HELLO_RESPONSE = 2
KIND_RESET = 3
KIND_RESET_RESPONSE = 4
KIND_STEP = 5
KIND_STEP_RESPONSE = 6
KIND_CLOSE = 7
KIND_CLOSE_RESPONSE = 8
KIND_ERROR = 0xFFFF

TERMINAL_NAMES = [
    "none", "passed", "contact", "wall", "stalled", "timeout"
]

COMPONENT_NAMES = [
    "own_progress", "relative_progress", "pass", "contact", "wall",
    "action_magnitude", "action_delta", "time", "timeout_outcome",
    "mode_budget",
]


def write_message(stream, kind: int, payload: bytes = b"") -> None:
    stream.write(HEADER.pack(MAGIC, VERSION, kind, len(payload)))
    stream.write(payload)
    stream.flush()


def read_message(stream) -> tuple[int, bytes]:
    header = stream.read(HEADER.size)
    if len(header) != HEADER.size:
        raise EOFError("host closed the protocol stream")
    magic, version, kind, length = HEADER.unpack(header)
    if magic != MAGIC or version != VERSION:
        raise ValueError(f"bad header magic=0x{magic:08X} version={version}")
    payload = stream.read(length) if length else b""
    if len(payload) != length:
        raise EOFError("truncated payload")
    if kind == KIND_ERROR:
        raise RuntimeError(payload.decode("utf-8", "replace"))
    return kind, payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--host-project",
        default=str(
            Path(__file__).resolve().parents[1]
            / "StintegyEVO.TrainingHost"
            / "StintegyEVO.TrainingHost.csproj"
        ),
    )
    args = parser.parse_args()

    process = subprocess.Popen(
        [
            "dotnet", "run", "-c", "Release", "--project",
            args.host_project, "--", "--batch", str(args.batch),
            "--seed-base", str(args.seed),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,
    )
    assert process.stdin and process.stdout
    rng = random.Random(args.seed)

    try:
        write_message(process.stdin, KIND_HELLO)
        kind, payload = read_message(process.stdout)
        assert kind == KIND_HELLO_RESPONSE, kind
        obs_size, action_size, batch, version = struct.unpack("<iiii", payload)
        print(
            f"hello: obs={obs_size} action={action_size} "
            f"batch={batch} protocol=v{version}"
        )

        seeds = struct.pack(
            f"<{batch}q", *[args.seed + i for i in range(batch)]
        )
        write_message(process.stdin, KIND_RESET, seeds)
        kind, payload = read_message(process.stdout)
        assert kind == KIND_RESET_RESPONSE, kind
        observations = struct.unpack(f"<{batch * obs_size}f", payload)
        finite = all(math.isfinite(v) for v in observations)
        print(f"reset: {len(observations)} floats, all finite: {finite}")

        total_reward = [0.0] * batch
        terminals: dict[str, int] = {}
        component_totals = [0.0] * len(COMPONENT_NAMES)
        alive = [True] * batch
        for step in range(args.steps):
            actions = []
            for i in range(batch):
                actions.append(rng.uniform(-0.3, 0.3))
                actions.append(rng.uniform(-0.2, 0.6))
            write_message(
                process.stdin,
                KIND_STEP,
                struct.pack(f"<{batch * action_size}f", *actions),
            )
            kind, payload = read_message(process.stdout)
            assert kind == KIND_STEP_RESPONSE, kind
            cursor = batch * obs_size * 4
            rewards = struct.unpack_from(f"<{batch}f", payload, cursor)
            cursor += batch * 4
            done = payload[cursor:cursor + batch]
            cursor += batch
            reasons = payload[cursor:cursor + batch]
            cursor += batch
            for c in range(len(COMPONENT_NAMES)):
                values = struct.unpack_from(f"<{batch}f", payload, cursor)
                cursor += batch * 4
                component_totals[c] += sum(values)
            for i in range(batch):
                total_reward[i] += rewards[i]
            if any(done):
                print(
                    f"step {step}: done={list(done)} "
                    f"reasons={[TERMINAL_NAMES[r] for r in reasons]}"
                )
                # MaskedReset payload is structure-of-arrays: every mask
                # byte first, then every seed.
                masks = bytearray()
                seeds_payload = bytearray()
                for i in range(batch):
                    if done[i]:
                        name = TERMINAL_NAMES[reasons[i]]
                        terminals[name] = terminals.get(name, 0) + 1
                    masks.append(1 if done[i] else 0)
                    seeds_payload += struct.pack(
                        "<q", args.seed + 1000 + step * batch + i
                    )
                write_message(process.stdin, 9, bytes(masks + seeds_payload))
                kind, _ = read_message(process.stdout)
                assert kind == 10, kind

        print(f"steps: {args.steps} x batch {batch}")
        print(f"episode terminals: {terminals or 'none'}")
        print(f"mean step reward: {sum(total_reward) / (batch * args.steps):+.4f}")
        for name, total in zip(COMPONENT_NAMES, component_totals):
            print(f"  {name:>18}: {total:+10.3f}")

        write_message(process.stdin, KIND_CLOSE)
        kind, _ = read_message(process.stdout)
        assert kind == KIND_CLOSE_RESPONSE, kind
        print("pipeline OK")
        return 0
    finally:
        process.stdin.close()
        process.wait(timeout=30)


if __name__ == "__main__":
    raise SystemExit(main())
