"""Batched racecraft duels, driven over the host's framed stdio protocol.

The host (Tools/RacecraftHost) runs ``batch`` duels on one circuit in
lockstep. ``reset`` returns one observation per duel; ``step`` takes one
action row per duel and returns the next observations, the observations
the ended duels ended on, rewards, ended and out-of-time flags, and the
reward's parts. Ended duels are drawn afresh by the host at once.
"""

from __future__ import annotations

import os
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

MAGIC = 0x52435654
VERSION = 1
HEADER = struct.Struct("<IHHi")

HELLO, HELLO_RESPONSE = 1, 2
RESET, RESET_RESPONSE = 3, 4
STEP, STEP_RESPONSE = 5, 6
CLOSE, CLOSE_RESPONSE = 7, 8
ERROR = 0xFFFF

COMPONENTS = ("gain_m", "passes", "contact_s", "off_s", "wall_s", "spin")

HOST_PROJECT = Path(__file__).resolve().parents[1] / "StintegyEVO.RacecraftHost.csproj"


class RacecraftEnv:
    def __init__(self, batch: int = 16, circuit: str = "silverstone", quiet: bool = True) -> None:
        binary = os.environ.get("STINTEGY_RACECRAFT_HOST")
        launcher = [binary] if binary else [
            "dotnet", "run", "-c", "Release", "--project", str(HOST_PROJECT), "--",
        ]
        self._process = subprocess.Popen(
            launcher + ["--circuit", circuit, "--batch", str(batch)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL if quiet else sys.stderr,
            env={**os.environ, "DOTNET_ROLL_FORWARD": "Major"},
        )
        self._write(HELLO)
        payload = self._expect(HELLO_RESPONSE)
        self.obs_size, self.action_size, self.batch, version, self.components = struct.unpack("<iiiii", payload)
        if version != VERSION:
            raise RuntimeError(f"the host speaks protocol v{version}, this client v{VERSION}")

    def reset(self, seed: int = 0) -> np.ndarray:
        self._write(RESET, struct.pack("<q", seed))
        payload = self._expect(RESET_RESPONSE)
        return np.frombuffer(payload, dtype=np.float32).reshape(self.batch, self.obs_size).copy()

    def step(self, actions: np.ndarray):
        actions = np.ascontiguousarray(actions, dtype=np.float32)
        assert actions.shape == (self.batch, self.action_size), actions.shape
        self._write(STEP, actions.tobytes())
        payload = self._expect(STEP_RESPONSE)
        b, o, c = self.batch, self.obs_size, self.components
        at = 0

        def take(count: int, dtype) -> np.ndarray:
            nonlocal at
            size = count * np.dtype(dtype).itemsize
            out = np.frombuffer(payload, dtype=dtype, count=count, offset=at).copy()
            at += size
            return out

        obs = take(b * o, np.float32).reshape(b, o)
        final = take(b * o, np.float32).reshape(b, o)
        rewards = take(b, np.float32)
        ended = take(b, np.uint8).astype(bool)
        out_of_time = take(b, np.uint8).astype(bool)
        parts = take(b * c, np.float32).reshape(b, c)
        return obs, final, rewards, ended, out_of_time, parts

    def close(self) -> None:
        if self._process.poll() is None:
            try:
                self._write(CLOSE)
                self._expect(CLOSE_RESPONSE)
            except (BrokenPipeError, RuntimeError):
                pass
            self._process.wait(timeout=10)

    def __enter__(self) -> "RacecraftEnv":
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # -- protocol --------------------------------------------------------

    def _write(self, kind: int, payload: bytes = b"") -> None:
        stdin = self._process.stdin
        stdin.write(HEADER.pack(MAGIC, VERSION, kind, len(payload)))
        stdin.write(payload)
        stdin.flush()

    def _read_exactly(self, count: int) -> bytes:
        data = self._process.stdout.read(count)
        if data is None or len(data) != count:
            raise RuntimeError("the host closed its output")
        return data

    def _expect(self, kind: int) -> bytes:
        magic, version, got, length = HEADER.unpack(self._read_exactly(HEADER.size))
        if magic != MAGIC or version != VERSION:
            raise RuntimeError(f"bad header magic 0x{magic:08X} version {version}")
        payload = self._read_exactly(length) if length else b""
        if got == ERROR:
            raise RuntimeError("host error:\n" + payload.decode("utf-8", "replace"))
        if got != kind:
            raise RuntimeError(f"expected message {kind}, got {got}")
        return payload
