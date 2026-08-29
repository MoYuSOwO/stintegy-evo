"""Vectorized environment client for the direct-drive training host.

Speaks the framed binary stdio protocol to a C# host subprocess and
presents it as a batched environment: ``reset`` returns one observation
per lane, ``step`` takes one action row per lane and returns observations,
rewards, done flags, terminal reasons, and the individual reward
components. Lanes that finish are re-seeded through the masked reset, so
the batch never stalls waiting for its slowest episode.
"""

from __future__ import annotations

import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

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
KIND_MASKED_RESET = 9
KIND_MASKED_RESET_RESPONSE = 10
KIND_ERROR = 0xFFFF

TERMINAL_NAMES = (
    "none", "passed", "contact", "wall", "stalled", "timeout"
)

COMPONENT_NAMES = (
    "own_progress", "relative_progress", "pass", "contact", "wall",
    "off_course", "tyre_slip", "time", "timeout_outcome",
    "mode_excess", "retirement",
)

DEFAULT_HOST_PROJECT = str(
    Path(__file__).resolve().parents[1]
    / "StintegyEVO.TrainingHost"
    / "StintegyEVO.TrainingHost.csproj"
)


class HostEnv:
    """One host subprocess driving ``batch`` environments in lockstep."""

    def __init__(
        self,
        batch: int = 16,
        seed_base: int = 0,
        solo: bool = False,
        track: str | None = None,
        episode_seconds: float | None = None,
        host_project: str = DEFAULT_HOST_PROJECT,
        quiet: bool = True,
    ) -> None:
        command = [
            "dotnet", "run", "-c", "Release", "--project", host_project,
            "--", "--batch", str(batch), "--seed-base", str(seed_base),
        ]
        if solo:
            command.append("--solo")
        if track:
            command += ["--track", track]
        if episode_seconds is not None:
            command += ["--episode-seconds", str(episode_seconds)]

        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL if quiet else sys.stderr,
        )
        assert self._process.stdin and self._process.stdout
        self._stdin = self._process.stdin
        self._stdout = self._process.stdout
        self._seed_counter = seed_base + 10_000_000

        self._write(KIND_HELLO)
        kind, payload = self._read()
        assert kind == KIND_HELLO_RESPONSE, kind
        obs_size, action_size, host_batch, version = struct.unpack(
            "<iiii", payload
        )
        if version != VERSION:
            raise RuntimeError(f"host speaks protocol v{version}")
        self.obs_size = obs_size
        self.action_size = action_size
        self.batch = host_batch

    # -- protocol ---------------------------------------------------------

    def _write(self, kind: int, payload: bytes = b"") -> None:
        self._stdin.write(HEADER.pack(MAGIC, VERSION, kind, len(payload)))
        self._stdin.write(payload)
        self._stdin.flush()

    def _read(self) -> tuple[int, bytes]:
        header = self._stdout.read(HEADER.size)
        if len(header) != HEADER.size:
            raise EOFError("host closed the protocol stream")
        magic, version, kind, length = HEADER.unpack(header)
        if magic != MAGIC or version != VERSION:
            raise ValueError(f"bad header magic=0x{magic:08X} v={version}")
        payload = b""
        while len(payload) < length:
            chunk = self._stdout.read(length - len(payload))
            if not chunk:
                raise EOFError("truncated payload")
            payload += chunk
        if kind == KIND_ERROR:
            raise RuntimeError(payload.decode("utf-8", "replace"))
        return kind, payload

    def _next_seeds(self, count: int) -> list[int]:
        seeds = [self._seed_counter + i for i in range(count)]
        self._seed_counter += count
        return seeds

    def _observations_from(self, payload: bytes) -> np.ndarray:
        flat = np.frombuffer(
            payload, dtype="<f4", count=self.batch * self.obs_size
        )
        return flat.reshape(self.batch, self.obs_size).astype(np.float32)

    # -- environment ------------------------------------------------------

    def reset(self) -> np.ndarray:
        seeds = self._next_seeds(self.batch)
        self._write(
            KIND_RESET, struct.pack(f"<{self.batch}q", *seeds)
        )
        kind, payload = self._read()
        assert kind == KIND_RESET_RESPONSE, kind
        return self._observations_from(payload)

    def step(
        self, actions: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Advance every lane, then re-seed the lanes that finished.

        Returns ``(next_obs, reward, done, reason, components)``. The
        returned ``next_obs`` is the post-reset observation for finished
        lanes, so the caller must store the transition using the terminal
        flag rather than bootstrapping through it.
        """
        flat = np.ascontiguousarray(
            np.clip(actions, -1.0, 1.0), dtype="<f4"
        )
        self._write(KIND_STEP, flat.tobytes())
        kind, payload = self._read()
        assert kind == KIND_STEP_RESPONSE, kind

        cursor = self.batch * self.obs_size * 4
        obs = self._observations_from(payload)
        reward = np.frombuffer(
            payload, dtype="<f4", count=self.batch, offset=cursor
        ).astype(np.float32)
        cursor += self.batch * 4
        done = np.frombuffer(
            payload, dtype=np.uint8, count=self.batch, offset=cursor
        ).astype(bool)
        cursor += self.batch
        reason = np.frombuffer(
            payload, dtype=np.uint8, count=self.batch, offset=cursor
        ).copy()
        cursor += self.batch
        components = np.frombuffer(
            payload,
            dtype="<f4",
            count=self.batch * len(COMPONENT_NAMES),
            offset=cursor,
        ).reshape(len(COMPONENT_NAMES), self.batch).astype(np.float32)

        if done.any():
            seeds = self._next_seeds(self.batch)
            payload = done.astype(np.uint8).tobytes() + struct.pack(
                f"<{self.batch}q", *seeds
            )
            self._write(KIND_MASKED_RESET, payload)
            kind, reset_payload = self._read()
            assert kind == KIND_MASKED_RESET_RESPONSE, kind
            obs = self._observations_from(reset_payload)

        return obs, reward, done, reason, components

    def close(self) -> None:
        try:
            self._write(KIND_CLOSE)
            self._read()
        except (OSError, EOFError, RuntimeError):
            pass
        finally:
            try:
                self._stdin.close()
            except OSError:
                pass
            self._process.wait(timeout=30)

    def __enter__(self) -> "HostEnv":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
