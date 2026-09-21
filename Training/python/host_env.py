"""Vectorized environment client for the direct-drive training host.

Speaks the framed binary stdio protocol to a C# host subprocess and
presents it as a batched environment: ``reset`` returns one observation
per lane, ``step`` takes one action row per lane and returns observations,
rewards, done flags, terminal reasons, and the individual reward
components. Lanes that finish are re-seeded through the masked reset, so
the batch never stalls waiting for its slowest episode.
"""

from __future__ import annotations

import os
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

MAGIC = 0x53544556
VERSION = 8

# The decision rate the host defaults to, mirrored from
# DirectDriveRaceDriver.DefaultDecisionHz. It lives here rather than in
# train.py because every script that drives the host needs it and none of
# them should be carrying its own copy of a step length: a hundred
# milliseconds was hardcoded in three places when the rate moved.
DEFAULT_DECISION_HZ = 15.0
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
    # A barrier no longer ends a race -- it is priced for as long as it
    # lasts, like leaving the track -- so nothing between "contact" and
    # "stalled" ends an episode any more.
    "none", "passed", "contact", "stalled", "timeout",
    # The race's flag (protocol 5): a true terminal, so the energy budget's
    # bill is settled there rather than bootstrapped past.
    "finished",
)

COMPONENT_NAMES = (
    "own_progress", "relative_progress", "pass", "contact", "wall",
    "off_course", "tyre_slip", "time", "timeout_outcome",
    "mode_excess", "retirement", "budget",
    # The wheel (protocol 8): the detour the front wheels took over the
    # last two decisions, and a small tax on how far they travelled. Sophy
    # names a cost over the acted steering history and publishes neither
    # formula nor coefficient; both the shape and the numbers are ours.
    "steering_detour", "steering_travel",
)

DEFAULT_HOST_PROJECT = str(
    Path(__file__).resolve().parents[1]
    / "StintegyEVO.TrainingHost"
    / "StintegyEVO.TrainingHost.csproj"
)


class HostEnv:
    def __init__(
        self,
        batch: int = 16,
        seed_base: int = 0,
        solo: bool = False,
        track: str | None = None,
        episode_seconds: float | None = None,
        randomise_episode_start: bool = False,
        hidden_curriculum: bool = False,
        delta_actions: bool = False,
        steering_detour_cost: float | None = None,
        steering_travel_cost: float | None = None,
        budget_gamma: float | None = None,
        race_km: float | None = None,
        host_project: str = DEFAULT_HOST_PROJECT,
        quiet: bool = True,
        duel: bool = False,
        ego_modes: tuple[int, int] | None = None,
        ego_analytic: bool = False,
        analytic_hz: float | None = None,
        decision_hz: float | None = None,
    ) -> None:
        """One host subprocess driving ``batch`` environments in lockstep.

        ``ego_modes`` fixes the pit-wall instruction - (tyre rung, power
        rung), counted from one - for every episode instead of drawing it
        from the seed. Training leaves it None so the policy meets all five
        settings; evaluation sets it, because a lap time taken under an
        instruction nobody wrote down is not comparable to another one.

        ``ego_analytic`` puts the analytic driver at the wheel and ignores
        the actions sent to it, which is how the baseline a learned lap is
        quoted against gets measured on the learner's own terms.

        ``duel`` puts a second car in every lane. Its observation comes back
        in ``opponent_obs`` and its action goes to ``step`` beside the
        ego's; the rewards, the terminals and the scoreboard stay the ego's
        alone, because the partner is what the ego is being trained
        against and not a second learner. Solo is the default and is
        unchanged by any of this, down to the bit.
        """
        # A published self-contained host binary, when one is provided,
        # spawns directly: no SDK on the machine, no rebuild on spawn, and
        # an evaluation can never race a half-edited source tree. This is
        # how the rented box runs; the laptop keeps the source path.
        host_binary = os.environ.get("STINTEGY_HOST_BIN")
        launcher = [host_binary] if host_binary else [
            "dotnet", "run", "-c", "Release", "--project", host_project, "--",
        ]
        command = launcher + ["--batch", str(batch), "--seed-base", str(seed_base)]
        if duel:
            command.append("--duel")
        elif solo:
            command.append("--solo")
        if track:
            command += ["--track", track]
        if episode_seconds is not None:
            command += ["--episode-seconds", str(episode_seconds)]
        if randomise_episode_start:
            command += ["--randomise-episode-start"]
        if hidden_curriculum:
            # The freeze design's hidden curriculum: a per-episode limiter
            # strength and perception noise, never shown to the policy.
            # Training only; evaluation stays nominal.
            command += ["--hidden-curriculum"]
        if delta_actions:
            # The pilot: action[0] moves the steering command instead of
            # being it. The host carries the integrator and shows it back
            # in the observation; nothing else changes.
            command.append("--delta-actions")
        # The two steering costs, per second; None leaves the host's own
        # numbers alone and zero switches one off.
        if steering_detour_cost is not None:
            command += ["--steering-detour-cost", str(steering_detour_cost)]
        if steering_travel_cost is not None:
            command += ["--steering-travel-cost", str(steering_travel_cost)]
        if budget_gamma is not None:
            # The host shapes with phi' - phi by default; see
            # EnergyBudget.DefaultGamma for why not the learner's gamma.
            command += ["--budget-gamma", str(budget_gamma)]
        if race_km is not None:
            command += ["--race-km", str(race_km)]
        if ego_modes is not None:
            command += ["--ego-modes", f"{ego_modes[0]},{ego_modes[1]}"]
        if ego_analytic:
            command.append("--ego-analytic")
        if analytic_hz is not None:
            command += ["--analytic-hz", str(analytic_hz)]
        if decision_hz is not None:
            command += ["--decision-hz", str(decision_hz)]
        self.duel = duel
        self.step_seconds = 1.0 / (decision_hz or DEFAULT_DECISION_HZ)
        self.delta_actions = delta_actions
        self.ego_modes = ego_modes
        self.four_wheels_off = np.zeros(batch, dtype=np.float64)
        # Who is in front, in metres, positive while the sparring partner
        # leads. Zero in a solo run, where there is nobody to lead.
        self.lead_metres = np.zeros(batch, dtype=np.float64)
        self.steer_angle = np.zeros(batch, dtype=np.float64)
        self.opponent_obs: np.ndarray | None = None
        self.opponent_final_obs: np.ndarray | None = None
        self.ego_analytic = ego_analytic

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
        obs_size, action_size, host_batch, version, cars = struct.unpack(
            "<iiiii", payload
        )
        if version != VERSION:
            raise RuntimeError(f"host speaks protocol v{version}")
        self.obs_size = obs_size
        self.action_size = action_size
        self.batch = host_batch
        # Seats per lane, from the host rather than from what was asked
        # for: two in a duel, one solo.
        self.cars = cars
        if duel and cars != 2:
            raise RuntimeError(f"asked for a duel, host gave {cars} seat(s)")

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

    def _seats_from(self, payload: bytes) -> tuple[np.ndarray, np.ndarray | None]:
        """Split a lane's seats: the ego's frame, then the partner's.

        The ego is always the first seat of its lane, so a solo payload is
        this one with the second seat absent rather than a different shape.
        """
        flat = np.frombuffer(
            payload,
            dtype="<f4",
            count=self.batch * self.cars * self.obs_size,
        )
        seats = flat.reshape(
            self.batch, self.cars, self.obs_size
        ).astype(np.float32)
        if self.cars == 1:
            return seats[:, 0], None
        return seats[:, 0], seats[:, 1]

    def _observations_from(self, payload: bytes) -> np.ndarray:
        ego, opponent = self._seats_from(payload)
        self.opponent_obs = opponent
        return ego

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
        self, actions: np.ndarray, opponent_actions: np.ndarray | None = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Advance every lane, then re-seed the lanes that finished.

        Returns ``(next_obs, reward, done, reason, components,
        race_distance, final_obs, spins)``. The returned ``next_obs`` is the
        post-reset observation for finished lanes, so the caller must store
        the transition using the terminal flag rather than bootstrapping
        through it.
        """
        ego = np.clip(np.asarray(actions, dtype=np.float32), -1.0, 1.0)
        if self.cars == 1:
            if opponent_actions is not None:
                raise ValueError("a solo host has no seat for a second action")
            flat = np.ascontiguousarray(ego, dtype="<f4")
        else:
            if opponent_actions is None:
                raise ValueError("a duel step needs the partner's action too")
            partner = np.clip(
                np.asarray(opponent_actions, dtype=np.float32), -1.0, 1.0
            )
            # Interleaved seat by seat within a lane, which is the order the
            # host reads them in: ego, partner, ego, partner.
            flat = np.ascontiguousarray(
                np.stack([ego, partner], axis=1), dtype="<f4"
            )
        self._write(KIND_STEP, flat.tobytes())
        kind, payload = self._read()
        assert kind == KIND_STEP_RESPONSE, kind

        cursor = self.batch * self.cars * self.obs_size * 4
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
        cursor += self.batch * len(COMPONENT_NAMES) * 4
        # Where each car is round the lap, continuous across the start line.
        # Not an observation and never given to the policy: it is how the
        # harness times a lap, which the progress reward cannot do because it
        # is masked whenever a car is off the road.
        race_distance = np.frombuffer(
            payload, dtype="<f4", count=self.batch, offset=cursor
        ).astype(np.float64)
        cursor += self.batch * 4
        # Spins begun during this step, per lane. An event count rather than
        # a running total, so a lane that re-seeds cannot make the number go
        # backwards and summing is the whole of the arithmetic.
        spins = np.frombuffer(
            payload, dtype=np.uint8, count=self.batch, offset=cursor
        ).astype(np.int64)
        cursor += self.batch
        # Seconds of the step each lane spent with all four wheels over the
        # white line: the race's track-limits ruler, for certification only.
        # Kept as an attribute rather than a ninth return value because a
        # dozen callers unpack this method's tuple and none of them want it;
        # the ones that do read it straight after the step it belongs to.
        self.four_wheels_off = np.frombuffer(
            payload, dtype="<f4", count=self.batch, offset=cursor
        ).astype(np.float64)
        cursor += self.batch * 4
        # Where the front wheels ended the step, in radians: the command
        # through the rack's rate limit, which is the steering a viewer
        # watches. Scoreboard only, like the three fields above it.
        self.steer_angle = np.frombuffer(
            payload, dtype="<f4", count=self.batch, offset=cursor
        ).astype(np.float64)
        cursor += self.batch * 4
        # Who is in front, in metres, positive while the partner leads. The
        # scoreboard's reading of the duel, on the same terms as the three
        # fields above it: never an observation, never a reward.
        if self.cars > 1:
            self.lead_metres = np.frombuffer(
                payload, dtype="<f4", count=self.batch, offset=cursor
            ).astype(np.float64)

        # The observation the episode actually ended on. The array returned
        # below is what the policy acts on next, so finished lanes carry the
        # fresh episode's first frame — but a learner bootstrapping across a
        # timeout needs the state the clock stopped at, and once the reset
        # has run this is the only copy of it.
        final_obs = obs
        self.opponent_final_obs = self.opponent_obs
        if done.any():
            final_obs = obs.copy()
            if self.opponent_obs is not None:
                self.opponent_final_obs = self.opponent_obs.copy()
            seeds = self._next_seeds(self.batch)
            payload = done.astype(np.uint8).tobytes() + struct.pack(
                f"<{self.batch}q", *seeds
            )
            self._write(KIND_MASKED_RESET, payload)
            kind, reset_payload = self._read()
            assert kind == KIND_MASKED_RESET_RESPONSE, kind
            obs = self._observations_from(reset_payload)

        return (
            obs, reward, done, reason, components, race_distance, final_obs,
            spins,
        )

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
