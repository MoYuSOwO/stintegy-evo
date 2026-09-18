"""Lifts a trained actor out of PyTorch and into something the game reads.

Two files come out of here and they are meant to be read together.

The first is the policy: the actor's weights, laid out flat, with just
enough of a header to say what shape they are. Nothing about SAC survives
the trip -- no critics, no entropy coefficient, no optimizer moments, and
not even the actor's second head. A trained actor emits a mean and a log
standard deviation; the game never samples, so the log standard deviation
is dead weight and the rows that produce it are dropped here rather than
skipped there. What is left is a plain multilayer perceptron whose output,
squashed through a tanh, is the action.

The second is the fixture, and it is the only reason the first can be
trusted. Reimplementing a forward pass in another language is four or five
chances to be quietly wrong -- a transposed weight matrix, a bias added to
the wrong axis, the two output heads swapped, a missing activation, an
endianness assumption -- and every one of them produces a policy that runs
without complaint and drives like something else. So this writes down what
this exact network answered to sixty-four exact observations, and a test on
the other side checks that the port answers the same. The observations are
not synthetic noise: most of them are lifted off a real lap of the circuit
this policy was trained on, so the fixture pins the network in the region
the game will actually run it in, where a saturated tanh cannot hide a
mistake by returning the same +/-1 to everything.

    Training/.venv/bin/python3 export_policy.py \
        checkpoints/bestalpha002.pt \
        --name silverstone-expert --track silverstone

Writes Assets/Drivers/<name>.nn and
Core/Tests/Fixtures/<name>-alignment.bin.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np
import torch

from sac import SacAgent, SacConfig

REPOSITORY = Path(__file__).resolve().parents[2]

# "STNN": a network. "STFX": a fixture. Both little-endian throughout,
# which is what every platform this ships on is.
NETWORK_MAGIC = b"STNN"
NETWORK_VERSION = 1
FIXTURE_MAGIC = b"STFX"
FIXTURE_VERSION = 1

ACTIVATION_NONE = 0
ACTIVATION_RELU = 1
SQUASH_TANH = 1

# How many of the fixture's cases come off a real lap rather than out of a
# random number generator. The random ones are there so the test also
# covers the corners of the input range, which a lap never visits.
ROLLOUT_CASES = 48
RANDOM_CASES = 16
FIXTURE_SEED = 20260907


def actor_layers(state: dict[str, torch.Tensor], action_size: int):
    """The actor's Linear layers in order, with the log-std rows removed.

    ``mlp`` builds Linear/ReLU pairs and a final bare Linear, so the state
    dict's even indices are the weights and the activation between any two
    of them is a ReLU. The final layer emits ``2 * action_size`` values and
    ``Actor.forward`` chunks them into a mean and a log standard deviation,
    in that order; deterministic inference uses only the first chunk, so
    only the first ``action_size`` rows of that layer are carried over.
    """
    indices = sorted(
        int(key.split(".")[1])
        for key in state
        if key.endswith(".weight")
    )
    layers = []
    for position, index in enumerate(indices):
        weight = state[f"net.{index}.weight"].cpu().numpy().astype(np.float32)
        bias = state[f"net.{index}.bias"].cpu().numpy().astype(np.float32)
        last = position == len(indices) - 1
        if last:
            if weight.shape[0] != 2 * action_size:
                raise ValueError(
                    f"The head emits {weight.shape[0]} values, not the "
                    f"{2 * action_size} a mean and a log-std would be."
                )
            weight = weight[:action_size]
            bias = bias[:action_size]
        layers.append(
            (weight, bias, ACTIVATION_NONE if last else ACTIVATION_RELU)
        )
    return layers


def write_network(path: Path, layers, input_size: int, action_size: int):
    blob = bytearray()
    blob += NETWORK_MAGIC
    blob += struct.pack(
        "<5i",
        NETWORK_VERSION,
        input_size,
        action_size,
        SQUASH_TANH,
        len(layers),
    )
    for weight, bias, activation in layers:
        blob += struct.pack(
            "<3i", int(weight.shape[1]), int(weight.shape[0]), activation
        )
    for weight, bias, _ in layers:
        # Row-major, one output's whole row of inputs at a time, which is
        # what torch stores and what a cache-friendly forward pass wants.
        blob += np.ascontiguousarray(weight, dtype="<f4").tobytes()
        blob += np.ascontiguousarray(bias, dtype="<f4").tobytes()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(bytes(blob))
    return len(blob)


def write_fixture(path: Path, observations: np.ndarray, actions: np.ndarray):
    blob = bytearray()
    blob += FIXTURE_MAGIC
    blob += struct.pack(
        "<4i",
        FIXTURE_VERSION,
        observations.shape[0],
        observations.shape[1],
        actions.shape[1],
    )
    for observation, action in zip(observations, actions):
        blob += np.ascontiguousarray(observation, dtype="<f4").tobytes()
        blob += np.ascontiguousarray(action, dtype="<f4").tobytes()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(bytes(blob))
    return len(blob)


def rollout_observations(
    agent: SacAgent, track: str, wanted: int, obs_size: int
) -> np.ndarray:
    """Observations this policy actually meets, driving where it was taught.

    Sampled every few decisions rather than consecutively, because sixty
    consecutive frames of a lap are nearly the same vector and would make
    the fixture sixty copies of one test.
    """
    from host_env import HostEnv

    lanes = 8
    stride = 5
    collected: list[np.ndarray] = []
    with HostEnv(
        batch=lanes,
        seed_base=900_001,
        solo=True,
        track=track,
        episode_seconds=400.0,
        ego_modes=(3, 3),
    ) as env:
        if env.obs_size != obs_size:
            raise ValueError(
                f"The host reports {env.obs_size} observation values and the "
                f"checkpoint expects {obs_size}."
            )
        obs = env.reset()
        step = 0
        while len(collected) < wanted:
            if step % stride == 0:
                collected.extend(obs.astype(np.float32))
            action = agent.act(obs, deterministic=True)
            obs = env.step(action)[0]
            step += 1
    return np.asarray(collected[:wanted], dtype=np.float32)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint")
    parser.add_argument("--name", default="silverstone-expert")
    parser.add_argument(
        "--track",
        default="silverstone",
        help="circuit the fixture's rollout observations are taken from",
    )
    parser.add_argument(
        "--no-rollout",
        action="store_true",
        help="build the fixture from random vectors alone, without a host",
    )
    parser.add_argument("--obs-size", type=int, default=452)
    parser.add_argument("--action-size", type=int, default=2)
    args = parser.parse_args()

    config = SacConfig(device="cpu")
    agent = SacAgent(args.obs_size, args.action_size, config)
    loaded = agent.load(args.checkpoint)
    agent.actor.eval()
    print(
        f"{args.checkpoint}: format {loaded['format']}, "
        f"step {loaded['step']}, hidden {config.hidden}",
        flush=True,
    )

    layers = actor_layers(agent.actor.state_dict(), args.action_size)
    shape = " -> ".join(
        [str(layers[0][0].shape[1])]
        + [str(weight.shape[0]) for weight, _, _ in layers]
    )
    parameters = sum(w.size + b.size for w, b, _ in layers)
    network_path = REPOSITORY / "Assets" / "Drivers" / f"{args.name}.nn"
    written = write_network(network_path, layers, args.obs_size, args.action_size)
    print(
        f"network: {shape} (tanh), {parameters} parameters, "
        f"{written} bytes -> {network_path.relative_to(REPOSITORY)}",
        flush=True,
    )

    generator = np.random.default_rng(FIXTURE_SEED)
    pieces = []
    if not args.no_rollout:
        pieces.append(
            rollout_observations(agent, args.track, ROLLOUT_CASES, args.obs_size)
        )
    wanted = RANDOM_CASES if pieces else ROLLOUT_CASES + RANDOM_CASES
    # Roughly the scale a normalized observation lives at, with a wide tail
    # so the test also sees inputs no lap would produce.
    pieces.append(
        generator.normal(0.0, 0.6, size=(wanted, args.obs_size)).astype(np.float32)
    )
    observations = np.concatenate(pieces, axis=0)

    with torch.no_grad():
        actions = agent.act(observations, deterministic=True).astype(np.float32)

    # A fixture whose every answer is pinned against the squash proves the
    # tanh and nothing before it. Worth saying out loud rather than
    # discovering later.
    saturated = float(np.mean(np.abs(actions) > 0.999))
    fixture_path = (
        REPOSITORY / "Core" / "Tests" / "Fixtures" / f"{args.name}-alignment.bin"
    )
    written = write_fixture(fixture_path, observations, actions)
    print(
        f"fixture: {observations.shape[0]} cases "
        f"({observations.shape[0] - wanted} driven, {wanted} random), "
        f"{written} bytes -> {fixture_path.relative_to(REPOSITORY)}",
        flush=True,
    )
    print(
        f"         action range [{actions.min():+.4f}, {actions.max():+.4f}], "
        f"{saturated * 100:.0f}% of components at the squash's limit",
        flush=True,
    )
    if saturated > 0.9:
        print(
            "         warning: almost everything is saturated, so this "
            "fixture would pass on a network that only got tanh right",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
