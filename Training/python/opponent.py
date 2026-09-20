"""Who the ego spars against.

The sparring partner is a checkpoint, frozen: the same actor network the
learner uses, loaded from a file, asked for a deterministic action every
step and never updated. Nothing about it is scripted, and nothing about it
learns -- which is what makes a duel's difficulty a thing the pipeline
chooses rather than a thing that drifts while training runs.

The pool is a stub, and deliberately marked as one. Self-play's usual shape
is to draw the partner from the last K checkpoints rather than from the
single newest, so the ego does not learn to beat one opponent and call it
racecraft; sampling that pool, and deciding how often to re-draw it, is a
training-design question the arena is not being asked to settle yet. What
is here is the seam: the pool is listed, the draw is made, and the only
policy is "the newest".
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from sac import SacAgent, SacConfig


class FrozenOpponent:
    """A checkpoint at the wheel of the second car."""

    def __init__(
        self,
        obs_size: int,
        action_size: int,
        config: SacConfig,
        checkpoint: str | Path,
    ) -> None:
        self.agent = SacAgent(obs_size, action_size, config)
        restored = self.agent.load(str(checkpoint))
        self.checkpoint = str(checkpoint)
        self.step = int(restored["step"])
        # Nothing here trains, so the networks sit in eval mode and the
        # buffer is never filled. act() is already under no_grad.
        self.agent.actor.eval()
        self.agent.critic.eval()

    def act(self, obs: np.ndarray) -> np.ndarray:
        """Deterministic: a sparring partner is a fixed opponent, not a
        source of exploration noise for somebody else's policy."""
        return self.agent.act(obs, deterministic=True)

    def describe(self) -> str:
        return f"{Path(self.checkpoint).name} @ step {self.step}"


def pool_candidates(
    checkpoint_dir: str | Path, tag: str, latest: int
) -> list[Path]:
    """The last ``latest`` evaluation checkpoints of a tag, newest first.

    STUB. This lists the pool; nothing yet draws from it per episode, which
    is the part that makes self-play self-play. See the module docstring.
    """
    directory = Path(checkpoint_dir)
    found = sorted(
        directory.glob(f"eval{tag}-*.pt"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    return found[:latest]


def draw_from_pool(
    checkpoint_dir: str | Path, tag: str, latest: int
) -> Path | None:
    """One partner out of the pool. STUB: always the newest."""
    pool = pool_candidates(checkpoint_dir, tag, latest)
    return pool[0] if pool else None
