"""Turning one-step transitions into n-step ones, lane by lane.

A one-step return tells a state only what happened in the next tenth of a
second and leaves the rest to a critic that is itself still learning. An
n-step return carries the actual reward of the next n tenths before handing
over, so a corner that goes wrong reaches the decision that caused it in one
backup instead of seven. Sony used seven steps.

Each lane keeps its own queue because lanes reset independently: when one
ends its episode, whatever is still in its queue is flushed with the
horizon it actually got, and nothing is carried across the boundary.
"""
from collections import deque

import numpy as np


class NStepBatcher:
    """Buffers per-lane transitions and emits n-step ones."""

    def __init__(self, lanes: int, n: int, gamma: float):
        if n < 1:
            raise ValueError("n-step needs at least one step")
        self.n = n
        self.gamma = gamma
        self._queues = [deque() for _ in range(lanes)]
        self._discounts = np.array(
            [gamma ** k for k in range(n)], dtype=np.float64
        )

    def add(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        reward: np.ndarray,
        next_obs: np.ndarray,
        done: np.ndarray,
    ) -> tuple[np.ndarray, ...] | None:
        """Take one step of every lane; return whatever is ready to store.

        Returns (obs, action, return, bootstrap_obs, done, horizon) or None
        when no lane has a complete window yet. `horizon` is how many steps
        each emitted return actually spans, which the discount on the
        bootstrap has to match — a flushed tail spans fewer than n.
        """
        out_obs, out_action, out_return = [], [], []
        out_next, out_done, out_horizon = [], [], []

        for lane, queue in enumerate(self._queues):
            queue.append((
                obs[lane].copy(),
                action[lane].copy(),
                float(reward[lane]),
                next_obs[lane].copy(),
                bool(done[lane]),
            ))

            if bool(done[lane]):
                # The episode is over, so every window still open ends here
                # with the horizon it managed and the terminal flag set.
                while queue:
                    self._emit(
                        queue, out_obs, out_action, out_return,
                        out_next, out_done, out_horizon,
                    )
                    queue.popleft()
                continue

            if len(queue) >= self.n:
                self._emit(
                    queue, out_obs, out_action, out_return,
                    out_next, out_done, out_horizon,
                )
                queue.popleft()

        if not out_obs:
            return None
        return (
            np.asarray(out_obs, dtype=np.float32),
            np.asarray(out_action, dtype=np.float32),
            np.asarray(out_return, dtype=np.float32),
            np.asarray(out_next, dtype=np.float32),
            np.asarray(out_done, dtype=np.float32),
            np.asarray(out_horizon, dtype=np.float32),
        )

    def _emit(self, queue, out_obs, out_action, out_return,
              out_next, out_done, out_horizon) -> None:
        horizon = min(len(queue), self.n)
        rewards = np.fromiter(
            (queue[k][2] for k in range(horizon)), dtype=np.float64,
            count=horizon,
        )
        out_obs.append(queue[0][0])
        out_action.append(queue[0][1])
        out_return.append(float((rewards * self._discounts[:horizon]).sum()))
        out_next.append(queue[horizon - 1][3])
        out_done.append(1.0 if queue[horizon - 1][4] else 0.0)
        out_horizon.append(float(horizon))

    def clear(self) -> None:
        for queue in self._queues:
            queue.clear()
