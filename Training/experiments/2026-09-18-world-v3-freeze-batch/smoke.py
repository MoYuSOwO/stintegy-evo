"""Rule-based smoke for the world-v3 freeze candidate.

No checkpoint can drive the 457-channel contract, and none needs to: a
fresh parent starts from zero. What has to hold is that the world and its
observation are sane under driving nobody learned. For every circuit,
with the hidden curriculum off and on, twelve lanes run 90 simulated
seconds under two rule policies -- zero input (coasting) and a constant
command -- and every step passes the Python layout self-check (width,
heading identity, absence written on capacity, every channel finite and
O(1)) and every reward component stays finite.
"""

from __future__ import annotations

import json
import sys

import numpy as np

sys.path.insert(0, sys.argv[1])
from host_env import COMPONENT_NAMES, HostEnv  # noqa: E402
from train import TRACKS, assert_observation_layout, BUDGET_DEVIATION  # noqa: E402

POLICIES = {"coast": (0.0, 0.0), "constant": (0.03, 0.5)}
rows = []
channel_max = np.zeros(457)
for track in TRACKS:
    for curriculum in (False, True):
        for name, (curvature, accel) in POLICIES.items():
            with HostEnv(batch=12, seed_base=4242, solo=True, track=track,
                         episode_seconds=90, randomise_episode_start=curriculum,
                         hidden_curriculum=curriculum) as env:
                obs = env.reset()
                assert_observation_layout(obs)
                largest = float(np.abs(obs).max())
                comp_total = np.zeros(len(COMPONENT_NAMES))
                reasons = {}
                deviations = [obs[:, BUDGET_DEVIATION].copy()]
                for _ in range(int(90 * 15)):
                    action = np.tile(np.array([curvature, accel], np.float32), (12, 1))
                    out = env.step(action)
                    obs = out[0]
                    assert_observation_layout(obs)
                    assert_observation_layout(out[6])
                    components = np.asarray(out[4])
                    assert np.all(np.isfinite(components)), "non-finite reward component"
                    comp_total += components.sum(axis=1)
                    largest = max(largest, float(np.abs(obs).max()))
                    channel_max = np.maximum(channel_max, np.abs(obs).max(axis=0))
                    channel_max = np.maximum(channel_max, np.abs(out[6]).max(axis=0))
                    for lane in np.nonzero(out[2])[0]:
                        r = str(out[3][lane])
                        reasons[r] = reasons.get(r, 0) + 1
                    deviations.append(obs[:, BUDGET_DEVIATION].copy())
            d = np.concatenate(deviations)
            row = {"track": track, "curriculum": curriculum, "policy": name,
                   "max_abs_obs": round(largest, 3),
                   "budget_deviation_range": [round(float(d.min()), 3), round(float(d.max()), 3)],
                   "terminals": reasons,
                   "components": {n: round(float(v), 2) for n, v in zip(COMPONENT_NAMES, comp_total) if v}}
            rows.append(row)
            print(json.dumps(row), flush=True)
json.dump({"runs": rows, "channel_max_abs": [round(float(x), 4) for x in channel_max]},
          open(sys.argv[2], "w"), indent=1)
top = np.argsort(channel_max)[::-1][:15]
print("largest channels:", [(int(c), round(float(channel_max[c]), 3)) for c in top], flush=True)
print("SMOKE PASS", flush=True)
