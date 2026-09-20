"""A fixed solo rollout through the host, hashed. Argument: a Training/python dir."""
import hashlib, sys
import numpy as np
sys.path.insert(0, sys.argv[1])
from host_env import HostEnv

digest = hashlib.sha256()
with HostEnv(batch=4, seed_base=42, solo=True, track="silverstone",
             episode_seconds=60.0, randomise_episode_start=True,
             hidden_curriculum=True) as env:
    obs = env.reset()
    digest.update(np.ascontiguousarray(obs, dtype="<f4").tobytes())
    for step in range(150):
        a = np.stack([
            np.sin(np.arange(4) + step * 0.11) * 0.8,
            np.cos(np.arange(4) + step * 0.07),
        ], axis=1).astype(np.float32)
        obs, reward, done, reason, comp, race, final, spins = env.step(a)
        for part in (obs, reward, comp, race.astype("<f4")):
            digest.update(np.ascontiguousarray(part, dtype="<f4").tobytes())
        digest.update(done.astype(np.uint8).tobytes())
        digest.update(reason.astype(np.uint8).tobytes())
        digest.update(spins.astype(np.uint8).tobytes())
print(digest.hexdigest())
