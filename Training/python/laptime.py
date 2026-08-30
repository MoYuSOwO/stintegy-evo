import numpy as np
from host_env import COMPONENT_NAMES, HostEnv
from sac import SacAgent, SacConfig

PROG = COMPONENT_NAMES.index("own_progress")
OFF = COMPONENT_NAMES.index("off_course")
LENGTH = {"silverstone": 5891, "shanghai": 5451, "zandvoort": 4259,
          "simple-right": 1804, "simple-left": 1804, "sepang": 5543,
          "monaco": 3337, "daytona": 4016, "speedway": 8512}
ANALYTIC = {"silverstone": 101.974, "shanghai": 103.729, "zandvoort": 83.241,
            "simple-right": 36.160, "simple-left": 36.160, "sepang": 105.714,
            "monaco": 80.857, "daytona": 54.572, "speedway": 110.216}
TRAIN = {"silverstone", "shanghai", "zandvoort", "simple-right", "simple-left"}

def fmt(t):
    return f"{int(t//60)}:{t%60:06.3f}" if t < 3600 else "  —"

agent = None
print(f"{'':2}{'赛道':<14}{'学到的圈速':>12}{'解析司机':>11}{'差':>9}{'出界':>8}")
for track in ["silverstone", "shanghai", "zandvoort", "simple-right",
              "simple-left", "sepang", "daytona", "speedway", "monaco"]:
    with HostEnv(batch=8, seed_base=900_000, solo=True, track=track,
                 quiet=True) as env:
        if agent is None:
            agent = SacAgent(env.obs_size, env.action_size, SacConfig())
            agent.load("checkpoints/latestv3.pt")
        obs = env.reset()
        # 4000 步 = 400 秒，丢掉起步的前 600 步
        dist = np.zeros(8); off = np.zeros(8); warm = None
        speeds = []
        for i in range(4000):
            obs, r, done, reason, comp = env.step(agent.act(obs, deterministic=True))
            dist += comp[PROG]; off += comp[OFF]
            speeds.append(obs[:, 232] * 100.0)
            if i == 599:
                warm = dist.copy()
    steady = (dist - warm).mean() / 0.02      # 米
    lap = LENGTH[track] * 340.0 / max(steady, 1.0)
    a = ANALYTIC[track]
    v2 = (np.array(speeds) ** 2).mean()
    off_sec = -off.mean() / (1e-3 * v2) if v2 > 0 else 0.0
    tag = "训" if track in TRAIN else "留"
    print(f"{tag} {track:<14}{fmt(lap):>12}{fmt(a):>11}"
          f"{lap-a:>+8.1f}s{off_sec:>7.1f}s")
