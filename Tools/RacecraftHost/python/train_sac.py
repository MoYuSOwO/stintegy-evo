"""Soft actor-critic for the racecraft layer, on the host's duels.

One host process runs ``--batch`` duels on one circuit; every decision of
every duel goes into one replay buffer, and the networks learn from it a
few batches per decision round. Episodes that run out of time are
bootstrapped from the row they ended on; a spin ends one for good.

Writes ``<run>/log.csv`` (one row per report) and ``<run>/latest.pt``.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from racecraft_env import COMPONENTS, RacecraftEnv

LOG_STD_MIN, LOG_STD_MAX = -5.0, 1.0


def mlp(inputs: int, outputs: int, hidden: int = 256, layers: int = 3) -> nn.Sequential:
    parts: list[nn.Module] = []
    size = inputs
    for _ in range(layers):
        parts += [nn.Linear(size, hidden), nn.ReLU()]
        size = hidden
    parts.append(nn.Linear(size, outputs))
    return nn.Sequential(*parts)


class Actor(nn.Module):
    def __init__(self, obs: int, act: int) -> None:
        super().__init__()
        self.body = mlp(obs, 2 * act)
        self.act = act

    def forward(self, obs: torch.Tensor, deterministic: bool = False):
        mean, log_std = self.body(obs).split(self.act, dim=-1)
        log_std = log_std.clamp(LOG_STD_MIN, LOG_STD_MAX)
        std = log_std.exp()
        raw = mean if deterministic else mean + std * torch.randn_like(mean)
        action = torch.tanh(raw)
        log_prob = (-0.5 * ((raw - mean) / std) ** 2 - log_std - 0.5 * math.log(2 * math.pi)).sum(-1)
        log_prob -= (2 * (math.log(2) - raw - F.softplus(-2 * raw))).sum(-1)
        return action, log_prob


class Critic(nn.Module):
    def __init__(self, obs: int, act: int) -> None:
        super().__init__()
        self.q1 = mlp(obs + act, 1)
        self.q2 = mlp(obs + act, 1)

    def forward(self, obs: torch.Tensor, act: torch.Tensor):
        x = torch.cat([obs, act], dim=-1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)


class Replay:
    def __init__(self, capacity: int, obs: int, act: int) -> None:
        self.obs = np.zeros((capacity, obs), np.float32)
        self.next = np.zeros((capacity, obs), np.float32)
        self.act = np.zeros((capacity, act), np.float32)
        self.rew = np.zeros(capacity, np.float32)
        self.done = np.zeros(capacity, np.float32)
        self.capacity, self.size, self.at = capacity, 0, 0

    def add(self, obs, act, rew, nxt, done) -> None:
        n = len(obs)
        idx = (self.at + np.arange(n)) % self.capacity
        self.obs[idx], self.act[idx], self.rew[idx], self.next[idx], self.done[idx] = obs, act, rew, nxt, done
        self.at = (self.at + n) % self.capacity
        self.size = min(self.size + n, self.capacity)

    def sample(self, n: int, device):
        idx = np.random.randint(0, self.size, n)
        t = lambda a: torch.as_tensor(a[idx], device=device)
        return t(self.obs), t(self.act), t(self.rew), t(self.next), t(self.done)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--run", required=True)
    p.add_argument("--minutes", type=float, default=5)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--circuit", default="silverstone")
    p.add_argument("--gamma", type=float, default=0.98)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--updates-per-round", type=int, default=8)
    p.add_argument("--minibatch", type=int, default=256)
    p.add_argument("--warmup", type=int, default=5000, help="decisions taken at random before learning")
    p.add_argument("--report-seconds", type=float, default=60)
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    run = Path(args.run)
    run.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = torch.device("cpu")
    torch.set_num_threads(4)

    env = RacecraftEnv(batch=args.batch, circuit=args.circuit)
    O, A, B = env.obs_size, env.action_size, env.batch
    actor, critic = Actor(O, A).to(device), Critic(O, A).to(device)
    target = Critic(O, A).to(device)
    target.load_state_dict(critic.state_dict())
    log_alpha = torch.zeros((), requires_grad=True, device=device)
    target_entropy = -float(A)
    opt_actor = torch.optim.Adam(actor.parameters(), lr=args.lr)
    opt_critic = torch.optim.Adam(critic.parameters(), lr=args.lr)
    opt_alpha = torch.optim.Adam([log_alpha], lr=args.lr)
    replay = Replay(1_000_000, O, A)

    obs = env.reset(seed=args.seed)
    episode_reward = np.zeros(B)
    episode_parts = np.zeros((B, len(COMPONENTS)))
    finished: list[tuple[float, np.ndarray]] = []
    decisions, updates = 0, 0
    start = last_report = time.perf_counter()
    losses = {"q": 0.0, "pi": 0.0, "alpha": 0.0, "n": 0}

    log_path = run / "log.csv"
    with open(log_path, "w", newline="") as f:
        csv.writer(f).writerow(["minutes", "decisions", "updates", "episodes", "reward", *COMPONENTS,
                                "alpha", "q_loss", "pi_loss", "decisions_per_s"])

    while time.perf_counter() - start < args.minutes * 60:
        if decisions < args.warmup:
            action = np.random.uniform(-1, 1, (B, A)).astype(np.float32)
        else:
            with torch.no_grad():
                action, _ = actor(torch.as_tensor(obs, device=device))
                action = action.cpu().numpy()
        nxt, final, rewards, ended, out_of_time, parts = env.step(action)
        # The row a duel ended on is its next state; a spin ends it for good,
        # running out of time does not.
        target_obs = np.where(ended[:, None], final, nxt)
        terminal = (ended & ~out_of_time).astype(np.float32)
        replay.add(obs, action, rewards, target_obs, terminal)
        episode_reward += rewards
        episode_parts += parts
        for i in np.flatnonzero(ended):
            finished.append((episode_reward[i], episode_parts[i].copy()))
            episode_reward[i] = 0
            episode_parts[i] = 0
        obs = nxt
        decisions += B

        if decisions >= args.warmup and replay.size >= args.minibatch:
            for _ in range(args.updates_per_round):
                o, a, r, n, d = replay.sample(args.minibatch, device)
                alpha = log_alpha.exp().detach()
                with torch.no_grad():
                    na, nlp = actor(n)
                    tq1, tq2 = target(n, na)
                    y = r + args.gamma * (1 - d) * (torch.min(tq1, tq2) - alpha * nlp)
                q1, q2 = critic(o, a)
                q_loss = F.mse_loss(q1, y) + F.mse_loss(q2, y)
                opt_critic.zero_grad()
                q_loss.backward()
                opt_critic.step()

                pa, lp = actor(o)
                pq1, pq2 = critic(o, pa)
                pi_loss = (alpha * lp - torch.min(pq1, pq2)).mean()
                opt_actor.zero_grad()
                pi_loss.backward()
                opt_actor.step()

                alpha_loss = -(log_alpha * (lp.detach() + target_entropy)).mean()
                opt_alpha.zero_grad()
                alpha_loss.backward()
                opt_alpha.step()

                with torch.no_grad():
                    for tp, cp in zip(target.parameters(), critic.parameters()):
                        tp.mul_(0.995).add_(0.005 * cp)
                updates += 1
                losses["q"] += q_loss.item()
                losses["pi"] += pi_loss.item()
                losses["n"] += 1

        now = time.perf_counter()
        if now - last_report >= args.report_seconds:
            minutes = (now - start) / 60
            if finished:
                rew = np.mean([e[0] for e in finished])
                comp = np.mean([e[1] for e in finished], axis=0)
            else:
                rew, comp = float("nan"), np.full(len(COMPONENTS), np.nan)
            n = max(1, losses["n"])
            row = [f"{minutes:.1f}", decisions, updates, len(finished), f"{rew:.2f}",
                   *[f"{c:.3f}" for c in comp], f"{log_alpha.exp().item():.4f}",
                   f"{losses['q'] / n:.4f}", f"{losses['pi'] / n:.4f}",
                   f"{decisions / (now - start):.0f}"]
            with open(log_path, "a", newline="") as f:
                csv.writer(f).writerow(row)
            print(" ".join(str(x) for x in row), flush=True)
            torch.save({"actor": actor.state_dict(), "critic": critic.state_dict(),
                        "obs_size": O, "action_size": A, "decisions": decisions}, run / "latest.pt")
            finished.clear()
            losses = {"q": 0.0, "pi": 0.0, "alpha": 0.0, "n": 0}
            last_report = now

    torch.save({"actor": actor.state_dict(), "critic": critic.state_dict(),
                "obs_size": O, "action_size": A, "decisions": decisions}, run / "latest.pt")
    env.close()


if __name__ == "__main__":
    main()
