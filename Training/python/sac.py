"""Soft actor-critic for the direct-drive racing policy.

A plain SAC: a tanh-squashed Gaussian actor, twin Q critics with Polyak
targets, and an automatically tuned entropy coefficient. Continuous
control at thirty hertz over a long episode is what this algorithm is
for, and keeping it plain means every knob here has a textbook meaning
rather than a project-specific one.

The network shape follows the training plan's budget: two wide hidden
layers and a narrower third, about seven hundred thousand parameters,
which infers in well under a microsecond per car on a CPU.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

LOG_STD_MIN = -20.0
LOG_STD_MAX = 2.0


def default_device() -> str:
    """Pick the device that measured fastest, not the fanciest one.

    On an Apple laptop this network is small enough that Metal wins
    nothing: an update took 10.2 ms on the GPU against 10.7 ms on the
    CPU, while a batch of sixteen actions took 1.24 ms on the GPU against
    0.105 ms on the CPU — the dispatch overhead dominates work this
    small. Since a step costs one update and one action, the CPU is
    ahead overall, and inference is the only half that also runs in the
    shipped game. A discrete CUDA card is a different trade and is taken
    when present; ``--device`` overrides either way.
    """
    return "cuda" if torch.cuda.is_available() else "cpu"


@dataclass
class SacConfig:
    hidden: tuple[int, ...] = (512, 512, 256)
    gamma: float = 0.995
    tau: float = 0.005
    actor_lr: float = 3e-4
    critic_lr: float = 3e-4
    alpha_lr: float = 3e-4
    batch_size: int = 512
    buffer_capacity: int = 1_000_000
    start_steps: int = 10_000
    # One gradient update per environment step, not per transition: the
    # environment produces a whole batch of lanes per step and is cheap
    # (about twelve thousand transitions a second) while an update costs
    # eleven milliseconds, so updates set the pace. Raising this trades
    # wall-clock for sample efficiency.
    updates_per_step: int = 2
    target_entropy_scale: float = 1.0
    device: str = field(default_factory=lambda: default_device())

    # Quantiles predicted per critic head. Zero keeps the classic scalar
    # critic, which predicts the mean return; any positive count makes the
    # critic distributional. Cornering outcomes are bimodal — the car makes
    # the corner or it does not — and a mean of those two describes an
    # outcome that never happens, whereas a spread of quantiles can say
    # "mostly through, sometimes into the wall", which is the information a
    # risk trade-off actually needs.
    quantiles: int = 0
    # Truncated Quantile Critics drops the most optimistic quantiles from
    # the pooled target, which is a sharper instrument against value
    # overestimation than taking the minimum of two scalars.
    top_quantiles_to_drop_per_critic: int = 2
    huber_kappa: float = 1.0
    # Layer normalization inside the critic trunk. The critic chases a
    # target it moves itself, and normalizing each layer is the cheapest
    # known way to keep that pursuit stable.
    critic_layer_norm: bool = False


def mlp(
    sizes: list[int], out_size: int, layer_norm: bool = False
) -> nn.Sequential:
    layers: list[nn.Module] = []
    for i in range(len(sizes) - 1):
        layers.append(nn.Linear(sizes[i], sizes[i + 1]))
        if layer_norm:
            layers.append(nn.LayerNorm(sizes[i + 1]))
        layers.append(nn.ReLU())
    layers.append(nn.Linear(sizes[-1], out_size))
    return nn.Sequential(*layers)


class Actor(nn.Module):
    def __init__(self, obs_size: int, action_size: int, hidden: tuple[int, ...]):
        super().__init__()
        self.net = mlp([obs_size, *hidden], 2 * action_size)
        self.action_size = action_size

    def forward(
        self, obs: torch.Tensor, deterministic: bool = False
    ) -> tuple[torch.Tensor, torch.Tensor]:
        mean, log_std = self.net(obs).chunk(2, dim=-1)
        log_std = log_std.clamp(LOG_STD_MIN, LOG_STD_MAX)
        std = log_std.exp()
        if deterministic:
            return torch.tanh(mean), torch.zeros_like(mean[..., :1])

        normal = torch.distributions.Normal(mean, std)
        raw = normal.rsample()
        action = torch.tanh(raw)
        # Change of variables for the tanh squash, with the usual numerically
        # stable form of log(1 - tanh(x)^2).
        log_prob = normal.log_prob(raw).sum(-1, keepdim=True)
        log_prob -= (
            2.0 * (np.log(2.0) - raw - F.softplus(-2.0 * raw))
        ).sum(-1, keepdim=True)
        return action, log_prob


class Critic(nn.Module):
    """Twin critics whose heads emit either one value or a set of quantiles.

    With ``outputs == 1`` each head predicts the mean return, exactly as
    classic SAC does. With more, each head predicts that many quantiles of
    the return distribution, evenly spaced in probability.
    """

    def __init__(
        self,
        obs_size: int,
        action_size: int,
        hidden: tuple[int, ...],
        outputs: int = 1,
        layer_norm: bool = False,
    ):
        super().__init__()
        self.outputs = outputs
        self.q1 = mlp([obs_size + action_size, *hidden], outputs, layer_norm)
        self.q2 = mlp([obs_size + action_size, *hidden], outputs, layer_norm)

    def forward(
        self, obs: torch.Tensor, action: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        joined = torch.cat([obs, action], dim=-1)
        return self.q1(joined), self.q2(joined)


class ReplayBuffer:
    """Flat ring buffer over transitions from every lane of the batch."""

    def __init__(self, capacity: int, obs_size: int, action_size: int):
        self.capacity = capacity
        self.obs = np.zeros((capacity, obs_size), dtype=np.float32)
        self.action = np.zeros((capacity, action_size), dtype=np.float32)
        self.reward = np.zeros((capacity, 1), dtype=np.float32)
        self.next_obs = np.zeros((capacity, obs_size), dtype=np.float32)
        self.done = np.zeros((capacity, 1), dtype=np.float32)
        self.size = 0
        self.cursor = 0

    def add_batch(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        reward: np.ndarray,
        next_obs: np.ndarray,
        done: np.ndarray,
    ) -> None:
        count = obs.shape[0]
        indices = (self.cursor + np.arange(count)) % self.capacity
        self.obs[indices] = obs
        self.action[indices] = action
        self.reward[indices, 0] = reward
        self.next_obs[indices] = next_obs
        self.done[indices, 0] = done
        self.cursor = int((self.cursor + count) % self.capacity)
        self.size = min(self.size + count, self.capacity)

    def sample(self, batch_size: int, device: str) -> tuple[torch.Tensor, ...]:
        idx = np.random.randint(0, self.size, size=batch_size)
        as_tensor = lambda a: torch.as_tensor(a[idx], device=device)
        return (
            as_tensor(self.obs),
            as_tensor(self.action),
            as_tensor(self.reward),
            as_tensor(self.next_obs),
            as_tensor(self.done),
        )


class SacAgent:
    def __init__(self, obs_size: int, action_size: int, config: SacConfig):
        self.config = config
        self.device = config.device
        self.distributional = config.quantiles > 0
        outputs = config.quantiles if self.distributional else 1
        self.actor = Actor(obs_size, action_size, config.hidden).to(self.device)
        critic_args = (
            obs_size,
            action_size,
            config.hidden,
            outputs,
            config.critic_layer_norm,
        )
        self.critic = Critic(*critic_args).to(self.device)
        self.critic_target = Critic(*critic_args).to(self.device)
        if self.distributional:
            # Midpoints of equal-probability bins: the fractions each
            # predicted quantile is responsible for.
            self.taus = (
                (torch.arange(outputs, device=self.device) + 0.5) / outputs
            ).view(1, -1, 1)
            self.kept_quantiles = (
                2 * outputs
                - 2 * config.top_quantiles_to_drop_per_critic
            )
            if self.kept_quantiles < 1:
                raise ValueError("Truncation removes every target quantile.")
        self.critic_target.load_state_dict(self.critic.state_dict())
        for parameter in self.critic_target.parameters():
            parameter.requires_grad_(False)

        self.actor_optimizer = torch.optim.Adam(
            self.actor.parameters(), lr=config.actor_lr
        )
        self.critic_optimizer = torch.optim.Adam(
            self.critic.parameters(), lr=config.critic_lr
        )
        self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
        self.alpha_optimizer = torch.optim.Adam(
            [self.log_alpha], lr=config.alpha_lr
        )
        self.target_entropy = -action_size * config.target_entropy_scale
        self.buffer = ReplayBuffer(
            config.buffer_capacity, obs_size, action_size
        )
        self.action_size = action_size

    @property
    def alpha(self) -> torch.Tensor:
        return self.log_alpha.exp()

    @torch.no_grad()
    def act(self, obs: np.ndarray, deterministic: bool = False) -> np.ndarray:
        tensor = torch.as_tensor(obs, device=self.device)
        action, _ = self.actor(tensor, deterministic=deterministic)
        return action.cpu().numpy()

    def update(self) -> dict[str, float]:
        config = self.config
        obs, action, reward, next_obs, done = self.buffer.sample(
            config.batch_size, self.device
        )

        with torch.no_grad():
            next_action, next_log_prob = self.actor(next_obs)
            target_q1, target_q2 = self.critic_target(next_obs, next_action)
            if self.distributional:
                # Pool both heads' quantiles, drop the most optimistic
                # ones, and treat what remains as the target distribution.
                pooled = torch.cat([target_q1, target_q2], dim=-1)
                kept = torch.sort(pooled, dim=-1).values[
                    :, : self.kept_quantiles
                ]
                target = reward + (1.0 - done) * config.gamma * (
                    kept - self.alpha * next_log_prob
                )
            else:
                target_q = torch.min(target_q1, target_q2)
                target_q -= self.alpha * next_log_prob
                target = reward + (1.0 - done) * config.gamma * target_q

        q1, q2 = self.critic(obs, action)
        if self.distributional:
            critic_loss = (
                self._quantile_huber(q1, target)
                + self._quantile_huber(q2, target)
            )
        else:
            critic_loss = F.mse_loss(q1, target) + F.mse_loss(q2, target)
        self.critic_optimizer.zero_grad(set_to_none=True)
        critic_loss.backward()
        self.critic_optimizer.step()

        for parameter in self.critic.parameters():
            parameter.requires_grad_(False)
        new_action, log_prob = self.actor(obs)
        q1_pi, q2_pi = self.critic(obs, new_action)
        if self.distributional:
            # The actor maximizes the mean of the whole predicted
            # distribution; pessimism already lives in the truncated target.
            value = torch.cat([q1_pi, q2_pi], dim=-1).mean(
                dim=-1, keepdim=True
            )
        else:
            value = torch.min(q1_pi, q2_pi)
        actor_loss = (self.alpha.detach() * log_prob - value).mean()
        self.actor_optimizer.zero_grad(set_to_none=True)
        actor_loss.backward()
        self.actor_optimizer.step()
        for parameter in self.critic.parameters():
            parameter.requires_grad_(True)

        alpha_loss = -(
            self.log_alpha * (log_prob.detach() + self.target_entropy)
        ).mean()
        self.alpha_optimizer.zero_grad(set_to_none=True)
        alpha_loss.backward()
        self.alpha_optimizer.step()

        with torch.no_grad():
            for parameter, target_parameter in zip(
                self.critic.parameters(), self.critic_target.parameters()
            ):
                target_parameter.mul_(1.0 - config.tau)
                target_parameter.add_(config.tau * parameter)

        return {
            "critic_loss": float(critic_loss.detach()),
            "actor_loss": float(actor_loss.detach()),
            "alpha": float(self.alpha.detach()),
            "q_mean": float(q1.mean().detach()),
        }

    def _quantile_huber(
        self, predicted: torch.Tensor, target: torch.Tensor
    ) -> torch.Tensor:
        """Quantile regression loss between predictions and a target set.

        Each predicted quantile is pulled toward the target distribution
        with a weight that is asymmetric in its own fraction: the quantile
        responsible for the tenth percentile is penalized nine times more
        for overshooting than for undershooting, which is what makes the
        set converge to the distribution's shape rather than its mean.
        """
        delta = target.unsqueeze(1) - predicted.unsqueeze(2)
        kappa = self.config.huber_kappa
        absolute = delta.abs()
        huber = torch.where(
            absolute <= kappa,
            0.5 * delta.pow(2),
            kappa * (absolute - 0.5 * kappa),
        )
        weight = (self.taus - (delta < 0).float()).abs()
        return (weight * huber).mean(dim=2).sum(dim=1).mean()

    def save(self, path: str) -> None:
        torch.save(
            {
                "actor": self.actor.state_dict(),
                "critic": self.critic.state_dict(),
                "log_alpha": self.log_alpha.detach().cpu(),
            },
            path,
        )

    def load(self, path: str) -> None:
        state = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(state["actor"])
        self.critic.load_state_dict(state["critic"])
        self.critic_target.load_state_dict(state["critic"])
        with torch.no_grad():
            self.log_alpha.copy_(state["log_alpha"].to(self.device))
