"""Actor Q aggregation: QR-SAC min-of-means, TQC average, scalar min."""

import torch

from sac import actor_q_value


def test_qr_takes_min_of_means() -> None:
    q1 = torch.full((2, 32), 9.0)
    q2 = torch.full((2, 32), 3.0)
    value = actor_q_value(q1, q2, distributional=True, drop_top=0)
    assert value.shape == (2, 1)
    assert torch.allclose(value, torch.full((2, 1), 3.0))


def test_tqc_averages_means() -> None:
    q1 = torch.full((2, 32), 9.0)
    q2 = torch.full((2, 32), 3.0)
    value = actor_q_value(q1, q2, distributional=True, drop_top=1)
    assert torch.allclose(value, torch.full((2, 1), 6.0))


def test_qr_does_not_min_quantilewise() -> None:
    q1 = torch.tensor([[0.0, 10.0]])
    q2 = torch.tensor([[4.0, 4.0]])
    value = actor_q_value(q1, q2, distributional=True, drop_top=0)
    # min(mean 5, mean 4) = 4, not mean(min pairwise) = mean(0, 4) = 2.
    assert torch.allclose(value, torch.tensor([[4.0]]))


def test_scalar_min() -> None:
    q1 = torch.tensor([[9.0]])
    q2 = torch.tensor([[3.0]])
    value = actor_q_value(q1, q2, distributional=False, drop_top=0)
    assert torch.allclose(value, torch.tensor([[3.0]]))


if __name__ == "__main__":
    test_qr_takes_min_of_means()
    test_tqc_averages_means()
    test_qr_does_not_min_quantilewise()
    test_scalar_min()
    print("ok")
