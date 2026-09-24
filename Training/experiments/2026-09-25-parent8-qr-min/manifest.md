# parent8a：QR-SAC Actor 取小 + LR 1e-4，从零 1M 步

2026-09-25。在 `era/world-v3`（`51a9751`）上从零点火，不 resume。tag `parent8a`，种子 3。PID 73245，PPID=1。

相对 7 系列仅两处配方变化，一起改，不能拆因果：

1. 默认 QR-SAC 的 Actor 用 `min(mean(Z1), mean(Z2))`，对齐 Sophy 公式 (3)。TQC 支路仍平均。
2. Actor / Critic 学习率 1e-4（命令行传入；优化器实测打印 `lr actor=0.0001 critic=0.0001`）。

alpha **自动**，谷底冻结（不传 `--fixed-alpha`）。其余与 7 系列相同：delta、solo 银石、64 车道、15 Hz、评估每 25k × 6 lane、无宽停、`--steps 1000000`。

    cd /Users/jayhuang/Code/stintegy-evo/.worktrees/steering-detour/Training/python && \
    /Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python -u train.py \
        --solo --delta-actions --track silverstone --batch 64 --seed 3 \
        --tag parent8a --steps 1000000 --eval-every 25000 --eval-batch 6 \
        --actor-lr 0.0001 --critic-lr 0.0001

7 系列检查点全部保留。对照至少包含 7d@550k。本腿未收。
