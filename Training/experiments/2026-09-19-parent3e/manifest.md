# parent3e：母本链第五棒

点火：2026-09-19 22:1x CST。按链式续炉长令（见 `2026-09-19-parent3d/manifest.md`）直接接棒，不请示。

## 续训起点

- `latestparent3d.pt`，step 975000，sha256 `dfbfd07024614ff06123fe0602268eaf3cfe5e338e8f16f3c618180e7b2d2b8e`。
- parent3d 于 975k 宽停（三评双平）。完整续训：optimizers yes，rng yes，replay no（日志已确认）。

## 世界与参数

世界指纹 `8720958`，工作树 `.worktrees/bake-parent3b`（`5d5ef16`，无改动）。同配方：`--fixed-alpha 0.0011 --steps 10000000`。

    nohup /Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python -u train.py \
        --solo --track silverstone --batch 64 --seed 1 --tag parent3e \
        --steps 10000000 --eval-every 25000 --eval-batch 6 \
        --fixed-alpha 0.0011 --resume checkpoints/latestparent3d.pt > run-parent3e.log 2>&1 &

- PID 28119，PPID 1（已核实）。日志 `.worktrees/bake-parent3b/Training/python/run-parent3e.log`。
- 首评（1000k）带续训疤，属预期。

## 链级账本（best 字典序 = (零旋转与否, 干净档, −配速)）

| 棒 | 区间 | 本棒 best | 键 | 链级进账 |
|---|---|---|---|---|
| 3b | 0–375k | （见 3b 日志） | — | 起点 |
| 3c | 375k–650k | 旋转 0，干净档有，1:49.725 | (1, 1, −109.725) | 进账（链上 best） |
| 3d | 650k–975k | 旋转 4，干净档过半，1:45.478 | (0, 2, −105.478) | **零进账**（第 1 棒） |
| 3e | 975k– | 进行中 | | |

终止条件之一是**连续两棒零进账**：若 3e 仍未超过 3c 的键，先报协调方（按长令照点下一棒，但先报）。
