# parent3f：母本链第六棒

点火：2026-09-20 00:2x CST。按链式续炉长令（见 `2026-09-19-parent3d/manifest.md`）直接接棒。
**连续两棒零进账（3d、3e）已先报协调方**；长令写明不自动停，故照点。

## 续训起点

- `latestparent3e.pt`，step 1175000，sha256 `1a49ab324e30350f805dd5fc2c1afd828ea4f9c25141fb07343fa2d3c14fe23c`。
- parent3e 于 1175k 宽停（三评双平）。完整续训：optimizers yes，rng yes，replay no（日志已确认）。

## 世界与参数

世界指纹 `8720958`，工作树 `.worktrees/bake-parent3b`（`5d5ef16`，无改动）。同配方：

    nohup /Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python -u train.py \
        --solo --track silverstone --batch 64 --seed 1 --tag parent3f \
        --steps 10000000 --eval-every 25000 --eval-batch 6 \
        --fixed-alpha 0.0011 --resume checkpoints/latestparent3e.pt > run-parent3f.log 2>&1 &

- PID 30872，PPID 1（已核实）。日志 `.worktrees/bake-parent3b/Training/python/run-parent3f.log`。

## 链级账本（best 字典序 = (零旋转与否, 干净档, −配速)）

| 棒 | 区间 | 本棒 best | 键 | 链级进账 |
|---|---|---|---|---|
| 3b | 0–375k | （见 3b 日志） | — | 起点 |
| 3c | 375k–650k | 旋转 0，干净档有，1:49.725 | (1, 1, −109.725) | 进账（链上 best） |
| 3d | 650k–975k | 旋转 4，干净档过半，1:45.478 | (0, 2, −105.478) | 零进账（第 1 棒） |
| 3e | 975k–1175k | 旋转 5，干净档有（8/26），1:45.968 | (0, 1, −105.968) | 零进账（第 2 棒） |
| 3f | 1175k– | 进行中 | | |

3e 八评旋转依次 13/9/6/4/5/1/6/2，无一归零；配速 1:44.9–1:49.4，干净圈 0–8/26。
