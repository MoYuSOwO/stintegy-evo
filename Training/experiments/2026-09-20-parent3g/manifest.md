# parent3g：母本链第七棒

按链式续炉长令（`2026-09-19-parent3d/manifest.md`）与 3f 预登记判据（`2026-09-20-parent3f/manifest.md`）接棒：3f 三线全破，链照常续。

## 3f 判决（预登记判据逐条，对日志核实）

| 判据 | 门槛 | 3f 实况 | 结果 |
|---|---|---|---|
| 计罚配速 | < 1:43.717 | 1:43.327（1425k） | 破 |
| 干净率 | > 18/25（0.72） | 19/26 = 0.731（1425k） | 破 |
| 零旋转且计罚 | < 1:49.725 | 1225k：旋转 0，计罚 1:46.002，干净均速 1:46.854，干净 18/26 | 破 |

3f 于 1500k 宽停（三评双平）。本棒 best = 1225k：(1, 2, −106.854)，**超过 3c 的 (1, 1, −109.725)，为新链上 best**（`bestparent3f.pt`）。

观察（与 3f manifest 的选材键观察呼应，只登记）：1425k 是本棒三线最好的一评（计罚 1:43.327，干净 19/26），却因旋转 1 次未被存为 best——一票否决把它排在 1225k 之后。

## 续训起点

- `latestparent3f.pt`，step 1500000，sha256 `a85ae3ffc90019244f28f75573135c52573d58be358b4d36ded22f18439a4993`。
- 完整续训：optimizers yes，rng yes，replay no（日志已确认）。

## 世界与参数

世界指纹 `8720958`，工作树 `.worktrees/bake-parent3b`（`5d5ef16`，无改动）。同配方：

    nohup /Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python -u train.py \
        --solo --track silverstone --batch 64 --seed 1 --tag parent3g \
        --steps 10000000 --eval-every 25000 --eval-batch 6 \
        --fixed-alpha 0.0011 --resume checkpoints/latestparent3f.pt > run-parent3g.log 2>&1 &

- PID 34831，PPID 1（已核实）。

## 链级账本

| 棒 | 区间 | 本棒 best（二值键） | 三线记账 |
|---|---|---|---|
| 3c | 375k–650k | (1, 1, −109.725) | 零旋转 1:49.725 |
| 3d | 650k–975k | (0, 2, −105.478) | 破两线：计罚 1:43.717，干净 18/25 |
| 3e | 975k–1175k | (0, 1, −105.968) | 三线全平（第一根平棒） |
| 3f | 1175k–1500k | **(1, 2, −106.854)** 链上 best | 三线全破：计罚 1:43.327，干净 19/26，零旋转 1:46.854 |
| 3g | 1500k– | 进行中 | |
