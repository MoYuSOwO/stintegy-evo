# parent3b：world-v3 上的从零母本（重点火）

点火：2026-09-19 02:42 CST，用户下令（"重新开始，这没法用了"）。耐心臂军令：不开案、不手术；评估异常先报，不先动。

## 为什么重点火

parent3 的世界（`0fa9841`）把预算目标线画成一条从发车满电到终点的绝对线。约六成回合一开局就在线下，
线下每少耗一点电都有奖励，一米最多为推进奖励的 3 倍，教 Normal 自动还债。用户约在 parent3 150k 步时发现，
parent3 于约 208k 步熄火废弃（见 `2026-09-18-parent3/manifest.md` 结局一节）。

修复后的语义（设计底稿三之勘误）：**每档是一个固定斜率，锚点只决定截距。** 斜率 = 全程开这一档恰好剩下
该档冲线余量的耗电速率（Normal 0.91 / 赛程，Eco 0.90，Save 0.85；Push、Attack 无线）。锚点是给出指令时的
（进度，电量）：回合开始时锚在起点，中途切档时锚在当时的状态。同一档永远同一种开法；切档不自动还债。
这一点用户专门核验过：代码里斜率只由档位决定（`EnergyBudget.Slope`），锚点只提供截距（`EnergyBudget.Anchor`）。

## 世界指纹

- 世界 `8720958a877cfdd9d04ad65a571c5f0b5c12c8da`（目标线锚定修复）。
- 点火提交 era/world-v3 @ `5d5ef16e0a02e12b89eea12326feafeaa129f2bc`：比 `8720958` 只多一个文档提交，
  `Core/`、宿主和 `Training/python/` 与之逐字相同（已 diff 核对）。
- 专用工作树 `.worktrees/bake-parent3b`，detached 在 `5d5ef16`，点火前无改动；Release 宿主由它构建（0 警告）。
  这一炉跑完前，这个工作树不许动。
- 协议 5，观测 457 维。

## 命令行（与 parent3 相同配方、相同参数）

    cd .worktrees/bake-parent3b/Training/python
    nohup /Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python -u train.py \
        --solo --track silverstone --batch 64 --seed 1 --tag parent3b \
        --steps 2000000 --eval-every 25000 --eval-batch 6 > run-parent3b.log 2>&1 &

- PID 81333，PPID 1（已核实）。
- 日志：`.worktrees/bake-parent3b/Training/python/run-parent3b.log`；检查点：同目录 `checkpoints/*parent3b*`。
- 首行：obs=457、lanes=64、alpha=auto、gamma=0.9931；1000 步时 1828 tps，α 0.609。

## 配方依据

与 parent3 相同（见 `2026-09-18-parent3/manifest.md` 配方一节）：归档生产工艺，α 自调开局 → 谷底冻结 → 宽停训；
隐藏课程三件套、二维起点、预算重核默认开启；评估保持名义条件。

## 观察哨（沿用，预先写死）

- **λ 观察条件**：若把 Normal 开成 Save 样（冲线余量显著高于 9%、白白牺牲圈速），λ 降档重议。
- **功率上限复活条件**：5×5 矩阵的档位单调性若坍塌（Attack 不再比 Save 快），重审上限。
- **新增：预算项量级**。锚定线下，预算项只对本段开法相对档位斜率的偏离计价，心跳里应在零附近、远小于推进项。
  若它再次稳定地占到推进项的相当份额，先报不先动。
