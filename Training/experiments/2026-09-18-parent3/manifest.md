# parent3：world-v3 上的从零母本

点火：2026-09-18，用户下令。耐心臂军令：不开案、不手术；评估异常先报，不先动。

## 世界指纹

- era/world-v3 @ `0fa98418e1c83ee3a3c62e831fc3a6eb8ab44f86`（封版候选，验收见 `2026-09-18-world-v3-freeze-batch`）。
- 专用工作树 `.worktrees/bake-parent3`，detached 在该提交，点火前工作树无改动；Release 宿主由该提交构建（0 警告）。
  这一炉跑完之前，这个工作树不许改动：宿主是 `dotnet run`，任何源码改动都会重新编译、覆盖运行中的程序。
- 协议 5，观测 457 维。

## 命令行

    cd .worktrees/bake-parent3/Training/python
    nohup /Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python -u train.py \
        --solo --track silverstone --batch 64 --seed 1 --tag parent3 \
        --steps 2000000 --eval-every 25000 --eval-batch 6 > run-parent3.log 2>&1 &

- PID 77763，PPID 1（nohup 孤儿化，应用退出不会带走炉子）。
- 日志：`.worktrees/bake-parent3/Training/python/run-parent3.log`；检查点：同目录 `checkpoints/*parent3*`。
- tag：`parent3`。

## 配方依据

照归档的生产工艺（`2026-09-09-parent/manifest.md`：`--solo --track silverstone --batch 64 --seed 1
--eval-every 25000 --eval-batch 6`），配方完整走：α 自调开局 → 谷底冻结 → 宽停训 → 字典序选材 → 全量存档。
上述各步都是 `train.py` 的默认行为：不给 `--fixed-alpha`，α 自调；回弹超过 2 倍持续 3 个窗口、
或到 75k 步上限时冻结；连续 3 次评估不进步即停。谷底 α 0.0009 是上一代定型段的读数，
从零开局不预先冻结，按正常熵计划走。

`--steps 2000000` 只是上限，何时收工由宽停规则决定（上一代母本 625k 宽停）。

封版批次的配置全部照验收时一样开着（`train.py` 默认）：
- 隐藏随机化课程三件套：限制器强度、感知噪声、轮胎应力标尺；
- 起点随机化，包括二维（比赛进度 × 电量）；
- 预算重核：目标线 Save 15% / Eco 10% / Normal 9%，λ 12,500，塑形 φ′−φ，赛距 194.4 km。
评估保持名义条件。

## 挂号的观察哨（预先写死，免得到时对着数据争论）

- **λ 观察条件**：若把 Normal 开成 Save 样（冲线余量显著高于 9%、白白牺牲圈速），λ 降档重议。
- **功率上限复活条件**：5×5 矩阵的档位单调性若坍塌（Attack 不再比 Save 快），重审上限。
- 两条都等成熟检查点的矩阵和认证读数，早期评估不作判决。
