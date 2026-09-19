# parent3c：parent3b 的续炉，不设上限

点火：2026-09-19 14:28 CST。用户令，原话："续 不设上限"。耐心臂军令照旧：不开案、不手术；评估异常先报，不先动。

## 续训起点

- `latestparent3b.pt`，step 375000，sha256 `a59fadb194be3ccc09c0462150bd7487984ef92ffe7b1fe3cd2707ae72d1d08a`。
- parent3b 已善终：375k 时"三评双平"触发宽停（`training stopped at step 375000: neither the clean criterion nor
  the charged mean improved for 3 evaluations`）。
- 完整续训：优化器、随机数状态恢复；回放池不恢复，按既有语义新起（日志：`optimizers yes, rng yes, replay no`）。

## 世界

指纹 `8720958`，与 parent3b 同一个世界。复用专用工作树 `.worktrees/bake-parent3b`（仍 detached 在 `5d5ef16`，
点火前无改动；parent3b 已结束，没有冲突）。协议 5，观测 457 维。

## α 依据：固定在它自己的谷底 0.0011

parent3b 的 α 自调在约早期回弹 2 倍持续 3 个窗口时冻结于 **0.0011**（日志：`alpha frozen at 0.0011`）。
续炉用 `--fixed-alpha 0.0011` 钉在这个谷底，不让续训重新自调。

记下的教训（协调方记录）：老母本 625k 续炉时让 α 自调重来，结果冻在 0.0017，多交熵税；
2f 那炉固定在谷底续训则出了纪录，对比 4:0。所以续炉一律固定谷底。
（`sac.py`：设了 `fixed_alpha` 时 `alpha` 属性直接返回固定值，续训加载的 `log_alpha` 不覆盖它，已核对代码。）

## 不设上限，宽停收工

用户令"不设上限"：`--steps 10000000` 只是一个实际到不了的数。何时收工只由宽停判据决定：
连续 3 次评估，干净判据与计罚均速都没有进步（三评双平）。

## 命令行（其余参数与 parent3b 一致）

    cd .worktrees/bake-parent3b/Training/python
    nohup /Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python -u train.py \
        --solo --track silverstone --batch 64 --seed 1 --tag parent3c \
        --steps 10000000 --eval-every 25000 --eval-batch 6 \
        --fixed-alpha 0.0011 --resume checkpoints/latestparent3b.pt > run-parent3c.log 2>&1 &

- PID 19438，PPID 1（已核实）。
- 日志：`.worktrees/bake-parent3b/Training/python/run-parent3c.log`；检查点：同目录 `checkpoints/*parent3c*`。
- 首行：alpha=0.0011，obs=457，lanes=64；续上后 376k 步时 2117 tps。

## 读评估的预先说明（防误读）

**首评（400k）会带续训疤，属预期**：回放池是新起的，最初几万步的数据分布和旧池不同。历来从第二评起才见真章，
首评读数不作判决。
