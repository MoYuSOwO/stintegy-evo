# parent3d：母本链第四棒，链式续炉长令

点火：2026-09-19 17:18 CST。

## 链式续炉长令（用户）

用户原话："怎么又停？我都说不停了。" 由协调方转达为长期命令：

- **母本炉链不停**。宽停只是换棒，不是收工。
- 本链任何一棒宽停，执行方直接点下一棒（3e、3f……），不再请示；每棒 manifest 引用本长令。
- 保留宽停判据本身（三评双平即换棒）。理由：每次换棒都会重起回放池，新回放正是每一棒买到进度的机制
  （parent3c 实测净赚约 1.4 秒）。这比单炉关掉判据一直干烤更有效。若用户另有指示再改。
- 链只有两种终止：用户喊停；或**连续两棒零进账**（best 的字典序完全没动）。后者出现时先报协调方，不自动停。

## 续训起点

- `latestparent3c.pt`，step 650000，sha256 `835cf57e6c1d28038049e9cba64a73c918724b1c68fdd19e1cd324c229a5cfa2`。
- parent3c 于 650k 宽停（三评双平）。它这一棒的 best 演进（日志 `saved best`）：
  1:47.747（旋转 5）→ 1:47.011（旋转 10）→ 1:45.030（旋转 4）→ 1:49.725（旋转 0）。
  选材判据（`train.clean_criterion_key`）是字典序 (零旋转与否, 干净档, −配速)：第一位只看旋转是否为 0，
  不看旋转次数。所以最后一条虽然更慢，因为零旋转而更优。（初稿写成"旋转 → 干净 → 均速"，不准确，已更正。）

## 结局（2026-09-19 22:1x 补记）

975k 宽停（三评双平）。本棒 best 最终为 旋转 4、干净档过半、均速 1:45.478，键 (0, 2, −105.478)，
低于 3c 的 best（旋转 0、干净档有、1:49.725，键 (1, 1, −109.725)）。**按链级字典序，本棒零进账**：
链上 best 仍是 3c 那个零旋转检查点。这是链上第一棒零进账。按长令直接点 parent3e。
- 完整续训：optimizers yes，rng yes，replay no（日志已确认）。

## 世界与参数

- 世界指纹 `8720958`，工作树 `.worktrees/bake-parent3b`（detached 在 `5d5ef16`，点火前无改动）。
- 与 parent3c 相同：`--fixed-alpha 0.0011`（谷底），`--steps 10000000`（不设上限），其余与 parent3b 一致。

      nohup /Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python -u train.py \
          --solo --track silverstone --batch 64 --seed 1 --tag parent3d \
          --steps 10000000 --eval-every 25000 --eval-batch 6 \
          --fixed-alpha 0.0011 --resume checkpoints/latestparent3c.pt > run-parent3d.log 2>&1 &

- PID 23940，PPID 1（已核实）。日志 `.worktrees/bake-parent3b/Training/python/run-parent3d.log`。
- 首评（675k）带续训疤，属预期；从第二评起见真章。
