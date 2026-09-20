# parent3l：母本链第十二棒（连平预警下的裁决棒）

## 续训起点

- `latestparent3k.pt`,step 2525000,sha256 前缀 `08700f1dc01b325f`。
- PID 52764,PPID 1（点火前 pgrep 冷、点火后 30 秒复查:全机唯一 train.py）。
  日志确认 `optimizers yes, rng yes, replay no`。
- 世界指纹 `8720958`,工作树 `.worktrees/bake-parent3b`（`5d5ef16`）。同配方。

## 本棒地位

3k 三线全平（连平 1/2）。**本棒即裁决棒**:
- 任一线破 → 连平计数清零,链照常续;
- 三线再平 → 链收官（先报用户再停）,链上四份候选检查点送认证:
  `bestparent3f.pt`（零旋转键 best）、`evalparent3g-1525000.pt`（干净率
  0.92）、`bestparent3j.pt`（配速 1:42.361）、`evalparent3k-2450000.pt`
  （最快准零旋转评,1:43.139/1 旋）;升级预案（新鲜度连续炉/租 CUDA）
  同时上会,由用户裁定是否加时。

## 起跑时的链纪录

计罚配速 1:42.361（3j）;干净率 23/25 = 0.92（3g）;零旋转最好
1:46.854（3f,链上 best 指针）。
