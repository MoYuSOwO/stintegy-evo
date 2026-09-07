# 银石 15Hz 专家 · 种子矩阵

修复管线时代的第一个真实验。回答两个问题:
1. 修好的管线能不能把专家训到解析水平?
   验收:计罚 gap 显著优于联训时代的 +13.4s,理想 ≤ 0。
2. sp1 那种"越练越慢"在修复管线上是否复发?

## 条件

| 项 | 值 |
|---|---|
| 提交 | f583fda222848086559f6efa8c142115fdadfe82 |
| 赛道 | silverstone (solo, --track 专家模式) |
| 解析基线 | 107.050 s (15Hz 干净飞驰圈, baseline_15_60.json) |
| 决策频率 | 15 Hz (步长 0.0667 s) |
| 种子 | 1, 2, 3 — 各自从零,**不热启动任何旧检查点** |
| 预算 | 每种子 400k 步 ≈ 7.4 模拟小时/lane |
| 评估 | 每 25k 步,计罚排名,固定 Normal/Normal,专家 + 2 哨兵 |
| 超参 | 全部库中现值 (γ 0.9931, n=10),一个未调 |
| 批 / lane | 512 / 64 |

## 为什么串行而不是并行

回放池预分配 3.6 GB/进程,三路并行需 ~15 GB(共 24 GB),且 PyTorch 线程
未设上限,三进程会在 10 核上抢 30 线程。历史实测两路并行时单路吞吐降到
39%(830 vs 2150 tps),**串行总墙钟更短**(约 10 h vs 13 h)且第一条曲线
在约 3.3 h 就能看到。故串行,按种子分 tag,日志与检查点互不共写。

## 运行

```
seed 1 → run-15hz-s1.log, checkpoints/{latest,best}15hz-s1.pt
seed 2 → run-15hz-s2.log, checkpoints/{latest,best}15hz-s2.pt
seed 3 → run-15hz-s3.log, checkpoints/{latest,best}15hz-s3.pt
```

命令(每种子):
```
Training/.venv/bin/python3 -u train.py --solo --track silverstone \
    --batch 64 --steps 400000 --seed <N> --tag 15hz-s<N>
```

起始时间:2026-09-07 06:16:03 CST

## 观察义务

每次评估后看:计罚 gap 趋势、转向变化率、干净圈比例。
若出现 sp1 型回退(峰值后持续恶化超过 100k 步),**不中途改任何东西**,
让它跑完 —— 完整的回退曲线本身就是证据。
