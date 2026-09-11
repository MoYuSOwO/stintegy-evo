# 热力学修正 · 550k → 575k 续训的模型实体

诊断全文：`Training/diagnostics/2026-09-11-thermal-adaptation/README_zh.md`。
本目录只存放那次续训留下、后续还要用的模型，原件在 `.tmp/`，随时可能被清。

**模型文件不进 git**（`Training/.gitignore` 的 `experiments/**/*.pt`），
随工作区保存；本文与 `manifest.json` 进 git，用于核对身份。

| 文件 | 步数 | 大小 | SHA-256 |
|---|---:|---:|---|
| `best.pt` | 575 000（该评估存为 best） | 28 612 856 | `8e5a2473de1da603444eee90f9df17c4bad65c99c51041585ef53dd1dce468d3` |
| `eval-575000.pt` | 575 000 | 28 622 777 | `f9e80fef024f52f2edffc8c3c8bda18a5c1f788bfbf6a198375047e649999109` |

起点（未复制，仍在原处）：`Training/python/checkpoints/bestparent.pt`，
550k，SHA-256 `1fd2f6ea4103f49133c7416225e5d6e4141ea19bd580c6db13bcd4916258cd85`。

`manifest.json` 是运行时写下的原始配置，路径是当时的绝对路径，
其中 `validation.status` 反映的是测试清理之前（368 过 / 13 败）的状态。

## 它证明了什么、没证明什么

- **证明**：删掉后轮重复发热后，这一代 550k 在 25k 步内适配到银石正赛带
  （最快干净圈 1:43.972，39/55 干净，0 旋转，12×600 s，暖胎评估）。
- **没证明**：它不是母本。续训撤掉了随机化起步课程（固定名义起步、600 s 回合、
  α 显式固定），雪邦崩回 0/43 干净、85 旋转，simple-right 未完成一圈；
  也没做冷胎认证。母本二幕从 550k 起、保留随机化课程，只换物理。
