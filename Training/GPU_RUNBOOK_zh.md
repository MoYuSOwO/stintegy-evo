# 租卡迁移手册：从 Mac 到 4090 / 5090

记录日期：2026-09-14。目标：**开机到点火 30 分钟以内**。适用于 `era/world-v2`
上的长炉从最新检查点迁移上卡续训。

## 〇、先知道三件事

1. **瓶颈在网络更新，不在环境——这正是上卡的理由。** 2026-09-14 的实测
   （见 `design-notes/2026-09-14-top-level-design-v2_zh.md` 第六节）：纯环境步进
   34704 tps，整炉 1526 tps，**学习器占墙钟约 96%**；GPU 对现网约 3–5 倍，对宽网 10 倍以上。
   训练主机仍用 `Parallel.For` 把 64 条车道铺满所有核，所以 CPU 也不能太弱，
   但它不是短板：16 核足够，不必为核数多付钱。
   （本手册初版把这一条写反了，写成"瓶颈可能在 CPU"——那是没有实测的推断，已按实测改正。）
2. **迁移后是"续训"，不是"逐位复现"。** 两处合理差异：
   - 赛道几何的线性代数：Mac 走 Apple Accelerate，Linux 走 OpenBLAS，浮点末位会不同；
   - 策略推理与更新改在 CUDA 上，且检查点只保存了 CPU 随机数状态。
   可复现性认 **era commit + 检查点哈希**，不认比特。自检里有一步专门对照 Mac 读数确认"足够接近"。
3. **本手册的脚本在 Mac 上没法验证 CUDA 那一步。** 编写时的 Mac 是 Apple Silicon：
   CUDA 路径只在代码层核查过，Linux 主机只做过交叉编译。**真正的 CUDA 冒烟是开机后的第一件事**，由自检脚本完成。

代码层已核查过的：`--device cuda` 会把网络、温度参数放上卡，动作取回 CPU；
CPU 上练出的检查点用 `map_location` 可直接加载到 CUDA，优化器状态随参数迁移；
`linux-x64` 交叉发布成功，原生库 `libopenblas.so`、`highs.so`（ELF 64 位）都已随包，
Linux 上自动选 OpenBLAS 后端。

## 一、选机

| 项 | 要求 | 理由 |
|---|---|---|
| GPU | RTX 4090（24 GB）或 5090（32 GB） | 网络小，显存绰绰有余 |
| CPU | ≥ 16 核 | 64 车道并行步进；实测环境不是短板，不必更多 |
| 内存 | ≥ 32 GB | 回放池 100 万条 ≈ 3.9 GB，外加主机与系统 |
| 磁盘 | ≥ 50 GB | 每份检查点约 28.6 MB，长炉 25k 一存，外加 .NET 与 torch |
| 系统 | **Ubuntu 24.04** | 自带 Python 3.12；22.04 默认源只有 3.10，需另加第三方源 |
| 驱动 | 与所装 PyTorch CUDA 轮子匹配；**5090（Blackwell）需要支持 CUDA 12.8 及以上的驱动** | 以 PyTorch 官网安装选择器当日给出的组合为准 |

## 二、装机（约 10 分钟）

```bash
sudo apt-get update && sudo apt-get install -y git tmux build-essential python3.12 python3.12-venv
```

```bash
curl -sSL https://dot.net/v1/dotnet-install.sh -o dotnet-install.sh && bash dotnet-install.sh --channel 8.0 && echo 'export PATH="$HOME/.dotnet:$PATH"' >> ~/.bashrc && export PATH="$HOME/.dotnet:$PATH"
```

```bash
git clone https://github.com/MoYuSOwO/stintegy-evo.git && cd stintegy-evo && git checkout <manifest 里记录的 era commit>
```

```bash
python3.12 -m venv Training/.venv && Training/.venv/bin/pip install --upgrade pip && Training/.venv/bin/pip install numpy==2.5.2
```

PyTorch 装与驱动匹配的 CUDA 版。版本尽量与 Mac 一致（2.13.0），装不到时取最近的，
**实际版本记进 manifest**。轮子索引以 PyTorch 官网安装选择器当日给出的为准，下面的 `cu128` 只是示例：

```bash
Training/.venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cu128
```

```bash
nvidia-smi && Training/.venv/bin/python -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

## 三、从 Mac 传检查点（约 2 分钟）

检查点不在 git 里。在 **Mac** 上先记哈希，再传两份：续训起点 + 参考检查点。

```bash
shasum -a 256 Training/python/checkpoints/latestparent2i.pt Training/python/checkpoints/evalparent2h-1225000.pt
```

```bash
rsync -avP Training/python/checkpoints/latestparent2i.pt Training/python/checkpoints/evalparent2h-1225000.pt <user>@<box>:~/stintegy-evo/Training/python/checkpoints/
```

## 四、构建（约 3 分钟）

```bash
dotnet build Training/StintegyEVO.TrainingHost -c Release
```

可选（约 2 分钟，确认物理与协议在这台机器上逐项通过）：

```bash
dotnet test Core/Tests -c Release && dotnet test Training/StintegyEVO.TrainingHost.Tests -c Release
```

## 五、开机自检（约 5 分钟）

```bash
cd Training/python && ../.venv/bin/python -u gpu_preflight.py --checkpoint checkpoints/latestparent2i.pt --sha256 <Mac 上记下的哈希> --reference ../gpu-reference.json
```

自检依次确认：Python 与包 → CUDA（在卡上真跑一次矩阵乘）→ .NET SDK ≥ 8 →
主机能构建、协议版本 4、观测 480 维、布局自检通过、四轮尺字段到位 →
检查点哈希一致且能加载到卡上（并报告续训是完整续训还是热启动）→
2000 步 16 车道训练冒烟（打印 tps）→ **对照 Mac 参考读数**。

参考读数（`Training/gpu-reference.json`）：`evalparent2h-1225000.pt`，种子 900001，
12 车道 × 600 秒，名义 3/3——56 圈、3 旋转、11 圈干净、最快干净圈 1:41.316（world-v3 读数；world-v2 时为 57 圈、0 旋转、27 圈干净、1:41.585，差异见 `experiments/2026-09-18-world-v3-physics-probe`）。
容差：圈数 ±2、最快干净圈 ±0.5 秒、干净圈数 ±25%。**超出容差即停**：
这台机器上的世界和 Mac 上的不够接近，先查再点火。

出现 `PREFLIGHT PASSED` 才进入下一步。

## 六、线程

PyTorch 默认会占满 CPU 线程做算子，和主机的 64 车道并行抢核。给 Python 进程限线程：

```bash
export OMP_NUM_THREADS=4
```

若想确认取值，用自检的训练冒烟分别在 `OMP_NUM_THREADS=2/4/8` 下看 tps，取最高的那个。

## 七、点火（约 1 分钟）

在 tmux 里点，断开 SSH 也不会杀掉进程。旗标照该炉 manifest，唯一差别是 `--device cuda`：

```bash
tmux new -s bake
```

```bash
cd ~/stintegy-evo/Training/python && export OMP_NUM_THREADS=4 && ../.venv/bin/python -u train.py --device cuda <manifest 中的其余旗标> --resume checkpoints/latestparent2i.pt 2>&1 | tee -a run-parent2i-gpu.log
```

点火后立刻把以下三项写进该炉 manifest：**era commit**、**续训检查点 sha256**、**机器型号（GPU/CPU 核数/驱动/torch 版本）**。

## 八、巡检与回传

在**租卡机器**上：

```bash
tail -f ~/stintegy-evo/Training/python/run-parent2i-gpu.log
```

```bash
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv -l 30
```

**显卡利用率长期很低、tps 与 Mac 相当**，先查三件事：`--device cuda` 是否生效
（日志第一行 `device:`）、`OMP_NUM_THREADS` 是否过小或过大、主机进程是否被限了核。
按实测，学习器占墙钟 96%，上卡后 tps 应明显高于 Mac。

定期把评估检查点和日志拉回 Mac（**在 Mac 上**执行；远程路径整体加引号，通配符才会在租卡机器上展开）：

```bash
rsync -avP '<user>@<box>:~/stintegy-evo/Training/python/checkpoints/evalparent2i-*.pt' Training/python/checkpoints/
```

```bash
rsync -avP '<user>@<box>:~/stintegy-evo/Training/python/run-parent2i-gpu.log' Training/python/
```

## 九、退租前

1. 确认炉子已按宽停或预算自然收工（日志末尾 `training finished`），或已人为停炉；
2. 最后一次 rsync，**在 Mac 上核对拉回的检查点 sha256**；
3. 再退租。云盘上的东西退租即丢。

## 十、时间预算

| 步骤 | 分钟 |
|---|---:|
| 装机 | 10 |
| 传检查点 | 2 |
| 构建 | 3 |
| 自检（含训练冒烟与参考认证） | 5 |
| 线程确认（可选） | 5 |
| 点火 | 1 |
| **合计** | **21–26** |
