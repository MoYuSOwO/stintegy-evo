# 3D low-poly 主场景

`Levels/lowpoly.tscn` 是当前主入口；旧 2D `Levels/root.tscn` 仅为遗留兼容保留，不纳入本次运行验收。

## 启动

```sh
dotnet build StintegyEVO.csproj -c Debug
/Applications/Godot_mono.app/Contents/MacOS/Godot --path .
```

必须使用 Godot **.NET/Mono** 版本。编辑器运行加载 Debug 应用程序集，单独编译 Release 不能更新它。
默认场景不加载驾驶器：只展示赛道、一辆静止车和 `NO CONTROLLER` HUD，不推进仿真时钟。

## 外部接入

在节点进入场景树前调用 `RaceView3D.BindSimulation(simulation)`，移交非空仿真。
绑定的世界正常步进，包括没有控制器但带初始速度的车辆。

等待 `IsInitialized` 后，可在 Godot 主线程调用：

```csharp
view.SetExternalInput(0, desiredCurvature: 0f, desiredAccel: 2f);
```

这是命令队列，不是控制算法。命令在后台物理任务释放世界后应用；首次提交会启动默认预览。
后续零命令仍继续物理（滑行、阻力、坡度），不代表暂停。暂停时提交命令不会越过暂停。
视图运行期间不要并发修改 `Simulation`；退出场景会等待在途任务结束，归还所有权。

## 操作

| 输入 | 行为 |
|---|---|
| 1 / 2 / 3 | 跟车、侧视、全局镜头 |
| 滚轮 / 右键拖动 | 缩放 / 绕观察点旋转 |
| 左右方向键 / 排名行 | 切换观察车辆 |
| F | 切换跟车与侧视 |
| 空格 / RUN、PAUSE | 暂停、恢复已经接入的仿真，不自动创建驾驶器 |
| Q / E | 轮胎使用档 |
| A / D | 动力输出档 |
| H | HUD 显隐 |

## 数据与验收

车辆姿态来自 Core，3D 不另建 Godot 刚体。物理在一个后台任务中以 1/60 秒步进；
渲染消费已完成步的姿态副本。HUD、策略、CSV 只在物理任务完成后读取世界。
`STINTEGY_CSV_TELEMETRY=1` 导出 `.tmp/telemetry.csv`，也可指定路径。

```sh
python3 Tools/verify_boundary.py --godot /Applications/Godot_mono.app/Contents/MacOS/Godot --render
```

- `lowpoly_smoke.gd`：静止预览、相机/HUD/策略、外部输入、零命令滑行、暂停和恢复；有窗口时保存五张真实截图。
- `lowpoly_motion_smoke.gd`：已知外部输入的位移、运动帧比例和仿真节奏。
- `CompositionSmoke`：测试专用外部控制器、无控制器滑行世界、输入验证和所有权归还；不进入普通产品构建。
- headless 模式明确跳过截图，不宣称视觉验证通过。脚本与外层运行器都有超时保护。

完整验收用 `python3 Tools/verify_boundary.py --godot <Godot .NET 可执行文件>`；加 `--render` 得到真实截图。
