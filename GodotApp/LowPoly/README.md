# Godot Low-poly 表现层

主入口为 `Levels/lowpoly.tscn`：赛道、车辆、相机、HUD、小地图和物理 CSV 遥测。
这里不实现控制算法，也不加载训练模型。

## 预览与外部所有权

- 默认是一辆 `Driver == null` 的静止预览车，`IsPreview == true`，时钟不推进。
- 在进入场景树前调用 `BindSimulation(simulation)`，将非空的已组装仿真交给视图。
  外部仿真即使没有控制器、仅有零输入的滑行车也会正常步进。
- 等待 `IsInitialized` 后，在 **Godot 主线程**调用 `SetExternalInput(index, curvature, accel, brakeBias)`。
  命令先排队，在后台物理任务结束后应用。首次提交激活默认预览；后续零输入只表示滑行，不冻结仿真。
- 暂停只由 `TogglePause()` 控制；暂停时可排队输入和调整策略，但不会推进时间。
- 视图拥有仿真期间，宿主不要从其他线程读取/修改可变车辆或调用 `Step`。
  控制器运行在物理工作线程上，应只使用 `DriverContext`，不要调用 Godot UI。
- 退出场景会等在途任务结束，再归还仿真所有权。

Core 以 1/60 秒步进；渲染使用已完成物理步的姿态副本插值。相机、HUD 和策略控件独立于驾驶实现。
设置 `STINTEGY_CSV_TELEMETRY=1` 导出到 `.tmp/telemetry.csv`，或给定输出路径；只在物理任务完成后记录。

## 运行与验收

Godot 编辑器/普通场景运行加载 **Debug 应用程序集**（Core 默认仍用 Release）：

```sh
dotnet build StintegyEVO.csproj -c Debug
/Applications/Godot_mono.app/Contents/MacOS/Godot --path .
python3 Tools/verify_boundary.py --godot /Applications/Godot_mono.app/Contents/MacOS/Godot --render
```

最后一条同时检查 Core、外部3D接入、相机/HUD/暂停/输入、真实渲染截图与 CSV。
去掉 `--render` 则无窗口运行，**明确跳过截图，不构成视觉验收**。
日志和结果在 `.tmp/boundary-verification/`；截图在 `.tmp/lowpoly/`。

测试专用 `Tools/Tests/CompositionSmoke.cs` 只在 `-p:PresentationSmoke=true` 时编译；
普通构建不包含其中的测试控制器。验收工具会在 fixture 运行后恢复普通 Debug 构建。

## 遗留 2D

`RaceView` / `Levels/root.tscn` 已废弃，暂为兼容保留。本次不移除、不继续扩展，也不将其列为运行验收对象。
