<p align="center">
  <img src="logo.svg" alt="StintegyEVO" width="500"/>
</p>

# StintegyEVO — 赛车仿真核心

[English](README.md)

StintegyEVO 是一个实时赛车仿真：车辆与轮胎物理、能量与磨损、赛道、比赛步进，以及 Godot 表现层。它提供世界，不提供车手。决定车怎么开的东西，无论是规则、学习模型还是人，都通过一个很小的接口从外部接入。

**开源，但不是社区驱动。** 代码以 AGPL 公开，欢迎阅读、运行、分叉和在其上构建。项目由维护者按自己的节奏开发；Issue 和 Pull Request 可能会被看到，但不承诺回复、审阅或合并。

> 我们不比谁开得快，我们比谁算得准——同时保持世界模型诚实。

## 快速开始：让一辆车动起来

需要 [.NET 8 SDK](https://dotnet.microsoft.com/download)。如需 3D 画面，还需要 [Godot 4.6 .NET 版](https://godotengine.org/download)。

```bash
git clone https://github.com/MoYuSOwO/stintegy-evo.git
```

```bash
cd stintegy-evo && dotnet test Core/Tests/StintegyEVO.Core.Tests.csproj -c Release
```

世界不会自己开车。要看到车跑起来，得给它一个控制器。下面这个控制器瞄准前方中心线上的一点，并按看得到的弯道曲率提前减速：

```csharp
sealed class CentrelineFollower : IDriverController
{
    public DriverInput GetControl(in DriverContext context, float dt)
    {
        RaceCarSnapshot car = context.Car;
        float lookahead = 12f + 0.6f * car.SpeedMetersPerSecond;
        Vector2 aim = context.Track.Sample(car.TrackS + lookahead).Center;

        // Pure pursuit: the arc through the aim point, in the car's frame.
        Vector2 offset = aim - car.Position;
        float left = -MathF.Sin(car.HeadingRadians) * offset.X +
                     MathF.Cos(car.HeadingRadians) * offset.Y;
        float curvature = 2f * left / MathF.Max(offset.LengthSquared(), 1f);

        // Slow for the tightest bend in the next 200 m.
        float tightest = 0f;
        for (float d = 0f; d <= 200f; d += 10f)
            tightest = MathF.Max(
                tightest,
                MathF.Abs(context.Track.Sample(car.TrackS + d).Curvature)
            );
        float target = MathF.Min(60f, MathF.Sqrt(14f / MathF.Max(tightest, 1e-4f)));
        float accel = Math.Clamp(2f * (target - car.SpeedMetersPerSecond), -20f, 8f);
        return new DriverInput(curvature, accel);
    }
}
```

把它装进一辆车，然后步进世界：

```csharp
TrackData track = TrackFactory.SilverstoneStyleTestTrack();
var simulation = new RaceSimulation(track);
var profile = new DriverProfile("example", new DriverAbilities());
TrackSample start = track.Sample(track.StartingLineS);
var car = new RaceCar(
    "car-1",
    new CarConfig(),
    new TireConfig(),
    new Driver(profile, new CentrelineFollower()),
    new CarState { Position = start.Center, Heading = start.Heading }
);
simulation.AddCar(car);
for (int i = 0; i < 60 * 240; i++)   // four simulated minutes
    simulation.Step(1f / 60f);
```

这个示例故意开得慢：四分钟仿真时间内跑完这条 5.9 km 赛道的一圈，零旋转。上面的代码原样就是一个测试 [`Core/Tests/ReadmeExampleTests.cs`](Core/Tests/ReadmeExampleTests.cs)，所以不会悄悄失效。

要看画面，用 Godot 打开项目。主场景 `Levels/lowpoly.tscn` 在宿主交给它一个世界之前是静止预览：按上面的方式建好仿真，在视图进入场景树之前调用 `RaceView3D.BindSimulation(simulation)`。[`Tools/Tests/CompositionSmoke.cs`](Tools/Tests/CompositionSmoke.cs) 演示了把"控制器驱动"和"宿主驱动"两种世界接入 3D 视图。

## 这里有什么

- **世界与物理：** 车辆状态、滑移角轮胎、动力系统资源、温度与磨损、抓地、道路姿态与倾角、碰撞、尾流、赛道界限与墙、确定性步进。
- **领域契约：** 稳定身份、带 0..100 能力评级的车手档案、车辆能力、有类型的资源槽位、赛道几何、不可变的帧快照，以及控制器接口。
- **表现层：** Godot 3D 视图、相机、HUD 和物理状态 CSV 记录。旧的 2D `RaceView` / `Levels/root.tscn` 仅作兼容保留。

这里**没有**：规则驾驶器或学习驾驶器、赛车线求解器、观测向量、神经网络、训练程序和模型文件。世界不偏好任何行车线，也不在物理里藏一个车手模型。

## 各部分如何衔接

- **控制器。** `IDriverController.GetControl` 读取 `DriverContext`（车手档案、只读赛道视图和一帧冻结快照），返回 `DriverInput`：期望曲率、期望纵向加速度和可选的刹车配比偏移。车也可以没有车手，此时由宿主设置 `RaceCar.ExternalInput`；零命令意味着滑行，不会把车冻住。
- **冻结、收集、推进。** 每个子步里，所有控制器读同一帧冻结快照，全部命令收集并校验完之后物理才推进。没有哪个控制器会看到被其他控制器改过的世界。
- **车上的装置。** 命令与轮胎之间的一切都是公示的装置，参数在 `CarConfig` 上。没装某个装置的车，该装置参数为零；接口不变。
  - **合成抓地限制器**（combined-grip limiter）是本系别允许的电子稳定装置。它裁剪每根轴的驱动与制动，使轮胎合成用量不超过车队轮胎档位授权的摩擦圆份额；车速高于 10 m/s 时，滑移角过峰的那根轴驱动清零。它不碰转向。`CombinedGripLimiterStrength = 0` 即未装机。
  - **减阻装置**（drag reduction）减少一部分气动阻力，并恢复一部分因前车尾流损失的下压力。激活由外部赛事主机设定，Core 不判定资格。
- **赛道是几何。** 赛道提供中心线、宽度、缓冲区、曲率、高程与倾角、路面和发车格，不提供赛车线。

这些契约写在上述类型的 XML 文档注释里，并由 `Core/Tests` 中的测试守住。

## 历史

当前的产品边界比项目早期原型更窄。原型自带解析驾驶器、学习驾驶器和训练栈，这段历史仍在 Git 里：基线 `d85e164` 与 `era/world-v2` 线。当前行为与它有意不同：车手效率参数已移除；旧的牵引力控制和 ABS 被合成抓地限制器取代；减阻装置由宿主激活，不再由过线时的一秒差距自动授予。旧结果是关于旧契约的证据，不要当作对当前契约的承诺。

## 验证

```bash
python3 Tools/verify_boundary.py --godot /path/to/godot-dotnet
```

它会依次跑源码边界检查、Core 测试、两种应用构建、无头 3D 组合与运动冒烟，以及遥测检查。加 `--render` 可得到真实渲染截图。

## 许可

- **代码**采用 [GNU Affero 通用公共许可证 v3.0](LICENSE)。
- **内容与素材**适用单独的[内容许可说明](CONTENT_LICENSE_zh.md)，涵盖项目素材、第三方材料和官方发行版。
- `Core/Track/Data/` 下的**第三方赛道数据**保留原有的 LGPL-3.0 或 MIT 条款，详见 [`Core/Track/Data/README.md`](Core/Track/Data/README.md)。
