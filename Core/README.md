# `StintegyEVO.Core` 边界说明

`Core/` 是 StintegyEVO 的物理世界与领域契约层。它可以被 Godot、测试程序、离线工具或其他主机引用，但不依赖 Godot 的节点生命周期，不拥有驾驶算法、训练奖励或行为处罚政策。

## 责任

Core 负责：

- 车辆、轮胎、动力系统、温度/磨损、资源消耗与回收、抓地和道路姿态；
- 赛道几何、表面、坡度/倾角、发车格、投影/绕回、边界接触与车车碰撞；
- `RaceSimulation` 的冻结帧、命令收集、物理推进和不可变快照；
- `DriverProfile`、`DriverAbilities`、`Driver`、`IDriverController`、`DriverContext` 等领域/集成契约；
- `CarStrategy` 这类由车队/主机拥有的策略状态，以及车辆可读的资源/能力描述。

## 明确不负责

Core 不实现：

- 规则驾驶器、学习驾驶器或任何隐式默认驾驶器；体育规则、裁判、处罚与赛制由外部规则层负责；
- 赛车线、参考线、QP/BLAS、路径规划、速度规划、跟踪控制或学习驾驶器；
- 观测向量、策略网络、神经网络推理、训练脚本、训练协议、检查点或模型资源；
- “没有控制器就自动使用某个驾驶器”的隐式回退。

历史树中仍可能存在旧时代的文件、名称或证据；它们不应被解释为当前 master 契约的一部分。若未来实现需要其中的想法，必须按当前接口重新适配，而不是恢复旧依赖。

## 驾驶边界

当前并行方案的最小形状如下：

```csharp
Driver driver = new Driver(profile, controller);
RaceCar car = new RaceCar(id, carConfig, tireConfig, driver: driver);
```

`RaceCar.Driver` 是只读引用，也可以为 `null`。没有控制器的车辆通过 `ExternalInput` 接收主机命令：

```csharp
RaceCar passive = new RaceCar(id, carConfig, tireConfig, driver: null);
passive.ExternalInput = new DriverInput(curvature, acceleration);
```

控制器只实现：

```csharp
void Initialize(in DriverContext context);
DriverInput GetControl(in DriverContext context, float dt);
```

`DriverContext` 中的 `Frame`、`Track` 和 `Profile` 都是只读语义。`RaceSimulation` 对每个驾驶步先捕获冻结的 `RaceFrameSnapshot`，所有控制器都从同一帧读取，再统一发布 `DriverInput`，最后执行物理子步。控制器的内部计划、计时器、策略编码和模型归调用方所有，不属于 Core 的世界状态。

## 物理语义

ABS、TC、轮胎曲线、热/磨损、能量与碰撞等真实物理机制仍在 Core。旧的三个驾驶器效率参数以及 `#56` 的 driver reflex/governor 已移出物理输入和物理层；驾驶能力不能再通过隐藏的物理捷径表达。这是行为变化，不是旧结果的逐位兼容承诺。

## 赛道语义

`TrackData` 表示道路本身：中心线采样、宽度、边缘缓冲、曲率、坡度/倾角、表面抓地、投影、绕回和发车格。它不表示“最佳线”或“策略线”。赛车线/QP/BLAS 不是 Core 的隐含依赖；任何外部控制器需要的路径都应在外部计算，并以当前 `DriverContext` 可消费的契约接入。

## 使用注意

- 创建 `RaceSimulation` 不会自动跑圈；必须显式 `Step(dt)`。
- 新的 Godot 接入和验证只针对 `RaceView3D`：可以在 `_Ready()` 前通过 `RaceView3D.BindSimulation(...)` 注入已有 `RaceSimulation`。旧 `RaceView` 仅作为废弃兼容表面保留。
- `Driver == null` 不应触发默认规则或默认驾驶器；Core 使用保持型 `ExternalInput`，其默认值为零命令。Core 仍会对零输入正常计算物理；独立 3D 预览是通过暂不调用 Step 保持静止，而非把零输入定义成冻结。
- 不要在 Core 中加入包/插件加载框架来解决外部策略、规则或模型分发问题。

更完整的契约、赛道几何、验证与迁移说明位于 [`../docs/architecture/README_zh.md`](../docs/architecture/README_zh.md)。
