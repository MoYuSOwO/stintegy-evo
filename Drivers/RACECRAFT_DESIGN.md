# 三层竞速控制设计规格

日期：2026-11-16  
分支：`research/driver`  
状态：这是当时定下的规格。刹车圆（`daf185b`）和画线倒推门控（`d603fc1`）已按此提交。六态战术脑以阶段形式提交在 `ff44351`，实测还差的三处和逐帧数据在 `Drivers/RACECRAFT_PHASE.md`，那份记录不改写本规格。  
范围：`LaneDriver` 的对称刹车能力圆、`TrafficCorridor` 的无折角走廊与空间限速、新建 `RacecraftTactics` 六态战术脑。不改车辆物理，不给 `FormulaAMixedCar` 加电子抓地限制器。

本规格里的每个类型、字段和方法都按下列源码的真实签名书写：

- `Drivers/Lanes/LaneDriver.cs`
- `Drivers/Lanes/LaneSpeedPlanner.cs`
- `Drivers/Lanes/CornerMap.cs`
- `Drivers/Lanes/DrawnLine.cs`
- `Drivers/Lanes/LanePerformance.cs`
- `Drivers/Lanes/TrafficCorridor.cs`
- `Core/Racing/RaceFrameSnapshot.cs`
- `Core/Drivers/DriverAbilities.cs`
- `Core/Drivers/IDriverController.cs`
- `Core/Cars/CarFeatures.cs`（`GripUse`、`AxleLayout`、`AxleBias`、`CarMotion`）
- `Core/Cars/RoadAttitude.cs`
- `mods/official/src/FormulaAMixed/FormulaAMixedCar.cs`

---

## 1. 背景与问题根因

### 1.1 左脚刹车没有能力圆

`FormulaAMixedCar` 的底盘明确关掉电子抓地限制器，并把刹车平衡固定在前轴：

```csharp
CombinedGripLimiterStrength = 0f,
FixedFrontBrakeShare = 0.57f,
MaxBrakeAccel = 57f,
```

`LaneDriver` 构造时把这个上限存进 `_maxBrake = _model.Spec.MaximumBrakeDeceleration`，也就是 **-57 m/s²**。

方向盘已经有实时轴限制。`LaneDriverGains.CapFromAxles` 默认为 `true`。`GetControl` 在 `axles.FrontGripAccel > 0.5f && axles.RearGripAccel > 0.5f && speed > 8f` 时，用两轴抓地的木桶和 `CornerOverstep` 误读去封顶曲率请求。

右脚也已经有摩擦圆。油门路径用 `ThrottleCircle` 和 `tractionRoom`：

```csharp
private float ThrottleCircle(float overuse) =>
    _style is { } style ? style.Traction + overuse : (Gains.TractionUse + overuse) * _tyreAuthority;
```

`Room(...)` 从 `GripUse.FrontGripAccel` / `RearGripAccel` 里扣掉当前或即将到来的侧向占用，再按 `AxleLayout.DriveFollowsGrip` 合成 `tractionRoom`。正加速度被夹到 `driveLimit`。

左脚没有对称的那一步。纵向指令先写成：

```csharp
float slope = (ahead - target) / 5f;
float speedChange = target * slope + Gains.SpeedGain * (target - speed);
float accel = (speedChange - att.AlongTrackGravity(StandardGravity, speed)) / att.LongitudinalDemandScale
              + _envelope.Loss(speed);
```

`Gains.SpeedGain` 默认 `6f`。速度表本身已经按 `LaneShares.Brake` 默认 **0.92** 建过，也就是计划里的 `target * slope` 已经吃掉约 92% 抓地。后面的 `SpeedGain * (target - speed)` 是比例项，动态改线或空间限速一旦让 `target < speed`，这一项会在一帧里把请求推向摩擦圆外。

随后的限幅是：

```csharp
accel = Math.Clamp(accel, -_maxBrake, MathF.Max(0f, driveLimit));
```

负向直接放行 `-_maxBrake = -57`。`BrakeBias` 要到踏板已经限幅之后才算，而且只影响 `DriverInput.Axles`，不缩小请求的模。车没有 `CombinedGripLimiterStrength`，物理层不会把这脚刹车拉回摩擦圆，前轴或后轴按 `FixedFrontBrakeShare = 0.57f` 先锁死，遥测里就是 `CarMotion.OverLimit` 升高、冒白烟。

`BrakeBias(GripUse tyres, float accel)` 的现有语义必须保留：固定分配车上，它返回的是相对 `FixedFrontBrakeShare` 的拨盘偏移，范围 `[-0.25, 0.25]`，里面已经包含 `LanePerformance.FrontBrakeBiasOffset`（`Technical` 能力）。物理侧 `FrontBrakeShare` 的真实前刹份额是 `clamp(fixedShare + offset, 0, 1)`。能力圆必须用这份真实份额做木桶，而不是用名义 0.57。

### 1.2 走廊绕过了摩纳哥防线，并画出折角

`LaneDriver.ScaleMove` 是原版防线。它按

```csharp
private static readonly float[] MoveScales = [1f, 0.6f, 0.35f, 0.15f, 0f];
private const float MoveSpeedToleranceMps = 0.5f;
```

从高到低试探，只有

```csharp
scale == 0f || _windowSpeed[0] >= MathF.Min(speed, _windowStillSpeed) - MoveSpeedToleranceMps
```

才接受这一档横向移动。含义是：这条线在窗口起点要求的速度，不能比「当前车速和不动那条线的窗口速度里较慢的那个」再低 0.5 m/s 以上。弯里或刹车区来不及的移动会自动退到更慢的档，最后可以是 `0`，也就是留在原线。

`TrafficCorridor` 不走这道门。它每 `CorridorIntervalSeconds = 0.08f` 调用 `DrawnLine.Set`，把一条新偏移塞进参考线。`SolveSmoothLine` 里有两处近场硬裁剪：

- 积分到 `k >= 6` 时，若 `y` 越出走廊，就把 `slope` 改成 `(clamped - _delta[idx - 1]) / spacing`，位置一步跳到墙上。
- 之后 8 趟平滑里，`idx >= BehindStations + 7`（`BehindStations = 6`）再次 `Clamp` 到 `_lowD/_highD`。

斜率被改写的那一站，曲率是折角。`LaneSpeedPlanner` 会把折角读成必须马上刹下来的弯，`candCeiling[0]` 远低于 `egoSpeed`。这就是已经错过刹车点的物理死路：60 Hz 的左脚对着 `-57 m/s²` 踩下去，入弯抱死并 `Spinning`。

### 1.3 直道跟车、记账和手写减速度

当前 `BuildSpatialCorridorAndLimits` 用一个与速度差无关的大间距：

```csharp
float desiredGap = halfLen + 2.4f + followGapSeconds * egoSpeed + 0.15f * egoSpeed;
```

15 m 量级外就开始把限速放进 `_windowLimits`。尾流还没吃完，油门先松了。

同一段里，`effGap <= 0` 时无条件注入 `recoverV = effV - 0.9f * (-effGap)`。起步时前车正在拉开（`Δv = egoSpeed - rival.Speed <= 0`）也会被这条限速拽慢。

对手预测在 `CaptureRivals` 里用固定减速度开根号，而不是规划器：

```csharp
float lookBrakeMeters = curV * curV / MathF.Max(2f * 0.85f * maxBrakeAccel, 20f);
float allowedHere = MathF.Sqrt(vCorner * vCorner + 2f * 0.85f * maxBrakeAccel * (m * _lanes.Spacing));
```

`maxBrakeAccel` 来自 `LaneDriver` 调用处的 `_envelope.Brake(speed)`，再乘死 0.85。它不知道坡度、`LaneRoad`、`BrakingCoupling`、尾流折进 `_brake[lane]` 的份额，也不知道 `LaneShares`。

直道变线没有「本直道只用一次」的记账，防守可以半路弹回自由线再切一次。深占内线之后没有 `Switchback`。套圈车没有蓝旗让行。`LaneDriver.BlendTargetCeiling` 里还有一处同类手写式：

```csharp
slowest = MathF.Min(slowest, MathF.Sqrt(cornerSpeed * cornerSpeed + 2f * brake * distance));
```

`brake` 在这里是 `0.8f * _planShares.Brake * _envelope.Brake(cornerSpeed)`，又是一个固定减速度。

`LaneSpeedPlanner.BrakeFrom` 内部用三遍中点重读 `Deceleration`，那是规划器自己的积分步，不是本规格要删的对象。要删的是规划器之外所有「拿一个常数 `a` 写 `v² = v0² + 2 a s`」的副本。`PlannedBraking` 用的是已经规划好的两站速度反推 `(v² - vAhead²) / (2 reach)`，它是在读计划，不是在发明减速度，保持不动。

---

## 2. 三层解耦与单向数据流

```text
RacecraftTactics.Update  (12 Hz, 战术意图)
        │  RacecraftIntent
        ▼
TrafficCorridor.Update   (同一 12 Hz 节拍, C¹ 几何 + PlanWindow 门控 + _windowLimits)
        │  DrawnLine.Set + ApplyWindowLimits
        ▼
LaneDriver.GetControl    (60 Hz, 方向盘 CapFromAxles + 右脚 tractionRoom + 左脚 brakeRoom)
        │
        ▼
DriverInput (steer, accel, AxleBias)
```

节拍：`CorridorIntervalSeconds = 1f / 12f`。战术和走廊在同一次节拍里先后跑，走廊用的是刚刚产出的意图。踏板、方向盘、`ScaleMove` 仍是 60 Hz。`LaneDriver.PlanIntervalSeconds` 默认 `0.1f`，速度表继续按这个节拍重算，不和 12 Hz 绑死。

`LaneDriver.GetControl` 里的调用顺序固定为：

1. 需要时 `PlanAhead`，刷新 `_envelope`、`_planShares`、`_corner`、`_brake`、`_road`、`_speed[0]`。
2. 每 `1/12` 秒：`_tactics.Update(...)` 得到 `RacecraftIntent`，再 `_corridor.Update(..., intent)`。
3. `ScaleMove`（车道混合自己的摩纳哥防线，保留）。
4. 用 `here` 而不是 `previewStation` 计算 `brakeRoom`，把负向加速度夹进 `-brakeRoom`。

全局铁律：

- 全系统在 `LaneSpeedPlanner` 之外禁止再写 `v * v + 2f * a * s`、`MathF.Sqrt(v * v + 2f * a * s)` 或 `v * v / (2f * a)` 来决定「多远之前要多慢」。减速曲线只来自 `OnlineCornerLimit`、`CornerLimits`、`BrakeFrom`、`PlanWindow`、`PlanAhead`。
- 这些函数跑在实时 `LaneEnvelope` 上。包络已经含有速度分段的下压力、气动阻力、`CornerDrag`、以及 `PlanTheWake` 写进 `_corner[lane]` / `_brake[lane]` 的尾流折减。道路用 `LaneRoad(Grade, Bank, Vertical)`，刹车横向耦合用 `LaneShares.BrakingCoupling`（默认 `true`）。
- 数据只向下流。`RacecraftTactics` 不读 `TrafficCorridor` 的私有场；`TrafficCorridor` 不决定超车或防守；`LaneDriver` 不在 60 Hz 里改战术相位。

`LaneDriver` 交给下面两层的访问器就是它自己已经算好的数组，不再把 `_envelope.Brake(speed)` 当成一个标量减速度传下去：

```csharp
_tactics.Update(
    in context,
    _speed[0],
    in _envelope,
    _planShares,
    station => _corner[0][_lanes.Wrap(station)],
    station => _brake[0][_lanes.Wrap(station)],
    station => _road[0][_lanes.Wrap(station)],
    _turnLag.Seconds);

_corridor.Update(
    in context,
    _speed[0],
    in _envelope,
    _planShares,
    station => _corner[0][_lanes.Wrap(station)],
    station => _brake[0][_lanes.Wrap(station)],
    station => _road[0][_lanes.Wrap(station)],
    _turnLag.Seconds,
    FollowGapSeconds,
    _drawn,
    ReferenceOffset,
    _tactics.Intent);
```

`TurnLag.Seconds(float speed)` 与 `LaneSpeedPlanner.LeadCurvatures` 的委托签名一致：`Func<float, float>`。

### 2.1 层间契约

```csharp
internal enum RacecraftPhase : byte
{
    Behind = 0,
    StandingStart = 1,
    DraftTow = 2,
    PullOutAttack = 3,
    OneMoveDefend = 4,
    SideBySide = 5,
    Switchback = 6,
    BlueFlagYield = 7,
}

internal readonly record struct RacecraftIntent
{
    public RacecraftPhase Phase { get; init; }
    /// <summary>+1 为 +d（左），-1 为 -d（右），0 为不锁边。</summary>
    public int SideSign { get; init; }
    /// <summary>相对自由线的目标偏移，米。NaN 表示走廊保持自由线或由槽位边界自然决定。</summary>
    public float TargetDeltaMeters { get; init; }
    /// <summary>可选的空间偏移剖面（从当前 anchor 起每站相对自由线的米数，如 Switchback 外入晚弯心内切线）。</summary>
    public float[]? SpatialDeltaProfile { get; init; }
    public bool FreezeLateral { get; init; }
    public bool EmergencyAvoid { get; init; }
    /// <summary>0 表示不加让车限速；否则限速为该比例乘当前车速。</summary>
    public float YieldSpeedScale { get; init; }
    public bool FullThrottleTow { get; init; }
    public string? FocusRivalId { get; init; }

    public static RacecraftIntent None => new() { TargetDeltaMeters = float.NaN };
}
```

`Behind = 0` 让 `default(RacecraftIntent)` 表示「跟车、不抽头」。它不是第七套战术，而是状态 2 抽头失败后的冷却锁，也是没有对手时的静止相位。编号上的六态是：

| 编号 | 相位 |
| --- | --- |
| 1 | `StandingStart` |
| 2 | `DraftTow` 与 `PullOutAttack`，失败时落入 `Behind` 并启动冷却 |
| 3 | `OneMoveDefend` |
| 4 | `SideBySide` |
| 5 | `Switchback` |
| 6 | `BlueFlagYield` |

走廊消费意图的方式：

- `TargetDeltaMeters` 写成相对自由线的 `_prefD`。`FreezeLateral` 为真时，偏好钉在本帧 `RaceCarSnapshot.TrackD`，积分器不再继续横移。
- `SideSign != 0` 时，焦点对手按该侧占住走廊墙，净空沿用现有 `SideMarginMeters`（0.90 m）加弯道外摆。
- `EmergencyAvoid` 只放开第 4.2 节的门控例外，不放大踏板。
- `FullThrottleTow` 为真时，这个对手不产生任何 `_windowLimits` 点。
- `YieldSpeedScale > 0` 时注入一条 `YieldSpeedScale * egoSpeed` 的空间限速。

`DrawnLine.Set(int from, ReadOnlySpan<float> offsets, float brakeFactor)` 仍是走廊交线的唯一入口，`brakeFactor` 保持 `1f`。近处冻结仍由 `DrawnLine` 自己的 `FreezeSeconds = 1f` 与 `MinFreezeMeters = 25f` 负责，走廊不再用 `k >= 6` 去补一刀。

---

## 3. 第 1 层：`LaneDriver` 对称能力刹车圆

### 3.1 能力闭环

在把 `accel` 夹进踏板之前计算。`familiar.Brake` 已经乘进 `_brake[lane]`（`ReadGrip` 里 `familiar.Brake[segment]`），规划器的 `BrakeFrom` 会读到它。能力圆不再乘一次。

```csharp
float overuse = _performance?.LimitOveruse ?? 0f;
float misread = _performance?.CornerOverstep ?? 0f;
float brakeCircle = (_style is { } s
        ? _feel.SustainablePeakShare * s.Brake + overuse
        : (Gains.BrakeFeelUse + overuse) * _tyreAuthority)
    * (1f + 2f * misread);
```

对应关系：

- `LimitOveruse`：`Pace` 留下的余量，加上 `Precision` 让手在余量两侧漂移。正值表示这一段用得比梯级更满。
- `CornerOverstep`：`Focus` 极少发生的整段走神，平时为 0。`(1 + 2 misread)` 与方向盘 `CapFromAxles`、右脚 `ThrottleCircle` 后面的误读系数同一形。
- `_style.Brake`：`TyreStyle` 的刹车份额。`LaneDriverGains.TyreStyleTable` 从 Protect 的 `0.9812f` 到 Attack 的 `1f`。有梯级时走第一支，不再乘 `_tyreAuthority`（梯级的 `Cap` 已经表达轮端权限）。
- 没有梯级时走 `BrakeFeelUse`（默认 `0.97f`）加 `overuse`，再乘 `_tyreAuthority`。
- `_feel.SustainablePeakShare` 来自 `ICarGripFeel`。库存方程式车返回 `TireSlipCurve.SustainablePeakShare`（`0.98f`）。

### 3.2 横向占用必须读车底，不读 18 m 外的预瞄

油门出弯看的是前瞻曲率 `wanted`：出弯时后轴要给「即将转过去」的弯留圆，这是 `tractionRoom` 的现有定义，保持不变。

刹车相反。直道末端重刹时，`previewStation = station + speed * turnIn / spacing` 会落在大约 18 m 外的入弯点。若用那里的曲率乘当前直道高速的 `v²`，摩擦圆会以为车已经在弯心里，于是直道上提前松刹。

刹车圆的几何需求只用车底这一站：

```csharp
float hereCorner = MathF.Abs(
    att.CurvatureDemandScale * speed * speed * here
    + att.LateralGravityDemand(StandardGravity, speed));
```

`here` 是已有的 `BlendedCurvature(station)`，不是 `feedforward`，也不是 `previewStation`。`att` 是同一函数里已经按 `Gains.UseRoadAttitude / UseGrade / UseBank / UseVertical` 装好的 `RoadAttitude`。`CurvatureDemandScale` 在平路上为 1，坡度倾角把它投到路面上；`LateralGravityDemand` 是重力沿法向的侧向分量。`StandardGravity` 已是 `9.80665f`。

再和实时轴占用取较大值。`GripUse` 的侧向分量是份额，几何需求是 m/s²，先除以该轴抓地：

```csharp
float frontW = Math.Clamp(_layout.FrontWeightShare, 0.05f, 0.95f);
float frontUse = MathF.Max(tyres.FrontLateral, frontW * hereCorner / tyres.FrontGripAccel);
float rearUse = MathF.Max(tyres.RearLateral, (1f - frontW) * hereCorner / tyres.RearGripAccel);
```

`FrontLateral` / `RearLateral` 是物理本帧报告的侧向占用。直道上它们接近 0，`here` 的曲率也接近 0，圆几乎全部留给刹车。真正入弯后，车底曲率和轴占用一起涨，圆自然让出横向。

各轴剩余纵向能力与右脚 `Room` 同一形式：

```csharp
float frontBrakeRoom = tyres.FrontGripAccel * MathF.Sqrt(MathF.Max(0f, brakeCircle * brakeCircle - frontUse * frontUse));
float rearBrakeRoom = tyres.RearGripAccel * MathF.Sqrt(MathF.Max(0f, brakeCircle * brakeCircle - rearUse * rearUse));
```

占用超过 `brakeCircle` 时，该轴剩余为 0，木桶把整车刹车压到 0，而不是放行 -57。

### 3.3 刹车比、`Technical` 与木桶

`BrakeBias(tyres, accel)` 提前到限幅之前调用，用的是尚未夹紧的 `accel`（小于 0 才会更新 `_brakeDial`）。返回值就是要写进 `AxleBias.FrontBrake` 的偏移，其中已含 `FrontBrakeBiasOffset`。本帧后面的 `return new DriverInput` 复用这个偏移，不再调用第二次。

真实前刹份额：

```csharp
float actualFrontShare = _layout.FixedFrontBrakeShare is { } fixedShare
    ? Math.Clamp(fixedShare + biasOffset, 0f, 1f)
    : Math.Clamp(optimalFront + biasOffset, 0f, 1f);
```

无固定分配时，`optimalFront` 与今天 `BrakeBias` 里的抓地比例相同：`front / (front + rear)`，其中 `front`、`rear` 是各轴扣掉侧向之后的剩余抓地。`FormulaAMixedCar` 有 `FixedFrontBrakeShare = 0.57f`，所以走第一支。`Technical` 差的车手 `FrontBrakeBiasOffset` 最大约 `±0.07`（`LanePerformance.MaximumBrakeBiasError`），份额会离开 0.57，木桶必须跟着变。

整车刹车间按 `CarPhysics.DistributedLongitudinalLimit` 的同一木桶，驱动侧自己算，不引用物理私有方法：

```csharp
float brakeRoom = float.PositiveInfinity;
if (actualFrontShare > 1e-3f)
    brakeRoom = MathF.Min(brakeRoom, frontBrakeRoom / actualFrontShare);
if (1f - actualFrontShare > 1e-3f)
    brakeRoom = MathF.Min(brakeRoom, rearBrakeRoom / (1f - actualFrontShare));
brakeRoom = float.IsFinite(brakeRoom) ? MathF.Min(_maxBrake, MathF.Max(0f, brakeRoom)) : _maxBrake;
```

首帧或离地后轴抓地还没建立时回退整段行程：

```csharp
if (tyres.FrontGripAccel <= 0.5f || tyres.RearGripAccel <= 0.5f)
    brakeRoom = _maxBrake;
```

0.5f 与 `CapFromAxles` 的轴抓地门槛相同。然后替换原来的负向夹紧：

```csharp
accel = Math.Clamp(accel, -brakeRoom, MathF.Max(0f, driveLimit));
```

夹紧之后的现有链保持原序：重返赛道的踏板限制、`BrakeFeelUse` 贴边回撤、`SlidePedals`、`ExitThrottle`、`FootBrakeRate = 600f` 的脚速。它们只能把已经进圆的请求再减小，不能把 `-brakeRoom` 重新放大到 -57。

平路、零侧向、`brakeCircle = 0.97`、前轴 30 m/s²、后轴 25 m/s²、份额 0.57 时：

```text
frontBrakeRoom = 29.1
rearBrakeRoom  = 24.25
brakeRoom = min(29.1 / 0.57, 24.25 / 0.43) = min(51.05, 56.40) = 51.05
```

小于 57。份额被 `Technical` 推到 0.64 时，木桶降到 `min(29.1/0.64, 24.25/0.36) = 45.47`。这就是拨盘打错时锁轴的能力差异，由圆的大小表达，而不是由 -57 的硬顶表达。

---

## 4. 第 2 层：无折角走廊、可达性门控、空间限速

`TrafficCorridor` 继续是 `internal sealed class`。`Update` 去掉标量 `maxBrakeAccel`，改为规划器输入：

```csharp
public void Update(
    in DriverContext context,
    ReadOnlySpan<float> freeSpeedPlan,
    in LaneEnvelope envelope,
    LaneShares shares,
    Func<int, float> cornerFactorAt,
    Func<int, float> brakeFactorAt,
    Func<int, LaneRoad> roadAt,
    Func<float, float> turnLagSeconds,
    float followGapSeconds,
    DrawnLine drawn,
    Func<int, float> currentReferenceOffset,
    in RacecraftIntent intent)
```

`cornerFactorAt` / `brakeFactorAt` / `roadAt` 的参数是未取模的站号，实现里用 `_lanes.Wrap`。`LaneEnvelope`、`LaneShares`、`LaneRoad` 都是现有类型：`LaneShares` 是 `sealed record`，`LaneRoad` 是 `readonly record struct`。

### 4.1 去掉近场硬裁剪，保留 C¹ 积分

`SolveSmoothLine` 删除两处硬裁剪：`k >= 6` 的斜率重写，以及 `idx >= BehindStations + 7` 的墙面 `Clamp`。

保留并作为唯一几何约束的是：

1. 边界内预平滑。目标偏移 `_targetDelta` 在 `_lowD/_highD` 内做 14 趟邻站平均，平滑结果再夹回边界。这一步发生在积分之前，夹的是目标，不是已经积出来的位置。
2. 从当前参考线（或车体，沿用现有 `startOnRef` 判定）对齐位置 `y` 和斜率 `slope`。
3. 向前积分时，斜率每站的变化不超过 `maxBend * spacing`，`maxBend = maxLatAccel / v²`。`maxLatAccel` 仍按场景取 `EmergencyMoveAccel = 7.2f`、并排 `5.0f` / `3.5f`、`LoadedMoveAccel = 2.8f`、`StraightMoveAccel = 4.6f`。
4. 预瞄点 `want` 先夹进近处走廊，让积分器提前弯向墙内，而不是撞上去再折返。
5. 积分后的 `y` 不再因为碰墙而改写 `slope`。事后平滑只做邻站平均，不再 `Clamp` 到墙上。

这样每一站的位置和一阶导数都由上一站连续积分得到，曲率变化率有界，线上没有折角。

### 4.2 画线倒推可达性门控

与 `ScaleMove` 同源，档位表显式写成：

```csharp
private static readonly float[] MoveScales = [1.0f, 0.6f, 0.35f, 0.15f, 0.0f];
private const float MoveSpeedToleranceMps = 0.5f;
```

步骤：

1. 以当前生效参考线为基底。`currentReferenceOffset(station)` 就是 `LaneDriver.ReferenceOffset`：有 `DrawnLine` 时是自由线加 `DrawnLine.At`，否则是 `LaneSet.BlendedOffset`。这条线记为 `still`。
2. 对 `still` 做一次窗口规划，得到 `stillSpeed = stillCeiling[0]`。
3. 预平滑后的攻击/防守目标记为 `target`。对每个 `scale`，候选为 `still + scale * (target - still)`，再按 4.1 的 C¹ 积分得到候选折线。
4. 对候选折线取曲率（弦长 3 站，与 `LaneDriver` 的 `CurvatureChordStations` 相同），用自由速度计划当 `speedGuess`，调用：

```csharp
LaneSpeedPlanner.LeadCurvatures(curvature, speedGuess, turnLagSeconds, _lanes.Spacing, lead);
LaneSpeedPlanner.CornerLimits(envelope, lead, factor, road, shares, limits);
LaneSpeedPlanner.PlanWindow(
    envelope, lead, step, factor, brake, road, shares,
    terminalSpeed, limits, ceiling);
```

`terminalSpeed` 取窗口末端的 `freeSpeedPlan`，不在地平线尽头发明一个停车点。`factor` 与 `brake` 来自 `cornerFactorAt` / `brakeFactorAt`，所以尾流和熟练度已经在里面。

5. 门槛：

```csharp
float bar = MathF.Min(egoSpeed, stillSpeed) - 0.5f;
bool pass = ceiling[0] >= bar;
```

从 `1.0` 往下找，第一个通过的档位就是最高可走档。`0.0` 恒通过，退回原线。弯中满载或重刹区内，高档的 `ceiling[0]` 会低于门槛，自动留下低档或原线。

唯一例外：`still` 本身在预测时域里会和故障车（`RivalForecast.Hazard`）或已经并排的车（`AlongsideNow`）发生横向重叠。此时允许一条通不过门槛的线，但只取能解开重叠的最低档，不取无条件的 `1.0`。`EmergencyAvoid` 为真时同样开启这个例外。其它情况，通不过就退档。

门控通过后才 `drawn.Set`。最高档为 0 且没有并排、没有承诺、偏移足够小（沿用现有 0.05 m 清线条件）时 `drawn.Clear()`。

`PassesDrivableGate(float candidateCeiling0, float egoSpeed, float stillCeiling0)` 是上述比较的纯函数，测试直接打它。

### 4.3 对手预测改用 `BrakeFrom`

`CaptureRivals` 删除 `maxBrakeAccel` 参数。故障车仍按现有方式减速停车并保持横向惯性，不给故障车编一套新模型。

正常对手在进入 36 步时间步循环**之前**，先沿对手前方 `look = Math.Min(60, _aheadStations)` 个站点做**单次** `BrakeFrom` 反向扫描（每名对手每次更新只扫 60 站，避免在 36 步循环内重复扫 `36 × 40` 次导致耗时超标），得到空间制动上限表 `rivalCeiling[m]`，禁止开根号：

```csharp
int look = Math.Min(60, _aheadStations);
Span<float> rivalCeiling = stackalloc float[look + 1];
int far = _lanes.Wrap(otherSt + look);
float offLineScale = MathF.Abs(other.TrackD - _lanes.Offset(LaneId.Free, otherSt)) > 2.0f ? 0.95f : 1.0f;
rivalCeiling[look] = offLineScale * PlanSpeedAt(freeSpeedPlan, far);
for (int m = look - 1; m >= 0; m--)
{
    int a = _lanes.Wrap(otherSt + m);
    float curvature = _lanes.CurvatureAtFreeOrLoaded(a); // 3站弦曲率
    float localLimit = offLineScale * PlanSpeedAt(freeSpeedPlan, a);
    rivalCeiling[m] = MathF.Min(
        localLimit,
        LaneSpeedPlanner.BrakeFrom(
            envelope, rivalCeiling[m + 1], curvature, _lanes.Spacing,
            cornerFactorAt(a), brakeFactorAt(a), roadAt(a), shares));
}
```
在随后的 36 个时间步积分中，直接按对手推进距离 `m = Clamp((curRelS - gap) / spacing, 0, look)` 线性读取 `rivalCeiling[m]` 作为当前位置的速度上限。

`BrakeFrom` 的签名是：

```csharp
internal static float BrakeFrom(
    in LaneEnvelope envelope, float vNext, float curvature, float ds,
    float factor, float brakeFactor, LaneRoad road, LaneShares shares)
```

它内部三次取步长中点的 `Deceleration`，`Deceleration` 再读 `LaneEnvelope.ForTurn`、`LaneRoad.Attitude`、`BrakingCoupling` 和 `Ellipse`。坡度、下压力和尾流因此自动进入对手的 `V[]`。

本步速度仍只在前 0.25 s 沿用测到的 `StepLongitudinalAccel`，之后用 `6f` 作为「计划允许范围内的恢复加速度」去贴近 `ceiling`，但不越过 `ceiling`。这里的 `6f` 是加速追随，不是减速公式。

`BuildEgoTimeline` 不再用 `+9f` / `-28f` 的固定加减速去猜本车。每一站直接取 `PlanSpeedAt(freeSpeedPlan, anchor + k)`，时间步是 `spacing / max(v, 6)`。采样对手预测时，时间轴和速度计划是同一条曲线。

### 4.4 诚实的 `_windowLimits`

`ApplyWindowLimits(int windowStartStation, Span<float> windowLimits)` 的签名和「把点限制平移进窗口并取 `Min`」的行为不变。变的是点从哪来。`LaneDriver.PlanBlendWindow` 在画线或存在交通限速时已经调用 `PlanAhead(..., _windowLimits, _windowSpeed)`，所以点上的一个速度会被反向扫成一条连续的收油/刹车曲线。走廊只负责把正确的点放进去。

对每个前方对手，按下面的顺序，命中一条就停止，不叠加第二条：

1. **已经抽头且横向净空打开。** `Slot != Behind`，并且 `|TrackD 差| >= 0.5 * (两车宽) + 0.30f`。该前车的限速贡献为 0：不添加 `SpeedLimitPoint`。`FullThrottleTow` 且相位是 `DraftTow` 时同样贡献为 0，好让直道把尾流吃完。
2. **直道被堵、无法抽头。** 相位不是入弯刹车区（沿用现有 `inBrakingZone` 判定：本车纵向加速度 `< -3`，或前方 10 站计划速度比当前站低 2.5 m/s 以上）。令 `Δv = egoSpeed - rival.Speed`。
   - `Δv <= 0`：前车在拉开，包括起步前车正在加速离开。禁止注入 `recoverV`，不添加限速。
   - `Δv > 0`：在预测追及站注入前车车速。追及距离 `gap * egoSpeed / Δv`，站号 `kCatch = floor(distance / spacing)`，`SpeedLimitPoint(kCatch, 6, rival.Speed)`。`PlanAhead` 从这一点反向扫出收油曲线，走廊不自己画那条曲线。
3. **`Behind` 跟车进入刹车区，防止手风琴追尾。** 把前车预测速度剖面 `V[]` 沿赛道向后（朝本车）平移 `d_tail + egoSpeed * t_react`，再写入经过的站。

```csharp
float boldness = Math.Clamp(abilities.Boldness * 0.01f, 0f, 1f);
float dTail = 3.8f - 1.3f * boldness;
float score = Math.Clamp(abilities.Reactions * 0.01f, 0f, 1f);
float tReact = 0.30f + (0.05f - 0.30f) * score;
float shift = dTail + egoSpeed * tReact;
```

`tReact` 与 `LanePerformance.ReactionSeconds` 同一闭式：`Reactions = 100` 时 0.05 s，`Reactions = 0` 时 0.30 s。`d_tail` 与第 5 节状态 2 共用，`boldness` 是 0..1 的评分份额，不是 0..100 的原始分。原始分 75 若直接乘 1.3 会得到负间距。

平移之后，站 `k` 看到的是对手在 `k` 对应弧长再往前 `shift` 米处的预测速度，也就是对手更晚、通常更慢的那一脚刹车被提前交给 `PlanAhead`。这是反应距离，不是另一套 `2 a s`。

夹击、三车并排把走廊压没时，保留现有 pinch 点的几何判定，但点的速度改成前车预测剖面在该站的 `V`，不再写 `pinchLeaderSpeed - 4f` 这种固定扣减。

---

## 5. 第 3 层：`RacecraftTactics` 六态闭环

新文件 `Drivers/Lanes/RacecraftTactics.cs`，`internal sealed class`。构造时预计算弯道表，之后只读：

```csharp
public RacecraftTactics(LaneSet lanes)
{
    _lanes = lanes;
    _freeK = CornerMap.FreeCurvature(lanes);
    _bends = CornerMap.Bends(lanes, _freeK);
    _pins = new CornerMap.Pins[_bends.Count];
    for (int i = 0; i < _bends.Count; i++)
        _pins[i] = CornerMap.PinsOf(lanes, _freeK, _bends, i);
}
```

签名就是源码里的：

```csharp
public readonly record struct Bend(int First, int Apex, int Last);
public readonly record struct Pins(int Entry, int Apex, int Exit, int Sign);
public static float[] FreeCurvature(LaneSet lanes);
public static List<Bend> Bends(LaneSet lanes, float[] k, float radius = 600f, float joinMeters = 30f);
public static Pins PinsOf(LaneSet lanes, float[] k, IReadOnlyList<Bend> bends, int index);
```

`Pins.Sign` 是弯心曲率的符号。`LaneSet` 的法向指向右侧（+d 为左）。左转曲率为正，内侧是 `+d`，因此内侧符号取 `Pins.Sign`，外侧取相反数。

每过一段弯的 `Exit`，把 `_defendedThisStraight = false`，并清掉这一段直道的防守符号。判定是：当前站相对该弯 `Exit` 的向前距离落在 `(0, 下一弯 Entry 之前)`，且与上一次记下的弯不同。没有弯的赛道上，防守态不启用，其它态仍按对手工作。

`Update` 的签名：

```csharp
public void Update(
    in DriverContext context,
    ReadOnlySpan<float> freeSpeedPlan,
    in LaneEnvelope envelope,
    LaneShares shares,
    Func<int, float> cornerFactorAt,
    Func<int, float> brakeFactorAt,
    Func<int, LaneRoad> roadAt,
    Func<float, float> turnLagSeconds);
```

公开读口：`RacecraftPhase Phase { get; }` 与 `RacecraftIntent Intent { get; }`。`DriverContext` 提供 `Profile.Abilities`、`Car`、`Frame`、`CarSnapshotIndex`、`RaceTimeSeconds`。对手运动学全部来自 `RaceFrameSnapshot` 里的 `RaceCarSnapshot`，不读走廊私有预测。

FIA 重叠与现有走廊同一几何：横向间距至少 `0.5f * (两车宽) + 0.38f`，纵向上本车前翼到达对方后轴，距离 `0.5f * me.LengthMeters + 0.5f * max(3f, other.WheelBaseMeters)`。并排滞回 0.8 m 只留在走廊的物理墙里；战术脑用无滞回的这一条，避免冷却锁和并排锁互相放行。

局部可行性一律走第 4.2 节那一组 `LeadCurvatures` + `CornerLimits` + `PlanWindow`，窗口 40 站。通过条件写成 `candSpeed[0] >= egoSpeed - 0.5f`。

### 5.1 状态 1 `StandingStart`

首次 `Update` 且 `egoSpeed < 5f && RaceTimeSeconds < 0.5f` 时记下 `_launchColumnD = me.TrackD` 与 `_launchStartS = me.TotalDistanceMeters`。在 `(TotalDistance - _launchStartS) < 150f && RaceTimeSeconds < 5f` 内，意图的 `TargetDeltaMeters` 等于 `_launchColumnD - freeOffset`，把车按在发车列上，抑制无谓抢线。

例外：正前方慢起步车满足 `Δv > 1.2f` 且左右至少一侧净空 `>= MinPassRoomMeters`（2.75 m）时，允许立即进入 `PullOutAttack`，不等待发车列结束。

进第一弯漏斗：下一个 `Pins.Entry` 距当前站不足 40 m，且与任何对手都没有 FIA 重叠时，结束发车列，回到 `Behind` 跟车。漏斗里已经重叠的车留在 `SideBySide`，不把它们挤回同一条发车线。

### 5.2 状态 2 `DraftTow` 与 `PullOutAttack`

只在直道或出弯上发起新的抽头。`inLoadedCorner` 为真且对手不是故障车时，不新开 `PullOutAttack`。`FollowGapSeconds > 0.05f` 表示车手被命令跟车，此时不抽头。

```csharp
float boldness = Math.Clamp(abilities.Boldness * 0.01f, 0f, 1f);
float dTail = 3.8f - 1.3f * boldness;
float dLat = MathF.Abs(targetDelta - (me.TrackD - freeHere));
float tMove = MathF.Sqrt(4f * dLat / 4.6f) + 0.15f;
float dPullout = dTail + MathF.Max(0f, egoSpeed - leader.Speed) * tMove;
```

`4.6f` 就是走廊直道横向加速度 `StraightMoveAccel`。`T = sqrt(4 |Δd| / a)` 是「加速半程、减速半程」走完横向距离的时间，再加 0.15 s 落稳。`d_pullout` 是尾距加上这段时间里按当前正速度差会追掉的距离。

- `gap > dPullout`：相位 `DraftTow`。`TargetDeltaMeters` 平滑偏向领跑车当前 `TrackD` 相对自由线的位置，也就是尾流中心。`FullThrottleTow = true`，走廊不注入限速，油门由第 1 层的 `tractionRoom` 自己顶住。
- `gap <= dPullout`（或已在 `_commitId == leader.Id` 锁定承诺中）：选边并锁定为 `PullOutAttack`。一旦 `_commitId` 锁定，后续帧不再因为 `gap` 在阈值附近微幅波动而退回 `DraftTow`（彻底防抖），一直保持到并排成立、完成超越、门关死中止（`< AbortRoomMeters`）或被拉开超过 32 m。

临近刹车区还想走内线时，先把内线候选（内侧符号乘 `HalfWidth - 1.48f`，1.48f 即走廊 `SafeEdgeMarginMeters`）送进局部 `PlanWindow`（120 站窗口，覆盖最长重刹区，复用预分配缓冲零 GC）。`candSpeed[0] < egoSpeed - 0.5f` 则否决这条内线，改试外线；外线也不行就留在 `DraftTow` / `Behind`。这是鱼雷检查：抽头不能变成一条当前速度已经走不完的线。

入弯前尚未形成 FIA 前翼对后轴重叠，且所选一侧剩余空间 `< AbortRoomMeters`（2.30 m）时，相位回到 `Behind`，`_abortUntil = now + 2.8f`（沿用 `AbortCooldownSeconds`）。冷却期间不允许对同一辆车再次 `PullOutAttack`。已经重叠的车不走这条退回，进入状态 4。

### 5.3 状态 3 `OneMoveDefend`

每段直道最多一次，且**一旦触发必须锁住防守线位持续输出直到入弯点 `Entry`**（绝不在下一帧因为 `_defendedThisStraight == true` 就丢失意图弹回自由线）。
- **触发条件**（`_defendedThisStraight == false`）：`Defending` 评分份额 `>= 0.30f`，本车速度 `> 30f`，后方对手在 `-(12f + 14f * defendingShare)` 到 `-4.2f` 米之间且闭合速度 `> 0.4f` m/s，下一弯 `Pins.Sign != 0`。触发时锁存 `_defendedThisStraight = true`、`_defendBend = bend` 以及防守路面横向份额 `_defendRoadShare`（内侧留白或中内线挤压位置）。
- **保持与刹车冻结**（`_defendedThisStraight == true` 且尚未到达 `_pins[_defendBend].Entry`）：后续每一帧持续输出 `OneMoveDefend` 与锁存的内侧偏移，禁止跳回自由线造成二次变线；一旦进入刹车区（`inBrakingZone`）或对手达成 FIA 重叠，立即置 `FreezeLateral = true` 将横向比例冻结在当前车道位置，严禁刹车区变线。过了 `Entry` 后释放直道防守偏移交给弯道几何，但 `_defendedThisStraight` 保持为 `true` 直到驶出 `Exit` 才重置。

### 5.4 状态 4 `SideBySide`

FIA 重叠成立后相位改为 `SideBySide`，`SideSign` 按 `me.TrackD >= other.TrackD ? +1 : -1` 锁定，`TargetDeltaMeters = float.NaN`（不硬锁死常数 `TrackD`，交由走廊的 `_lowD / _highD` 对手预测边界自然挤出平顺无碰的并排内外弧线）。走廊用这个符号占槽，并在弯道上附加已有的曲率外摆 `min(0.60f, 16f * |k|)`。第三辆车出现时仍走现有的传递留空：按横向次序把 `Width + StackSlotMarginMeters (0.82f) + 0.8f * bendSweep` 从两侧墙往里推，本车在中间时偏好取走廊中线。战术脑不在这一态里改槽。

重叠解开且本车已经超过对方后轴 `halfLen + 1.8f` 之后，相位离开 `SideBySide`。若当时仍在受载弯里，走廊现有的出弯保持 1.4 s 继续生效，避免刚超完就切回自由线。

### 5.5 状态 5 `Switchback`

同时满足才考虑：

- 下一弯弯心的自由线计划速度 `< 58f` m/s。
- 正前方对手 `0 < gap < 14f`。
- 对手相对该弯自由入弯线向内偏离 `> 1.6f` m（内偏 = `Pins.Sign * (other.TrackD - freeEntry)`）。
- 内侧剩余 `< AbortRoomMeters`（2.30 m），外侧剩余 `>= MinPassRoomMeters`（2.75 m）。
- 本车不在发车列，不在冷却，不在蓝旗。

参考偏移的形状，站号从当前走到 `Exit`：

- 现在到 `Entry`：收到外侧宽入弯，`outside * max(0.4f, HalfWidth - 1.48f)`。
- `Entry` 到弯心之后 4 站：继续留在外侧，形成晚弯心。
- 弯心之后到 `Exit`：收到内侧出弯边，`inside * max(0.4f, HalfWidth - 1.48f)`。

这条偏移送进局部 `PlanWindow`。同时要求 `candSpeed[0] >= egoSpeed - 0.5f`，并且候选在 `Exit` 那一站的速度高于留在对手内侧后面时的窗口速度。两个都成立才把相位设为 `Switchback`，并将完整空间数组写入 `Intent.SpatialDeltaProfile` 交给走廊逐站覆盖 `_prefD`（绝不只传单站标量）；否则保持 `Behind` 或已有的 `PullOutAttack`。过了 `Exit` 或已经超到前面，相位结束。

### 5.6 状态 6 `BlueFlagYield`

对手满足：

```csharp
other.RaceDistanceMeters - me.RaceDistanceMeters > 0.75f * me.TrackLengthMeters
```

并且人在本车后方、正在接近。追及时间门槛：

```csharp
float awareness = Math.Clamp(abilities.Awareness * 0.01f, 0f, 1f);
float catchSeconds = 1.20f + 0.40f * (awareness - 0.5f);
```

`Awareness = 50` 时正好 1.20 s，0 分时 1.00 s，100 分时 1.40 s。追及时间是后方负间距的绝对值除以 `max(0.1, other.Speed - egoSpeed)`。

让行只发生在非高载区域：不是 `inLoadedCorner`（弯心 `|k| v² > 13` 或正在刹车）。意图把本车平滑移到自由线相对来车的另一侧，两车边缘净空至少 3.0 m，也就是中心距至少 `0.5f * (两车宽) + 3.0f`，并且结果夹在 `HalfWidth - 1.48f` 之内。高载弯里不横移，只保持已经在直道上完成的偏移，避免弯心里再变线。

来车沿赛道的间距进入 10 m 以内，或已经并排时，`YieldSpeedScale = 0.88f`。走廊把它写成 `0.88 * egoSpeed` 的空间限速，由 `PlanAhead` 扫成缓和的收油，而不是一脚 -57。对方前翼超过本车后轴 `halfLen + 1.8f` 之后，相位回到 `Behind`，限速和横向让行一起撤销。

优先级从高到低：`BlueFlagYield`、紧急故障避让、`SideBySide`、`Switchback`、`PullOutAttack`、`OneMoveDefend`、`StandingStart`、`DraftTow`、`Behind`。高优先级成立时不让低优先级改写 `Intent`。

---

## 6. 测试矩阵与验收硬指标

Release 构建：

```text
dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release
```

六车网格沿用 `TrafficCorridorTests.SixDistinctAbilityCarsStandingStartFromGridBenchmarkAndCleanRace` 与 `LaneDriverDemo` 的同一发车表：`TrackData.Grids` 的 1 到 6 号位，`DevelopmentTyres.SuperId`，起步速度 0，`DriverAbilities` 从 P1-Rookie 到 P6-Champion 与演示程序一致。仿真 180 s，步长 `1/60` s。

硬指标：

| 量 | 要求 |
| --- | --- |
| `Contacts` | 0。某一 60 Hz 步上任一车 `HitCarThisStep` 即计 1，全程为 0 |
| `WallTouch` | 0。任一车 `BoundaryContactSeconds > 0` 即计 1，全程为 0 |
| `Spins` | 0。每车结束时 `ICarMotion.Motion(state).Spins == 0` |
| `OffTrackSeconds` | 0。`Progress.Region != TrackRegion.RacingSurface` 的累计秒数为 0 |
| `SmokeSeconds` | P5-Contender 与 P6-Champion 为 0。冒烟帧定义与现测相同：`OverLimit > 0.1f` 或 `Spinning`，且 `Speed > 9f`，且人还在赛道上 |
| 耗时 | 六车每步均值 `< 500` µs。计时包住 `RaceSimulation.Step`，不含断言本身 |
| 发车 | 前 70 m 相对发车列的横向摆动 `< 0.35f` m，与现测相同 |

P1 到 P4 不要求零冒烟。能力低的车手允许 `LimitOveruse` / `CornerOverstep` 把圆用满，但不允许因此撞车、碰墙、旋转或离线。

六项专项，各自一条测试，不靠 180 s 总测代替：

| 专项 | 测试 | 必须看到的行为 |
| --- | --- | --- |
| 发车列 | `StandingStart_HoldsColumnAndPullsOnlyASlowStarter` | 前 70 m 摆动 `< 0.35` m；前方慢车 `Δv > 1.2` 时 8 s 内横向分开 `> 1.5` m；无接触 |
| 尾流与抽头 | `DraftTow_HoldsThrottleUntilPulloutGap` | 间距仍大于 `d_pullout` 的前 2 s，追赶车速度不低于起步速度减 3 m/s；随后允许变线；无接触、无出界 |
| 一次防守 | `OneMoveDefend_MovesInsideOnceAndHoldsToEntry` | 直道上内侧偏移只跨过自由线一次，入弯前不回到另一侧；无接触 |
| 并排 | `SideBySide_ThreeWideHoldsSlotsWithoutContact` | 三车横向净空保持到弯道之后，`HitCarThisStep` 与 `BoundaryContactSeconds` 全 0 |
| 交叉线 | `Switchback_ClearsDeepInsideDefenderWithoutSpin` | 慢弯、前车深内线时追赶车从外侧完成超越或并排，`Spins == 0`，无接触、无出界 |
| 蓝旗 | `BlueFlag_YieldsOnlyWhenLappedAndCloses` | 纯函数：赛程差 `> 0.75 * L` 且追及时间 `< catchSeconds` 才为真；净空公式 `>= 3` m；未套圈的近距离后车不会被让到反侧 |

另外两条能力圆与走廊的门控测试不进入六大场景，但属于验收的一部分：

- `BrakeFeelAbilityTests`：平路木桶约 51 m/s² 而不是 57；车底零曲率比 40 m/s² 的预瞄曲率至少多 15 m/s² 刹车余量；任一轮抓地 `<= 0.5f` 时回到 `_maxBrake`；`Technical` 把份额从 0.57 推到 0.64 时木桶变小；`BrakeCircle` 在有 `TyreStyle.Brake` 时走 `SustainablePeakShare * Brake + overuse`，不乘 `familiar.Brake`。
- `CorridorDrivabilityTests`：`PassesDrivableGate` 与 0.5 m/s 门槛一致；直道上 `Δv <= 0` 的近距离前车不会把后车刹下去。

演示程序 `Tools/Demo/LaneDriverDemo.cs` 在环境变量 `STINTEGY_HEADLESS_BENCH=1` 时，用与测试相同的六车发车跑 30 s，打印 `Contacts`、`WallTouch`、`Spins`、`OffTrackSeconds`、P5/P6 的 `SmokeSeconds` 和每步均值微秒，然后 `GetTree().Quit()`。不设该变量时，演示仍只做现在的三维发车，不改变画面路径。
