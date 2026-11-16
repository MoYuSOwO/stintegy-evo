# 三层竞速控制 Implementation Plan

日期：2026-11-16  
状态：这是当时按任务拆开的实现计划，勾选框保持写计划时的样子。任务 1、2 已提交（`daf185b`、`d603fc1`）。任务 3 以阶段形式提交在 `ff44351`，还差的三处和逐帧在 `Drivers/RACECRAFT_PHASE.md`。任务 4（六车网格、锁死、出界、打转、Godot）还没做。规格在 `Drivers/RACECRAFT_DESIGN.md`。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把竞速控制拆成 12 Hz 的 `RacecraftTactics`、12 Hz 的 `TrafficCorridor` 和 60 Hz 的 `LaneDriver`，左脚用对称能力刹车圆，画线必须通过 `PlanWindow` 可达性门控，空间限速只来自 `LaneSpeedPlanner`。

**Architecture:** 战术脑只产出 `RacecraftIntent`。走廊把意图积成一条 C¹ 偏移，用 `LeadCurvatures` + `CornerLimits` + `PlanWindow` 从高档退到低档，并把诚实的速度点写进 `_windowLimits`。`LaneDriver` 在踏板限幅前用车底曲率和真实前刹份额计算 `brakeRoom`，负向加速度不得再直接放行 `-_maxBrake`。三层只向下传数据。

**Tech Stack:** .NET 8、xUnit 2.9.3、现有 `StintegyEVO.Drivers` 程序集（`InternalsVisibleTo` 已经开放给 `StintegyEVO.Drivers.Tests`）、`LaneSpeedPlanner`、`CornerMap`、`DrawnLine`、`FormulaAMixedCar`（`CombinedGripLimiterStrength = 0f`，`FixedFrontBrakeShare = 0.57f`，`MaxBrakeAccel = 57f`）。

## Global Constraints

- 禁止在 `LaneSpeedPlanner` 之外新写 `v * v + 2f * a * s`、`MathF.Sqrt(v * v + 2f * a * s)` 或 `v * v / (2f * a)` 来决定刹车点。`BrakeFrom` 内部的三遍中点积分保留。`PlannedBraking` 是在读已有速度表，保留。
- `SolveSmoothLine` 禁止 `k >= 6` 的斜率重写，禁止 `idx >= BehindStations + 7` 的墙面 `Clamp`。
- `brakeCircle` 禁止再乘 `familiar.Brake`。熟练度已经在 `_brake[lane]` 里。
- 刹车圆的几何侧向用 `BlendedCurvature(station)` 的 `here` 加 `RoadAttitude`，禁止用 `previewStation` 或 `wanted`。
- `GripUse.FrontGripAccel <= 0.5f` 或 `RearGripAccel <= 0.5f` 时 `brakeRoom = _maxBrake`。
- 门控档位就是 `MoveScales = [1.0f, 0.6f, 0.35f, 0.15f, 0.0f]`，门槛 `0.5f` m/s。
- 战术与走廊同拍：`CorridorIntervalSeconds = 1f / 12f`。踏板仍是 60 Hz。
- `boldness`、`defending`、`awareness`、`overtaking` 都是 `Math.Clamp(rating * 0.01f, 0f, 1f)`，不是 0..100 的原始分。
- `d_tail = 3.8f - 1.3f * boldness`。`T_move = MathF.Sqrt(4f * |Δd_lat| / 4.6f) + 0.15f`。`d_pullout = d_tail + MathF.Max(0f, Δv) * T_move`。
- `t_react = 0.30f + (0.05f - 0.30f) * reactionsScore`，与 `LanePerformance.ReactionSeconds` 相同。
- 交叉线只在弯心计划速度 `< 58f`、前车间距 `< 14f`、内偏 `> 1.6f` m 时生成。
- 蓝旗：`other.RaceDistanceMeters - me.RaceDistanceMeters > 0.75f * TrackLengthMeters`，追及门槛 `1.20f + 0.40f * (awareness - 0.5f)` 秒，净空 `>= 3.0f` m，10 m 内限速比例 `0.88f`。
- 六车验收：`Contacts == 0`、`WallTouch == 0`、`Spins == 0`、`OffTrackSeconds == 0`、P5 与 P6 的 `SmokeSeconds == 0`、每步均值 `< 500` µs。
- 不修改 `FormulaAMixedCar` 的 `CombinedGripLimiterStrength` 或 `FixedFrontBrakeShare`。
- 每个任务单独提交。本计划的文档提交不包含任何 `.cs` 改动。

## 文件边界

- `Drivers/Lanes/LaneDriver.cs`：60 Hz 踏板。任务 1 加入 `brakeRoom`。任务 2 把走廊调用改成包络访问器。任务 3 加上 12 Hz 战术调用。
- `Drivers/Lanes/TrafficCorridor.cs`：几何、门控、`_windowLimits`。任务 2 改预测和门控，暂时仍保留旧的选边函数。任务 3 删掉选边函数，改吃 `RacecraftIntent`。
- `Drivers/Lanes/RacecraftTactics.cs`：任务 3 新建。枚举、意图和六态都在这个文件里。任务 2 先把同一份枚举和意图放在 `TrafficCorridor.cs` 末尾，任务 3 原样搬过来并删掉走廊里的副本。
- `Drivers/Tests/BrakeFeelAbilityTests.cs`、`CorridorDrivabilityTests.cs`、`RacecraftTacticsTests.cs`：各任务自己的失败测试。
- `Drivers/Tests/TrafficCorridorTests.cs` 与 `Tools/Demo/LaneDriverDemo.cs`：任务 4 的网格硬指标和演示核对。

---

### Task 1: Symmetric Ability-Linked Brake Friction Circle (`brakeRoom`)

**Files:**

- Modify: `Drivers/Lanes/LaneDriver.cs`（`GetControl` 里 `accel = Math.Clamp(accel, -_maxBrake, ...)` 那一行，约 1096 行；`BrakeBias` 之后，约 1699 行）
- Test: `Drivers/Tests/BrakeFeelAbilityTests.cs`

**Interfaces:**

- Consumes: `GripUse(float FrontGripAccel, float RearGripAccel, float FrontLateral, float RearLateral, float FrontLongitudinal, float RearLongitudinal, float RearSlideSeverity)`；`ICarGripFeel.SustainablePeakShare`；`TyreStyle.Brake`；`LanePerformance.LimitOveruse`；`LanePerformance.CornerOverstep`；`LanePerformance.FrontBrakeBiasOffset`；`AxleLayout.FixedFrontBrakeShare`；`AxleLayout.FrontWeightShare`；`LaneDriverGains.BrakeFeelUse`（默认 `0.97f`）；`RoadAttitude.CurvatureDemandScale`；`RoadAttitude.LateralGravityDemand(float gravity, float speedMetersPerSecond)`；现有私有方法 `BrakeBias(GripUse tyres, float accel)`。
- Produces:

```csharp
internal static float BrakeCircle(
    float sustainablePeakShare, float? styleBrake, float brakeFeelUse,
    float tyreAuthority, float overuse, float misread);

internal static float DistributedBrakeRoom(
    GripUse tyres, float hereCornerAccel, float frontWeightShare,
    float brakeCircle, float actualFrontShare, float maxBrake);
```

`GetControl` 在限幅前调用二者，负向夹紧改为 `-brakeRoom`，`DriverInput.Axles` 使用同一次 `BrakeBias` 的返回值。

- [ ] **Step 1: Write the failing test**

新建 `Drivers/Tests/BrakeFeelAbilityTests.cs`：

```csharp
using StintegyEVO.Core.Cars;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class BrakeFeelAbilityTests
{
    private static GripUse Axles(float front, float rear, float frontLat = 0f, float rearLat = 0f) =>
        new(front, rear, frontLat, rearLat, 0f, 0f, 0f);

    [Fact]
    public void BrakeCircle_UsesStyleBrakeWithoutFamiliarity()
    {
        float noStyle = LaneDriver.BrakeCircle(0.98f, null, 0.97f, 1f, 0f, 0f);
        float styled = LaneDriver.BrakeCircle(0.98f, 1f, 0.97f, 0.5f, 0.02f, 0.05f);
        Assert.Equal(0.97f, noStyle, 3);
        Assert.Equal(1.10f, styled, 3);
    }

    [Fact]
    public void StraightBrakeRoom_IsTheAxleBarrelNotRawMaxBrake()
    {
        float room = LaneDriver.DistributedBrakeRoom(Axles(30f, 25f), 0f, 0.47f, 0.97f, 0.57f, 57f);
        Assert.InRange(room, 51.0f, 51.1f);
        Assert.True(room < 57f);
    }

    [Fact]
    public void HereCorner_KeepsMoreBrakeThanAnEntryPreview()
    {
        GripUse tyres = Axles(30f, 25f, 0.05f, 0.05f);
        float here = LaneDriver.DistributedBrakeRoom(tyres, 0f, 0.47f, 0.97f, 0.57f, 57f);
        float preview = LaneDriver.DistributedBrakeRoom(tyres, 40f, 0.47f, 0.97f, 0.57f, 57f);
        Assert.True(here > preview + 15f, $"here {here:F2} should not spend the preview corner {preview:F2}");
    }

    [Fact]
    public void ColdAxles_FallBackToMaxBrake()
    {
        float frontCold = LaneDriver.DistributedBrakeRoom(Axles(0.4f, 25f), 0f, 0.47f, 0.97f, 0.57f, 57f);
        float rearCold = LaneDriver.DistributedBrakeRoom(Axles(30f, 0.5f), 0f, 0.47f, 0.97f, 0.57f, 57f);
        Assert.Equal(57f, frontCold);
        Assert.Equal(57f, rearCold);
    }

    [Fact]
    public void TechnicalOffset_ShrinksTheBarrel()
    {
        GripUse tyres = Axles(30f, 25f);
        float balanced = LaneDriver.DistributedBrakeRoom(tyres, 0f, 0.47f, 0.97f, 0.57f, 57f);
        float forward = LaneDriver.DistributedBrakeRoom(tyres, 0f, 0.47f, 0.97f, 0.64f, 57f);
        Assert.True(forward < balanced - 4f, $"forward {forward:F2} balanced {balanced:F2}");
    }
}
```

`styled` 的手算：`(0.98f * 1f + 0.02f) * (1f + 2f * 0.05f) = 1.10f`。平路木桶：`min(30*0.97/0.57, 25*0.97/0.43) = 51.0526`。40 m/s² 的预瞄需求把后轴占用抬到 `0.53*40/25 = 0.848`，木桶约 27，比车底至少少 15。

- [ ] **Step 2: Run test to verify it fails**

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~BrakeFeelAbilityTests`

Expected: FAIL，编译错误 `CS0117`：`LaneDriver` 没有 `BrakeCircle`。

- [ ] **Step 3: Implement the brake circle**

在 `BrakeBias` 方法之后加入：

```csharp
internal static float BrakeCircle(
    float sustainablePeakShare, float? styleBrake, float brakeFeelUse,
    float tyreAuthority, float overuse, float misread)
{
    float baseShare = styleBrake is { } brake
        ? sustainablePeakShare * brake + overuse
        : (brakeFeelUse + overuse) * tyreAuthority;
    return baseShare * (1f + 2f * misread);
}

internal static float DistributedBrakeRoom(
    GripUse tyres, float hereCornerAccel, float frontWeightShare,
    float brakeCircle, float actualFrontShare, float maxBrake)
{
    if (tyres.FrontGripAccel <= 0.5f || tyres.RearGripAccel <= 0.5f)
        return maxBrake;
    float frontW = Math.Clamp(frontWeightShare, 0.05f, 0.95f);
    float frontUse = Math.Clamp(MathF.Max(tyres.FrontLateral, frontW * hereCornerAccel / tyres.FrontGripAccel), 0f, 1.5f);
    float rearUse = Math.Clamp(MathF.Max(tyres.RearLateral, (1f - frontW) * hereCornerAccel / tyres.RearGripAccel), 0f, 1.5f);
    float frontRoom = tyres.FrontGripAccel * MathF.Sqrt(MathF.Max(0f, brakeCircle * brakeCircle - frontUse * frontUse));
    float rearRoom = tyres.RearGripAccel * MathF.Sqrt(MathF.Max(0f, brakeCircle * brakeCircle - rearUse * rearUse));
    float share = Math.Clamp(actualFrontShare, 0f, 1f);
    float limit = float.PositiveInfinity;
    if (share > 1e-3f)
        limit = MathF.Min(limit, frontRoom / share);
    if (1f - share > 1e-3f)
        limit = MathF.Min(limit, rearRoom / (1f - share));
    return float.IsFinite(limit) ? MathF.Min(maxBrake, MathF.Max(0f, limit)) : maxBrake;
}

private float ActualFrontShare(GripUse tyres, float biasOffset)
{
    if (_layout.FixedFrontBrakeShare is { } fixedShare)
        return Math.Clamp(fixedShare + biasOffset, 0f, 1f);
    float front = tyres.FrontGripAccel * MathF.Sqrt(MathF.Max(0f, 1f - tyres.FrontLateral * tyres.FrontLateral));
    float rear = tyres.RearGripAccel * MathF.Sqrt(MathF.Max(0f, 1f - tyres.RearLateral * tyres.RearLateral));
    float optimal = front + rear > 1f ? front / (front + rear) : 0.5f;
    return Math.Clamp(optimal + biasOffset, 0f, 1f);
}
```

把 `GetControl` 里这一行：

```csharp
accel = Math.Clamp(accel, -_maxBrake, MathF.Max(0f, driveLimit));
```

换成：

```csharp
float biasOffset = BrakeBias(tyres, accel);
float overuse = _performance?.LimitOveruse ?? 0f;
float misread = _performance?.CornerOverstep ?? 0f;
float brakeCircle = BrakeCircle(
    _feel.SustainablePeakShare, _style?.Brake, Gains.BrakeFeelUse, _tyreAuthority, overuse, misread);
float hereCorner = MathF.Abs(
    att.CurvatureDemandScale * speed * speed * here
    + att.LateralGravityDemand(StandardGravity, speed));
float brakeRoom = DistributedBrakeRoom(
    tyres, hereCorner, _layout.FrontWeightShare, brakeCircle, ActualFrontShare(tyres, biasOffset), _maxBrake);
accel = Math.Clamp(accel, -brakeRoom, MathF.Max(0f, driveLimit));
```

`here`、`att`、`speed`、`tyres` 都是 `GetControl` 里已经存在的局部变量。`here` 是 `BlendedCurvature(station)`，不要换成 `wanted` 或 `previewStation`。

把返回处的：

```csharp
Axles = new AxleBias(BrakeBias(tyres, accel), 0f),
```

换成：

```csharp
Axles = new AxleBias(biasOffset, 0f),
```

`biasOffset` 必须在夹紧之前就算好，后面的脚速限制不能再调用 `BrakeBias`。重返赛道、`BrakeFeelUse` 回撤、`SlidePedals`、`ExitThrottle`、`FootBrakeRate` 仍留在夹紧之后。

- [ ] **Step 4: Run test to verify it passes**

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~BrakeFeelAbilityTests`

Expected: PASS，5 个事实全部通过。

- [ ] **Step 5: Commit**

```bash
git add Drivers/Lanes/LaneDriver.cs Drivers/Tests/BrakeFeelAbilityTests.cs
git commit -m "feat(drivers): cap braking with the ability friction circle"
```

---

### Task 2: Pure Geometry Corridor, `PlanWindow` Drivable-Line Gate and Spatial Limits

**Files:**

- Modify: `Drivers/Lanes/TrafficCorridor.cs`
- Modify: `Drivers/Lanes/LaneDriver.cs`（走廊调用约 813 行；`BlendTargetCeiling` 约 2237 行；`CorridorIntervalSeconds` 约 279 行）
- Test: `Drivers/Tests/CorridorDrivabilityTests.cs`

**Interfaces:**

- Consumes: 任务 1 不改变走廊签名。本任务使用已有的

```csharp
public static void LeadCurvatures(ReadOnlySpan<float> curvature, ReadOnlySpan<float> speedGuess, Func<float, float> lagSeconds, float spacing, Span<float> into);
public static void CornerLimits(in LaneEnvelope envelope, ReadOnlySpan<float> curvature, ReadOnlySpan<float> factor, ReadOnlySpan<LaneRoad> road, LaneShares shares, Span<float> limits);
public static void PlanWindow(in LaneEnvelope envelope, ReadOnlySpan<float> curvature, ReadOnlySpan<float> stepLength, ReadOnlySpan<float> factor, ReadOnlySpan<float> brakeFactor, ReadOnlySpan<LaneRoad> road, LaneShares shares, float terminalSpeed, ReadOnlySpan<float> limits, Span<float> speed);
internal static float BrakeFrom(in LaneEnvelope envelope, float vNext, float curvature, float ds, float factor, float brakeFactor, LaneRoad road, LaneShares shares);
public void ApplyWindowLimits(int windowStartStation, Span<float> windowLimits);
public float Seconds(float speed); // TurnLag
```

- Produces: `RacecraftPhase`、`RacecraftIntent`（本任务先放在 `TrafficCorridor.cs` 末尾，任务 3 搬进 `RacecraftTactics.cs`），以及

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
    in RacecraftIntent intent);

internal static bool PassesDrivableGate(float candidateCeiling0, float egoSpeed, float stillCeiling0);
```

`LaneDriver` 的调用去掉 `_envelope.Brake(...)` 这个标量。旧的 `UpdateTacticalCommitments` 和 `UpdateOneMoveDefence` 本任务保留，任务 3 再删。

- [ ] **Step 1: Write the failing test**

新建 `Drivers/Tests/CorridorDrivabilityTests.cs`：

```csharp
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class CorridorDrivabilityTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void Gate_MatchesTheHalfMeterMonacoBar()
    {
        Assert.True(TrafficCorridor.PassesDrivableGate(70f, 70f, 69f));
        Assert.True(TrafficCorridor.PassesDrivableGate(68.5f, 70f, 69f));
        Assert.False(TrafficCorridor.PassesDrivableGate(68.4f, 70f, 69f));
        Assert.False(TrafficCorridor.PassesDrivableGate(40f, 75f, 74f));
        Assert.True(TrafficCorridor.PassesDrivableGate(50f, 80f, 49.6f));
    }

    [Fact]
    public void OpeningLeaderOnAStraight_DoesNotDragTheFollower()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        float start = track.StartingLineS;

        (RaceCar leader, LaneDriver leaderDriver) = LaneGrid.Seat(
            simulation, file, lanes, "leader", StockCar.Build(compound), compound,
            start + 6f, 0f, 50f,
            abilities: new DriverAbilities { Pace = 90, Consistency = 95, Focus = 95, Precision = 95 });
        leaderDriver.SpeedCap = 50f;

        (RaceCar follower, LaneDriver followerDriver) = LaneGrid.Seat(
            simulation, file, lanes, "follower", StockCar.Build(compound), compound,
            start, 0f, 48f,
            abilities: new DriverAbilities { Pace = 90, Consistency = 95, Focus = 95, Precision = 95 });
        followerDriver.SpeedCap = 48f;

        float slowest = float.PositiveInfinity;
        bool hit = false;
        for (int step = 0; step < 60 * 3; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || follower.HitCarThisStep;
            if (step > 30)
                slowest = MathF.Min(slowest, follower.State.Speed);
        }

        Assert.False(hit, "opening pair made contact");
        Assert.True(slowest > 44f, $"follower was braked to {slowest:F1} m/s while the leader was pulling away");
    }
}
```

门槛手算：`Min(70, 69) - 0.5 = 68.5`，所以 68.5 通过、68.4 失败。`Min(80, 49.6) - 0.5 = 49.1`，候选 50 通过。行为测试里两车中心距 6 m，后车 48、前车 50，`Δv < 0`。旧代码的 `desiredGap` 大约 14 m，会注入 `recoverV ≈ 42`，3 秒内后车会被拉到 44 以下。

- [ ] **Step 2: Run test to verify it fails**

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~CorridorDrivabilityTests`

Expected: FAIL。`CS0117`：`TrafficCorridor` 没有 `PassesDrivableGate`。行为测试在这一步还编不过，门控函数落地后若减速公式还在，第二条会断言失败。

- [ ] **Step 3: Implement the corridor and the call site**

把 `CorridorIntervalSeconds` 改为：

```csharp
private const float CorridorIntervalSeconds = 1f / 12f;
```

`LaneDriver` 里走廊调用整段换成：

```csharp
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
    _drawn!,
    ReferenceOffset,
    RacecraftIntent.None);
```

`BlendTargetCeiling` 整段换成规划器窗口，删掉 `MathF.Sqrt(cornerSpeed * cornerSpeed + 2f * brake * distance)`：

```csharp
private float BlendTargetCeiling(float station)
{
    if (MathF.Abs(TargetRange - Range) < 0.02f && MathF.Abs(TargetSide - Side) < 0.02f)
        return float.PositiveInfinity;
    int n = _windowSpeed.Length;
    int start = (int)MathF.Floor(station);
    float range = Math.Clamp(TargetRange, 0f, 1f);
    float side = Math.Clamp(TargetSide, -1f, 1f);
    var curvature = new float[n];
    var step = new float[n];
    var factor = new float[n];
    var brake = new float[n];
    var road = new LaneRoad[n];
    var guess = new float[n];
    var limits = new float[n];
    var speed = new float[n];
    for (int k = 0; k < n; k++)
    {
        int i = start + k;
        curvature[k] = BlendedCurvatureFor(i, range, side);
        step[k] = _lanes.Spacing;
        factor[k] = FactorFor(i, range, side);
        brake[k] = BlendOf(lane => _brake[(int)lane], i, range, side);
        road[k] = RoadAt(i, _lanes.BlendedOffset(i, range, side));
        guess[k] = MathF.Max(8f, _carSpeed);
    }
    LeadCurvatures(curvature, guess, n);
    int end = _lanes.Wrap(start + n - 1);
    float terminal = MathF.Min(_speed[0][end], MathF.Min(_speed[(int)LaneId.Left][end], _speed[(int)LaneId.Middle][end]));
    LaneSpeedPlanner.CornerLimits(_envelope, curvature, factor, road, _planShares, limits);
    LaneSpeedPlanner.PlanWindow(
        _envelope, curvature, step, factor, brake, road, _planShares, terminal, limits, speed);
    return speed[0];
}
```

`LeadCurvatures(float[] curvature, float[] guess, int n)` 是 `LaneDriver` 已有的私有包装，它调用 `LaneSpeedPlanner.LeadCurvatures(..., _turnLag.Seconds, _lanes.Spacing, ...)`。这里继续用那一层，保证滞后和踏板一致。

在 `TrafficCorridor.cs` 文件末尾、命名空间之内，先放上任务 3 要搬走的契约：

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
    public int SideSign { get; init; }
    public float TargetDeltaMeters { get; init; }
    public float[]? SpatialDeltaProfile { get; init; }
    public bool FreezeLateral { get; init; }
    public bool EmergencyAvoid { get; init; }
    public float YieldSpeedScale { get; init; }
    public bool FullThrottleTow { get; init; }
    public string? FocusRivalId { get; init; }

    public static RacecraftIntent None => new() { TargetDeltaMeters = float.NaN };
}
```

`Update` 签名改为接口里的那一版。方法体开头仍清 `_limitPoints`、捕获发车列、调用 `BuildEgoTimeline`、`CaptureRivals`、`UpdateTacticalCommitments`、`UpdateOneMoveDefence`。`CaptureRivals` 去掉 `maxBrakeAccel`，加上 `envelope`、`shares` 和三个访问器。在 `SolveSmoothLine` 之前，若 `intent.TargetDeltaMeters` 有限，用它覆盖 `_prefD`；`FreezeLateral` 时改钉 `me.TrackD`。`SolveSmoothLine` 增加门控参数。`YieldSpeedScale > 0` 时在限速列表末尾追加 `new SpeedLimitPoint(0, _aheadStations, intent.YieldSpeedScale * egoSpeed)`。

`BuildEgoTimeline` 整段换成：

```csharp
private void BuildEgoTimeline(int anchor, float egoSpeed, ReadOnlySpan<float> freeSpeedPlan)
{
    float spacing = _lanes.Spacing;
    float t = 0f;
    _egoS[0] = 0f;
    _egoT[0] = 0f;
    for (int k = 1; k <= _aheadStations; k++)
    {
        float v = MathF.Max(6f, PlanSpeedAt(freeSpeedPlan, anchor + k));
        t += spacing / v;
        _egoS[k] = k * spacing;
        _egoT[k] = t;
    }
    _ = egoSpeed;
}
```

`CaptureRivals` 在进入 36 个时间步循环**之前**，对每个非故障对手先做单次 60 站 `BrakeFrom` 反向扫描（零循环内嵌套扫描，保证 6 车单步 `< 500 us`），替换原先的 `lookBrakeMeters` 和 `MathF.Sqrt(... 2f * 0.85f * maxBrakeAccel ...)`：

```csharp
// 进入 for (int step = 1; step <= HorizonSteps; step++) 之前预扫一次：
int look = Math.Min(60, _aheadStations);
Span<float> rivalCeiling = stackalloc float[look + 1];
float offLineScale = MathF.Abs(other.TrackD - _lanes.Offset(LaneId.Free, otherSt)) > 2.0f ? 0.95f : 1.0f;
rivalCeiling[look] = offLineScale * PlanSpeedAt(freeSpeedPlan, _lanes.Wrap(otherSt + look));
for (int m = look - 1; m >= 0; m--)
{
    int a = _lanes.Wrap(otherSt + m);
    float curvature = LaneSet.Curvature(
        _lanes.Position(a - 3, _lanes.Offset(LaneId.Free, a - 3)),
        _lanes.Position(a, _lanes.Offset(LaneId.Free, a)),
        _lanes.Position(a + 3, _lanes.Offset(LaneId.Free, a + 3)));
    rivalCeiling[m] = MathF.Min(
        offLineScale * PlanSpeedAt(freeSpeedPlan, a),
        LaneSpeedPlanner.BrakeFrom(
            envelope, rivalCeiling[m + 1], curvature, _lanes.Spacing,
            cornerFactorAt(a), brakeFactorAt(a), roadAt(a), shares));
}

// 在 36 步循环的非故障分支内直接查表：
float mFloat = Math.Clamp((curRelS - gap) / _lanes.Spacing, 0f, look);
int m0 = Math.Min((int)mFloat, look - 1);
float cornerCeiling = rivalCeiling[m0] + (rivalCeiling[m0 + 1] - rivalCeiling[m0]) * (mFloat - m0);
float accel = t <= 0.25f ? measuredAccel : 6.0f;
curV = Math.Clamp(curV + accel * HorizonDt, 0f, MathF.Max(8f, cornerCeiling));
curRelS += curV * HorizonDt;
curAbsS = WrapS(curAbsS + curV * HorizonDt, length);
```

横向预测循环保持原样，紧接在这段速度更新之后。

把跟车限速的 `desiredGap` / `recoverV` / `groundCatchMeters` 整段换成下面这个方法，并在 `BuildSpatialCorridorAndLimits` 里对每个前方对手调用它：

```csharp
private void AddFollowerLimit(
    RaceCarSnapshot me, RivalForecast rival, RivalState state,
    float egoSpeed, bool inBrakingZone, float boldness, float reactionsScore,
    bool suppressForTow)
{
    if (rival.GapS <= 0f || rival.AlongsideNow)
        return;
    float clearCross = 0.5f * (me.WidthMeters + rival.Width) + 0.30f;
    bool laterallyClear = MathF.Abs(me.TrackD - rival.TrackD) >= clearCross;
    if (suppressForTow || (state.Slot != Slot.Behind && laterallyClear))
        return;

    float dv = egoSpeed - rival.Speed;
    if (!inBrakingZone)
    {
        if (dv <= 0f)
            return;
        float catchMeters = rival.GapS * egoSpeed / dv;
        int kCatch = Math.Clamp((int)MathF.Floor(catchMeters / _lanes.Spacing), 0, _aheadStations - 1);
        _limitPoints.Add(new SpeedLimitPoint(kCatch, 6, rival.Speed));
        return;
    }

    float dTail = 3.8f - 1.3f * boldness;
    float tReact = 0.30f + (0.05f - 0.30f) * reactionsScore;
    float shift = dTail + egoSpeed * tReact;
    for (int step = 0; step <= HorizonSteps; step++)
    {
        float rel = rival.S[step] - shift;
        if (rel < 0f || rel > _aheadStations * _lanes.Spacing)
            continue;
        int k = Math.Clamp((int)MathF.Floor(rel / _lanes.Spacing), 0, _aheadStations - 1);
        _limitPoints.Add(new SpeedLimitPoint(k, 2, rival.V[step]));
    }
}
```

`BuildSpatialCorridorAndLimits` 末尾对每个前方对手调用它。`boldness` 与 `reactionsScore` 用 `DriverAbilities` 的 `Boldness`、`Reactions` 各乘 `0.01f` 再夹到 0..1。`suppressForTow` 为 `intent.FullThrottleTow && intent.FocusRivalId == rival.Id`。pinch 点的速度改为该站预测 `V`，不再减 4 m/s：找到重叠对手后取 `rival.V[0]`。

`SolveSmoothLine` 删除 `if (k >= 6) { ... slope = (clamped - _delta[idx - 1]) / spacing; ... }` 整块，也删除事后平滑里 `if (idx >= BehindStations + 7) smoothed = Math.Clamp(...)` 那一行。积分本身、预平滑、`maxBend` 和 `drawn.Set` 的清线条件保留。在 `drawn.Set` 之前插入门控，通不过的高档不保留：

```csharp
internal static bool PassesDrivableGate(float candidateCeiling0, float egoSpeed, float stillCeiling0) =>
    candidateCeiling0 >= MathF.Min(egoSpeed, stillCeiling0) - 0.5f;

private static readonly float[] MoveScales = [1.0f, 0.6f, 0.35f, 0.15f, 0.0f];

private void GateAndCommit(
    int anchor, RaceCarSnapshot me, float egoSpeed, bool emergency,
    ReadOnlySpan<float> freeSpeedPlan, in LaneEnvelope envelope, LaneShares shares,
    Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
    Func<float, float> turnLagSeconds, Func<int, float> currentReferenceOffset,
    DrawnLine drawn, bool inLaunchHold)
{
    int n = Math.Min(_aheadStations + 1, 120); // 240 m 窗口完整覆盖最长重刹区（复用成员缓冲数组避免 GC）
    var still = new float[n];
    var target = new float[n];
    float carDeltaHere = me.TrackD - _lanes.Offset(LaneId.Free, anchor);
    for (int k = 0; k < n; k++)
    {
        int st = _lanes.Wrap(anchor + k);
        float free = _lanes.Offset(LaneId.Free, st);
        // 若尚未处于 DrawnLine 且车显著偏离自由线（如发车格起步），基底线从车当前横向偏移平滑保持，避免向 4m 外的自由线跳变
        still[k] = !drawn.Active && MathF.Abs(carDeltaHere) > 0.5f
            ? Math.Clamp(me.TrackD, -(_lanes.HalfWidth(st) - SafeEdgeMarginMeters), _lanes.HalfWidth(st) - SafeEdgeMarginMeters) - free
            : currentReferenceOffset(st) - free;
        target[k] = _delta[BehindStations + k];
    }
    float stillCeiling = Ceiling0(anchor, still, n, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds);
    float barStill = stillCeiling;
    float chosen = 0f;
    float escape = -1f;
    var chosenLine = still;
    foreach (float scale in MoveScales)
    {
        var candidate = new float[n];
        for (int k = 0; k < n; k++)
            candidate[k] = still[k] + scale * (target[k] - still[k]);
        float ceiling = Ceiling0(anchor, candidate, n, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds);
        bool pass = scale == 0f || PassesDrivableGate(ceiling, egoSpeed, barStill);
        bool overlaps = emergency && LineOverlaps(anchor, candidate, n);
        if (pass && !(scale == 0f && overlaps))
        {
            chosen = scale;
            chosenLine = candidate;
            break;
        }
        if (emergency && !overlaps && escape < 0f)
            escape = scale;
    }
    if (chosen == 0f && escape >= 0f)
    {
        chosen = escape;
        chosenLine = new float[n];
        for (int k = 0; k < n; k++)
            chosenLine[k] = still[k] + chosen * (target[k] - still[k]);
    }
    for (int k = 0; k < n; k++)
        _delta[BehindStations + k] = chosenLine[k];
    float maxDelta = 0f;
    for (int k = 0; k < n; k++)
        maxDelta = MathF.Max(maxDelta, MathF.Abs(_delta[BehindStations + k]));
    if (!inLaunchHold && maxDelta < 0.05f && !HasAlongsideRival && _committedPassId is null && !emergency)
    {
        if (_active)
        {
            _active = false;
            drawn.Clear();
        }
        return;
    }
    _active = true;
    drawn.Set(_lanes.Wrap(anchor - BehindStations), _delta, 1f);
}

private float Ceiling0(
    int anchor, float[] delta, int n, ReadOnlySpan<float> freeSpeedPlan,
    in LaneEnvelope envelope, LaneShares shares,
    Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
    Func<float, float> turnLagSeconds)
{
    var curvature = new float[n];
    var lead = new float[n];
    var step = new float[n];
    var factor = new float[n];
    var brake = new float[n];
    var road = new LaneRoad[n];
    var guess = new float[n];
    var limits = new float[n];
    var speed = new float[n];
    float Delta(int rel)
    {
        if (rel < 0) return delta[0];
        if (rel >= n) return delta[n - 1];
        return delta[rel];
    }
    for (int k = 0; k < n; k++)
    {
        int st = anchor + k;
        Vector2 Point(int rel) =>
            _lanes.Position(anchor + rel, _lanes.Offset(LaneId.Free, anchor + rel) + Delta(rel));
        curvature[k] = LaneSet.Curvature(Point(k - 3), Point(k), Point(k + 3));
        step[k] = (Point(k + 1) - Point(k)).Length();
        int wrapped = _lanes.Wrap(st);
        factor[k] = cornerFactorAt(wrapped);
        brake[k] = brakeFactorAt(wrapped);
        road[k] = roadAt(wrapped);
        guess[k] = MathF.Max(8f, PlanSpeedAt(freeSpeedPlan, st));
    }
    LaneSpeedPlanner.LeadCurvatures(curvature, guess, turnLagSeconds, _lanes.Spacing, lead);
    float terminal = PlanSpeedAt(freeSpeedPlan, anchor + n - 1);
    LaneSpeedPlanner.CornerLimits(envelope, lead, factor, road, shares, limits);
    LaneSpeedPlanner.PlanWindow(envelope, lead, step, factor, brake, road, shares, terminal, limits, speed);
    return speed[0];
}

private bool LineOverlaps(int anchor, float[] delta, int n)
{
    for (int k = 0; k < n; k += 2)
    {
        float t = MathF.Min(_egoT[Math.Min(k, _egoT.Length - 1)], HorizonSteps * HorizonDt);
        float egoD = _lanes.Offset(LaneId.Free, anchor + k) + delta[k];
        foreach (RivalForecast rival in _forecasts)
        {
            if (!rival.Hazard && !rival.AlongsideNow)
                continue;
            (float s, float d, _) = SampleForecast(rival, t);
            if (s < -2f || s > k * _lanes.Spacing + 8f)
                continue;
            if (MathF.Abs(egoD - d) < 0.5f * rival.Width + 1.1f)
                return true;
        }
    }
    return false;
}
```

`SolveSmoothLine` 在积完 `_delta` 之后调用 `GateAndCommit`，不再自己 `drawn.Set`。`inLaunchHold` 已经是它的参数。`emergency` 用现有的 `BuildSpatialCorridorAndLimits` 返回值，或 `intent.EmergencyAvoid`。

`_egoT` 的长度是 `_aheadStations + 1`。`GateAndCommit` 的 `n` 不超过它，索引要先 `Math.Min`。

- [ ] **Step 4: Run test to verify it passes**

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~CorridorDrivabilityTests`

Expected: PASS。再跑现有走廊测试，确认直道超车没有被门控误杀：

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~TrafficCorridorTests`

Expected: PASS。若 `FasterCarCatchesAndOvertakesSlowerCarCleanly` 失败，先看失败是接触还是没有完成超车。接触说明门控把车留在了重叠线上；没有超车说明最高可走档被错误拒绝。直道上 `ceiling[0]` 应接近 `stillCeiling[0]`，档位 1.0 必须能通过。

- [ ] **Step 5: Commit**

```bash
git add Drivers/Lanes/TrafficCorridor.cs Drivers/Lanes/LaneDriver.cs Drivers/Tests/CorridorDrivabilityTests.cs
git commit -m "feat(drivers): gate corridor lines with PlanWindow and honest limits"
```

---

### Task 3: Six-State Tactical Brain

**Files:**

- Create: `Drivers/Lanes/RacecraftTactics.cs`
- Modify: `Drivers/Lanes/TrafficCorridor.cs`（删掉文件末尾的枚举和结构体副本；删掉 `UpdateTacticalCommitments` 与 `UpdateOneMoveDefence`；`Update` 不再调用它们，只应用 `RacecraftIntent`）
- Modify: `Drivers/Lanes/LaneDriver.cs`（`Initialize` 里构造 `_tactics`；12 Hz 块里先 `Update` 战术再把 `Intent` 传给走廊）
- Test: `Drivers/Tests/RacecraftTacticsTests.cs`

**Interfaces:**

- Consumes: 任务 2 的 `RacecraftPhase`、`RacecraftIntent`、`PassesDrivableGate`、`TrafficCorridor.Update(..., in RacecraftIntent intent)`、`CornerMap.FreeCurvature`、`CornerMap.Bends`、`CornerMap.PinsOf`、`CornerMap.Bend`、`CornerMap.Pins`、`LaneSpeedPlanner.PlanWindow` 的 limits 重载、`LaneSpeedPlanner.LeadCurvatures`、`LaneSpeedPlanner.CornerLimits`、`DriverAbilities`、`DriverContext`、`RaceCarSnapshot.RaceDistanceMeters`、`RaceCarSnapshot.TrackLengthMeters`、`RaceCarSnapshot.WheelBaseMeters`。
- Produces:

```csharp
internal sealed class RacecraftTactics
{
    public const float PullLateralAccel = 4.6f;
    public const float MinPassRoomMeters = 2.75f;
    public const float AbortRoomMeters = 2.30f;
    public const float SafeEdgeMarginMeters = 1.48f;
    public const float AbortCooldownSeconds = 2.8f;
    public RacecraftTactics(LaneSet lanes);
    public RacecraftPhase Phase { get; }
    public RacecraftIntent Intent { get; }
    public void Update(
        in DriverContext context,
        ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope,
        LaneShares shares,
        Func<int, float> cornerFactorAt,
        Func<int, float> brakeFactorAt,
        Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds);
    internal static bool BlueFlagDue(float otherRaceDistance, float myRaceDistance, float trackLength, float gapMeters, float closingSpeed, float catchSeconds);
    internal static float CatchSeconds(float awarenessRating);
    internal static float YieldClearanceMeters(float myWidth, float otherWidth);
}
```

- [ ] **Step 1: Write the failing test**

新建 `Drivers/Tests/RacecraftTacticsTests.cs`：

```csharp
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class RacecraftTacticsTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void BlueFlag_RequiresThreeQuartersOfALapAndAClosingWindow()
    {
        float length = 5890f;
        float catchSeconds = RacecraftTactics.CatchSeconds(50f);
        Assert.Equal(1.20f, catchSeconds, 3);
        Assert.Equal(1.00f, RacecraftTactics.CatchSeconds(0f), 3);
        Assert.Equal(1.40f, RacecraftTactics.CatchSeconds(100f), 3);
        Assert.True(RacecraftTactics.BlueFlagDue(length, 0f, length, -20f, 20f, 1.2f));
        Assert.False(RacecraftTactics.BlueFlagDue(0.5f * length, 0f, length, -20f, 20f, 1.2f));
        Assert.False(RacecraftTactics.BlueFlagDue(length, 0f, length, -40f, 20f, 1.2f));
        Assert.True(RacecraftTactics.YieldClearanceMeters(1.9f, 1.9f) >= 3f + 1.9f);
    }

    [Fact]
    public void StandingStart_HoldsColumnAndPullsOnlyASlowStarter()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        float start = track.StartingLineS;
        int station = (int)MathF.Round(lanes.StationOf(start));
        float column = 2.2f;

        (RaceCar slow, LaneDriver slowDriver) = LaneGrid.Seat(
            simulation, file, lanes, "slow-start", StockCar.Build(compound), compound,
            start + 8f, column, 0f,
            abilities: new DriverAbilities { Pace = 60, Defending = 40, Boldness = 40 });
        slowDriver.SpeedCap = 18f;

        (RaceCar fast, _) = LaneGrid.Seat(
            simulation, file, lanes, "fast-start", StockCar.Build(compound), compound,
            start, column, 0f,
            abilities: new DriverAbilities { Pace = 95, Overtaking = 90, Boldness = 70, Focus = 95, Precision = 95 });

        float maxSway = 0f;
        float maxSep = 0f;
        bool hit = false;
        for (int step = 0; step < 60 * 8; step++)
        {
            simulation.Step(Dt);
            hit |= slow.HitCarThisStep || fast.HitCarThisStep;
            if (fast.Progress.TotalDistance < 70f)
                maxSway = MathF.Max(maxSway, MathF.Abs(fast.Progress.CurrentD - (lanes.Offset(LaneId.Free, station) + column)));
            maxSep = MathF.Max(maxSep, MathF.Abs(fast.Progress.CurrentD - slow.Progress.CurrentD));
        }

        Assert.False(hit);
        Assert.True(maxSway < 0.35f, $"launch sway {maxSway:F2} m");
        Assert.True(maxSep > 1.5f, $"no pull-out, separation {maxSep:F2} m");
    }

    [Fact]
    public void DraftTow_HoldsThrottleUntilPulloutGap()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        float start = track.StartingLineS;

        (RaceCar leader, LaneDriver leaderDriver) = LaneGrid.Seat(
            simulation, file, lanes, "tow-leader", StockCar.Build(compound), compound,
            start + 40f, 0f, 62f,
            abilities: new DriverAbilities { Pace = 80, Defending = 70, Boldness = 50 });
        leaderDriver.SpeedCap = 64f;

        (RaceCar chaser, _) = LaneGrid.Seat(
            simulation, file, lanes, "tow-chaser", StockCar.Build(compound), compound,
            start, 0f, 66f,
            abilities: new DriverAbilities { Pace = 95, Overtaking = 90, Boldness = 50, Focus = 95, Precision = 95 });

        float slowest = float.PositiveInfinity;
        bool hit = false;
        bool off = false;
        for (int step = 0; step < 60 * 2; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || chaser.HitCarThisStep;
            off |= leader.BoundaryContactSeconds > 0f || chaser.BoundaryContactSeconds > 0f;
            if (step > 15)
                slowest = MathF.Min(slowest, chaser.State.Speed);
        }

        Assert.False(hit);
        Assert.False(off);
        Assert.True(slowest > 63f, $"chaser lifted to {slowest:F1} while still well outside d_pullout");
    }

    [Fact]
    public void OneMoveDefend_MovesInsideOnceAndHoldsToEntry()
    {
        (RaceSimulation simulation, LaneSet lanes, RaceCar leader, RaceCar chaser) =
            PairAt(1350f, leaderAhead: 0f, chaserBehind: 10f, leaderSpeed: 62f, chaserSpeed: 68f,
                leader: new DriverAbilities { Pace = 85, Defending = 95, Awareness = 90, Boldness = 40 },
                chaser: new DriverAbilities { Pace = 80, Overtaking = 40, Boldness = 30 });
        int anchor = (int)MathF.Round(lanes.StationOf(1350f));
        int sign = NextInsideSign(lanes, anchor);
        float free = lanes.Offset(LaneId.Free, anchor);
        int crosses = 0;
        int lastSide = 0;
        bool hit = false;
        for (int step = 0; step < 60 * 8; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || chaser.HitCarThisStep;
            float delta = leader.Progress.CurrentD - free;
            int side = MathF.Abs(delta) < 0.4f ? 0 : MathF.Sign(delta) == sign ? 1 : -1;
            if (lastSide == 0 && side != 0)
                crosses++;
            if (lastSide == 1 && side == -1)
                crosses++;
            if (side != 0)
                lastSide = side;
        }
        Assert.False(hit);
        Assert.True(crosses <= 1, $"defender changed side {crosses} times");
    }

    [Fact]
    public void SideBySide_ThreeWideHoldsSlotsWithoutContact()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        const float start = 1450f;
        int st = (int)MathF.Round(lanes.StationOf(start));
        float free = lanes.Offset(LaneId.Free, st);
        float heading = track.Sample(start).Heading;
        (RaceCar left, _) = LaneGrid.Seat(simulation, file, lanes, "slot-left", StockCar.Build(compound), compound, start, -free + 3.0f, 60f);
        (RaceCar mid, _) = LaneGrid.Seat(simulation, file, lanes, "slot-mid", StockCar.Build(compound), compound, start, -free, 60f);
        (RaceCar right, _) = LaneGrid.Seat(simulation, file, lanes, "slot-right", StockCar.Build(compound), compound, start, -free - 3.0f, 60f);
        left.State.Heading = mid.State.Heading = right.State.Heading = heading;
        bool hit = false;
        bool wall = false;
        for (int step = 0; step < 60 * 12; step++)
        {
            simulation.Step(Dt);
            foreach (RaceCar car in new[] { left, mid, right })
            {
                hit |= car.HitCarThisStep;
                wall |= car.BoundaryContactSeconds > 0f;
            }
        }
        Assert.False(hit);
        Assert.False(wall);
    }

    [Fact]
    public void Switchback_ClearsDeepInsideDefenderWithoutSpin()
    {
        (RaceSimulation simulation, _, RaceCar leader, RaceCar chaser) =
            PairAt(1680f, leaderAhead: 8f, chaserBehind: 0f, leaderSpeed: 42f, chaserSpeed: 48f,
                leader: new DriverAbilities { Pace = 70, Defending = 95, Boldness = 80, Focus = 90 },
                chaser: new DriverAbilities { Pace = 95, Overtaking = 95, Boldness = 75, Focus = 98, Precision = 98 });
        bool hit = false;
        bool wall = false;
        for (int step = 0; step < 60 * 18; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || chaser.HitCarThisStep;
            wall |= leader.BoundaryContactSeconds > 0f || chaser.BoundaryContactSeconds > 0f;
        }
        int spins = ((ICarMotion)chaser.Car).Motion(chaser.State).Spins;
        Assert.False(hit);
        Assert.False(wall);
        Assert.Equal(0, spins);
        Assert.True(chaser.Progress.RaceDistanceMeters + 5f > leader.Progress.RaceDistanceMeters);
    }

    private static (RaceSimulation Simulation, LaneSet Lanes, RaceCar Leader, RaceCar Chaser) PairAt(
        float start, float leaderAhead, float chaserBehind, float leaderSpeed, float chaserSpeed,
        DriverAbilities leader, DriverAbilities chaser)
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        (RaceCar leaderCar, _) = LaneGrid.Seat(
            simulation, file, lanes, "pair-leader", StockCar.Build(compound), compound,
            start + leaderAhead, 0f, leaderSpeed, abilities: leader);
        (RaceCar chaserCar, _) = LaneGrid.Seat(
            simulation, file, lanes, "pair-chaser", StockCar.Build(compound), compound,
            start - chaserBehind, 0f, chaserSpeed, abilities: chaser);
        return (simulation, lanes, leaderCar, chaserCar);
    }

    private static int NextInsideSign(LaneSet lanes, int anchor)
    {
        float[] k = CornerMap.FreeCurvature(lanes);
        List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
        int best = int.MaxValue;
        int sign = 1;
        for (int i = 0; i < bends.Count; i++)
        {
            CornerMap.Pins pins = CornerMap.PinsOf(lanes, k, bends, i);
            int dist = lanes.Wrap(pins.Entry - anchor);
            if (dist < best)
            {
                best = dist;
                sign = pins.Sign == 0 ? 1 : pins.Sign;
            }
        }
        return sign;
    }
}
```

`BlueFlagDue(..., gap -40, closing 20, horizon 1.2)` 的追及时间是 2.0 s，必须为 false。`-20 / 20 = 1.0 s`，且赛程差等于一整圈，必须为 true。半圈赛程差必须为 false。

- [ ] **Step 2: Run test to verify it fails**

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~RacecraftTacticsTests`

Expected: FAIL，`CS0246` 或 `CS0117`：没有 `RacecraftTactics`。

- [ ] **Step 3: Implement the brain and delete the old selectors**

新建 `Drivers/Lanes/RacecraftTactics.cs`。文件同时拥有任务 2 末尾的 `RacecraftPhase` 和 `RacecraftIntent`，正文与那里逐字相同，然后删掉 `TrafficCorridor.cs` 末尾的那两份副本。

```csharp
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers.Lanes;

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
    public int SideSign { get; init; }
    public float TargetDeltaMeters { get; init; }
    public float[]? SpatialDeltaProfile { get; init; }
    public bool FreezeLateral { get; init; }
    public bool EmergencyAvoid { get; init; }
    public float YieldSpeedScale { get; init; }
    public bool FullThrottleTow { get; init; }
    public string? FocusRivalId { get; init; }

    public static RacecraftIntent None => new() { TargetDeltaMeters = float.NaN };
}

internal sealed class RacecraftTactics
{
    public const float PullLateralAccel = 4.6f;
    public const float MinPassRoomMeters = 2.75f;
    public const float AbortRoomMeters = 2.30f;
    public const float SafeEdgeMarginMeters = 1.48f;
    public const float AbortCooldownSeconds = 2.8f;
    private const int LocalStations = 120;
    private const float MoveSpeedToleranceMps = 0.5f;

    private readonly LaneSet _lanes;
    private readonly float[] _freeK;
    private readonly List<CornerMap.Bend> _bends;
    private readonly CornerMap.Pins[] _pins;

    private bool _launchCaptured;
    private bool _isStandingStart;
    private float _launchStartS;
    private float _launchColumnD;
    private bool _defendedThisStraight;
    private int _defendBend = -1;
    private int _defendSign;
    private float _defendShare;
    private bool _defendFrozen;
    private int _commitSign;
    private string? _commitId;
    private float _abortUntil = -10f;
    private RacecraftPhase _phase = RacecraftPhase.Behind;
    private RacecraftIntent _intent = RacecraftIntent.None;

    public RacecraftTactics(LaneSet lanes)
    {
        _lanes = lanes;
        _freeK = CornerMap.FreeCurvature(lanes);
        _bends = CornerMap.Bends(lanes, _freeK);
        _pins = new CornerMap.Pins[_bends.Count];
        for (int i = 0; i < _bends.Count; i++)
            _pins[i] = CornerMap.PinsOf(lanes, _freeK, _bends, i);
    }

    public RacecraftPhase Phase => _phase;
    public RacecraftIntent Intent => _intent;

    public void Update(
        in DriverContext context,
        ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope,
        LaneShares shares,
        Func<int, float> cornerFactorAt,
        Func<int, float> brakeFactorAt,
        Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds)
    {
        RaceCarSnapshot me = context.Car;
        float now = context.RaceTimeSeconds;
        int anchor = (int)MathF.Floor(_lanes.StationOf(me.TrackS));
        float ego = MathF.Max(0f, me.SpeedMetersPerSecond);
        NoteStraight(anchor);
        CaptureLaunch(me, now, ego);

        DriverAbilities abilities = context.Profile.Abilities;
        float boldness = Score(abilities.Boldness);
        float defending = Score(abilities.Defending);
        float overtaking = Score(abilities.Overtaking);
        bool loaded = Loaded(anchor, ego, me);
        bool braking = Braking(anchor, ego, me, freeSpeedPlan);
        Rival? ahead = NearestAhead(context, me, freeSpeedPlan);
        Rival? behind = NearestBehind(context, me, freeSpeedPlan);
        Rival? blue = LappingCar(context, me, abilities);

        RacecraftIntent intent = RacecraftIntent.None;
        if (blue is { } yieldTo && !loaded)
        {
            float side = MathF.Abs(me.TrackD - yieldTo.TrackD) < 0.3f ? 1f : MathF.Sign(me.TrackD - yieldTo.TrackD);
            float edge = MathF.Max(0.4f, _lanes.HalfWidth(anchor) - SafeEdgeMarginMeters);
            float delta = Math.Clamp(side * (0.5f * (me.WidthMeters + yieldTo.Width) + 3.0f), -edge, edge);
            bool close = yieldTo.Gap > -10f && yieldTo.Gap < 10f;
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.BlueFlagYield,
                TargetDeltaMeters = delta,
                YieldSpeedScale = close ? 0.88f : 0f,
                FocusRivalId = yieldTo.Id,
            };
        }
        else if (ahead is { Hazard: true })
        {
            int sign = Room(ahead.Value, me, +1) >= Room(ahead.Value, me, -1) ? +1 : -1;
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.PullOutAttack,
                SideSign = sign,
                TargetDeltaMeters = SideDelta(anchor, sign),
                EmergencyAvoid = true,
                FocusRivalId = ahead.Value.Id,
            };
        }
        else if (Overlap(me, ahead) || Overlap(me, behind))
        {
            Rival other = Overlap(me, ahead) ? ahead!.Value : behind!.Value;
            int sign = me.TrackD >= other.TrackD ? +1 : -1;
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.SideBySide,
                SideSign = sign,
                TargetDeltaMeters = float.NaN, // 由走廊对手预测边界自然挤出平滑并排线
                FocusRivalId = other.Id,
            };
        }
        else if (TrySwitchback(me, ahead, anchor, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, out RacecraftIntent switchback))
        {
            intent = switchback;
        }
        else if (TryPull(me, ahead, anchor, ego, now, loaded, boldness, overtaking, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, out RacecraftIntent pull))
        {
            intent = pull;
        }
        else if (TryDefend(me, behind, anchor, ego, braking, defending, out RacecraftIntent defend))
        {
            intent = defend;
        }
        else if (InLaunch(me, now) && !(ahead is { } slow && ego - slow.Speed > 1.2f && MathF.Max(Room(slow, me, +1), Room(slow, me, -1)) >= MinPassRoomMeters))
        {
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.StandingStart,
                TargetDeltaMeters = _launchColumnD - Free(anchor),
            };
        }
        else if (ahead is { } leader && leader.Gap > PullDistance(me, leader, anchor, boldness) && !loaded)
        {
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.DraftTow,
                TargetDeltaMeters = leader.TrackD - Free(anchor),
                FullThrottleTow = true,
                FocusRivalId = leader.Id,
            };
        }
        else
        {
            intent = new RacecraftIntent { Phase = RacecraftPhase.Behind, TargetDeltaMeters = float.NaN };
        }

        if (InLaunch(me, now) && AheadOfFirstEntry(anchor) && !Overlap(me, ahead) && !Overlap(me, behind))
        {
            intent = new RacecraftIntent { Phase = RacecraftPhase.Behind, TargetDeltaMeters = float.NaN };
        }

        _intent = intent;
        _phase = intent.Phase;
    }

    internal static float CatchSeconds(float awarenessRating)
    {
        float awareness = Score(awarenessRating);
        return 1.20f + 0.40f * (awareness - 0.5f);
    }

    internal static bool BlueFlagDue(
        float otherRaceDistance, float myRaceDistance, float trackLength,
        float gapMeters, float closingSpeed, float catchSeconds)
    {
        if (otherRaceDistance - myRaceDistance <= 0.75f * trackLength)
            return false;
        if (gapMeters >= 0f || closingSpeed <= 0.1f)
            return false;
        return -gapMeters / closingSpeed <= catchSeconds;
    }

    internal static float YieldClearanceMeters(float myWidth, float otherWidth) =>
        0.5f * (myWidth + otherWidth) + 3.0f;

    private bool TryPull(
        RaceCarSnapshot me, Rival? ahead, int anchor, float ego, float now, bool loaded,
        float boldness, float overtaking, ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope, LaneShares shares,
        Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds, out RacecraftIntent intent)
    {
        intent = default;
        if (ahead is not { } leader)
        {
            _commitId = null;
            _commitSign = 0;
            return false;
        }
        bool alreadyCommitted = _commitId == leader.Id && _commitSign != 0;
        if (!alreadyCommitted && loaded && !leader.Hazard)
            return false;
        if (now < _abortUntil && _commitId == leader.Id)
            return false;
        float gap = leader.Gap;
        if (alreadyCommitted && gap > 32f && ego < leader.Speed)
        {
            _commitId = null;
            _commitSign = 0;
            return false;
        }
        float dPull = PullDistance(me, leader, anchor, boldness);
        if (!alreadyCommitted && gap > dPull)
            return false;
        int inside = NextInsideSign(anchor);
        int sign;
        if (alreadyCommitted)
        {
            sign = _commitSign;
        }
        else
        {
            int preferred = inside == 0 ? (Room(leader, me, +1) >= Room(leader, me, -1) ? +1 : -1) : inside;
            int other = -preferred;
            sign = Room(leader, me, preferred) >= MinPassRoomMeters ? preferred
                : Room(leader, me, other) >= MinPassRoomMeters ? other : 0;
            if (sign == 0)
                return false;
            if (Braking(anchor, ego, me, freeSpeedPlan) && sign == inside)
            {
                float[] delta = ConstantDelta(anchor, SideDelta(anchor, sign));
                float cand = WindowSpeed(anchor, delta, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, 0);
                if (cand < ego - MoveSpeedToleranceMps)
                {
                    if (Room(leader, me, other) < MinPassRoomMeters)
                        return false;
                    sign = other;
                }
            }
        }
        if (Room(leader, me, sign) < AbortRoomMeters && !Fia(me, leader))
        {
            _commitId = null;
            _commitSign = 0;
            _abortUntil = now + AbortCooldownSeconds;
            intent = new RacecraftIntent { Phase = RacecraftPhase.Behind, TargetDeltaMeters = float.NaN, FocusRivalId = leader.Id };
            return true;
        }
        _commitId = leader.Id;
        _commitSign = sign;
        intent = new RacecraftIntent
        {
            Phase = RacecraftPhase.PullOutAttack,
            SideSign = sign,
            TargetDeltaMeters = SideDelta(anchor, sign),
            FocusRivalId = leader.Id,
        };
        _ = overtaking;
        return true;
    }

    private bool TryDefend(
        RaceCarSnapshot me, Rival? behind, int anchor, float ego, bool braking, float defending,
        out RacecraftIntent intent)
    {
        intent = default;
        if (_bends.Count == 0)
            return false;

        // 已触发本直道防守：持续锁住防守内线直到入弯钉点 Entry，开始刹车或并排后冻结横向份额
        if (_defendedThisStraight && _defendBend >= 0)
        {
            int toEntry = _lanes.Wrap(_pins[_defendBend].Entry - anchor);
            if (toEntry > 0 && toEntry < _lanes.Count / 2)
            {
                if (braking || (behind is { } r && Fia(me, r)))
                {
                    if (!_defendFrozen)
                    {
                        float curEdge = MathF.Max(0.4f, _lanes.HalfWidth(anchor) - SafeEdgeMarginMeters);
                        _defendShare = Math.Clamp(me.TrackD / curEdge, -1f, 1f);
                        _defendFrozen = true;
                    }
                }
                float edge = MathF.Max(0.4f, _lanes.HalfWidth(anchor) - SafeEdgeMarginMeters);
                intent = new RacecraftIntent
                {
                    Phase = RacecraftPhase.OneMoveDefend,
                    SideSign = _defendSign,
                    TargetDeltaMeters = _defendShare * edge - Free(anchor),
                    FreezeLateral = _defendFrozen,
                    FocusRivalId = behind?.Id,
                };
                return true;
            }
            return false;
        }

        if (behind is not { } rival || _defendedThisStraight || defending < 0.30f || ego < 30f || braking)
            return false;
        if (rival.Gap > -4.2f || rival.Gap < -(12f + 14f * defending) || rival.Speed < ego + 0.4f)
            return false;
        int sign = NextInsideSign(anchor);
        if (sign == 0)
            return false;
        int bend = NextBend(anchor);
        _defendedThisStraight = true;
        _defendBend = bend;
        _defendSign = sign;
        _defendShare = sign * (0.72f + 0.22f * defending);
        _defendFrozen = false;
        float edgeHere = MathF.Max(0.4f, _lanes.HalfWidth(anchor) - SafeEdgeMarginMeters);
        intent = new RacecraftIntent
        {
            Phase = RacecraftPhase.OneMoveDefend,
            SideSign = sign,
            TargetDeltaMeters = _defendShare * edgeHere - Free(anchor),
            FreezeLateral = false,
            FocusRivalId = rival.Id,
        };
        return true;
    }

    private bool TrySwitchback(
        RaceCarSnapshot me, Rival? ahead, int anchor, float ego, ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope, LaneShares shares,
        Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds, out RacecraftIntent intent)
    {
        intent = default;
        if (_bends.Count == 0 || ahead is not { } leader || leader.Gap <= 0f || leader.Gap >= 14f)
            return false;
        int bend = NextBend(anchor);
        CornerMap.Pins pins = _pins[bend];
        if (PlanSpeed(freeSpeedPlan, pins.Apex) >= 58f)
            return false;
        float inside = pins.Sign;
        if (inside == 0)
            return false;
        float entryFree = _lanes.Offset(LaneId.Free, pins.Entry);
        if (inside * (leader.TrackD - entryFree) <= 1.6f)
            return false;
        if (Room(leader, me, (int)inside) >= AbortRoomMeters)
            return false;
        if (Room(leader, me, (int)-inside) < MinPassRoomMeters)
            return false;
        float[] wide = SwitchLine(anchor, pins);
        float[] stay = ConstantDelta(anchor, 0f);
        float cand0 = WindowSpeed(anchor, wide, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, 0);
        if (cand0 < ego - MoveSpeedToleranceMps)
            return false;
        int exitRel = Math.Clamp(_lanes.Wrap(pins.Exit - anchor), 0, wide.Length - 1);
        float exitWide = WindowSpeed(anchor, wide, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, exitRel);
        float exitStay = WindowSpeed(anchor, stay, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, exitRel);
        if (exitWide <= exitStay)
            return false;
        intent = new RacecraftIntent
        {
            Phase = RacecraftPhase.Switchback,
            SideSign = (int)-inside,
            TargetDeltaMeters = wide[0],
            SpatialDeltaProfile = wide,
            FocusRivalId = leader.Id,
        };
        return true;
    }

    private float[] SwitchLine(int anchor, CornerMap.Pins pins)
    {
        var delta = new float[LocalStations];
        for (int k = 0; k < delta.Length; k++)
        {
            int st = _lanes.Wrap(anchor + k);
            float edge = MathF.Max(0.4f, _lanes.HalfWidth(st) - SafeEdgeMarginMeters);
            int fromEntry = _lanes.Wrap(st - pins.Entry);
            int fromApex = _lanes.Wrap(st - pins.Apex);
            float side = fromEntry > _lanes.Count / 2 ? -pins.Sign : fromApex <= 4 ? -pins.Sign : pins.Sign;
            delta[k] = side * edge;
        }
        return delta;
    }

    private float WindowSpeed(
        int anchor, float[] delta, float ego, ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope, LaneShares shares,
        Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds, int sample)
    {
        int n = Math.Min(delta.Length, LocalStations);
        var curvature = new float[n];
        var lead = new float[n];
        var step = new float[n];
        var factor = new float[n];
        var brake = new float[n];
        var road = new LaneRoad[n];
        var guess = new float[n];
        var limits = new float[n];
        var speed = new float[n];
        float At(int rel) => rel < 0 ? delta[0] : rel >= n ? delta[n - 1] : delta[rel];
        for (int k = 0; k < n; k++)
        {
            Vector2 P(int rel) => _lanes.Position(anchor + rel, _lanes.Offset(LaneId.Free, anchor + rel) + At(rel));
            curvature[k] = LaneSet.Curvature(P(k - 3), P(k), P(k + 3));
            step[k] = MathF.Max(0.5f, (P(k + 1) - P(k)).Length());
            int wrapped = _lanes.Wrap(anchor + k);
            factor[k] = cornerFactorAt(wrapped);
            brake[k] = brakeFactorAt(wrapped);
            road[k] = roadAt(wrapped);
            guess[k] = MathF.Max(8f, ego);
        }
        LaneSpeedPlanner.LeadCurvatures(curvature, guess, turnLagSeconds, _lanes.Spacing, lead);
        LaneSpeedPlanner.CornerLimits(envelope, lead, factor, road, shares, limits);
        float terminal = PlanSpeed(freeSpeedPlan, anchor + n - 1);
        LaneSpeedPlanner.PlanWindow(envelope, lead, step, factor, brake, road, shares, terminal, limits, speed);
        return speed[Math.Clamp(sample, 0, n - 1)];
    }

    private float[] ConstantDelta(int anchor, float value)
    {
        var delta = new float[LocalStations];
        Array.Fill(delta, value);
        _ = anchor;
        return delta;
    }

    private float PullDistance(RaceCarSnapshot me, Rival leader, int anchor, float boldness)
    {
        float dTail = 3.8f - 1.3f * boldness;
        float dLat = MathF.Abs(SideDelta(anchor, _commitSign == 0 ? NextInsideSign(anchor) : _commitSign) - (me.TrackD - Free(anchor)));
        float tMove = MathF.Sqrt(4f * MathF.Max(dLat, 0.5f) / PullLateralAccel) + 0.15f;
        return dTail + MathF.Max(0f, me.SpeedMetersPerSecond - leader.Speed) * tMove;
    }

    private float SideDelta(int station, int sign)
    {
        int st = _lanes.Wrap(station);
        float edge = MathF.Max(0.4f, _lanes.HalfWidth(st) - SafeEdgeMarginMeters);
        return sign * edge;
    }

    private float Room(Rival rival, RaceCarSnapshot me, int sign)
    {
        int st = _lanes.Wrap((int)MathF.Round(_lanes.StationOf(me.TrackS)));
        float edge = MathF.Max(0.4f, _lanes.HalfWidth(st) - SafeEdgeMarginMeters);
        float free = Free(st);
        return sign > 0 ? free + edge - rival.TrackD : rival.TrackD - (free - edge);
    }

    private void NoteStraight(int anchor)
    {
        if (_bends.Count == 0 || _defendBend < 0)
            return;
        int sinceExit = _lanes.Wrap(anchor - _pins[_defendBend].Exit);
        int nextEntry = _pins[(_defendBend + 1) % _pins.Length].Entry;
        int untilNext = _lanes.Wrap(nextEntry - anchor);
        if (sinceExit > 0 && sinceExit < _lanes.Count / 2 && untilNext < _lanes.Count / 2)
        {
            _defendedThisStraight = false;
            _defendBend = -1;
        }
    }

    private int NextBend(int anchor)
    {
        int best = 0;
        int bestDist = int.MaxValue;
        for (int i = 0; i < _pins.Length; i++)
        {
            int dist = _lanes.Wrap(_pins[i].Entry - anchor);
            if (dist < bestDist)
            {
                bestDist = dist;
                best = i;
            }
        }
        return best;
    }

    private int NextInsideSign(int anchor)
    {
        if (_pins.Length == 0)
            return 0;
        int sign = _pins[NextBend(anchor)].Sign;
        return sign == 0 ? 0 : sign;
    }

    private bool AheadOfFirstEntry(int anchor)
    {
        if (_pins.Length == 0)
            return false;
        return _lanes.Wrap(_pins[NextBend(anchor)].Entry - anchor) < (int)MathF.Round(40f / _lanes.Spacing);
    }

    private void CaptureLaunch(RaceCarSnapshot me, float now, float ego)
    {
        if (_launchCaptured)
            return;
        _launchCaptured = true;
        _isStandingStart = ego < 5f && now < 0.5f;
        _launchStartS = me.TotalDistanceMeters;
        _launchColumnD = me.TrackD;
    }

    private bool InLaunch(RaceCarSnapshot me, float now) =>
        _isStandingStart && me.TotalDistanceMeters - _launchStartS < 150f && now < 5f;

    private bool Loaded(int anchor, float ego, RaceCarSnapshot me)
    {
        float k = MathF.Abs(_freeK.Length == 0 ? 0f : _freeK[_lanes.Wrap(anchor)]);
        return k * ego * ego > 13f || (me.Motion?.StepLongitudinalAccel ?? 0f) < -3f;
    }

    private bool Braking(int anchor, float ego, RaceCarSnapshot me, ReadOnlySpan<float> plan) =>
        (me.Motion?.StepLongitudinalAccel ?? 0f) < -3f
        || PlanSpeed(plan, anchor + 10) < PlanSpeed(plan, anchor) - 2.5f;

    private float PlanSpeed(ReadOnlySpan<float> plan, int station)
    {
        if (plan.IsEmpty)
            return 85f;
        float v = plan[_lanes.Wrap(station)];
        return v > 1f ? v : 85f;
    }

    private float Free(int station) => _lanes.Offset(LaneId.Free, station);

    private static float Score(float rating) => Math.Clamp(rating * 0.01f, 0f, 1f);

    private readonly struct Rival
    {
        public string Id { get; init; }
        public float Gap { get; init; }
        public float TrackD { get; init; }
        public float Speed { get; init; }
        public float Width { get; init; }
        public float Length { get; init; }
        public float WheelBase { get; init; }
        public float RaceDistance { get; init; }
        public bool Hazard { get; init; }
    }

    private Rival? NearestAhead(in DriverContext context, RaceCarSnapshot me, ReadOnlySpan<float> freeSpeedPlan) =>
        Scan(context, me, freeSpeedPlan, ahead: true);

    private Rival? NearestBehind(in DriverContext context, RaceCarSnapshot me, ReadOnlySpan<float> freeSpeedPlan) =>
        Scan(context, me, freeSpeedPlan, ahead: false);

    private Rival? Scan(in DriverContext context, RaceCarSnapshot me, ReadOnlySpan<float> freeSpeedPlan, bool ahead)
    {
        Rival? best = null;
        float length = MathF.Max(me.TrackLengthMeters, _lanes.LengthMeters);
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            RaceCarSnapshot other = context.Frame[i];
            float gap = MathF.IEEERemainder(other.TrackS - me.TrackS, length);
            if (ahead ? gap <= 0f || gap > 140f : gap >= 0f || gap < -36f)
                continue;
            if (best is { } have && MathF.Abs(gap) > MathF.Abs(have.Gap))
                continue;
            int st = _lanes.Wrap((int)MathF.Round(_lanes.StationOf(other.TrackS)));
            bool skewed = MathF.Abs(MathHelper.NormalizeAngle(other.HeadingRadians - me.HeadingRadians)) > 0.35f;
            bool slow = other.SpeedMetersPerSecond < 0.48f * PlanSpeed(freeSpeedPlan, st)
                        && other.SpeedMetersPerSecond < 28f;
            best = new Rival
            {
                Id = other.Id,
                Gap = gap,
                TrackD = other.TrackD,
                Speed = other.SpeedMetersPerSecond,
                Width = other.WidthMeters,
                Length = other.LengthMeters,
                WheelBase = MathF.Max(3f, other.WheelBaseMeters),
                RaceDistance = other.RaceDistanceMeters,
                Hazard = skewed || slow,
            };
        }
        return best;
    }

    private Rival? LappingCar(in DriverContext context, RaceCarSnapshot me, DriverAbilities abilities)
    {
        float horizon = CatchSeconds(abilities.Awareness);
        float length = MathF.Max(me.TrackLengthMeters, _lanes.LengthMeters);
        Rival? best = null;
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            RaceCarSnapshot other = context.Frame[i];
            float gap = MathF.IEEERemainder(other.TrackS - me.TrackS, length);
            float closing = other.SpeedMetersPerSecond - me.SpeedMetersPerSecond;
            if (!BlueFlagDue(other.RaceDistanceMeters, me.RaceDistanceMeters, length, gap, closing, horizon))
                continue;
            best = new Rival
            {
                Id = other.Id,
                Gap = gap,
                TrackD = other.TrackD,
                Speed = other.SpeedMetersPerSecond,
                Width = other.WidthMeters,
                Length = other.LengthMeters,
                WheelBase = MathF.Max(3f, other.WheelBaseMeters),
                RaceDistance = other.RaceDistanceMeters,
            };
        }
        return best;
    }

    private static bool Fia(RaceCarSnapshot me, Rival rival)
    {
        float beside = 0.5f * (me.WidthMeters + rival.Width) + 0.38f;
        if (MathF.Abs(me.TrackD - rival.TrackD) < beside)
            return false;
        float reach = rival.Gap >= 0f
            ? 0.5f * me.LengthMeters + 0.5f * rival.WheelBase
            : 0.5f * rival.Length + 0.5f * MathF.Max(3f, me.WheelBaseMeters);
        return MathF.Abs(rival.Gap) <= reach;
    }

    private static bool Overlap(RaceCarSnapshot me, Rival? rival) =>
        rival is { } other && Fia(me, other);
}
```

`Scan` 的故障判定已经用 `PlanSpeed(freeSpeedPlan, st)`：车头偏过 0.35 rad，或车速低于该站自由线计划的 0.48 倍且低于 28 m/s。这与 `TrafficCorridor.CaptureRivals` 里现有的 `hazard` 条件一致。

`TrafficCorridor.Update` 删除对 `UpdateTacticalCommitments` 和 `UpdateOneMoveDefence` 的调用，并删除这两个方法。发车列几何改由意图负责：`BuildOpenCorridor` 里 `inLaunchHold` 分支只在 `intent.Phase == RacecraftPhase.StandingStart` 时把 `_prefD` 设为 `_launchColumnD` 的混合，否则用自由线。`intent.TargetDeltaMeters` 有限时，在对手墙收紧之后再写一遍偏好：

```csharp
if (intent.SpatialDeltaProfile is { Length: > 0 } spatial)
{
    int limit = Math.Min(spatial.Length - 1, _aheadStations);
    for (int k = 0; k <= limit; k++)
    {
        int idx = BehindStations + k;
        int st = _lanes.Wrap(anchor + k);
        float abs = _lanes.Offset(LaneId.Free, st) + spatial[k];
        _prefD[idx] = Math.Clamp(abs, _lowD[idx], _highD[idx]);
    }
}
else if (float.IsFinite(intent.TargetDeltaMeters))
{
    for (int k = 0; k <= _aheadStations; k++)
    {
        int idx = BehindStations + k;
        int st = _lanes.Wrap(anchor + k);
        float free = _lanes.Offset(LaneId.Free, st);
        float abs = intent.FreezeLateral ? me.TrackD : free + intent.TargetDeltaMeters;
        _prefD[idx] = Math.Clamp(abs, _lowD[idx], _highD[idx]);
    }
}
```

`intent.SideSign != 0` 且 `FocusRivalId` 命中时，该对手的 `effectiveSlot` 用 `SideSign > 0 ? Slot.Left : Slot.Right`，不再由走廊自己选边。

`LaneDriver.Initialize` 在 `_corridor = new TrafficCorridor(...)` 之后加 `_tactics = new RacecraftTactics(_lanes);`。字段是 `private RacecraftTactics? _tactics;`。12 Hz 块里先：

```csharp
_tactics?.Update(
    in context,
    _speed[0],
    in _envelope,
    _planShares,
    station => _corner[0][_lanes.Wrap(station)],
    station => _brake[0][_lanes.Wrap(station)],
    station => _road[0][_lanes.Wrap(station)],
    _turnLag.Seconds);
```

再把走廊最后一个参数从 `RacecraftIntent.None` 换成 `_tactics?.Intent ?? RacecraftIntent.None`。加一个只读口 `internal RacecraftPhase TacticPhase => _tactics?.Phase ?? RacecraftPhase.Behind;`，测试可以读相位，生产路径不依赖它。

- [ ] **Step 4: Run test to verify it passes**

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~RacecraftTacticsTests`

Expected: PASS，6 个事实全部通过。蓝旗纯函数若差 0.01，先核对 `CatchSeconds(50) = 1.20`，不要放宽赛程比例。

然后确认任务 1 和任务 2 的测试仍通过：

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~BrakeFeelAbilityTests|FullyQualifiedName~CorridorDrivabilityTests`

Expected: PASS。

- [ ] **Step 5: Commit**

```bash
git add Drivers/Lanes/RacecraftTactics.cs Drivers/Lanes/TrafficCorridor.cs Drivers/Lanes/LaneDriver.cs Drivers/Tests/RacecraftTacticsTests.cs
git commit -m "feat(drivers): add the six-state racecraft brain"
```

---

### Task 4: Full 6-Car Grid Integration and Demo Verification

**Files:**

- Modify: `Drivers/Tests/TrafficCorridorTests.cs`（`SixDistinctAbilityCarsStandingStartFromGridBenchmarkAndCleanRace` 的断言，约 301 行之后）
- Modify: `Tools/Demo/LaneDriverDemo.cs`（`_Ready` 开头）
- Test: `Drivers/Tests/TrafficCorridorTests.cs`

**Interfaces:**

- Consumes: 任务 1 的 `brakeRoom`、任务 2 的门控和限速、任务 3 的 `RacecraftTactics`。测试只通过 `RaceSimulation`、`LaneDriver`、`RaceCar`、`ICarMotion.Motion`、`CarMotion.Spins`、`CarMotion.OverLimit`、`CarMotion.Spinning`、`RaceCar.HitCarThisStep`、`RaceCar.BoundaryContactSeconds`、`RaceProgress.Region` 观察。
- Produces: 无新的生产类型。演示在 `STINTEGY_HEADLESS_BENCH=1` 时打印同一组计数后退出。

- [ ] **Step 1: Write the failing test**

在 `SixDistinctAbilityCarsStandingStartFromGridBenchmarkAndCleanRace` 末尾、现有 `Assert.Equal(0, contacts)` 之后，把硬指标写全。现有测试还没有 `Spins == 0`、`OffTrackSeconds == 0`、P5/P6 `SmokeSeconds == 0` 和 `< 500` µs，所以这一步先红：

```csharp
Assert.Equal(0, contacts);
Assert.False(wall, "a car hit the wall during the 6-car standing-start race");
Assert.True(meanUs < 500d, $"6-car step mean {meanUs:F1} us exceeds 500 us");
for (int i = 0; i < cars.Length; i++)
{
    int spins = ((ICarMotion)cars[i].Car).Motion(cars[i].State).Spins;
    Assert.Equal(0, spins);
    Assert.Equal(0f, offTrackSeconds[i]);
    if (cars[i].Id is "P5-Contender" or "P6-Champion")
        Assert.Equal(0f, smokeSeconds[i]);
}
```

`meanUs`、`offTrackSeconds`、`smokeSeconds`、`contacts`、`wall` 都是这个测试里已经存在的局部变量。冒烟计数保持原条件：`OverLimit > 0.1f` 或 `Spinning`，速度 `> 9f`，且 `Region == TrackRegion.RacingSurface`。

- [ ] **Step 2: Run test to verify it fails**

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~SixDistinctAbilityCarsStandingStartFromGridBenchmarkAndCleanRace`

Expected: FAIL。在任务 3 之前，直道提前松油、折角急刹或固定 `recoverV` 会让 P5/P6 的 `smokeSeconds` 大于 0，或 `meanUs` 的断言还没写上时编译失败。写上断言之后，未完成的行为必须让 `Assert.Equal(0f, smokeSeconds[i])` 或接触断言失败。不要把 500 µs 放宽，也不要只在 Debug 下跑。

- [ ] **Step 3: Implement the demo check and close remaining metric gaps**

`LaneDriver`、`TrafficCorridor`、`RacecraftTactics` 不再加新策略。若步骤 2 里 P5/P6 仍冒烟，只允许回到任务 1 的 `brakeRoom` 或任务 2 的门控检查调用顺序，不允许重新引入 `2f * a * s`。本任务要写进演示程序的是同一套计数，方便肉眼核对。

在 `LaneDriverDemo._Ready` 的发车循环之后、`new RaceView3D` 之前插入：

```csharp
if (System.Environment.GetEnvironmentVariable("STINTEGY_HEADLESS_BENCH") == "1")
{
    const float dt = 1f / 60f;
    const int steps = 60 * 30;
    int contacts = 0;
    int wall = 0;
    var smoke = new float[simulation.Cars.Count];
    var off = new float[simulation.Cars.Count];
    var clock = System.Diagnostics.Stopwatch.StartNew();
    for (int step = 0; step < steps; step++)
    {
        simulation.Step(dt);
        for (int i = 0; i < simulation.Cars.Count; i++)
        {
            RaceCar car = simulation.Cars[i];
            CarMotion motion = ((ICarMotion)car.Car).Motion(car.State);
            bool outside = car.Progress.Region != TrackRegion.RacingSurface;
            if ((motion.Spinning || motion.OverLimit > 0.1f) && car.State.Speed > 9f && !outside)
                smoke[i] += dt;
            if (outside)
                off[i] += dt;
            if (car.HitCarThisStep && i == 0)
                contacts++;
            if (car.BoundaryContactSeconds > 0f)
                wall++;
        }
    }
    clock.Stop();
    double meanUs = clock.Elapsed.TotalMicroseconds / steps;
    GD.Print($"[Bench] contacts={contacts} wall={wall} meanUs={meanUs:F1}");
    for (int i = 0; i < simulation.Cars.Count; i++)
    {
        RaceCar car = simulation.Cars[i];
        int spins = ((ICarMotion)car.Car).Motion(car.State).Spins;
        GD.Print($"[Bench] {car.Id} spins={spins} off={off[i]:F2} smoke={smoke[i]:F2}");
    }
    GetTree().Quit();
    return;
}
```

先核对 `RaceSimulation` 暴露车辆的属性名。现有测试保存的是自己的 `RaceCar[]`，演示程序若没有 `simulation.Cars`，就在发车循环里把 `RaceCar` 放进 `List<RaceCar>`，基准循环改用那份列表。接触计数按「这一步有任意一车 `HitCarThisStep`」计一次，与测试的 `contacts++` 一致，不要按车重复加。墙的计数是帧次数，打印即可；测试断言的是「有没有发生」，演示打印的是同一事件。

`GetTree().Quit()` 之后的 `return` 必须跳过 `RaceView3D`，否则无头进程还会去建场景。

- [ ] **Step 4: Run test to verify it passes**

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~SixDistinctAbilityCarsStandingStartFromGridBenchmarkAndCleanRace`

Expected: PASS。日志里的六车步长均值小于 500 µs，P5-Contender 与 P6-Champion 的 `lockup/smoke` 为 `0.00s`，每车 `spins=0`、`offTrack=0.00s`，`contacts` 为 0，没有墙。

再跑全部驾驶测试，确认六项专项还在：

Run: `dotnet test Drivers/Tests/StintegyEVO.Drivers.Tests.csproj -c Release --filter FullyQualifiedName~BrakeFeelAbilityTests|FullyQualifiedName~CorridorDrivabilityTests|FullyQualifiedName~RacecraftTacticsTests|FullyQualifiedName~TrafficCorridorTests`

Expected: PASS。

演示核对：

```bash
STINTEGY_HEADLESS_BENCH=1 STINTEGY_CARS=6 STINTEGY_CIRCUIT=silverstone
```

在 Godot 演示宿主里启动 `LaneDriverDemo`。预期控制台出现 `[Bench]`，`contacts=0`，`wall=0`，P5 与 P6 的 `smoke=0.00`、`spins=0`、`off=0.00`，`meanUs` 小于 500。不设 `STINTEGY_HEADLESS_BENCH` 时画面发车与改动前相同。

- [ ] **Step 5: Commit**

```bash
git add Drivers/Tests/TrafficCorridorTests.cs Tools/Demo/LaneDriverDemo.cs
git commit -m "test(drivers): require a clean six-car grid and a headless bench"
```

---

## 规格覆盖

- 第 1 节根因：任务 1 替换 `-_maxBrake`；任务 2 删折角、删 `recoverV`、删开根号；任务 3 补一次变线和蓝旗。
- 第 2 节单向数据流：任务 3 的调用顺序是战术、走廊、踏板。`RacecraftTactics` 不读走廊私有场。
- 第 3 节 `brakeCircle` 与车底曲率：任务 1 的五个测试锁住公式、木桶、0.5f 回退和预瞄坑。
- 第 4 节门控与空间限速：任务 2 的 `PassesDrivableGate`、`Ceiling0`、`BrakeFrom` 倒推和 `AddFollowerLimit`。
- 第 5 节六态：任务 3 的 `Update` 优先级与六条测试一一对应。`Behind` 只作为状态 2 的冷却回退和空闲相位。
- 第 6 节硬指标：任务 4。

实现时不要把 `LaneSpeedPlanner.ChooseAhead` 里的步长积分改掉。那是规划器内部实现，不是走廊里的固定减速度副本。
