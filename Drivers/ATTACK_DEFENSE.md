# 攻防层与赛会干事设计（草案，2026-10-19）

状态：**设计，未实现**。先给用户审，审过再写代码。

## 0. 范围

目标是在现有三线司机（`Drivers/Lanes/`）之上加两层：

1. **赛会干事（Race Control / Stewards）**：Core 里的确定性判罚引擎，负责事件检测、旗语、罚则和成绩；
2. **攻防层（Racecraft）**：AI 对 AI 的跟车、超车、防守、并排、让车、回场，只消费赛会发布后的状态。

共同原则：

- 赛会干事是**世界服务**，不是 driver 的一部分；所有车看到同一份判罚/旗语；
- 车手只使用 `IDriverController` 能看到的冻结帧，不读对手私有计划，也不读干事内部证据；
- 能力值（`DriverAbilities`）决定感知、决断和风险偏好，不改变 Core 物理；
- 确定性、可复现、可做场景测试。

不做：

- 玩家输入、手柄/键盘；
- 隐藏的车辆优势、读心、全图感知；
- 进站/换胎的完整策略（等 #91 和 `CarState.Compound` 打通后再接）。

## 1. 现有接口

控制器每步拿到：

- `DriverContext.Profile`：`DriverAbilities`，含 `Awareness` / `Overtaking` / `Defending` / `Reactions` / `Consistency`；
- `DriverContext.Track`：只读的 `DriverTrackView`（采样、投影、路宽、曲率）；
- `DriverContext.Frame`：所有车的 `RaceCarSnapshot`，含位置、航向、速度、`TrackS/TrackD`、圈数、区域、策略、上一帧输入、轮胎、能量；
- `LaneDriver` 已有的动作面：`TargetRange`（0..1，离开 free 线多远）、`TargetSide`（-1 右 / +1 左）、`LaneShares`、轮胎/能量档。

目前没有、本设计要补的：

- 赛会控制、罚则、旗语、事件日志；
- `CarState.Compound` 没进 snapshot（进站换胎前要补）；
- “换线窗口/换线速度”表。

## 2. 状态机

建议一个显式状态机，状态之间带滞回和最短驻留时间，避免每帧抖：

| 状态 | 进入条件 | 主要动作 | 退出 |
|---|---|---|---|
| `FreeRun` | 感知范围内没有相关车 | 目标 free 线，按计划跑 | 检测到前车/后车 |
| `Follow` | 前车在感知距离内，且不满足超车条件 | 保持 1.0–1.5 s 间距，准备侧移 | 超车窗口出现 / 前车拉开 |
| `Attack` | 有超车窗口，风险可接受 | 选边、完成换线、在直线/刹车区完成超越 | 完成 / 放弃 / 被反超 |
| `Defend` | 后车进入威胁距离 | 占内线、留一车宽、只做一次变线 | 后车拉开 / 并排 |
| `SideBySide` | 横向重叠 | 保持线位，不向对手打方向，留空间 | 一方领先 / 进入弯道 |
| `Recover` | 出赛道、打滑、旋转 | 降速、回场、让行 | 回到赛道且稳定 |
| `Yield` | 蓝旗 / 被套圈 | 让出赛车线，不阻挡 | 后车通过 |
| `ServingPenalty` | 赛会发布 drive-through / stop-go | 按罚则进站/限速/让行 | 罚则执行完 |

规则：

- **最短驻留**：`Attack`/`Defend` 至少保持 0.3–0.5 s，除非出现碰撞风险；
- **一次性变线**：一次直线/刹车区只做一次方向选择，不来回摆；
- **承诺**：进入 `Attack` 后，只有“对手已占位/空间消失/将出赛道”才 abort；
- **不回头**：并排后不再读对手的“意图”，只按位置和空间做反应；
- **让罚则优先**：`ServingPenalty` 压过 `Attack/Defend`，先执行赛会命令。

## 3. 感知

`Awareness` 映射为：

- 检测距离：`40 + 0.6 * Awareness` 米（0..100 → 40..100 m）；
- 更新间隔：`1 / (10 + 0.1 * Awareness)` 秒（10–20 Hz）；
- 反应延迟：`Reactions` 映射 0.05–0.25 s；
- 滞回：进入/退出的距离差 5–10 m，防止边界抖动。

每步只从 snapshot 计算：

- 沿赛道差 `ds = WrapS(other.TrackS - ego.TrackS)`；
- 横向差 `dd = other.TrackD - ego.TrackD`；
- 接近率 `closing = ego.Speed - other.Speed`；
- 横向重叠：`|dd| < (ego.Width + other.Width) / 2 + margin`；
- 预测位置：按当前速度/航向推 0.5–2 s，再投影回赛道；不预测对手会突然变线；
- 旗语/罚则：只读 `RaceControlSnapshot` 里已发布的状态，不读事件证据。

不给 AI 额外信息；`Reactions` 延迟用控制器内部的时间戳实现，不读墙钟。

## 4. 候选线与换线

三条基础线来自 `LaneSet`：`Free`、`Left`、`Right`，中间用 `BlendedOffset` 连续混合。

候选动作：

- 保持当前 `TargetRange/TargetSide`；
- 向 `Left`/`Right` 变线（range 0→1）；
- 回到 free 线；
- 在左右线之间做一次 S 形换线（已有 `RangeAcceleration/SideAcceleration`）。

可行性检查：

- 目标偏移处的路面宽度、路肩/护墙、对手占位；
- 换线所需时间：`RangeRate/SideRate` + 加速度限制；
- 弯道曲率与抓地：用 `LaneSpeedPlanner`/`LaneEnvelope` 估算目标线在弯里的速度；
- 刹车区/入弯点不做非必要变线；
- 对手在目标线上时，换线窗口不成立；
- 黄旗/双黄旗区间内不做超车性变线。

“哪里能换线、能换多快”的表：

- 以 2 m 站点为粒度，记录每个站点的可用横向范围、曲率、路面、对手占位；
- 运行时先查表，再做连续检查；表可以离线生成，也可以运行时缓存；
- 不在弯心/刹车区做“最后一刻”变线。

## 5. 超车

按优先级：

1. **直线/DRS 区**：速度差 + 尾流，选对手空出的一侧；在刹车点前完成并排；
2. **刹车区**：只在对手留出空间、自己不会锁死/出赛道时；晚刹车幅度受 `CarControl`/`Consistency` 限制；
3. **弯后 switchback**：对手为防守走内线、出弯慢时，走外线换内线；
4. **外线**：只在弯道半径/抓地允许、且不会出赛道时；否则放弃。

放弃条件：

- 对手已占住目标线；
- 自己将超出赛道/护墙；
- 进入弯道前没有完成并排；
- 黄旗区间；
- 被罚风险超过阈值（见第 7 节）。

`Overtaking` 映射：

- 攻击距离/窗口宽度：越高越早判断、越敢在刹车区晚一点；
- 风险阈值：越高越能接受“贴边”；
- 但 `CarControl`/`Consistency` 低时，晚刹车幅度要收回来，避免“能力值高就无视物理”。

## 6. 防守

- 在直线上占内线，给对手留至少一车宽；
- 只做一次变线，不连续横移；
- 对手已经并排时，不向对手方向打方向；
- 弯道里不把对手挤出赛道；出弯给空间；
- 被套圈/蓝旗时进入 `Yield`，让出赛车线，不防守。

`Defending` 映射：

- 防守距离/反应：越高越早占位；
- 留空间：始终至少一车宽；`Defending` 不降低这个下限；
- 风险偏好：越高越敢在边缘留空间，但不允许“逼出赛道”。

## 7. 赛会干事（Race Control / Stewards）

### 7.1 定位

赛会干事是**世界服务**，不是 driver：

- 建议放在 Core：`Core/Racing/RaceControl/`，由 `RaceSimulation` 在每个物理子步/接触解算之后推进；
- 它读所有车、接触、越界、位置和赛道，写一份 `RaceControlSnapshot`；
- 所有车看到同一份判罚/旗语；车手不能读干事内部证据、阈值或“下一步会不会罚”；
- 确定性：同样的接触/越界/位置，得到同样的判罚；事件日志可回放；
- 不改车物理：罚时加在成绩上，drive-through/stop-go 走维修区，DSQ 从结果移除。

### 7.2 事件检测

| 事件 | 输入 | 判定 |
|---|---|---|
| 赛道边界 | 四轮位置、`TrackD`、路宽、区域 | 四轮出白线；警告累计；必要时删圈/罚时 |
| 把对手逼出赛道 | 并排重叠、横向间距、对手到边缘距离 | 防守方留不足一车宽 -> 防守方；进攻方冲进弯内且自己出界 -> 进攻方 |
| 碰撞 | `CarContactResolver` 的接触、相对速度/航向、前后位置、转向输入 | 有明显责任方 -> 罚；责任不清 -> racing incident |
| 不安全回场 | 回场轨迹、来车距离/速度 | 回场前未让行且造成冲突 -> 罚 |
| 阻挡/二次变线 | 变线次数、横向移动、对手距离 | 一次变线；连续横移 -> 警告/罚 |
| 蓝旗 | 圈数差、后车距离 | 被套圈车须让行；未让行 -> 罚 |
| 黄旗超车 | 黄旗区间、位置变化 | 黄旗区间完成超车 -> 罚 |
| 抢跑/维修区限速 | 起跑、pit lane | 后续阶段（等 #91） |

原则：

- 先有实测接触/越界，再判；不预判、不读心；
- 责任不清就不罚（racing incident），不要为了“有接触”硬找人背锅；
- 判罚阈值和罚则写在 `RaceRules` 里，可按赛事配置，不散落在代码各处。

### 7.3 罚则

| 罚则 | 执行 |
|---|---|
| Warning | 记录；累计到阈值升级 |
| Lap deleted | 删除该圈成绩（赛道边界） |
| 5s / 10s time penalty | 加在最终成绩上 |
| Drive-through | 进维修区通过，限速；未执行 -> 转罚时 |
| Stop-go | 进站停 N 秒；未执行 -> 转罚时 |
| Grid drop | 排位赛/下一场（现在没有排位，先留接口） |
| DSQ | 从结果移除 |
| Penalty points | 记在 driver/season 上，供后续赛事使用 |

`PenaltyState` 至少包含：

```text
Pending / Served
Type
Seconds
Reason
IncidentId
Lap / RaceTime
```

事件日志 `IncidentLog` 记录：时间、地点 `TrackS/TrackD`、涉及车、规则、证据摘要、判罚。用于回放、测试和赛后报告。

### 7.4 旗语

`RaceControlSnapshot` 按 sector 发布：

- Green；
- Yellow / Double Yellow（禁止超车，减速）；
- Blue（被套圈让行）；
- Red（停止，后续）；
- Safety Car（后续）。

车手只看到已发布的旗语；黄旗/蓝旗的行为在 `Racecraft` 里执行，判罚仍由赛会做。

### 7.5 车手风险 vs 赛会判罚

这是两层，不能混：

- **赛会判罚**是权威的、确定性的，由 `RaceControl` 做；
- **车手风险**是车手自己的估计，受 `Awareness` / `Reactions` / `Overtaking` / `Defending` 影响，可能估错；
- 车手不能因为“我觉得不会被罚”就免罚；也不能读赛会的内部风险分；
- `Overtaking`/`Defending` 只改车手的风险阈值和留空间习惯，不改判罚结果；
- `CarControl`/`Consistency` 影响车手能否把车留在线上，不改变规则。

### 7.6 可复现与反作弊

- 判罚只依赖冻结帧/接触/赛道，不依赖墙钟、随机数或网络；
- 事件日志和 `RaceControlSnapshot` 可重放；
- 不允许隐藏抓地/动力、读对手私有计划、提前知道判罚；
- 测试必须覆盖“同样事件 -> 同样判罚”。

## 8. 代码落点

### 8.1 Core：赛会干事

建议新增：

```text
Core/Racing/RaceControl/
  RaceControl.cs             // 每步推进，持有事件/罚则状态
  RaceRules.cs               // 阈值、罚则、旗语配置
  Incident.cs                // 事件类型、涉及车、证据
  Penalty.cs                 // 罚则、执行状态
  RaceControlSnapshot.cs     // 发布给所有车的旗语/罚则/事件摘要
```

`RaceSimulation`：

1. 物理子步/接触解算；
2. `RaceControl.Step(...)`；
3. 下一帧 `CaptureFrame` 带上 `Control`；
4. 车手只读 `RaceControlSnapshot`，不读 `RaceControl` 内部。

`RaceCarSnapshot` 增加每个车的 `PenaltyState`（pending/served、time added）；`DriverContext` 增加 `Control`。

### 8.2 Drivers：攻防层

```text
Drivers/Racecraft/
  RacecraftState.cs          // 状态枚举 + 决策记录
  OpponentModel.cs           // 相对位置、接近率、重叠、预测
  RacecraftDecision.cs       // 目标 range/side、风险、原因
  AttackDefenseController.cs // IDriverController 包装 LaneDriver
  RacecraftGains.cs          // 能力值映射、滞回、阈值
```

`AttackDefenseController` 持有 `LaneDriver`，每步：

1. 读 `DriverContext.Frame` 里除自己外的车；
2. 读 `DriverContext.Control` 的旗语/罚则；
3. 更新状态机/对手模型；
4. 输出 `TargetRange/TargetSide` 给 `LaneDriver`；
5. `LaneDriver` 仍负责曲率、速度、换线 S 和轮胎/能量档。

攻防层不放进 Core；Core 只认 `DriverInput`。

## 9. 测试与验收

### 9.1 赛会干事

- 赛道边界：四轮出线 -> 警告 -> 删圈/罚时；不误判压线；
- 逼出赛道：并排时留不足一车宽 -> 罚；racing incident 不罚；
- 碰撞：明显责任方 -> 罚；责任不清 -> 不罚；
- 不安全回场：来车距离内回场 -> 罚；
- 阻挡/二次变线：连续横移 -> 罚；
- 蓝旗未让行 -> 罚；
- 黄旗超车 -> 罚；
- 罚则执行：5s/10s 加成绩，drive-through/stop-go 未执行转罚时；
- 确定性：同一事件重放 -> 同一 `IncidentLog` / 罚则。

### 9.2 攻防层

场景（两车、确定性、无随机）：

- 直道超车：快车从 1.0 s 后追上，能完成且不碰；
- 刹车区 dive-bomb：防守车留空间，进攻车不锁死/不出界；
- switchback：内线防守后被外线反超；
- 并排过弯：两车都留在赛道内；
- 蓝旗：被套圈车让行，不阻挡；
- 出界回场：不碰撞、不抢线；
- 抖动：同一场景跑 60 s，`TargetSide` 不来回跳。

指标：

- 碰撞次数、出界秒数、罚则数量/罚时总和；
- 超车完成率、被反超率；
- 与 `FreeRun` 的圈速差；
- 决策频率/变线次数（防抖）。

## 10. 分阶段

1. **S0**：赛会干事数据模型 + 赛道边界/警告 + `RaceControlSnapshot`；不接车手行为；
2. **S1**：碰撞/逼出赛道/不安全回场 + 罚则执行（时间罚）；
3. **S2**：黄旗/蓝旗/红旗 + 车手反应（`Yield`/减速）；
4. **P1**：直道 `Attack/Defend`，接 `RaceControlSnapshot`；
5. **P2**：刹车区/switchback/外线，接 `CarControl`；
6. **P3**：完整风险整合、事件回放、指标门槛；
7. **P4**：进站/drive-through/compound（等 #91 和 `CarState.Compound`）。

每阶段先给场景/指标，再合代码。

## 11. 待你定

1. 罚则做到多严？只做警告/罚时，还是也要 drive-through/stop-go/DSQ/罚分？
2. 赛道边界怎么算？四轮出白线、警告几次升级、是否删圈？
3. 碰撞/逼出赛道的责任判定：严格找人背锅，还是“不清楚就不罚”？
4. 旗语现在做到哪一步？黄旗/蓝旗够不够，红旗/安全车要不要？
5. 赛会干事放 `Core/Racing/RaceControl/` 是否合适，还是独立项目？
6. 攻防层放 `Drivers/Racecraft/` 是否合适，还是并进 `Lanes`？
7. 优先级：先做赛会干事，还是先做低抓地力/中性胎标定？
