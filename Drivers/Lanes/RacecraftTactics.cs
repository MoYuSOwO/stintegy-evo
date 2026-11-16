using System.Numerics;
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
    private const float LaunchColumnMeters = 75f;
    private const float LookAheadMeters = 140f;

    private readonly LaneSet _lanes;
    private readonly float[] _freeK;
    private readonly List<CornerMap.Bend> _bends;
    private readonly CornerMap.Pins[] _pins;
    private readonly float[] _switchProfile = new float[LocalStations];
    private readonly float[] _holdProfile = new float[LocalStations];
    private readonly float[] _flatProfile = new float[LocalStations];
    private readonly float[] _zeroProfile = new float[LocalStations];
    private readonly float[] _winCurvature = new float[LocalStations];
    private readonly float[] _winLead = new float[LocalStations];
    private readonly float[] _winStep = new float[LocalStations];
    private readonly float[] _winFactor = new float[LocalStations];
    private readonly float[] _winBrake = new float[LocalStations];
    private readonly LaneRoad[] _winRoad = new LaneRoad[LocalStations];
    private readonly float[] _winGuess = new float[LocalStations];
    private readonly float[] _winLimits = new float[LocalStations];
    private readonly float[] _winSpeed = new float[LocalStations];

    private bool _launchCaptured;
    private bool _isStandingStart;
    private float _launchStartS;
    private float _launchColumnD;
    private bool _defendedThisStraight;
    private int _defendBend = -1;
    private int _defendSign;
    private float _defendShare;
    private float _defendAbs;
    private bool _defendFrozen;
    private int _commitSign;
    private string? _commitId;
    private int _switchBend = -1;
    private string? _switchRival;
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
        Func<float, float> turnLagSeconds,
        float followGapSeconds)
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
        bool ordersHoldGap = followGapSeconds > 0.05f;
        bool launch = InLaunch(me, now);
        bool columnHold = launch && me.TotalDistanceMeters - _launchStartS < LaunchColumnMeters;
        bool loaded = Loaded(anchor, ego, me);
        Rival? ahead = NearestAhead(context, me, freeSpeedPlan);
        Rival? behind = NearestBehind(context, me, freeSpeedPlan);
        Rival? hazard = NearestHazardAhead(in context, me, freeSpeedPlan);
        Rival? blue = LappingCar(context, me, abilities);

        bool latchedDefend = _defendedThisStraight && _defendBend >= 0;
        RacecraftIntent intent;
        if (latchedDefend && TryDefend(me, behind, anchor, ego, Braking(anchor, ego, me, freeSpeedPlan), defending, out intent))
        {
        }
        else if (TryHoldBeside(me, in context, freeSpeedPlan, out intent))
        {
        }
        else if (ordersHoldGap)
        {
            _commitId = null;
            _commitSign = 0;
            intent = BehindIntent();
        }
        else if (columnHold)
        {
            intent = ColumnIntent(anchor);
        }
        else if (blue is { } yieldTo && !loaded)
        {
            float side = MathF.Abs(me.TrackD - yieldTo.TrackD) < 0.3f ? 1f : MathF.Sign(me.TrackD - yieldTo.TrackD);
            float edge = MathF.Max(0.4f, _lanes.HalfWidth(anchor) - SafeEdgeMarginMeters);
            float absTargetD = Math.Clamp(side * (0.5f * (me.WidthMeters + yieldTo.Width) + 3.0f), -edge, edge);
            bool close = yieldTo.Gap > -10f && yieldTo.Gap < 10f;
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.BlueFlagYield,
                TargetDeltaMeters = absTargetD - Free(anchor),
                YieldSpeedScale = close ? 0.88f : 0f,
                FocusRivalId = yieldTo.Id,
            };
        }
        else if (hazard is { } danger && !(launch && !danger.Skewed))
        {
            int sign = _commitId == danger.Id && _commitSign != 0
                ? _commitSign
                : OpenSide(danger);
            _commitId = danger.Id;
            _commitSign = sign;
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.PullOutAttack,
                SideSign = sign,
                TargetDeltaMeters = float.NaN, // 由走廊沿绝对 TrackD 平滑漏斗直接绕开障碍车，禁止把单点相对差加到远方 Free(st) 上
                EmergencyAvoid = true,
                FocusRivalId = danger.Id,
            };
        }
        else if (TrySwitchback(me, ahead, anchor, ego, in context, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, out intent))
        {
        }
        else if (TryPull(me, ahead, anchor, ego, now, loaded, boldness, overtaking, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, out intent))
        {
        }
        else if (TryDefend(me, behind, anchor, ego, Braking(anchor, ego, me, freeSpeedPlan), defending, out intent))
        {
        }
        else if (launch && !SlowStarterRoom(me, ahead, ego))
        {
            intent = ColumnIntent(anchor);
        }
        else if (ahead is { } leader && leader.Gap > PullDistance(me, leader, anchor, boldness) && !loaded && leader.Gap < 48f)
        {
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.DraftTow,
                TargetDeltaMeters = float.NaN, // 保持最佳自由线出弯/推进，靠 FullThrottleTow 吃尾流而不跟着慢车当前瞬时偏航横移
                FullThrottleTow = true,
                FocusRivalId = leader.Id,
            };
        }
        else
        {
            intent = BehindIntent();
        }

        if (launch && AheadOfFirstEntry(anchor) && !Overlap(me, ahead) && !Overlap(me, behind)
            && intent.Phase is RacecraftPhase.PullOutAttack or RacecraftPhase.Switchback or RacecraftPhase.DraftTow)
        {
            _commitId = null;
            _commitSign = 0;
            intent = BehindIntent();
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
        // A loaded bend still allows a pass when the closing speed is large.
        // PlanWindow scales the move down to the grip that is actually there.
        if (!alreadyCommitted && loaded && !leader.Hazard && ego - leader.Speed <= 4.0f)
            return false;
        if (now < _abortUntil && _commitId == leader.Id)
            return false;
        float gap = leader.Gap;
        float dPull = PullDistance(me, leader, anchor, boldness);
        float maxStrikeGap = leader.Hazard
            ? LookAheadMeters
            : Math.Clamp(dPull, 12f, 24f + 10f * overtaking + MathF.Max(0f, ego - leader.Speed) * 0.8f);
        // 尚未建立并排重叠时，一旦被拉开超出有效抽头窗口（或弯中已无闭合速度），立即清除旧抽头承诺回到最佳自由线追近
        if (alreadyCommitted && !leader.Hazard && !Fia(me, leader) &&
            (gap > maxStrikeGap + 4f || (gap > 10f && ego < leader.Speed + 0.15f)))
        {
            _commitId = null;
            _commitSign = 0;
            return false;
        }
        // 未锁定时只有在具备真实闭合速度或直道潜在速度优势时才发起新抽头
        float freePotential = PlanSpeed(freeSpeedPlan, anchor + 20);
        if (!alreadyCommitted && !leader.Hazard && ego < leader.Speed + 0.25f && freePotential <= leader.Speed + 1.2f)
            return false;
        if (!alreadyCommitted && gap > maxStrikeGap)
            return false;
        int inside = NextInsideSign(anchor);
        int sign;
        if (alreadyCommitted)
        {
            sign = _commitSign;
        }
        else
        {
            float roomLeft = Room(leader, me, -1);
            float roomRight = Room(leader, me, +1);
            bool canLeft = roomLeft >= MinPassRoomMeters;
            bool canRight = roomRight >= MinPassRoomMeters;
            if (!canLeft && !canRight)
                return false;
            if (canLeft && canRight)
            {
                float clearSide = 0.5f * (me.WidthMeters + leader.Width) + 0.90f;
                float leftScore = roomLeft * 0.30f - MathF.Abs(me.TrackD - (leader.TrackD - clearSide)) * 0.25f + (inside < 0 ? 1.1f : -0.2f);
                float rightScore = roomRight * 0.30f - MathF.Abs(me.TrackD - (leader.TrackD + clearSide)) * 0.25f + (inside > 0 ? 1.1f : -0.2f);
                sign = rightScore >= leftScore ? +1 : -1;
            }
            else
            {
                sign = canRight ? +1 : -1;
            }
            int other = -sign;
            if (Braking(anchor, ego, me, freeSpeedPlan) && sign == inside)
            {
                FillFlat(PassDelta(me, leader, anchor, sign));
                float cand = WindowSpeed(anchor, _flatProfile, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, 0);
                float still = WindowSpeed(anchor, _zeroProfile, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, 0);
                if (!TrafficCorridor.PassesDrivableGate(cand, ego, still))
                {
                    if (Room(leader, me, other) < MinPassRoomMeters)
                        return false;
                    sign = other;
                }
            }
        }
        int aheadPinDist = _pins.Length > 0 ? _lanes.Wrap(_pins[NextBend(anchor)].Apex - anchor) : 0;
        float checkRoom = aheadPinDist is > 4 and < 65
            ? MathF.Min(Room(leader, me, sign), RoomAtStation(leader, anchor + Math.Min(aheadPinDist, 25), sign))
            : Room(leader, me, sign);
        if (checkRoom < AbortRoomMeters && !Fia(me, leader) && !RearHold(me, leader, ego))
        {
            int other = -sign;
            bool onShutSide = sign * (me.TrackD - leader.TrackD) > 0.4f;
            if (!onShutSide && Room(leader, me, other) >= MinPassRoomMeters)
            {
                sign = other;
            }
            else
            {
                _commitId = null;
                _commitSign = 0;
                _abortUntil = now + AbortCooldownSeconds;
                float halfLen = 0.5f * (me.LengthMeters + leader.Length);
                bool close = leader.Gap < halfLen + 2.5f;
                intent = new RacecraftIntent
                {
                    Phase = RacecraftPhase.Behind,
                    TargetDeltaMeters = float.NaN,
                    FreezeLateral = close, // 先纵向退回车尾后方，严禁在后保险杠旁往正在关门的前车身上斜切
                    FocusRivalId = leader.Id,
                };
                return true;
            }
        }
        _commitId = leader.Id;
        _commitSign = sign;
        intent = new RacecraftIntent
        {
            Phase = RacecraftPhase.PullOutAttack,
            SideSign = sign,
            TargetDeltaMeters = float.NaN, // 由走廊贴对手一车宽（rD ± clearSide）干净超越，不横跨整条马路去贴对面白线
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

        if (_defendedThisStraight && _defendBend >= 0)
        {
            int toEntry = _lanes.Wrap(_pins[_defendBend].Entry - anchor);
            if (toEntry > 0 && toEntry < _lanes.Count / 2)
            {
                if ((braking || (behind is { } r && Fia(me, r))) && !_defendFrozen)
                    _defendFrozen = true;
                FillHold(anchor, _defendAbs);
                intent = new RacecraftIntent
                {
                    Phase = RacecraftPhase.OneMoveDefend,
                    SideSign = _defendSign,
                    TargetDeltaMeters = _holdProfile[0],
                    SpatialDeltaProfile = _holdProfile,
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
        _defendAbs = _defendShare * edgeHere;
        FillHold(anchor, _defendAbs);
        intent = new RacecraftIntent
        {
            Phase = RacecraftPhase.OneMoveDefend,
            SideSign = sign,
            TargetDeltaMeters = _holdProfile[0],
            SpatialDeltaProfile = _holdProfile,
            FreezeLateral = false,
            FocusRivalId = rival.Id,
        };
        return true;
    }

    private bool TrySwitchback(
        RaceCarSnapshot me, Rival? ahead, int anchor, float ego, in DriverContext context,
        ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope, LaneShares shares,
        Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds, out RacecraftIntent intent)
    {
        intent = default;
        if (_bends.Count == 0)
            return false;

        if (_switchBend >= 0)
        {
            CornerMap.Pins held = _pins[_switchBend];
            int pastExit = _lanes.Wrap(anchor - held.Exit);
            bool passedExit = pastExit > 0 && pastExit < _lanes.Count / 2;
            bool passClear = !TryRival(in context, me, _switchRival, freeSpeedPlan, out Rival heldRival)
                || PassCleared(me, heldRival, ego);
            if (!passedExit && !passClear)
            {
                FillSwitch(anchor, held);
                intent = SwitchIntent(anchor, heldRival);
                return true;
            }
            _switchBend = -1;
            _switchRival = null;
        }

        if (ahead is not { } leader || leader.Gap <= 0f || leader.Gap >= 14f)
            return false;
        int bend = NextBend(anchor);
        CornerMap.Pins pins = _pins[bend];
        if (PlanSpeed(freeSpeedPlan, pins.Apex) >= 58f || pins.Sign == 0)
            return false;
        int inside = -pins.Sign;
        int outside = pins.Sign;
        float entryFree = _lanes.Offset(LaneId.Free, pins.Entry);
        bool deepInside = inside * (leader.TrackD - entryFree) > 1.8f;
        if (!deepInside || Room(leader, me, inside) >= AbortRoomMeters)
            return false;
        if (Room(leader, me, outside) < MinPassRoomMeters)
            return false;
        FillSwitch(anchor, pins);
        FillFlat(0f);
        float cand0 = WindowSpeed(anchor, _switchProfile, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, 0);
        if (cand0 < ego - MoveSpeedToleranceMps)
            return false;
        int exitDist = _lanes.Wrap(pins.Exit - anchor);
        if (exitDist < _switchProfile.Length)
        {
            float exitWide = WindowSpeed(anchor, _switchProfile, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, exitDist);
            float exitStay = WindowSpeed(anchor, _flatProfile, ego, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, exitDist);
            if (exitWide <= exitStay)
                return false;
        }
        _switchBend = bend;
        _switchRival = leader.Id;
        intent = SwitchIntent(anchor, leader);
        return true;
    }

    private RacecraftIntent SwitchIntent(int anchor, Rival leader)
    {
        float abs = Free(anchor) + _switchProfile[0];
        int side = abs >= leader.TrackD ? +1 : -1;
        return new RacecraftIntent
        {
            Phase = RacecraftPhase.Switchback,
            SideSign = side,
            TargetDeltaMeters = _switchProfile[0],
            SpatialDeltaProfile = _switchProfile,
            FocusRivalId = leader.Id,
        };
    }

    private void FillSwitch(int anchor, CornerMap.Pins pins)
    {
        int inside = -pins.Sign;
        int outside = pins.Sign;
        int cutStart = _lanes.Wrap(pins.Apex + 4);
        int half = _lanes.Count / 2;
        int cutLen = Math.Max(1, _lanes.Wrap(pins.Exit - cutStart));
        for (int k = 0; k < LocalStations; k++)
        {
            int st = _lanes.Wrap(anchor + k);
            float edge = MathF.Max(0.4f, _lanes.HalfWidth(st) - SafeEdgeMarginMeters);
            float free = Free(st);
            float outsideD = outside * edge;
            float insideD = inside * edge;
            int aheadCut = _lanes.Wrap(cutStart - st);
            int aheadExit = _lanes.Wrap(pins.Exit - st);
            float abs;
            if (aheadCut <= half)
            {
                abs = outsideD;
            }
            else if (aheadExit <= half)
            {
                int fromCut = _lanes.Wrap(st - cutStart);
                float t = Math.Clamp(fromCut / (float)cutLen, 0f, 1f);
                float s = t * t * (3f - 2f * t);
                abs = outsideD * (1f - s) + insideD * s;
            }
            else
            {
                int pastExit = _lanes.Wrap(st - pins.Exit);
                float t = Math.Clamp(pastExit / 40f, 0f, 1f);
                float s = t * t * (3f - 2f * t);
                abs = insideD * (1f - s) + free * s;
            }
            _switchProfile[k] = abs - free;
        }
    }

    private void FillHold(int anchor, float absD)
    {
        for (int k = 0; k < LocalStations; k++)
        {
            int st = _lanes.Wrap(anchor + k);
            _holdProfile[k] = absD - Free(st);
        }
    }

    private void FillFlat(float value) => Array.Fill(_flatProfile, value);

    private float WindowSpeed(
        int anchor, float[] delta, float ego, ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope, LaneShares shares,
        Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds, int sample)
    {
        int n = Math.Min(delta.Length, LocalStations);
        for (int k = 0; k < n; k++)
        {
            _winCurvature[k] = LaneSet.Curvature(
                OffsetPoint(anchor, k - 3, delta, n),
                OffsetPoint(anchor, k, delta, n),
                OffsetPoint(anchor, k + 3, delta, n));
            _winStep[k] = MathF.Max(0.5f, (OffsetPoint(anchor, k + 1, delta, n) - OffsetPoint(anchor, k, delta, n)).Length());
            int wrapped = _lanes.Wrap(anchor + k);
            _winFactor[k] = cornerFactorAt(wrapped);
            _winBrake[k] = brakeFactorAt(wrapped);
            _winRoad[k] = roadAt(wrapped);
            _winGuess[k] = MathF.Max(8f, ego);
        }
        LaneSpeedPlanner.LeadCurvatures(_winCurvature.AsSpan(0, n), _winGuess.AsSpan(0, n), turnLagSeconds, _lanes.Spacing, _winLead.AsSpan(0, n));
        LaneSpeedPlanner.CornerLimits(envelope, _winLead.AsSpan(0, n), _winFactor.AsSpan(0, n), _winRoad.AsSpan(0, n), shares, _winLimits.AsSpan(0, n));
        float terminal = PlanSpeed(freeSpeedPlan, anchor + n - 1);
        LaneSpeedPlanner.PlanWindow(
            envelope, _winLead.AsSpan(0, n), _winStep.AsSpan(0, n), _winFactor.AsSpan(0, n), _winBrake.AsSpan(0, n),
            _winRoad.AsSpan(0, n), shares, terminal, _winLimits.AsSpan(0, n), _winSpeed.AsSpan(0, n));
        return _winSpeed[Math.Clamp(sample, 0, n - 1)];
    }

    private Vector2 OffsetPoint(int anchor, int rel, float[] delta, int n)
    {
        float d = rel < 0 ? delta[0] : rel >= n ? delta[n - 1] : delta[rel];
        return _lanes.Position(anchor + rel, _lanes.Offset(LaneId.Free, anchor + rel) + d);
    }

    private float PullDistance(RaceCarSnapshot me, Rival leader, int anchor, float boldness)
    {
        float dTail = 3.8f - 1.3f * boldness;
        int side = _commitSign == 0 ? NextInsideSign(anchor) : _commitSign;
        if (side == 0)
            side = Room(leader, me, +1) >= Room(leader, me, -1) ? +1 : -1;
        float passAbsD = PassAbsD(me, leader, anchor, side);
        float dLat = MathF.Abs(passAbsD - me.TrackD);
        float tMove = MathF.Sqrt(4f * MathF.Max(dLat, 0.5f) / PullLateralAccel) + 0.15f;
        return dTail + MathF.Max(0f, me.SpeedMetersPerSecond - leader.Speed) * tMove;
    }

    private float PassAbsD(RaceCarSnapshot me, Rival leader, int station, int sign)
    {
        (float lo, float hi) = Road(station);
        float clear = 0.5f * (me.WidthMeters + leader.Width) + (leader.Hazard ? 1.50f : 0.90f);
        float projLeaderD = leader.Hazard ? Math.Clamp(leader.TrackD + leader.LatSpeed * 0.6f, lo, hi) : leader.TrackD;
        return Math.Clamp(projLeaderD + sign * clear, lo, hi);
    }

    private float PassDelta(RaceCarSnapshot me, Rival leader, int station, int sign)
    {
        int st = _lanes.Wrap(station);
        return PassAbsD(me, leader, st, sign) - Free(st);
    }

    private (float Lo, float Hi) Road(int station)
    {
        int st = _lanes.Wrap(station);
        float edge = MathF.Max(0.4f, _lanes.HalfWidth(st) - SafeEdgeMarginMeters);
        float free = Free(st);
        return (MathF.Min(-edge, free), MathF.Max(edge, free));
    }

    private float SideDelta(int station, int sign)
    {
        if (sign == 0)
            return 0f;
        int st = _lanes.Wrap(station);
        (float lo, float hi) = Road(st);
        float absD = sign > 0 ? hi : lo;
        return absD - Free(st);
    }

    private float Room(Rival rival, RaceCarSnapshot me, int sign)
    {
        _ = me;
        int st = _lanes.Wrap((int)MathF.Round(_lanes.StationOf(rival.AbsS)));
        (float lo, float hi) = Road(st);
        return sign > 0 ? hi - rival.TrackD : rival.TrackD - lo;
    }

    private float RoomAtStation(Rival rival, int station, int sign)
    {
        int st = _lanes.Wrap(station);
        (float lo, float hi) = Road(st);
        float rivalRelFree = rival.TrackD - Free(_lanes.Wrap((int)MathF.Round(_lanes.StationOf(rival.AbsS))));
        float projRivalD = Math.Clamp(Free(st) + rivalRelFree, lo, hi);
        return sign > 0 ? hi - projRivalD : projRivalD - lo;
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
        return sign == 0 ? 0 : -sign;
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

    private static RacecraftIntent BehindIntent(string? focus = null) => new()
    {
        Phase = RacecraftPhase.Behind,
        TargetDeltaMeters = float.NaN,
        FocusRivalId = focus,
    };

    private RacecraftIntent ColumnIntent(int anchor) => new()
    {
        Phase = RacecraftPhase.StandingStart,
        TargetDeltaMeters = _launchColumnD - Free(anchor),
    };

    private readonly struct Rival
    {
        public string Id { get; init; }
        public float Gap { get; init; }
        public float AbsS { get; init; }
        public float TrackD { get; init; }
        public float Speed { get; init; }
        public float Width { get; init; }
        public float Length { get; init; }
        public float WheelBase { get; init; }
        public float RaceDistance { get; init; }
        public bool Hazard { get; init; }
        public bool Skewed { get; init; }
        public float LatSpeed { get; init; }
    }

    private Rival? NearestAhead(in DriverContext context, RaceCarSnapshot me, ReadOnlySpan<float> freeSpeedPlan) =>
        Scan(context, me, freeSpeedPlan, ahead: true, hazardOnly: false);

    private Rival? NearestBehind(in DriverContext context, RaceCarSnapshot me, ReadOnlySpan<float> freeSpeedPlan) =>
        Scan(context, me, freeSpeedPlan, ahead: false, hazardOnly: false);

    private Rival? NearestHazardAhead(in DriverContext context, RaceCarSnapshot me, ReadOnlySpan<float> freeSpeedPlan) =>
        Scan(context, me, freeSpeedPlan, ahead: true, hazardOnly: true);

    private Rival? Scan(in DriverContext context, RaceCarSnapshot me, ReadOnlySpan<float> freeSpeedPlan, bool ahead, bool hazardOnly)
    {
        Rival? best = null;
        float length = MathF.Max(me.TrackLengthMeters, _lanes.LengthMeters);
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            RaceCarSnapshot other = context.Frame[i];
            float gap = MathF.IEEERemainder(other.TrackS - me.TrackS, length);
            if (ahead ? gap <= 0f || gap > LookAheadMeters : gap >= 0f || gap < -36f)
                continue;
            Rival rival = MakeRival(me, other, gap, freeSpeedPlan);
            if (hazardOnly && !rival.Hazard)
                continue;
            if (best is { } have && MathF.Abs(gap) > MathF.Abs(have.Gap))
                continue;
            best = rival;
        }
        return best;
    }

    private bool TryHoldBeside(RaceCarSnapshot me, in DriverContext context, ReadOnlySpan<float> freeSpeedPlan, out RacecraftIntent intent)
    {
        intent = default;
        bool held = false;
        float bestAbs = float.MaxValue;
        float length = MathF.Max(me.TrackLengthMeters, _lanes.LengthMeters);
        float ego = MathF.Max(0f, me.SpeedMetersPerSecond);
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            RaceCarSnapshot other = context.Frame[i];
            float gap = MathF.IEEERemainder(other.TrackS - me.TrackS, length);
            if (gap > LookAheadMeters || gap < -36f)
                continue;
            Rival rival = MakeRival(me, other, gap, freeSpeedPlan);
            if (_commitId == rival.Id && PassCleared(me, rival, ego))
            {
                _commitId = null;
                _commitSign = 0;
            }
            if (!ShouldHold(me, rival, ego))
                continue;
            float abs = MathF.Abs(rival.Gap);
            if (abs >= bestAbs)
                continue;
            bestAbs = abs;
            int sign = me.TrackD >= rival.TrackD ? +1 : -1;
            if (_commitId == rival.Id && _commitSign != 0 && MathF.Abs(me.TrackD - rival.TrackD) < 0.4f)
                sign = _commitSign;
            intent = new RacecraftIntent
            {
                Phase = RacecraftPhase.SideBySide,
                SideSign = sign,
                TargetDeltaMeters = float.NaN,
                EmergencyAvoid = rival.Hazard,
                FocusRivalId = rival.Id,
            };
            held = true;
        }
        return held;
    }

    private bool ShouldHold(RaceCarSnapshot me, Rival rival, float ego)
    {
        float halfLen = 0.5f * (me.LengthMeters + rival.Length);
        float front = 0.5f * me.LengthMeters + 0.5f * rival.WheelBase;
        bool lateral = MathF.Abs(me.TrackD - rival.TrackD) >= 0.5f * (me.WidthMeters + rival.Width) - 0.3f;
        bool inWindow = rival.Gap >= -(halfLen + 2.6f) && rival.Gap <= front;
        if (lateral && inWindow)
            return true;
        if (_commitId != rival.Id || _commitSign == 0 || PassCleared(me, rival, ego))
            return false;
        return RearHold(me, rival, ego);
    }

    private static bool RearHold(RaceCarSnapshot me, Rival rival, float ego)
    {
        float halfLen = 0.5f * (me.LengthMeters + rival.Length);
        bool overhang = rival.Gap >= -(halfLen + 2.6f) && rival.Gap <= 0f;
        bool reoverlap = rival.Gap > -(halfLen + 4.5f) && rival.Gap < -(halfLen + 2.6f) && ego < rival.Speed - 0.3f;
        return overhang || reoverlap;
    }

    private bool TryRival(in DriverContext context, RaceCarSnapshot me, string? id, ReadOnlySpan<float> freeSpeedPlan, out Rival rival)
    {
        rival = default;
        if (id is null)
            return false;
        float length = MathF.Max(me.TrackLengthMeters, _lanes.LengthMeters);
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            RaceCarSnapshot other = context.Frame[i];
            if (other.Id != id)
                continue;
            float gap = MathF.IEEERemainder(other.TrackS - me.TrackS, length);
            rival = MakeRival(me, other, gap, freeSpeedPlan);
            return true;
        }
        return false;
    }

    private static bool PassCleared(RaceCarSnapshot me, Rival rival, float ego)
    {
        float halfLen = 0.5f * (me.LengthMeters + rival.Length);
        return rival.Gap < -(halfLen + 2.6f)
            && (ego >= rival.Speed - 0.3f || rival.Gap < -(halfLen + 4.5f));
    }

    private int OpenSide(Rival rival)
    {
        int st = _lanes.Wrap((int)MathF.Round(_lanes.StationOf(rival.AbsS)));
        (float lo, float hi) = Road(st);
        // 用障碍车自身站点 Road(st) 及滑动 0.8s 后的预计横向位置选空旷侧
        float projD = Math.Clamp(rival.TrackD + rival.LatSpeed * 0.8f, lo, hi);
        float roomRight = hi - projD;
        float roomLeft = projD - lo;
        return roomRight >= roomLeft ? +1 : -1;
    }

    private bool SlowStarterRoom(RaceCarSnapshot me, Rival? ahead, float ego) =>
        ahead is { } slow && ego - slow.Speed > 1.2f && MathF.Max(Room(slow, me, +1), Room(slow, me, -1)) >= MinPassRoomMeters;

    private Rival MakeRival(RaceCarSnapshot me, RaceCarSnapshot other, float gap, ReadOnlySpan<float> freeSpeedPlan)
    {
        int st = _lanes.Wrap((int)MathF.Round(_lanes.StationOf(other.TrackS)));
        Vector2 vel = new(MathF.Cos(other.VelocityHeadingRadians), MathF.Sin(other.VelocityHeadingRadians));
        float lat = Vector2.Dot(vel, _lanes.Normal(st)) * MathF.Max(0f, other.SpeedMetersPerSecond);
        bool skewed = MathF.Abs(MathHelper.NormalizeAngle(other.HeadingRadians - me.HeadingRadians)) > 0.35f;
        bool slow = other.SpeedMetersPerSecond < 0.48f * PlanSpeed(freeSpeedPlan, st)
                    && other.SpeedMetersPerSecond < 28f;
        return new Rival
        {
            Id = other.Id,
            Gap = gap,
            AbsS = other.TrackS,
            TrackD = other.TrackD,
            Speed = other.SpeedMetersPerSecond,
            Width = other.WidthMeters,
            Length = other.LengthMeters,
            WheelBase = MathF.Max(3f, other.WheelBaseMeters),
            RaceDistance = other.RaceDistanceMeters,
            Hazard = skewed || slow,
            Skewed = skewed,
            LatSpeed = lat,
        };
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
            best = MakeRival(me, other, gap, ReadOnlySpan<float>.Empty);
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
