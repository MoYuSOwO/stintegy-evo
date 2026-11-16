using System.Numerics;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// Clean three-part multi-car racecraft &amp; corridor layer:
/// <list type="number">
/// <item><b>Standing-start launch hold</b>: holds the car's starting grid column
/// straight off the line before blending smoothly onto the racing line.</item>
/// <item><b>Latched one-way tactical commitment</b>: chooses a passing side once
/// on a straight and holds it through the corner; only downgrades to
/// <see cref="Slot.Behind"/> if the door shuts before the FIA rear-axle overlap
/// line, with a cooldown that prevents weaving.</item>
/// <item><b>Near-field frozen corridor + spatial <c>_windowLimits</c></b>: never
/// bends the line inside the braking/steering freeze horizon, and feeds cars ahead
/// directly into <see cref="LaneSpeedPlanner"/>'s spatial speed limits so braking
/// always respects the tyre friction circle without sudden lockups.</item>
/// </list>
/// </summary>
internal sealed class TrafficCorridor
{
    public enum Slot : byte
    {
        Behind = 0,
        /// <summary>Ego runs on the +d (left) side of this rival.</summary>
        Left = 1,
        /// <summary>Ego runs on the -d (right) side of this rival.</summary>
        Right = 2,
    }

    private sealed class RivalState
    {
        public Slot Slot;
        public bool Alongside;
        public float LastSeenTime;
    }

    private readonly struct RivalForecast
    {
        public required string Id { get; init; }
        public required float GapS { get; init; }
        public required float TrackD { get; init; }
        public required float Speed { get; init; }
        public required float LateralSpeed { get; init; }
        public required float Accel { get; init; }
        public required float Length { get; init; }
        public required float Width { get; init; }
        public required float WheelBase { get; init; }
        public required bool Hazard { get; init; }
        /// <summary>FIA apex right: front wing has reached the other's rear axle and the cars are laterally offset.</summary>
        public required bool AlongsideNow { get; init; }
        /// <summary>
        /// Bodies still share the bumper zone (rear diffuser through a closing nose),
        /// even after the FIA rear-axle line has been crossed.
        /// </summary>
        public required bool PhysicallyBeside { get; init; }
        public required float[] S { get; init; }
        public required float[] D { get; init; }
        public required float[] V { get; init; }
    }

    private readonly record struct SpeedLimitPoint(int OffsetStation, int SpanStations, float MaxSpeed);

    private const int BehindStations = 6;
    private const int HorizonSteps = 36;
    private const float HorizonDt = 0.05f;
    private const float LookAheadMeters = 140f;
    private const float LookBehindMeters = 36f;

    private const float SafeEdgeMarginMeters = 1.48f;
    private const float SideMarginMeters = 0.90f;
    private const float StackSlotMarginMeters = 0.82f;

    // Near-field freeze and gentle lateral acceleration guarantee no un-brakable kinks!
    private const float FreezeTimeSeconds = 0.18f;
    private const float MinFreezeMeters = 6f;
    private const float MaxFreezeMeters = 12f;
    private const float StraightMoveAccel = 4.6f;
    private const float LoadedMoveAccel = 2.8f;
    private const float EmergencyMoveAccel = 7.2f;
    private const int GateStations = 120;
    private static readonly float[] MoveScales = [1.0f, 0.6f, 0.35f, 0.15f, 0.0f];

    private readonly LaneSet _lanes;
    private readonly int _aheadStations;
    private readonly int _totalStations;
    private readonly Dictionary<string, RivalState> _states = new(StringComparer.Ordinal);
    private readonly List<RivalForecast> _forecasts = new(16);
    private readonly List<SpeedLimitPoint> _limitPoints = new(8);

    private readonly float[] _egoS;
    private readonly float[] _egoT;
    private readonly float[] _lowD;
    private readonly float[] _highD;
    private readonly float[] _prefD;
    private readonly float[] _targetDelta;
    private readonly float[] _delta;
    private readonly float[] _gateStill = new float[GateStations];
    private readonly float[] _gateTarget = new float[GateStations];
    private readonly float[] _gateCandidate = new float[GateStations];
    private readonly float[] _ceilCurvature = new float[GateStations];
    private readonly float[] _ceilLead = new float[GateStations];
    private readonly float[] _ceilStep = new float[GateStations];
    private readonly float[] _ceilFactor = new float[GateStations];
    private readonly float[] _ceilBrake = new float[GateStations];
    private readonly LaneRoad[] _ceilRoad = new LaneRoad[GateStations];
    private readonly float[] _ceilGuess = new float[GateStations];
    private readonly float[] _ceilLimits = new float[GateStations];
    private readonly float[] _ceilSpeed = new float[GateStations];

    private bool _active;
    private bool _launchCaptured;
    private bool _isStandingStart;
    private float _launchStartS;
    private float _launchColumnD;

    private string? _committedPassId;
    private float _postPassHoldDelta;
    private float _postPassHoldUntil = -10f;

    private int _limitsAnchorStation;

    public TrafficCorridor(LaneSet lanes, float windowMeters = 510f)
    {
        _lanes = lanes;
        _aheadStations = (int)MathF.Ceiling(windowMeters / lanes.Spacing);
        _totalStations = BehindStations + _aheadStations + 1;
        _egoS = new float[_aheadStations + 1];
        _egoT = new float[_aheadStations + 1];
        _lowD = new float[_totalStations];
        _highD = new float[_totalStations];
        _prefD = new float[_totalStations];
        _targetDelta = new float[_totalStations];
        _delta = new float[_totalStations];
    }

    public bool Active => _active;
    public bool HasAlongsideRival { get; private set; }
    public bool HasWindowLimits => _limitPoints.Count > 0;
    public int LimitsVersion { get; private set; }

    public void ApplyWindowLimits(int windowStartStation, Span<float> windowLimits)
    {
        if (_limitPoints.Count == 0)
            return;
        int shift = _lanes.Wrap(windowStartStation - _limitsAnchorStation);
        if (shift > _lanes.Count / 2)
            shift -= _lanes.Count;

        foreach (SpeedLimitPoint pt in _limitPoints)
        {
            int from = pt.OffsetStation - shift;
            int to = from + pt.SpanStations;
            int clampedFrom = Math.Max(0, from);
            int clampedTo = Math.Min(windowLimits.Length - 1, to);
            for (int k = clampedFrom; k <= clampedTo; k++)
                windowLimits[k] = MathF.Min(windowLimits[k], pt.MaxSpeed);
        }
    }

    public void Update(
        in DriverContext context,
        ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope,
        LaneShares shares,
        Func<int, float> cornerFactorAt,
        Func<int, float> brakeFactorAt,
        Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds,
        DrawnLine drawn,
        Func<int, float> currentReferenceOffset,
        in RacecraftIntent intent)
    {
        _limitPoints.Clear();
        HasAlongsideRival = false;

        RaceCarSnapshot me = context.Car;
        float now = context.RaceTimeSeconds;
        float length = _lanes.LengthMeters;
        int anchor = (int)MathF.Floor(_lanes.StationOf(me.TrackS));
        _limitsAnchorStation = anchor;
        float egoSpeed = MathF.Max(0f, me.SpeedMetersPerSecond);

        if (!_launchCaptured)
        {
            _launchCaptured = true;
            _isStandingStart = egoSpeed < 5f && now < 0.5f;
            _launchStartS = me.TotalDistanceMeters;
            _launchColumnD = me.TrackD;
        }
        bool inLaunchHold = _isStandingStart && (me.TotalDistanceMeters - _launchStartS) < 150f && now < 5.0f;

        if (context.Frame.Count <= 1 && !inLaunchHold)
        {
            if (_active)
            {
                _active = false;
                drawn.Clear();
            }
            LimitsVersion++;
            return;
        }

        float egoAccel = me.Motion?.StepLongitudinalAccel ?? 0f;
        float roadCurvNow = MathF.Abs(LaneSet.Curvature(
            _lanes.Position(anchor - 3, _lanes.Offset(LaneId.Free, anchor - 3)),
            _lanes.Position(anchor, _lanes.Offset(LaneId.Free, anchor)),
            _lanes.Position(anchor + 3, _lanes.Offset(LaneId.Free, anchor + 3))));
        bool inBrakingZone = egoAccel < -3.0f ||
                             PlanSpeedAt(freeSpeedPlan, anchor + 10) < PlanSpeedAt(freeSpeedPlan, anchor) - 2.5f;
        bool inLoadedCorner = inBrakingZone || roadCurvNow * egoSpeed * egoSpeed > 13f;

        DriverAbilities abilities = context.Profile.Abilities;
        float awareness = Math.Clamp(abilities.Awareness * 0.01f, 0f, 1f);
        float boldness = Math.Clamp(abilities.Boldness * 0.01f, 0f, 1f);
        float reactionsScore = Math.Clamp(abilities.Reactions * 0.01f, 0f, 1f);
        bool holdColumn = intent.Phase == RacecraftPhase.StandingStart;

        BuildEgoTimeline(anchor, egoSpeed, freeSpeedPlan, envelope, shares, cornerFactorAt, roadAt);
        CaptureRivals(in context, me, now, egoSpeed, inLaunchHold, length, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, awareness);
        HasAlongsideRival = _forecasts.Any(f => f.AlongsideNow);

        if (_states.Count > _forecasts.Count + 4)
        {
            foreach (string stale in _states.Where(kv => now - kv.Value.LastSeenTime > 3f).Select(kv => kv.Key).ToList())
                _states.Remove(stale);
        }

        if (_forecasts.Count == 0 && !inLaunchHold)
        {
            if (_active)
            {
                if (MathF.Abs(me.TrackD - _lanes.Offset(LaneId.Free, anchor)) < 0.12f)
                {
                    _active = false;
                    drawn.Clear();
                    LimitsVersion++;
                    return;
                }
                BuildOpenCorridor(anchor, holdColumn, inLaunchHold, me.TotalDistanceMeters - _launchStartS);
                ApplyIntentLine(anchor, me, intent);
                AppendYield(intent, egoSpeed);
                SolveSmoothLine(anchor, me, inLaunchHold, inLoadedCorner, intent.EmergencyAvoid, freeSpeedPlan, drawn, currentReferenceOffset,
                    egoSpeed, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds);
            }
            LimitsVersion++;
            return;
        }

        ApplyRacecraftSlots(me, anchor, now, egoSpeed, inLoadedCorner, intent);
        bool emergency = BuildSpatialCorridorAndLimits(
            me, anchor, now, egoSpeed, inLaunchHold, holdColumn, me.TotalDistanceMeters - _launchStartS);
        ApplyIntentLine(anchor, me, intent);
        AppendYield(intent, egoSpeed);
        SolveSmoothLine(anchor, me, inLaunchHold, inLoadedCorner, emergency || intent.EmergencyAvoid, freeSpeedPlan, drawn, currentReferenceOffset,
            egoSpeed, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds);
        foreach (RivalForecast rival in _forecasts)
        {
            bool suppressForTow = intent.FullThrottleTow && intent.FocusRivalId == rival.Id;
            AddFollowerLimit(anchor, me, rival, egoSpeed, inBrakingZone, boldness, reactionsScore, suppressForTow);
        }
        LimitsVersion++;
    }

    private void ApplyIntentLine(int anchor, RaceCarSnapshot me, in RacecraftIntent intent)
    {
        if (intent.Phase == RacecraftPhase.StandingStart)
            return;
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
        else if (float.IsFinite(intent.TargetDeltaMeters) || intent.FreezeLateral)
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
    }

    private void AppendYield(in RacecraftIntent intent, float egoSpeed)
    {
        if (intent.YieldSpeedScale > 0f)
            _limitPoints.Add(new SpeedLimitPoint(0, _aheadStations, intent.YieldSpeedScale * egoSpeed));
    }

    private float PlanSpeedAt(ReadOnlySpan<float> freeSpeedPlan, int station)
    {
        float v = freeSpeedPlan[_lanes.Wrap(station)];
        return v > 1f ? v : 85f;
    }

    private (float RoadLow, float RoadHigh, float Free) SafeRoadAt(int station, bool keepColumn = false)
    {
        int st = _lanes.Wrap(station);
        float free = _lanes.Offset(LaneId.Free, st);
        float safeEdge = MathF.Max(0.2f, _lanes.HalfWidth(st) - SafeEdgeMarginMeters);
        float low = MathF.Min(-safeEdge, free);
        float high = MathF.Max(safeEdge, free);
        if (keepColumn)
        {
            low = MathF.Min(low, _launchColumnD);
            high = MathF.Max(high, _launchColumnD);
        }
        return (low, high, free);
    }

    private void BuildEgoTimeline(
        int anchor,
        float egoSpeed,
        ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope,
        LaneShares shares,
        Func<int, float> cornerFactorAt,
        Func<int, LaneRoad> roadAt)
    {
        float spacing = _lanes.Spacing;
        float t = 0f;
        float v = MathF.Max(egoSpeed, 6f);
        _egoS[0] = 0f;
        _egoT[0] = 0f;
        for (int k = 1; k <= _aheadStations; k++)
        {
            int st = _lanes.Wrap(anchor + k);
            float curvature = LaneSet.Curvature(
                _lanes.Position(st - 3, _lanes.Offset(LaneId.Free, st - 3)),
                _lanes.Position(st, _lanes.Offset(LaneId.Free, st)),
                _lanes.Position(st + 3, _lanes.Offset(LaneId.Free, st + 3)));
            v = MathF.Min(
                LaneSpeedPlanner.Driven(envelope, v, curvature, spacing, cornerFactorAt(st), roadAt(st), shares),
                PlanSpeedAt(freeSpeedPlan, anchor + k));
            t += spacing / MathF.Max(v, 1f);
            _egoS[k] = k * spacing;
            _egoT[k] = t;
        }
    }

    private void CaptureRivals(
        in DriverContext context,
        RaceCarSnapshot me,
        float now,
        float egoSpeed,
        bool inLaunchHold,
        float length,
        ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope,
        LaneShares shares,
        Func<int, float> cornerFactorAt,
        Func<int, float> brakeFactorAt,
        Func<int, LaneRoad> roadAt,
        float awareness)
    {
        _forecasts.Clear();
        float lookBehind = LookBehindMeters * (0.75f + 0.35f * awareness);
        Span<float> rivalCeiling = stackalloc float[61];
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            RaceCarSnapshot other = context.Frame[i];
            float gap = MathF.IEEERemainder(other.TrackS - me.TrackS, length);
            if (gap > LookAheadMeters || gap < -lookBehind)
                continue;

            int otherSt = _lanes.Wrap((int)MathF.Round(_lanes.StationOf(other.TrackS)));
            Vector2 tangent = Vector2.Normalize(_lanes.Centre(otherSt + 1) - _lanes.Centre(otherSt - 1));
            Vector2 normal = _lanes.Normal(otherSt);
            float roadHeading = MathF.Atan2(tangent.Y, tangent.X);
            float bodyDiff = MathHelper.NormalizeAngle(other.HeadingRadians - roadHeading);
            float vAlong = MathF.Max(0f, Vector2.Dot(other.Velocity, tangent));
            float vLat = Vector2.Dot(other.Velocity, normal);

            float refSpeed = MathF.Max(14f, PlanSpeedAt(freeSpeedPlan, otherSt));
            bool hazard = MathF.Abs(bodyDiff) > 0.35f ||
                          (!inLaunchHold &&
                           other.SpeedMetersPerSecond < 0.48f * refSpeed &&
                           other.SpeedMetersPerSecond < 28f);

            if (!_states.TryGetValue(other.Id, out RivalState? state))
            {
                state = new RivalState();
                _states[other.Id] = state;
            }
            state.LastSeenTime = now;

            // FIA overlap: front wing at rear axle AND laterally offset (not directly nose-to-tail).
            float meFrontToOtherRearAxle = 0.5f * me.LengthMeters + 0.5f * MathF.Max(3.0f, other.WheelBaseMeters);
            float otherFrontToMeRearAxle = 0.5f * other.LengthMeters + 0.5f * MathF.Max(3.0f, me.WheelBaseMeters);
            float hysteresis = state.Alongside ? 0.8f : 0f;
            float otherCurv = MathF.Abs(LaneSet.Curvature(
                _lanes.Position(otherSt - 3, 0f),
                _lanes.Position(otherSt, 0f),
                _lanes.Position(otherSt + 3, 0f)));
            float sideClearForOverlap = 0.5f * (me.WidthMeters + other.WidthMeters) + (state.Alongside ? 0.18f : 0.38f) + MathF.Min(0.55f, 15f * otherCurv);
            bool beside = MathF.Abs(me.TrackD - other.TrackD) >= sideClearForOverlap;
            bool alongsideNow = beside && (gap >= 0f
                ? gap <= meFrontToOtherRearAxle + hysteresis
                : -gap <= otherFrontToMeRearAxle + hysteresis);
            state.Alongside = alongsideNow;

            // Bumper overlap outlasts the rear-axle flag: half-lengths leave the diffuser
            // and a closing nose still in the same longitudinal window.
            float bodyHalf = 0.5f * (me.LengthMeters + other.LengthMeters);
            float physCross = 0.5f * (me.WidthMeters + other.WidthMeters) - 0.3f;
            bool physicallyBeside = MathF.Abs(me.TrackD - other.TrackD) >= physCross
                && gap >= -(bodyHalf + 2.4f)
                && gap <= bodyHalf + 2.0f;
            bool noseToDiffuser = physicallyBeside || (MathF.Abs(gap) <= bodyHalf + 2.5f && beside);

            var sArr = new float[HorizonSteps + 1];
            var dArr = new float[HorizonSteps + 1];
            var vArr = new float[HorizonSteps + 1];
            sArr[0] = gap;
            dArr[0] = other.TrackD;
            vArr[0] = vAlong;

            float curRelS = gap;
            float curAbsS = other.TrackS;
            float curD = other.TrackD;
            float curV = vAlong;
            float measuredAccel = Math.Clamp(
                other.Motion?.StepLongitudinalAccel ?? 0f,
                -envelope.Brake(MathF.Max(10f, other.SpeedMetersPerSecond)),
                10f);
            float edge0 = MathF.Max(1f, _lanes.HalfWidth(otherSt) - SafeEdgeMarginMeters);
            float startShare = Math.Clamp(other.TrackD / edge0, -1f, 1f);
            float startFree = _lanes.Offset(LaneId.Free, otherSt);
            float startRelFree = other.TrackD - startFree;

            int look = 0;
            if (!hazard)
            {
                look = Math.Min(60, _aheadStations);
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
            }

            for (int step = 1; step <= HorizonSteps; step++)
            {
                float t = step * HorizonDt;
                int st = _lanes.Wrap((int)MathF.Round(_lanes.StationOf(curAbsS)));
                (float roadLo, float roadHi, float freeAtSt) = SafeRoadAt(st);
                float edge = MathF.Max(1f, 0.5f * (roadHi - roadLo));

                if (hazard)
                {
                    // A cruising hazard stays at its measured speed. Only a real brake
                    // (never a scripted full-stop) is allowed to decay the forecast.
                    float hazardAccel = Math.Clamp(measuredAccel, -6f, 0f);
                    curV = MathF.Max(0f, curV + hazardAccel * HorizonDt);
                    curRelS += curV * HorizonDt;
                    curAbsS = WrapS(curAbsS + curV * HorizonDt, length);
                    curD = Math.Clamp(curD + vLat * MathF.Exp(-1.5f * t) * HorizonDt, roadLo, roadHi);
                }
                else
                {
                    float mFloat = Math.Clamp((curRelS - gap) / _lanes.Spacing, 0f, look);
                    int m0 = Math.Min((int)mFloat, look - 1);
                    float cornerCeiling = rivalCeiling[m0] + (rivalCeiling[m0 + 1] - rivalCeiling[m0]) * (mFloat - m0);
                    float accel = t <= 0.25f ? measuredAccel : 6.0f;
                    curV = Math.Clamp(curV + accel * HorizonDt, 0f, MathF.Max(8f, cornerCeiling));
                    curRelS += curV * HorizonDt;
                    curAbsS = WrapS(curAbsS + curV * HorizonDt, length);

                    if (inLaunchHold)
                    {
                        curD = Math.Clamp(other.TrackD + vLat * MathF.Min(t, 0.25f), roadLo, roadHi);
                    }
                    else
                    {
                        float inertiaD = other.TrackD + vLat * MathF.Min(t, 0.28f);
                        float shareD = startShare * edge;
                        float freeD = freeAtSt + startRelFree * 0.35f;
                        // Nose-to-diffuser still owns the rival's lane. Blending toward the
                        // racing line here predicts them teleporting across our rear bumper.
                        float targetD = noseToDiffuser ? shareD : 0.55f * shareD + 0.45f * freeD;
                        float blend = Math.Clamp((t - 0.18f) / 0.55f, 0f, 1f);
                        curD = Math.Clamp(inertiaD + (targetD - inertiaD) * blend, roadLo, roadHi);
                    }
                }

                sArr[step] = curRelS;
                dArr[step] = curD;
                vArr[step] = curV;
            }

            _forecasts.Add(new RivalForecast
            {
                Id = other.Id,
                GapS = gap,
                TrackD = other.TrackD,
                Speed = vAlong,
                LateralSpeed = vLat,
                Accel = measuredAccel,
                Length = other.LengthMeters,
                Width = other.WidthMeters,
                WheelBase = MathF.Max(3.0f, other.WheelBaseMeters),
                Hazard = hazard,
                AlongsideNow = alongsideNow,
                PhysicallyBeside = physicallyBeside,
                S = sArr,
                D = dArr,
                V = vArr,
            });
        }
    }

    private void ApplyRacecraftSlots(
        RaceCarSnapshot me, int anchor, float now, float egoSpeed, bool inLoadedCorner, in RacecraftIntent intent)
    {
        bool finishedInCorner = false;
        foreach (RivalForecast rival in _forecasts)
        {
            RivalState state = _states[rival.Id];
            if (state.Slot == Slot.Behind)
                continue;
            float halfLen = 0.5f * (me.LengthMeters + rival.Length);
            bool passCleared = rival.GapS < -(halfLen + 2.6f)
                && (egoSpeed >= rival.Speed - 0.3f || rival.GapS < -(halfLen + 4.5f));
            if (!passCleared)
                continue;
            if (inLoadedCorner)
                finishedInCorner = true;
            state.Slot = Slot.Behind;
        }
        if (finishedInCorner)
        {
            _postPassHoldDelta = me.TrackD - _lanes.Offset(LaneId.Free, anchor);
            _postPassHoldUntil = now + 1.4f;
        }

        Slot open = intent.SideSign > 0 ? Slot.Left : Slot.Right;
        foreach (RivalForecast rival in _forecasts)
        {
            RivalState state = _states[rival.Id];
            if (state.Slot == Slot.Behind)
                continue;
            bool renewed = intent.SideSign != 0 && (rival.Id == intent.FocusRivalId
                || (rival.Hazard && rival.GapS > 0f && rival.GapS <= LookAheadMeters));
            if (renewed)
                continue;
            float halfLen = 0.5f * (me.LengthMeters + rival.Length);
            bool overhang = rival.GapS >= -(halfLen + 2.6f) && rival.GapS <= 0f;
            bool reoverlap = rival.GapS > -(halfLen + 4.5f) && rival.GapS < -(halfLen + 2.6f)
                && egoSpeed < rival.Speed - 0.3f;
            if (overhang || reoverlap)
                continue;
            state.Slot = Slot.Behind;
        }

        if (intent.SideSign != 0 && intent.Phase != RacecraftPhase.OneMoveDefend)
        {
            foreach (RivalForecast rival in _forecasts)
            {
                RivalState state = _states[rival.Id];
                if (rival.Id == intent.FocusRivalId)
                    state.Slot = open;
                else if (rival.Hazard && rival.GapS > 0f && rival.GapS <= LookAheadMeters && state.Slot == Slot.Behind)
                    state.Slot = open;
            }
            _committedPassId = intent.FocusRivalId;
        }
        else if (_committedPassId is not null)
        {
            bool still = false;
            foreach (RivalForecast rival in _forecasts)
            {
                if (rival.Id == _committedPassId && _states[rival.Id].Slot != Slot.Behind)
                    still = true;
            }
            if (!still)
                _committedPassId = null;
        }
    }

    private void BuildOpenCorridor(int anchor, bool holdColumn, bool keepColumn, float launchRunMeters)
    {
        for (int idx = 0; idx < _totalStations; idx++)
        {
            int st = _lanes.Wrap(anchor - BehindStations + idx);
            (float roadLo, float roadHi, float free) = SafeRoadAt(st, keepColumn);
            _lowD[idx] = roadLo;
            _highD[idx] = roadHi;
            if (holdColumn)
            {
                float sAhead = launchRunMeters + (idx - BehindStations) * _lanes.Spacing;
                float blendToFree = SmoothStep((sAhead - 75f) / 75f);
                _prefD[idx] = Math.Clamp(_launchColumnD + (free - _launchColumnD) * blendToFree, roadLo, roadHi);
            }
            else
            {
                _prefD[idx] = free;
            }
        }
    }

    private bool BuildSpatialCorridorAndLimits(
        RaceCarSnapshot me,
        int anchor,
        float now,
        float egoSpeed,
        bool inLaunchHold,
        bool holdColumn,
        float launchRunMeters)
    {
        BuildOpenCorridor(anchor, holdColumn, inLaunchHold, launchRunMeters);
        bool emergency = false;
        foreach (RivalForecast rival in _forecasts)
        {
            if (rival.Hazard && rival.GapS is > 0f and < LookAheadMeters)
            {
                emergency = true;
                break;
            }
        }

        if (!inLaunchHold && now < _postPassHoldUntil)
        {
            for (int k = 0; k <= Math.Min(50, _aheadStations); k++)
            {
                int idx = BehindStations + k;
                float free = _lanes.Offset(LaneId.Free, anchor + k);
                float w = 1f - k / 52f;
                _prefD[idx] = Math.Clamp(free + _postPassHoldDelta * w, _lowD[idx], _highD[idx]);
            }
        }

        var overlapping = new List<(float D, bool IsMe, float GapS, float Speed)>(8);
        int pinchStation = -1;
        float pinchLeaderSpeed = float.PositiveInfinity;

        for (int k = 0; k <= _aheadStations; k++)
        {
            int idx = BehindStations + k;
            int st = _lanes.Wrap(anchor + k);
            (float roadLo, float roadHi, _) = SafeRoadAt(st, inLaunchHold);
            float lowAbs = roadLo;
            float highAbs = roadHi;
            float prefAbs = _prefD[idx];

            float t = MathF.Min(_egoT[k], HorizonSteps * HorizonDt);
            float egoS = _egoS[k];
            float roadCurv = MathF.Abs(LaneSet.Curvature(
                _lanes.Position(st - 3, 0f),
                _lanes.Position(st, 0f),
                _lanes.Position(st + 3, 0f)));
            float bendSweep = MathF.Min(0.60f, 16f * roadCurv);

            overlapping.Clear();
            overlapping.Add((k == 0 ? me.TrackD : prefAbs, true, 0f, egoSpeed));

            foreach (RivalForecast rival in _forecasts)
            {
                RivalState state = _states[rival.Id];
                if (rival.Hazard && rival.GapS is > 0f and < 50f)
                    emergency = true;

                (float rS, float rD, _) = SampleForecast(rival, t);
                float halfLen = 0.5f * (me.LengthMeters + rival.Length);
                float clearSide = 0.5f * (me.WidthMeters + rival.Width) + SideMarginMeters + bendSweep;

                Slot effectiveSlot = state.Slot;
                // Any car already beside us laterally (even if we committed our pass to someone else!) is a physical side wall, never a ghost!
                if (effectiveSlot == Slot.Behind &&
                    (rival.AlongsideNow ||
                     (MathF.Abs(rival.GapS) < halfLen + 4.0f && MathF.Abs(me.TrackD - rival.TrackD) >= 0.5f * (me.WidthMeters + rival.Width) + 0.15f)))
                {
                    effectiveSlot = me.TrackD >= rival.TrackD ? Slot.Left : Slot.Right;
                }

                if (effectiveSlot == Slot.Left || effectiveSlot == Slot.Right)
                {
                    float wallD = effectiveSlot == Slot.Left ? rD + clearSide : rD - clearSide;
                    // Cover the diffuser and a faster rival catching back from ~6 m behind.
                    float overlapReach = halfLen + (rival.Hazard ? 3.5f : 2.2f);
                    if (MathF.Abs(rS - egoS) <= overlapReach)
                    {
                        if (effectiveSlot == Slot.Left)
                            lowAbs = MathF.Max(lowAbs, wallD);
                        else
                            highAbs = MathF.Min(highAbs, wallD);

                        overlapping.Add((rD, false, rival.GapS, rival.V[0]));
                        if (rival.GapS > -(halfLen + 2.6f))
                            prefAbs = wallD;
                    }
                    else if (rival.GapS > 0f && egoS < rS - overlapReach)
                    {
                        // S-curve lead-in: the wall leaves me.TrackD at rampMeters and arrives at wallD
                        // as the overlap begins, so the bounded lateral move is finished before the bumper.
                        float latNeed = effectiveSlot == Slot.Left
                            ? MathF.Max(0f, wallD - me.TrackD)
                            : MathF.Max(0f, me.TrackD - wallD);
                        float moveAccel = rival.Hazard ? EmergencyMoveAccel : StraightMoveAccel;
                        float rampMeters = MathF.Max(18f, MathF.Max(egoSpeed, 12f) * (MathF.Sqrt(4f * latNeed / moveAccel) + 0.25f));
                        float distToOverlap = (rS - overlapReach) - egoS;
                        if (distToOverlap > 0f && distToOverlap < rampMeters)
                        {
                            float rampD = me.TrackD + (wallD - me.TrackD) * SmoothStep(1f - distToOverlap / rampMeters);
                            if (effectiveSlot == Slot.Left)
                                lowAbs = MathF.Max(lowAbs, rampD);
                            else
                                highAbs = MathF.Min(highAbs, rampD);
                        }

                        if (distToOverlap < rampMeters + 36f || rival.Hazard)
                            prefAbs = wallD;
                    }
                }
            }

            // Transitive multi-car stack width (3-wide / 4-wide).
            if (overlapping.Count >= 3)
            {
                overlapping.Sort((a, b) => a.D.CompareTo(b.D));
                int myOrder = overlapping.FindIndex(o => o.IsMe);
                if (myOrder >= 0)
                {
                    int rightCount = myOrder;
                    int leftCount = overlapping.Count - 1 - myOrder;
                    float slotW = me.WidthMeters + StackSlotMarginMeters + 0.8f * bendSweep;
                    lowAbs = MathF.Max(lowAbs, roadLo + rightCount * slotW);
                    highAbs = MathF.Min(highAbs, roadHi - leftCount * slotW);
                    if (rightCount > 0 && leftCount > 0)
                        prefAbs = 0.5f * (lowAbs + highAbs);
                }
            }

            // Pinch bailout fed into spatial _windowLimits instead of sudden braking!
            if (lowAbs > highAbs + 0.05f && k is >= 4 and <= 60 && overlapping.Count >= 2 && pinchStation < 0)
            {
                int myOrder = overlapping.FindIndex(o => o.IsMe);
                bool someoneAhead = overlapping.Any(o => !o.IsMe && o.GapS > 0.4f);
                bool middleThreeWide = overlapping.Count >= 3 && myOrder > 0 && myOrder < overlapping.Count - 1;
                if (someoneAhead || middleThreeWide)
                {
                    pinchStation = k;
                    pinchLeaderSpeed = overlapping.Where(o => !o.IsMe).Min(o => o.Speed);
                }
            }

            if (lowAbs > highAbs)
                lowAbs = highAbs = Math.Clamp(0.5f * (lowAbs + highAbs), roadLo, roadHi);

            _lowD[idx] = lowAbs;
            _highD[idx] = highAbs;
            _prefD[idx] = Math.Clamp(prefAbs, lowAbs, highAbs);
        }

        // Spatial longitudinal limits injected directly into LaneSpeedPlanner._windowLimits!
        if (pinchStation >= 0 && float.IsFinite(pinchLeaderSpeed))
            _limitPoints.Add(new SpeedLimitPoint(Math.Max(2, pinchStation - 4), 10, MathF.Max(12f, pinchLeaderSpeed)));

        return emergency;
    }

    private void SolveSmoothLine(
        int anchor,
        RaceCarSnapshot me,
        bool inLaunchHold,
        bool inLoadedCorner,
        bool emergency,
        ReadOnlySpan<float> freeSpeedPlan,
        DrawnLine drawn,
        Func<int, float> currentReferenceOffset,
        float egoSpeed,
        in LaneEnvelope envelope,
        LaneShares shares,
        Func<int, float> cornerFactorAt,
        Func<int, float> brakeFactorAt,
        Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds)
    {
        float spacing = _lanes.Spacing;
        for (int idx = 0; idx < _totalStations; idx++)
        {
            int st = _lanes.Wrap(anchor - BehindStations + idx);
            _targetDelta[idx] = _prefD[idx] - _lanes.Offset(LaneId.Free, st);
        }

        // Smooth the target relative profile inside corridor bounds so corridor walls have zero sharp corners.
        for (int pass = 0; pass < 14; pass++)
        {
            for (int idx = 1; idx < _totalStations - 1; idx++)
            {
                int st = _lanes.Wrap(anchor - BehindStations + idx);
                float free = _lanes.Offset(LaneId.Free, st);
                float avg = 0.5f * (_targetDelta[idx - 1] + _targetDelta[idx + 1]);
                float next = _targetDelta[idx] + (avg - _targetDelta[idx]) * 0.45f;
                _targetDelta[idx] = Math.Clamp(next, _lowD[idx] - free, _highD[idx] - free);
            }
        }

        // Anchor C¹-continuously to the active reference line at the car (Werling continuity),
        // then integrate forward with bounded lateral acceleration |d²(delta)/ds²| <= maxLatAccel / v².
        // Because position and heading match at anchor and curvature change is physically bounded (<0.5g),
        // it can NEVER create an un-brakable near-field kink AND suffers zero rolling-freeze Zeno lag!
        float freeHere = _lanes.Offset(LaneId.Free, anchor);
        float freeSlope = (_lanes.Offset(LaneId.Free, anchor + 1) - _lanes.Offset(LaneId.Free, anchor - 1)) / (2f * spacing);
        float refDeltaHere = currentReferenceOffset(anchor) - freeHere;
        float refDeltaSlope = ((currentReferenceOffset(anchor + 1) - _lanes.Offset(LaneId.Free, anchor + 1)) -
                               (currentReferenceOffset(anchor - 1) - _lanes.Offset(LaneId.Free, anchor - 1))) / (2f * spacing);

        Vector2 roadTan = Vector2.Normalize(_lanes.Centre(anchor + 1) - _lanes.Centre(anchor - 1));
        Vector2 velDir = new(MathF.Cos(me.VelocityHeadingRadians), MathF.Sin(me.VelocityHeadingRadians));
        float carAbsSlope = me.SpeedMetersPerSecond > 3f
            ? Vector2.Dot(velDir, _lanes.Normal(anchor)) / MathF.Max(0.25f, Vector2.Dot(velDir, roadTan))
            : 0f;
        float carDeltaHere = me.TrackD - freeHere;
        float carDeltaSlope = inLaunchHold ? -freeSlope : Math.Clamp(carAbsSlope - freeSlope, -0.22f, 0.22f);

        bool startOnRef = drawn.Active &&
                          MathF.Abs(carDeltaHere - refDeltaHere) < 0.90f &&
                          MathF.Abs(carDeltaSlope - refDeltaSlope) < 0.14f;
        float startY = startOnRef ? refDeltaHere : carDeltaHere;
        float startSlope = Math.Clamp(startOnRef ? refDeltaSlope : carDeltaSlope, -0.22f, 0.22f);

        for (int idx = 0; idx <= BehindStations; idx++)
        {
            int st = _lanes.Wrap(anchor - BehindStations + idx);
            (float rLo, float rHi, float f) = SafeRoadAt(st, inLaunchHold);
            _delta[idx] = Math.Clamp(startY + (idx - BehindStations) * spacing * startSlope, rLo - f, rHi - f);
        }

        bool activelyPassingOrAlongside = HasAlongsideRival || _committedPassId is not null;
        float maxLatAccel = emergency
            ? EmergencyMoveAccel
            : activelyPassingOrAlongside
                ? (inLoadedCorner ? 3.5f : 5.0f)
                : (inLoadedCorner ? LoadedMoveAccel : StraightMoveAccel);

        float y = startY;
        float slope = startSlope;
        for (int k = 1; k <= _aheadStations; k++)
        {
            int idx = BehindStations + k;
            int st = _lanes.Wrap(anchor + k);
            float v = MathF.Max(12f, PlanSpeedAt(freeSpeedPlan, st));
            float maxBend = maxLatAccel / (v * v);
            float preview = MathF.Max(14f, v * 0.30f);
            int lookIdx = Math.Min(_totalStations - 1, idx + (int)MathF.Ceiling(preview / spacing));
            int nearIdx = Math.Min(_totalStations - 1, idx + 4);

            float want = _targetDelta[lookIdx];
            want = Math.Clamp(want, _lowD[nearIdx] - _lanes.Offset(LaneId.Free, anchor + k + Math.Min(4, _aheadStations - k)),
                                    _highD[nearIdx] - _lanes.Offset(LaneId.Free, anchor + k + Math.Min(4, _aheadStations - k)));
            float err = want - y;
            float maxSlope = MathF.Sqrt(2f * maxBend * MathF.Abs(err));
            float wantSlope = Math.Clamp(err / preview, -maxSlope, maxSlope);
            slope += Math.Clamp(wantSlope - slope, -maxBend * spacing, maxBend * spacing);
            y += slope * spacing;

            (float rLo, float rHi, float f) = SafeRoadAt(st, inLaunchHold);
            y = Math.Clamp(y, rLo - f, rHi - f);
            _delta[idx] = y;
        }

        // Smooth out any k>=6 wall contact so curvature stays continuous everywhere.
        for (int pass = 0; pass < 8; pass++)
        {
            for (int idx = BehindStations + 2; idx < _totalStations - 1; idx++)
            {
                int st = _lanes.Wrap(anchor - BehindStations + idx);
                (float rLo, float rHi, float f) = SafeRoadAt(st, inLaunchHold);
                float avg = 0.5f * (_delta[idx - 1] + _delta[idx + 1]);
                float smoothed = _delta[idx] + (avg - _delta[idx]) * 0.42f;
                _delta[idx] = Math.Clamp(smoothed, rLo - f, rHi - f);
            }
        }

        GateAndCommit(anchor, me, egoSpeed, emergency, freeSpeedPlan, envelope, shares,
            cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds, currentReferenceOffset, drawn, inLaunchHold);
    }

    private void AddFollowerLimit(
        int anchor, RaceCarSnapshot me, RivalForecast rival,
        float egoSpeed, bool inBrakingZone, float boldness, float reactionsScore,
        bool suppressForTow)
    {
        float halfLen = 0.5f * (me.LengthMeters + rival.Length);
        if (rival.GapS <= -halfLen || rival.AlongsideNow)
            return;
        if (rival.Hazard && _states[rival.Id].Slot != Slot.Behind && rival.GapS > 20f)
            return;

        float dTail = 3.8f - 1.3f * boldness;
        float netGap = rival.GapS - halfLen - dTail;
        float clearCross = 0.5f * (me.WidthMeters + rival.Width) + 0.35f;
        bool laterallyClearNow = MathF.Abs(me.TrackD - rival.TrackD) >= clearCross;
        if (CommittedLineClears(anchor, rival, halfLen, clearCross) &&
            (rival.GapS >= halfLen + 2f || laterallyClearNow))
            return;

        if (suppressForTow && netGap > 6f)
            return;

        if (!inBrakingZone)
        {
            float dv = egoSpeed - rival.Speed;
            if (dv <= 0f)
                return;
            if (netGap <= 0f)
            {
                _limitPoints.Add(new SpeedLimitPoint(0, 8, rival.Speed));
                return;
            }

            // PlanAhead reaches effRivalV at kCatch. While ego slows from egoSpeed to that
            // speed over D, the rival covers rivalV * 2D / (egoSpeed + rivalV), so
            // D = netGap * (egoSpeed + effRivalV) / effDv leaves exactly halfLen + dTail.
            float effRivalV = MathF.Min(rival.Speed, rival.V[HorizonSteps]);
            float effDv = egoSpeed - effRivalV;
            float catchMeters = netGap * (egoSpeed + effRivalV) / effDv;
            int kCatch = Math.Clamp((int)MathF.Floor(catchMeters / _lanes.Spacing), 0, _aheadStations - 1);
            _limitPoints.Add(new SpeedLimitPoint(kCatch, 6, effRivalV));
            return;
        }

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

    private bool CommittedLineClears(int anchor, RivalForecast rival, float halfLen, float clearCross)
    {
        float reach = halfLen + 1.2f;
        bool sawOverlap = false;
        for (int k = 0; k <= _aheadStations; k++)
        {
            float t = MathF.Min(_egoT[k], HorizonSteps * HorizonDt);
            (float rS, float rD, _) = SampleForecast(rival, t);
            if (MathF.Abs(rS - _egoS[k]) >= reach)
                continue;
            sawOverlap = true;
            int st = _lanes.Wrap(anchor + k);
            float egoD = _lanes.Offset(LaneId.Free, st) + _delta[BehindStations + k];
            if (MathF.Abs(egoD - rD) < clearCross)
                return false;
        }

        if (sawOverlap)
            return true;

        // Outside the 1.8 s horizon a chosen side is not a collision. A follow slot
        // yields only when the planned line still sits in the rival's lane.
        if (_states[rival.Id].Slot != Slot.Behind)
            return true;

        int relSt = Math.Clamp((int)MathF.Round(rival.GapS / _lanes.Spacing), 0, _aheadStations);
        int rivalSt = _lanes.Wrap(anchor + relSt);
        float plannedD = _lanes.Offset(LaneId.Free, rivalSt) + _delta[BehindStations + relSt];
        return MathF.Abs(plannedD - rival.TrackD) >= clearCross;
    }

    internal static bool PassesDrivableGate(float candidateCeiling0, float egoSpeed, float stillCeiling0) =>
        candidateCeiling0 >= MathF.Min(egoSpeed, stillCeiling0) - 0.5f;

    private void GateAndCommit(
        int anchor, RaceCarSnapshot me, float egoSpeed, bool emergency,
        ReadOnlySpan<float> freeSpeedPlan, in LaneEnvelope envelope, LaneShares shares,
        Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds, Func<int, float> currentReferenceOffset,
        DrawnLine drawn, bool inLaunchHold)
    {
        if (inLaunchHold)
        {
            _active = true;
            drawn.Set(_lanes.Wrap(anchor - BehindStations), _delta, 1f);
            return;
        }
        int n = Math.Min(_aheadStations + 1, GateStations);
        float carDeltaHere = me.TrackD - _lanes.Offset(LaneId.Free, anchor);
        bool holdCarLine = !drawn.Active && MathF.Abs(carDeltaHere) > 0.5f;
        for (int k = 0; k < n; k++)
        {
            int st = _lanes.Wrap(anchor + k);
            float free = _lanes.Offset(LaneId.Free, st);
            float edge = _lanes.HalfWidth(st) - SafeEdgeMarginMeters;
            _gateStill[k] = holdCarLine
                ? Math.Clamp(me.TrackD, -edge, edge) - free
                : currentReferenceOffset(st) - free;
            _gateTarget[k] = _delta[BehindStations + k];
        }
        float stillCeiling = Ceiling0(anchor, _gateStill.AsSpan(0, n), freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds);
        float chosen = -1f;
        float escape = -1f;
        foreach (float scale in MoveScales)
        {
            for (int k = 0; k < n; k++)
                _gateCandidate[k] = _gateStill[k] + scale * (_gateTarget[k] - _gateStill[k]);
            ReadOnlySpan<float> candidate = _gateCandidate.AsSpan(0, n);
            float ceiling = Ceiling0(anchor, candidate, freeSpeedPlan, envelope, shares, cornerFactorAt, brakeFactorAt, roadAt, turnLagSeconds);
            bool overlaps = emergency && LineOverlaps(anchor, me, candidate, n);
            bool pass = (scale == 0f && !overlaps) || PassesDrivableGate(ceiling, egoSpeed, stillCeiling);
            if (emergency && !overlaps)
                escape = scale;
            if (pass && !overlaps)
            {
                chosen = scale;
                break;
            }
        }
        if (chosen < 0f)
            chosen = escape >= 0f ? escape : (emergency ? 1.0f : 0f);
        for (int k = 0; k < n; k++)
            _delta[BehindStations + k] = _gateStill[k] + chosen * (_gateTarget[k] - _gateStill[k]);
        if (n < _aheadStations + 1 && chosen < 1f)
        {
            float tailShift = (_gateStill[n - 1] + chosen * (_gateTarget[n - 1] - _gateStill[n - 1])) - _gateTarget[n - 1];
            for (int k = n; k <= _aheadStations; k++)
            {
                float fade = Math.Clamp(1f - (k - (n - 1)) / 25f, 0f, 1f);
                _delta[BehindStations + k] += tailShift * fade;
            }
        }
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
        int anchor, ReadOnlySpan<float> delta, ReadOnlySpan<float> freeSpeedPlan,
        in LaneEnvelope envelope, LaneShares shares,
        Func<int, float> cornerFactorAt, Func<int, float> brakeFactorAt, Func<int, LaneRoad> roadAt,
        Func<float, float> turnLagSeconds)
    {
        int n = delta.Length;
        for (int k = 0; k < n; k++)
        {
            int st = anchor + k;
            _ceilCurvature[k] = LaneSet.Curvature(
                OffsetPoint(anchor, k - 3, delta, n),
                OffsetPoint(anchor, k, delta, n),
                OffsetPoint(anchor, k + 3, delta, n));
            _ceilStep[k] = (OffsetPoint(anchor, k + 1, delta, n) - OffsetPoint(anchor, k, delta, n)).Length();
            int wrapped = _lanes.Wrap(st);
            _ceilFactor[k] = cornerFactorAt(wrapped);
            _ceilBrake[k] = brakeFactorAt(wrapped);
            _ceilRoad[k] = roadAt(wrapped);
            _ceilGuess[k] = MathF.Max(8f, PlanSpeedAt(freeSpeedPlan, st));
        }
        LaneSpeedPlanner.LeadCurvatures(_ceilCurvature.AsSpan(0, n), _ceilGuess.AsSpan(0, n), turnLagSeconds, _lanes.Spacing, _ceilLead.AsSpan(0, n));
        float terminal = PlanSpeedAt(freeSpeedPlan, anchor + n - 1);
        LaneSpeedPlanner.CornerLimits(envelope, _ceilLead.AsSpan(0, n), _ceilFactor.AsSpan(0, n), _ceilRoad.AsSpan(0, n), shares, _ceilLimits.AsSpan(0, n));
        LaneSpeedPlanner.PlanWindow(
            envelope, _ceilLead.AsSpan(0, n), _ceilStep.AsSpan(0, n), _ceilFactor.AsSpan(0, n), _ceilBrake.AsSpan(0, n),
            _ceilRoad.AsSpan(0, n), shares, terminal, _ceilLimits.AsSpan(0, n), _ceilSpeed.AsSpan(0, n));
        return _ceilSpeed[0];
    }

    private Vector2 OffsetPoint(int anchor, int rel, ReadOnlySpan<float> delta, int n)
    {
        float d = rel < 0 ? delta[0] : rel >= n ? delta[n - 1] : delta[rel];
        return _lanes.Position(anchor + rel, _lanes.Offset(LaneId.Free, anchor + rel) + d);
    }

    private bool LineOverlaps(int anchor, RaceCarSnapshot me, ReadOnlySpan<float> delta, int n)
    {
        for (int k = 0; k < n; k++)
        {
            int kk = Math.Min(k, _egoT.Length - 1);
            float t = MathF.Min(_egoT[kk], HorizonSteps * HorizonDt);
            float egoS = _egoS[kk];
            float egoD = _lanes.Offset(LaneId.Free, anchor + k) + delta[k];
            foreach (RivalForecast rival in _forecasts)
            {
                (float rS, float rD, _) = SampleForecast(rival, t);
                float halfLen = 0.5f * (me.LengthMeters + rival.Length);
                if (MathF.Abs(rS - egoS) >= halfLen + 1.2f)
                    continue;
                float clear = 0.5f * (me.WidthMeters + rival.Width) + 0.35f;
                if (MathF.Abs(egoD - rD) < clear)
                    return true;
            }
        }
        return false;
    }

    private static float SmoothStep(float x)
    {
        x = Math.Clamp(x, 0f, 1f);
        return x * x * (3f - 2f * x);
    }

    private static (float S, float D, float V) SampleForecast(RivalForecast rival, float time)
    {
        float f = Math.Clamp(time / HorizonDt, 0f, HorizonSteps);
        int i0 = Math.Min((int)f, HorizonSteps - 1);
        float u = f - i0;
        return (
            rival.S[i0] + (rival.S[i0 + 1] - rival.S[i0]) * u,
            rival.D[i0] + (rival.D[i0 + 1] - rival.D[i0]) * u,
            rival.V[i0] + (rival.V[i0 + 1] - rival.V[i0]) * u);
    }

    private static float WrapS(float s, float length)
    {
        float r = s % length;
        return r < 0f ? r + length : r;
    }
}
