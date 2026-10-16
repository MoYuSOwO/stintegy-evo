using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>Follow gains. Steering is in wheel-angle terms, then turned back into curvature.</summary>
public sealed record LaneDriverGains
{
    /// <summary>
    /// How far ahead the curvature feedforward is read, in seconds of travel.
    /// 0.15 since the F1E car: lighter and driven at both ends, it answers
    /// the wheel sooner than the car 0.20 was set on, and at 0.20 it turned
    /// into Silverstone's fast left-right early enough to sit a metre wide
    /// on the way out and put a wheel over. Swept on the relearned file:
    /// 0.15 has the smallest worst error (0.70 m against 1.07); shorter
    /// lowers the mean but lets the slow hairpin run wide.
    /// </summary>
    public float CurvaturePreviewSeconds { get; init; } = 0.15f;

    /// <summary>
    /// Feedback is pure pursuit in the path's frame: aim at a point this many
    /// seconds of travel ahead, so the correction's lateral acceleration does
    /// not grow with the square of the speed.
    /// </summary>
    public float PursuitSeconds { get; init; } = 0.6f;
    public float MinimumPursuitMeters { get; init; } = 10f;

    /// <summary>
    /// The command may spend this share of the lateral limit the axles have at
    /// the braking actually being done. Past it the rear, unloaded by the
    /// brakes, asks for so much slip angle that the wheel is wound off and
    /// the car goes straight on.
    /// </summary>
    public float CommandLateralShare { get; init; } = 1.0f;

    /// <summary>Cap the command from the axles' live grip rather than the envelope.</summary>
    public bool CapFromAxles { get; init; } = true;

    /// <summary>
    /// Plan each corner on the tyres that hold it: the outside pair, at their
    /// own temperature and wear, rather than the four-tyre average. On by
    /// default since the F1E relearn, which was learned with it: averaged,
    /// a hot outside front was planned as if the cool inside one shared the
    /// load, and the second hot lap went off where the first had not.
    /// </summary>
    public bool TurnAwareEnvelope { get; init; } = true;

    /// <summary>
    /// The tyre rung as a way of driving, set by the player and carried out
    /// by the driver: how much of the corner, the throttle, the brakes and
    /// the wheel each rung spends. Off, the rung only scales the tyre's
    /// allowance, which moves almost nothing. On by default: the top row of
    /// the table is the plan as it is learned, so every other rung drives
    /// inside what the learning proved.
    /// </summary>
    public bool TyreRungStyles { get; init; } = true;

    /// <summary>Protect, Light, Normal, Push, Attack.</summary>
    public TyreStyle[] TyreStyleTable { get; init; } =
    [
        new(Corner: 0.99250f, Brake: 0.99475f, Traction: 0.882f, Cap: 0.99825f),
        new(Corner: 0.99175f, Brake: 0.99450f, Traction: 0.881f, Cap: 0.99800f),
        new(Corner: 0.99150f, Brake: 0.99400f, Traction: 0.880f, Cap: 0.99750f),
        new(Corner: 0.99350f, Brake: 0.99550f, Traction: 0.885f, Cap: 0.99850f),
        new(Corner: 1.00000f, Brake: 1.00000f, Traction: 0.900f, Cap: 1.00000f),
    ];

    /// <summary>Share of the axles' live grip the command may ask for: what a tyre holds, not its peak.</summary>
    public float AxleCapShare { get; init; } = 0.90f;

    public float SpeedPreviewSeconds { get; init; } = 0.30f;

    /// <summary>
    /// The same look ahead while the plan is slowing. Kept apart because
    /// reading the table early on the way into a corner is braking early.
    /// </summary>
    public float BrakePreviewSeconds { get; init; } = 0.35f;
    public float SpeedGain { get; init; } = 6f;

    /// <summary>Combined use (lateral and longitudinal together) the throttle may take the driven axle to.</summary>
    public float TractionUse { get; init; } = 0.90f;

    /// <summary>Combined tyre use past which the brakes are eased, and over what band.</summary>
    public float BrakeFeelUse { get; init; } = 0.97f;
    public float BrakeFeelBand { get; init; } = 0.06f;
    public float BrakeFeelFloor { get; init; } = 0.5f;

    /// <summary>
    /// The energy rungs as ways of driving, Save to Attack: seconds of lift
    /// before each braking zone. The player picks the rung and watches the
    /// pack; the driver only lifts as the rung says. Set on Silverstone with
    /// the 320 kW generator so that Normal all race spends the pack, about
    /// 28 of its 30 kWh over 52 laps: Attack spends 0.73 kWh a lap net and
    /// runs it flat with laps to go, Save puts back about 0.34 for just
    /// under a second a lap, and the rungs between spend in steps.
    /// </summary>
    public float[] EnergyRungLiftSeconds { get; init; } = [1.25f, 0.7f, 0.47f, 0.35f, 0f];

    /// <summary>Whether the tyre rung scales how much of the tyre the driver uses.</summary>
    public bool TyreRungAuthority { get; init; } = true;

    /// <summary>Whether the plan and the command read the road's lie. Off only for comparisons.</summary>
    public bool UseRoadAttitude { get; init; } = true;
    public bool UseGrade { get; init; } = true;
    /// <summary>
    /// The bank is read too. It took the command cap and the throttle both
    /// reading the axles' live grip before it held: estimated, the bank's
    /// extra corner pushed the command over the steering cliff on entry and
    /// the throttle over the rear's circle on exit.
    /// </summary>
    public bool UseBank { get; init; } = true;
    public bool UseVertical { get; init; } = true;

    /// <summary>Whether the command's lateral cap reads the bank (the plan always does when UseBank).</summary>
    public bool BankInCommand { get; init; } = true;

    /// <summary>Body sideslip where the driver starts handing the slide back.</summary>
    public float SlideOnsetRadians { get; init; } = 0.10f;
    public float SlideFullRadians { get; init; } = 0.20f;
}

/// <summary>
/// The three-lane driver. The plan is a blend of three baked lanes, chosen
/// by two numbers that move at limited rates: how far from the free lane,
/// and to which side. Speed is planned on whatever blend is in force, from
/// this car's live envelope times the track's learned factors, so tyres,
/// battery and wake move it without anything being re-baked.
/// </summary>
public sealed class LaneDriver : IDriverController
{
    private const int CurvatureChordStations = 3;
    private const float RefreshSeconds = 0.5f;
    private const float WindowMeters = 450f;

    private readonly LaneTrackFile _file;
    private readonly CarConfig _car;
    private readonly TireConfig _tires;
    private readonly CarState _plan = new();
    private readonly float[][] _curvature = new float[3][];
    private readonly LaneRoad[][] _road = new LaneRoad[3][];
    private DriverTrackView? _trackView;
    private readonly float[][] _stepLength = new float[3][];
    private readonly float[][] _speed = new float[3][];
    private LaneSet _lanes = null!;
    private LaneEnvelope _envelope;
    private float _referenceLateral = 1f;
    private LanePerformance? _performance;
    private readonly float[][] _corner = new float[3][];
    private readonly float[][] _brake = new float[3][];
    private float _sinceRefresh = float.PositiveInfinity;

    private float[] _windowSpeed = [];
    private float[] _windowCurvature = [];
    private LaneRoad[] _windowRoad = [];
    private float[] _windowStep = [];
    private float[] _windowFactor = [];
    private float[] _windowBrake = [];
    private int _windowStart = -1;
    private float _windowRange = -1f;
    private float _windowSide;

    public LaneDriver(LaneTrackFile file, CarConfig? car = null, TireConfig? tires = null)
    {
        _file = file ?? throw new ArgumentNullException(nameof(file));
        _car = car ?? new CarConfig();
        _tires = tires ?? new TireConfig();
    }

    public LaneDriverGains Gains { get; init; } = new();

    /// <summary>Whether the driver's ratings shape the drive. Off only for probes of the plan itself.</summary>
    public bool UseAbilities { get; init; } = true;

    public LanePerformance? Performance => _performance;

    /// <summary>How long before each braking zone the driver lifts, seconds: the energy rung's.</summary>
    public float CoastSeconds { get; private set; }

    /// <summary>Holds the lift at this instead of the rung's, for probes that sweep it.</summary>
    public float? CoastOverride { get; set; }

    /// <summary>Most the driver will lift early: past it the car looks like it is cruising.</summary>
    public float MaximumCoastSeconds { get; init; } = 1.5f;

    private void ManageEnergy(RaceCarSnapshot car)
    {
        float[] table = Gains.EnergyRungLiftSeconds;
        float lift = CoastOverride ??
                     (table.Length == 0 ? 0f : table[Math.Clamp(car.Strategy.PowerRung - 1, 0, table.Length - 1)]);
        CoastSeconds = Math.Clamp(lift, 0f, MaximumCoastSeconds);
    }

    /// <summary>
    /// The tyre rung as an authority over the driver: the share of the tyre
    /// the rung allows, against Normal's. Push and Attack let the driver use
    /// more of the circle, Protect and Light less, and the plan, the steering
    /// cap, the throttle and the brakes all follow it.
    /// </summary>
    private float _tyreAuthority = 1f;
    private TyreStyle? _style;
    private LaneShares _planShares = new();
    public LaneShares Shares { get; init; } = new();

    /// <summary>How far from the free lane the car is driving, 0..1.</summary>
    public float Range { get; private set; }

    /// <summary>Which side: +1 the positive-offset lane, -1 the negative-offset lane.</summary>
    public float Side { get; private set; }

    /// <summary>Where the tactics want the blend; the blend moves there at a limited rate.</summary>
    public float TargetRange { get; set; }
    public float TargetSide { get; set; }

    /// <summary>Blend travel per second, in units of the blend.</summary>
    public float RangeRate { get; init; } = 0.6f;
    public float SideRate { get; init; } = 0.8f;

    public LaneDriverTelemetry LastTelemetry { get; private set; }
    public LaneSet Lanes => _lanes;

    public void Initialize(in DriverContext context)
    {
        _lanes = _file.LanesFor(context.Track);
        _trackView = context.Track;
        for (int lane = 0; lane < 3; lane++)
        {
            int count = _lanes.Count;
            _curvature[lane] = new float[count];
            _road[lane] = new LaneRoad[count];
            _stepLength[lane] = new float[count];
            _speed[lane] = new float[count];
            _corner[lane] = new float[count];
            _brake[lane] = new float[count];
            for (int i = 0; i < count; i++)
            {
                _curvature[lane][i] = CurvatureAt((LaneId)lane, i);
                _road[lane][i] = RoadAt(i, _lanes.Offset((LaneId)lane, i));
                _stepLength[lane][i] = (PositionOn((LaneId)lane, i + 1) - PositionOn((LaneId)lane, i)).Length();
            }
        }

        int window = (int)MathF.Ceiling(WindowMeters / _lanes.Spacing);
        _windowSpeed = new float[window];
        _windowCurvature = new float[window];
        _windowRoad = new LaneRoad[window];
        _windowStep = new float[window];
        _windowFactor = new float[window];
        _windowBrake = new float[window];
        Range = TargetRange;
        Side = TargetSide;
        _sinceRefresh = float.PositiveInfinity;
        _referenceLateral = ReferenceLateral();
        _performance = UseAbilities ? new LanePerformance(context.Profile) : null;
        _performance?.Update(0, context.Car.TrackS, 0f);
        Refresh(context.Car, context.Environment.SurfaceGripScalar);
    }

    /// <summary>
    /// What this car corners at 40 m/s on new tyres at their best temperature
    /// on a clean road: the full grip every grip level is a share of.
    /// </summary>
    private float ReferenceLateral()
    {
        var ideal = new CarState();
        float temp = 0.5f * (_tires.IdealSurfaceTempLowC + _tires.IdealSurfaceTempHighC);
        foreach (TireState tyre in new[] { ideal.FrontLeft, ideal.FrontRight, ideal.RearLeft, ideal.RearRight })
        {
            tyre.SurfaceTempC = temp;
            tyre.CoreTempC = temp;
            tyre.Wear = 0f;
            tyre.SurfaceGrip = 1f;
        }
        return MathF.Max(1f, LaneEnvelope.From(ideal, _car, _tires, CarStrategy.Default).Lateral(40f));
    }

    /// <summary>Grip as the driver reads it, as a share of the reference, from the last refresh.</summary>
    public float Grip { get; private set; } = 1f;

    /// <summary>Grip as it really is, as a share of the reference, from the last refresh.</summary>
    public float ActualGrip { get; private set; } = 1f;

    public DriverInput GetControl(in DriverContext context, float dt)
    {
        RaceCarSnapshot car = context.Car;
        _sinceRefresh += dt;
        ManageEnergy(car);
        if (_performance is not null)
        {
            int lap = (int)MathF.Floor(car.RaceDistanceMeters / MathF.Max(car.TrackLengthMeters, 1f));
            _performance.Update(lap, car.TrackS, dt);
        }
        if (_sinceRefresh >= RefreshSeconds)
            Refresh(car, context.Environment.SurfaceGripScalar);
        MoveBlend(dt);

        float speed = car.SpeedMetersPerSecond;
        float station = _lanes.StationOf(car.TrackS);

        // Where the plan is, at the car and a little ahead.
        (Vector2 refPosition, Vector2 tangent) = BlendedPose(station);
        // Consistency: the line this segment, a little off the lane.
        if (_performance is not null)
            refPosition += _lanes.Normal((int)MathF.Round(station)) * _performance.LateralOffsetMeters;
        float pathHeading = MathF.Atan2(tangent.Y, tangent.X);
        Vector2 toCar = car.Position - refPosition;
        float leftError = tangent.X * toCar.Y - tangent.Y * toCar.X;
        float headingError = MathHelper.NormalizeAngle(pathHeading - car.VelocityHeadingRadians);
        float previewStation = station + MathF.Max(0f, speed) * Gains.CurvaturePreviewSeconds / _lanes.Spacing;
        float feedforward = BlendedCurvature(previewStation) + BlendedSteer(previewStation);
        float here = BlendedCurvature(station);

        float pursuit = MathF.Max(Gains.MinimumPursuitMeters, MathF.Max(speed, 0f) * Gains.PursuitSeconds);
        // What the axles will hold at the braking being done right now.
        float braking = Math.Clamp(-car.Telemetry.ActualLongitudinalAccel / MathF.Max(_envelope.Brake(speed), 1f), 0f, 1f);
        LaneEnvelope turn = _envelope.ForTurn(here);
        float lateralLimit = turn.Lateral(speed) +
                             (turn.LateralBraking(speed) - turn.Lateral(speed)) * braking;

        float curvature = feedforward +
                          2f * MathF.Sin(headingError) / pursuit -
                          2f * leftError / (pursuit * pursuit);

        // The corner the tyres can carry, on the road as it lies here: a bank
        // takes part of it off them and presses them down, so on Daytona's
        // banking the command may ask for far more than it could on the flat.
        TrackSample under = context.Track.Sample(car.TrackS);
        RoadAttitude att = Gains.UseRoadAttitude
            ? new(Gains.UseGrade ? under.Grade : 0f,
                  Gains.UseBank && Gains.BankInCommand ? under.BankSlopeAt(car.TrackD) : 0f,
                  Gains.UseVertical ? under.VerticalRate : 0f)
            : RoadAttitude.Flat;
        float load = att.NormalGravity(StandardGravity, speed, curvature) / StandardGravity;
        float available = Gains.CommandLateralShare * MathF.Max(1f, lateralLimit + turn.A * (load - 1f));

        // Better than any estimate: what the axles have this instant, as the
        // physics reports it, with the brakes' share of each circle taken out
        // and the corner split between them the way the physics splits it.
        // The physics winds the wheel off the moment either axle is asked for
        // more than it has, so this is a cliff, not a slope, and the only
        // number accurate enough to stand at the edge of it is its own.
        CarTelemetry axles = car.Telemetry;
        if (Gains.CapFromAxles && axles.FrontGripAccel > 0.5f && axles.RearGripAccel > 0.5f && speed > 8f)
        {
            float frontShare = Math.Clamp(_car.FrontStaticLoadShare, 0.05f, 0.95f);
            float frontLong = Math.Clamp(axles.FrontLongitudinalUse, 0f, 1f);
            float rearLong = Math.Clamp(axles.RearLongitudinalUse, 0f, 1f);
            float front = axles.FrontGripAccel * MathF.Sqrt(1f - frontLong * frontLong) / frontShare;
            float rear = axles.RearGripAccel * MathF.Sqrt(1f - rearLong * rearLong) / (1f - frontShare);
            // A driver with less feel leaves the tyre nearer its peak.
            // A mistake is a misreading of the grip, so the hands believe it too.
            float misread = _performance?.CornerOverstep ?? 0f;
            float capShare = _style is { } capStyle
                ? (Gains.AxleCapShare * capStyle.Cap + (_performance?.LimitOveruse ?? 0f)) * (1f + 2f * misread)
                : (Gains.AxleCapShare + (_performance?.LimitOveruse ?? 0f)) * _tyreAuthority * (1f + 2f * misread);
            available = Gains.CommandLateralShare * capShare * MathF.Max(1f, MathF.Min(front, rear));
        }
        float lean = att.LateralGravityDemand(StandardGravity, speed);
        float scale = MathF.Max(att.CurvatureDemandScale * MathF.Max(speed * speed, 1f), 1e-3f);
        float maxRequest = car.Capabilities.MaxCurvatureRequest;
        float upper = MathF.Min(maxRequest, MathF.Max((available - lean) / scale, 1e-4f));
        float lower = MathF.Max(-maxRequest, MathF.Min((-available - lean) / scale, -1e-4f));
        float wanted = curvature;
        curvature = Math.Clamp(curvature, lower, upper);

        float slip = MathF.Abs(car.SideslipAngleRadians);
        float slide;
        if (_performance is null)
        {
            slide = Math.Clamp(
                (slip - Gains.SlideOnsetRadians) / MathF.Max(Gains.SlideFullRadians - Gains.SlideOnsetRadians, 1e-3f),
                0f,
                1f
            );
            if (slide > 0f)
                curvature += (here - curvature) * slide;
        }
        else
        {
            slide = Recover(car, ref curvature, dt);
        }

        // Longitudinal: follow the table with its own slope as feedforward,
        // and pay the drag the axles will be charged for.
        // Consistency: the brake marker, read a few metres early or late,
        // and the speed this segment, a little over or under.
        float marker = (_performance?.BrakeMarkerMeters ?? 0f) / _lanes.Spacing;
        // Car control: a segment the driver has got wrong is taken faster
        // than the plan, not merely allowed more wheel.
        float speedScale = 1f + (_performance?.SpeedError ?? 0f) + (_performance?.CornerOverstep ?? 0f);
        station += marker;
        float hereSpeed = SpeedAt(station);
        float slowing = SpeedAt(station + 5f / _lanes.Spacing) < hereSpeed ? 1f : 0f;
        float preview = slowing > 0f
            ? Gains.BrakePreviewSeconds + (_performance?.ExtraBrakePreviewSeconds ?? 0f)
            : Gains.SpeedPreviewSeconds;
        float target = SpeedAt(station + MathF.Max(0f, speed) * preview / _lanes.Spacing);
        float ahead = SpeedAt(station + (MathF.Max(0f, speed) * preview + 5f) / _lanes.Spacing);
        target *= speedScale;
        ahead *= speedScale;
        float slope = (ahead - target) / 5f;
        float accel = target * slope + Gains.SpeedGain * (target - speed) + _envelope.Loss(speed);

        // Lift and coast: off the throttle this long before a braking zone.
        // The speed given up would have gone into the brakes a moment later,
        // and the regeneration cap only takes a small share of peak braking.
        if (CoastSeconds > 0f && accel > 0f)
        {
            float lookahead = MathF.Max(0f, speed) * CoastSeconds / _lanes.Spacing;
            if (SpeedAt(station + lookahead) < hereSpeed - 1f)
                accel = 0f;
        }
        // Pedals against the grip the tyres actually have this instant, as
        // the physics reports it, not against what the envelope expects.
        // Throttle: whatever the driven axle has left after the corner it is
        // already carrying. Hot, that is more than the motor can use; cold,
        // it is less, and the throttle comes in only as fast as it is taken.
        CarTelemetry tyres = car.Telemetry;
        float driveLimit = car.Capabilities.AvailableDriveAcceleration;
        // What the tyres will carry of the coming corner on this road: a bank
        // takes its share off them, so the throttle is not held back on
        // banking by load the banking is carrying.
        float tyreCorner = MathF.Abs(att.CurvatureDemandScale * speed * speed * curvature +
                                     att.LateralGravityDemand(StandardGravity, speed));
        float circle = (_style is { } throttleStyle
                ? throttleStyle.Traction + (_performance?.LimitOveruse ?? 0f)
                : (Gains.TractionUse + (_performance?.LimitOveruse ?? 0f)) * _tyreAuthority)
            * (1f + 2f * (_performance?.CornerOverstep ?? 0f));
        // One axle's room for drive: the circle a tyre holds (about nine
        // tenths of its peak, TireSlipCurve.SustainablePeakShare) with the
        // corner it is carrying, or about to carry, taken out. Below walking
        // pace the lateral model is kinematic and its use figure means
        // nothing; only the corner coming counts.
        float Room(float grip, float axleShare, float lateralUseNow)
        {
            if (grip <= 0.1f)
                return 0f;
            float coming = axleShare * tyreCorner / grip;
            float carrying = speed > 10f ? lateralUseNow : 0f;
            float lateralUse = Math.Clamp(MathF.Max(coming, carrying), 0f, 1f);
            return grip * MathF.Sqrt(MathF.Max(0f, circle * circle - lateralUse * lateralUse));
        }
        float motorShare = Math.Clamp(_car.FrontDriveShare, 0f, 1f);
        float rearRoom = Room(tyres.RearGripAccel, 1f - _car.FrontStaticLoadShare, tyres.RearLateralUse);
        float frontRoom = Room(tyres.FrontGripAccel, _car.FrontStaticLoadShare, tyres.FrontLateralUse);
        float tractionRoom;
        if (_car.AdaptiveDriveSplit)
        {
            // The car shares drive between its motors by the grip each axle
            // has left, so the room is the two together, each held to what
            // its own motor can give.
            float drive = car.Capabilities.AvailableDriveAcceleration;
            tractionRoom = MathF.Min(frontRoom, drive * motorShare) +
                           MathF.Min(rearRoom, drive * (1f - motorShare));
        }
        else
        {
            // A fixed split: the drive runs out where the first axle does.
            tractionRoom = MathF.Min(
                motorShare > 1e-3f ? frontRoom / motorShare : float.PositiveInfinity,
                motorShare < 1f - 1e-3f ? rearRoom / (1f - motorShare) : float.PositiveInfinity);
        }
        // The command is what the axles are asked to push; drag comes off
        // after the tyre, so it does not widen the tyre's circle.
        driveLimit = MathF.Min(driveLimit, tractionRoom);
        accel = Math.Clamp(accel, -_car.MaxBrakeAccel, MathF.Max(0f, driveLimit));

        // Brakes: eased the moment either axle is near the edge of its circle,
        // which is what the combined-grip limiter does at the Normal rung and
        // stops doing at Attack.
        if (accel < 0f)
        {
            float front = Combined(tyres.FrontLateralUse, tyres.FrontLongitudinalUse);
            float rear = Combined(tyres.RearLateralUse, tyres.RearLongitudinalUse);
            float feel = Gains.BrakeFeelUse * (_style?.Cap ?? _tyreAuthority);
            float over = MathF.Max(front, rear) - MathF.Min(0.995f, feel) - 2f * (_performance?.CornerOverstep ?? 0f);
            if (over > 0f)
                accel *= Math.Clamp(1f - over / Gains.BrakeFeelBand, Gains.BrakeFeelFloor, 1f);
        }
        if (slide > 0f)
            accel = MathF.Min(accel, accel * (1f - slide));

        LastTelemetry = new LaneDriverTelemetry(
            -leftError, headingError, target, feedforward, curvature, accel, Range, Side, wanted, curvature >= 0f ? upper : lower);
        return new DriverInput(curvature, accel, FrontBrakeBiasOffset: _performance?.FrontBrakeBiasOffset ?? 0f);
    }

    private float _perceivedSlip;
    private float _perceivedYawError;
    private float _slidingFor;
    private bool _recovering;

    /// <summary>Whether the driver is catching a slide right now.</summary>
    public bool IsRecovering => _recovering;

    /// <summary>Slides caught (or not) since the start, for probes.</summary>
    public int RecoveryEvents { get; private set; }

    /// <summary>
    /// Catching a slide, the historical driver's way: perceived through a
    /// filter whose speed is the driver's car control, acted on only after
    /// their reaction time, then countersteer on the perceived sideslip and
    /// yaw error with a firmness drawn for this one catch, and the throttle
    /// lifted for as long as it lasts. Returns how severe the slide is, 0..1.
    /// </summary>
    private float Recover(RaceCarSnapshot car, ref float curvature, float dt)
    {
        LanePerformance performance = _performance!;
        float speed = car.SpeedMetersPerSecond;
        float sideslip = car.SideslipAngleRadians;
        float yawError = car.YawRateRadiansPerSecond - speed * curvature;
        float filter = 1f - MathF.Exp(-MathF.Max(0f, dt) / MathF.Max(performance.PerceptionSeconds, 1e-3f));
        _perceivedSlip += (sideslip - _perceivedSlip) * filter;
        _perceivedYawError += (yawError - _perceivedYawError) * filter;

        // In this physics two to four degrees of body slip is an ordinary
        // corner; what says the car is going is the rear sliding, a slip past
        // that, or a yaw rate running away from the one asked for.
        float slipSeverity = Math.Clamp((MathF.Abs(sideslip) - 0.07f) / 0.08f, 0f, 1f);
        float yawSeverity = Math.Clamp((MathF.Abs(yawError) - 0.10f) / 0.6f, 0f, 1f) *
                            Math.Clamp(MathF.Abs(sideslip) / 0.06f, 0f, 1f);
        float rearSeverity = Math.Clamp((car.Telemetry.RearSlideSeverity - 0.15f) / 0.6f, 0f, 1f);
        float severity = MathF.Max(slipSeverity, MathF.Max(yawSeverity, rearSeverity));

        if (!_recovering)
        {
            _slidingFor = severity > 0.15f ? _slidingFor + dt : 0f;
            if (_slidingFor >= performance.ReactionSeconds)
            {
                _recovering = true;
                RecoveryEvents++;
                performance.BeginRecovery();
            }
        }
        else if (severity < 0.05f)
        {
            _recovering = false;
            _slidingFor = 0f;
        }

        if (!_recovering)
            return 0f;
        float wheelBase = MathF.Max(car.WheelBaseMeters, 1.5f);
        float correction = (0.35f * _perceivedSlip / wheelBase -
                            0.25f * _perceivedYawError / MathF.Max(speed, 5f)) * performance.RecoveryGain;
        float limit = car.Capabilities.MaxCurvatureRequest;
        curvature = Math.Clamp(curvature + correction, -limit, limit);
        return severity;
    }

    private static float Combined(float lateral, float longitudinal) =>
        MathF.Sqrt(lateral * lateral + longitudinal * longitudinal);

    /// <summary>The planned speed at the car, for probes.</summary>
    public float PlannedSpeedAt(float s) => SpeedAt(_lanes.StationOf(s));

    public float PlannedLapSeconds(LaneId lane)
    {
        float seconds = 0f;
        for (int i = 0; i < _lanes.Count; i++)
            seconds += _stepLength[(int)lane][i] / MathF.Max(_speed[(int)lane][i], 1f);
        return seconds;
    }

    private void MoveBlend(float dt)
    {
        Range += Math.Clamp(Math.Clamp(TargetRange, 0f, 1f) - Range, -RangeRate * dt, RangeRate * dt);
        Side += Math.Clamp(Math.Clamp(TargetSide, -1f, 1f) - Side, -SideRate * dt, SideRate * dt);
    }

    private void Refresh(RaceCarSnapshot car, float roadGrip)
    {
        PlanState.Hydrate(_plan, car);
        _tyreAuthority = Gains.TyreRungAuthority
            ? _tires.GetAccelerationUsage(car.Strategy) / MathF.Max(_tires.NormalAccelerationUsage, 0.5f)
            : 1f;
        _style = Gains.TyreRungStyles
            ? Gains.TyreStyleTable[Math.Clamp((int)car.Strategy.TireMode - 1, 0, Gains.TyreStyleTable.Length - 1)]
            : null;
        _planShares = _style is { } style
            ? Shares with { Lateral = Shares.Lateral * style.Corner, Brake = Shares.Brake * style.Brake }
            : Shares;
        LaneEnvelope truth = LaneEnvelope.From(_plan, _car, _tires, car.Strategy, Gains.TurnAwareEnvelope).WithRoadGrip(roadGrip);
        ActualGrip = truth.Lateral(40f) / _referenceLateral;
        // Adaptability: the plan is made on the grip the driver believes in.
        // The hands still feel the axles as they are; only the plan is late.
        Grip = _performance?.PerceiveGrip(ActualGrip, _sinceRefresh) ?? ActualGrip;
        float belief = Math.Clamp(Grip / MathF.Max(ActualGrip, 1e-3f), 0.9f, 1.1f);
        _envelope = belief == 1f ? truth : LaneEnvelope.From(_plan, _car, _tires, car.Strategy, Gains.TurnAwareEnvelope).WithRoadGrip(roadGrip * belief);
        for (int lane = 0; lane < 3; lane++)
        {
            _file.FactorsAtGrip((LaneId)lane, Grip, _corner[lane], _brake[lane]);
            // Pace: the share of the plan this driver takes this lap.
            float pace = _performance?.PaceShare ?? 1f;
            if (pace < 1f)
            {
                for (int i = 0; i < _corner[lane].Length; i++)
                {
                    _corner[lane][i] *= pace;
                    _brake[lane][i] *= pace;
                }
            }
            LaneSpeedPlanner.PlanClosed(
                _envelope,
                _curvature[lane],
                _stepLength[lane],
                _corner[lane],
                _brake[lane],
                _road[lane],
                _planShares,
                _speed[lane]
            );
        }
        _sinceRefresh = 0f;
        _windowStart = -1;
    }

    private float SpeedAt(float station)
    {
        if (Range <= 1e-3f && TargetRange <= 1e-3f)
            return Interpolate(_speed[0], station);

        // Off the free lane: plan the blend actually in force over a window,
        // ending on the slower of the pure lanes it lies between.
        int start = (int)MathF.Floor(station);
        if (start != _windowStart || MathF.Abs(Range - _windowRange) > 0.02f || MathF.Abs(Side - _windowSide) > 0.02f)
            PlanBlendWindow(start);
        float local = station - _windowStart;
        int i0 = Math.Clamp((int)MathF.Floor(local), 0, _windowSpeed.Length - 2);
        float u = Math.Clamp(local - i0, 0f, 1f);
        float planned = _windowSpeed[i0] + (_windowSpeed[i0 + 1] - _windowSpeed[i0]) * u;
        // A blend that is still moving is planned at both ends of its travel.
        return MathF.Min(planned, BlendTargetCeiling(station));
    }

    private float BlendTargetCeiling(float station)
    {
        if (MathF.Abs(TargetRange - Range) < 0.02f && MathF.Abs(TargetSide - Side) < 0.02f)
            return float.PositiveInfinity;
        float range = Math.Clamp(TargetRange, 0f, 1f);
        float side = Math.Clamp(TargetSide, -1f, 1f);
        float slowest = float.PositiveInfinity;
        int start = (int)MathF.Floor(station);
        for (int k = 0; k < _windowSpeed.Length; k += 4)
        {
            float k2 = BlendedCurvatureFor(start + k, range, side);
            float cornerSpeed = LaneSpeedPlanner.CornerLimit(
                _envelope, k2, FactorFor(start + k, range, side) * _planShares.Lateral,
                RoadAt(start + k, _lanes.BlendedOffset(start + k, range, side)));
            float brake = 0.8f * _planShares.Brake * _envelope.Brake(cornerSpeed);
            float distance = MathF.Max(0f, (k - (station - start)) * _lanes.Spacing);
            slowest = MathF.Min(slowest, MathF.Sqrt(cornerSpeed * cornerSpeed + 2f * brake * distance));
        }
        return slowest;
    }

    private void PlanBlendWindow(int start)
    {
        int n = _windowSpeed.Length;
        for (int k = 0; k < n; k++)
        {
            int i = start + k;
            _windowCurvature[k] = BlendedCurvatureFor(i, Range, Side);
            _windowRoad[k] = RoadAt(i, _lanes.BlendedOffset(i, Range, Side));
            _windowStep[k] = (BlendedPosition(i + 1, Range, Side) - BlendedPosition(i, Range, Side)).Length();
            _windowFactor[k] = FactorFor(i, Range, Side);
            _windowBrake[k] = BlendOf(lane => _brake[(int)lane], i, Range, Side);
        }

        int end = _lanes.Wrap(start + n - 1);
        int sideLane = Side >= 0f ? (int)LaneId.Left : (int)LaneId.Right;
        float terminal = MathF.Min(_speed[0][end], _speed[sideLane][end]);
        LaneSpeedPlanner.PlanWindow(_envelope, _windowCurvature, _windowStep, _windowFactor, _windowBrake, _windowRoad, _planShares, terminal, _windowSpeed);
        _windowStart = start;
        _windowRange = Range;
        _windowSide = Side;
    }

    private float FactorFor(int station, float range, float side) =>
        BlendOf(lane => _corner[(int)lane], station, range, side);

    private float BlendOf(Func<LaneId, float[]> table, int station, float range, float side)
    {
        int i = _lanes.Wrap(station);
        float free = table(LaneId.Free)[i];
        if (range <= 0f)
            return free;
        float left = free + (table(LaneId.Left)[i] - free) * range;
        float right = free + (table(LaneId.Right)[i] - free) * range;
        float towardLeft = (1f + side) * 0.5f;
        return right + (left - right) * towardLeft;
    }

    private const float StandardGravity = 9.80665f;

    /// <summary>How the road lies under a station at this offset, as the physics reads it.</summary>
    private LaneRoad RoadAt(int station, float offset)
    {
        if (_trackView is null || !Gains.UseRoadAttitude)
            return default;
        TrackSample sample = _trackView.Sample(_lanes.Wrap(station) * _lanes.Spacing);
        return new LaneRoad(
            Gains.UseGrade ? sample.Grade : 0f,
            Gains.UseBank ? sample.BankSlopeAt(offset) : 0f,
            Gains.UseVertical ? sample.VerticalRate : 0f);
    }

    private Vector2 PositionOn(LaneId lane, int station) =>
        _lanes.Position(station, _lanes.Offset(lane, station));

    private float CurvatureAt(LaneId lane, int station) =>
        LaneSet.Curvature(
            PositionOn(lane, station - CurvatureChordStations),
            PositionOn(lane, station),
            PositionOn(lane, station + CurvatureChordStations));

    private Vector2 BlendedPosition(int station, float range, float side) =>
        _lanes.Position(station, _lanes.BlendedOffset(station, range, side));

    private float BlendedCurvatureFor(int station, float range, float side) =>
        LaneSet.Curvature(
            BlendedPosition(station - CurvatureChordStations, range, side),
            BlendedPosition(station, range, side),
            BlendedPosition(station + CurvatureChordStations, range, side));

    private float BlendedCurvature(float station)
    {
        int i0 = (int)MathF.Floor(station);
        float u = station - i0;
        float a = BlendedCurvatureFor(i0, Range, Side);
        float b = BlendedCurvatureFor(i0 + 1, Range, Side);
        return a + (b - a) * u;
    }

    /// <summary>The learned steering correction under the blend in force.</summary>
    private float BlendedSteer(float station)
    {
        int i0 = (int)MathF.Floor(station);
        float u = station - i0;
        float a = BlendOf(_file.Steer, i0, Range, Side);
        float b = BlendOf(_file.Steer, i0 + 1, Range, Side);
        return a + (b - a) * u;
    }

    private (Vector2 Position, Vector2 Tangent) BlendedPose(float station)
    {
        int i0 = (int)MathF.Floor(station);
        float u = station - i0;
        Vector2 a = BlendedPosition(i0, Range, Side);
        Vector2 b = BlendedPosition(i0 + 1, Range, Side);
        Vector2 tangent = b - a;
        float length = tangent.Length();
        tangent = length > 1e-5f ? tangent / length : new Vector2(1f, 0f);
        return (a + (b - a) * u, tangent);
    }

    private float Interpolate(float[] values, float station)
    {
        int i0 = (int)MathF.Floor(station);
        float u = station - i0;
        float a = values[_lanes.Wrap(i0)];
        float b = values[_lanes.Wrap(i0 + 1)];
        return a + (b - a) * u;
    }
}

public readonly record struct LaneDriverTelemetry(
    float LateralErrorMeters,
    float HeadingErrorRadians,
    float TargetSpeed,
    float FeedforwardCurvature,
    float CommandCurvature,
    float CommandAccel,
    float Range,
    float Side,
    float WantedCurvature = 0f,
    float CurvatureCap = 0f
);

/// <summary>
/// One tyre rung's way of driving. Corner and brake scale the plan's shares
/// of the envelope; traction is the share of the driven axle's circle the
/// throttle may fill; cap scales how far the wheel may be wound on.
/// </summary>
public readonly record struct TyreStyle(float Corner, float Brake, float Traction, float Cap);

internal static class PlanState
{
    public static void Hydrate(CarState plan, RaceCarSnapshot car)
    {
        plan.Position = car.Position;
        plan.Heading = car.HeadingRadians;
        plan.SideslipAngleRadians = car.SideslipAngleRadians;
        plan.YawRateRadiansPerSecond = car.YawRateRadiansPerSecond;
        plan.Speed = car.SpeedMetersPerSecond;
        plan.SteerAngleRadians = car.SteerAngleRadians;
        plan.AirVelocityDeficit = car.AirVelocityDeficit;
        plan.DownforceVelocityDeficit = car.DownforceVelocityDeficit;
        plan.WakeDownforceLoss = car.WakeDownforceLoss;
        plan.ActiveAero = car.ActiveAero;
        if (car.Resources.Length > 0)
        {
            float primary = car.Resources[0].Fraction;
            float secondary = car.Resources.Length > 1 ? car.Resources[1].Fraction : primary;
            plan.Energy = new PowertrainState(primary, secondary);
        }
        Copy(plan.FrontLeft, car.FrontLeft);
        Copy(plan.FrontRight, car.FrontRight);
        Copy(plan.RearLeft, car.RearLeft);
        Copy(plan.RearRight, car.RearRight);
    }

    private static void Copy(TireState target, TireSnapshot source)
    {
        target.SurfaceTempC = source.SurfaceTempC;
        target.CoreTempC = source.CoreTempC;
        target.Wear = source.Wear;
        target.LoadN = source.LoadN;
        target.SurfaceGrip = source.SurfaceGrip;
    }
}
