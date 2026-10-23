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
    /// How far ahead the curvature feedforward is read, in seconds of travel,
    /// in place of the car's own turn-in delay (<see cref="TurnLag"/>), which
    /// is what it reads ahead by when this is unset. Only for probes.
    ///
    /// It used to be a fixed 0.15 s, swept on a car that took that long to
    /// answer the wheel. The car since answers in 0.05 to 0.07 s, and read
    /// 0.15 s ahead every exit was steered as if it had already opened up:
    /// the car ran half a metre wide of the line out of every corner, by as
    /// much as the feedback needed to pull the difference back, and onto
    /// the barrier out of Portier, where the wall stands on the kerb.
    /// </summary>
    public float? CurvaturePreviewSeconds { get; init; }

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

    /// <summary>The plan corners on the sharpest of the road the hands are already steering for (LaneDriver.LeadCurvatures).</summary>
    public bool PlanTurnLead { get; init; } = true;

    /// <summary>Lane moves eased in and out (LaneDriver.RangeAcceleration); off, at a fixed rate.</summary>
    public bool EaseLaneMoves { get; init; } = true;

    /// <summary>Steering for a lane move's own sideways motion (LaneDriver.MoveAt).</summary>
    public bool FollowLaneMoves { get; init; } = true;

    /// <summary>
    /// The tyre rung as a way of driving, set by the player and carried out
    /// by the driver: how much of the corner, the throttle, the brakes and
    /// the wheel each rung spends. Off, the rung only scales the tyre's
    /// allowance, which moves almost nothing. On by default: the top row of
    /// the table is the plan as it is learned, so every other rung drives
    /// inside what the learning proved.
    /// </summary>
    public bool TyreRungStyles { get; init; } = true;

    /// <summary>
    /// Protect, Light, Normal, Push, Attack: one line from the plan as
    /// learned down to Protect, spaced so each rung is about 0.3 s a lap from
    /// the next on Silverstone (Attack to Protect 1.17 s, Protect wearing a
    /// fifth less), with the car's limiter the same at every rung.
    /// </summary>
    public TyreStyle[] TyreStyleTable { get; init; } =
    [
        new(Corner: 0.9731f, Brake: 0.9812f, Traction: 0.8355f, Cap: 0.9937f),
        new(Corner: 0.9786f, Brake: 0.9850f, Traction: 0.8485f, Cap: 0.9950f),
        new(Corner: 0.9848f, Brake: 0.9893f, Traction: 0.8634f, Cap: 0.9964f),
        new(Corner: 0.9918f, Brake: 0.9943f, Traction: 0.8803f, Cap: 0.9981f),
        new(Corner: 1.0000f, Brake: 1.0000f, Traction: 0.9000f, Cap: 1.0000f),
    ];

    /// <summary>
    /// Share of the axles' live grip the command may ask for: what a tyre
    /// holds, not its peak, so the physics' own figure for it. It was 0.90,
    /// set when a tyre held nine tenths of its peak, and stayed there when
    /// the steering stopped winding itself off and the tyre came to hold
    /// 0.98 (Tire.SustainablePeakShare). The envelope the plan is
    /// drawn on moved with the physics and the hands did not: the plan asked
    /// 93% of the peak of hands that would give 90, and with nothing over to
    /// steer back with, a car a little late into a long corner ran wider all
    /// the way round it, at Zandvoort's Scheivlak off the road.
    /// </summary>
    public float AxleCapShare { get; init; } = Tire.SustainablePeakShare;

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
    /// pack; the driver only lifts as the rung says. A circuit sets its own
    /// in its <c>driver.json</c>, since a second of lift saves a different
    /// amount on every track; this table stands in where a circuit has none.
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
    private const float WindowMeters = 450f;

    // Only the road ahead is planned, every step, on the grip the tyres have
    // this instant. A tread heats and cools within a corner, and a plan even
    // a tenth of a second old had the car arrive at the next one on grip it
    // no longer had. Five hundred metres covers the longest braking the car
    // does (about 150 m from top speed at Silverstone) with room to spare,
    // so the far end may stop on its own corner's limit: what lies past it
    // cannot reach back to the car. The cost is the same on any circuit,
    // a Nordschleife as a Monaco, and the same every step.
    private const float AheadMeters = 500f;

    private readonly LaneTrackFile _file;
    private float[]? _energyTable;
    private readonly CarConfig _car;
    private readonly TireConfig _tires;
    private readonly TurnLag _turnLag;
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
    private readonly bool[] _planned = new bool[3];
    private float[] _aheadCurvature = [];
    private float[] _aheadStep = [];
    private float[] _aheadFactor = [];
    private float[] _aheadBrake = [];
    private LaneRoad[] _aheadRoad = [];
    private float[] _aheadSpeed = [];

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
        _turnLag = TurnLag.Measure(_car, _tires);
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
        float[] table = _energyTable ??= _file.Energy?.LiftSeconds() ?? Gains.EnergyRungLiftSeconds;
        float lift = CoastOverride ??
                     (table.Length == 0 ? 0f : table[Math.Clamp(car.Strategy.Rung(0, _car.Powertrain.OutputLadder) - 1, 0, table.Length - 1)]);
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

    /// <summary>
    /// How fast that travel may itself change, per second. At a fixed rate
    /// the line the car is asked to follow kinked sideways the instant a move
    /// began and again where it ended, a jump in lateral speed no car can
    /// make, and every switch ran a metre and more off it. Eased in and out,
    /// as Simplix's Runaround moves its offset, the move is an S the car can
    /// drive: at 0.6 a second of travel it takes the first half second to get
    /// up to speed, and with lanes eight metres apart asks some 10 m/s² of
    /// the tyres at its sharpest.
    /// </summary>
    public float RangeAcceleration { get; init; } = 1.2f;
    public float SideAcceleration { get; init; } = 1.6f;

    private float _rangeVelocity;
    private float _sideVelocity;

    /// <summary>
    /// The share of the full rate the move under way is made at: the
    /// quickest of these the tyres can carry, read again this often. A move
    /// asked for in a corner the car is already at the edge of was made at
    /// full rate all the same, and at Monaco's swimming pool, begun under
    /// braking, the S it asked for needed more slowing than the road before
    /// the corner had room for: the car ran two metres wide. A driver
    /// eases across there, or waits, and goes once the corner lets go.
    /// </summary>
    private static readonly float[] MoveScales = [1f, 0.6f, 0.35f, 0.15f, 0f];
    private const float MoveCheckSeconds = 0.1f;

    /// <summary>How much faster than its plan the car may be at the start of a move's window and still make it.</summary>
    private const float MoveSpeedToleranceMps = 0.5f;

    private float _moveScale = 1f;
    private float _moveCheckIn;

    /// <summary>
    /// The most the move may be made at, held down by the hands: wherever a
    /// move adding to the corner finds the wheel at the axles' limit, a step
    /// down MoveScales, and a step back up once it has been clear this long.
    /// The plan's grip under braking is the lanes' learned one, and a move
    /// between lanes is on neither: at Monaco the plan said the S could be
    /// made where the axles, braking, had less than it thought.
    /// </summary>
    private float _boundScale = 1f;
    private float _boundClearFor;
    private float _boundStepIn;
    private float _moveCurvature;
    private bool _moveBound;
    private const float BoundClearSeconds = 0.3f;

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
        int ahead = (int)MathF.Ceiling(AheadMeters / _lanes.Spacing);
        _aheadCurvature = new float[ahead];
        _aheadStep = new float[ahead];
        _aheadFactor = new float[ahead];
        _aheadBrake = new float[ahead];
        _aheadRoad = new LaneRoad[ahead];
        _aheadSpeed = new float[ahead];
        _leadGuess = new float[Math.Max(ahead, window)];
        _leadBuffer = new float[Math.Max(ahead, window)];
        Range = TargetRange;
        Side = TargetSide;
        _rangeVelocity = 0f;
        _sideVelocity = 0f;
        _moveScale = 1f;
        _moveCheckIn = 0f;
        _boundScale = 1f;
        _boundClearFor = 0f;
        _boundStepIn = 0f;
        Array.Clear(_planned);
        _referenceLateral = ReferenceLateral();
        _performance = UseAbilities ? new LanePerformance(context.Profile) : null;
        _performance?.Update(0, context.Car.TrackS, 0f);
        PlanAhead(context.Car, context.Environment.SurfaceGripScalar, 0f);
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

    /// <summary>Grip as the driver reads it, as a share of this circuit at its best, from the last refresh.</summary>
    public float Grip { get; private set; } = 1f;

    /// <summary>Grip as it really is, as a share of this circuit at its best, from the last refresh.</summary>
    public float ActualGrip { get; private set; } = 1f;

    public DriverInput GetControl(in DriverContext context, float dt)
    {
        RaceCarSnapshot car = context.Car;
        ManageEnergy(car);
        if (_performance is not null)
        {
            int lap = (int)MathF.Floor(car.RaceDistanceMeters / MathF.Max(car.TrackLengthMeters, 1f));
            _performance.Update(lap, car.TrackS, dt);
        }
        PlanAhead(car, context.Environment.SurfaceGripScalar, dt);
        ScaleMove(car, dt);
        MoveBlend(dt);

        _axles = AxlesOf(car);
        float speed = car.SpeedMetersPerSecond;
        float station = _lanes.StationOf(car.TrackS);

        // Where the plan is, at the car and a little ahead.
        (Vector2 refPosition, Vector2 tangent) = BlendedPose(station);
        // Consistency: the line this segment, a little off the lane.
        if (_performance is not null)
            refPosition += _lanes.Normal((int)MathF.Round(station)) * _performance.LateralOffsetMeters;
        // A move under way carries the line sideways: it heads that much
        // across the lane's own heading, and bends by what its sideways
        // speed changes over the road the car covers.
        float moveSpeed = MoveAt((int)MathF.Round(station), 0f).Speed;
        float pathHeading = MathF.Atan2(tangent.Y, tangent.X) - MathF.Atan2(moveSpeed, MathF.Max(speed, 5f));
        Vector2 toCar = car.Position - refPosition;
        float leftError = tangent.X * toCar.Y - tangent.Y * toCar.X;
        float headingError = MathHelper.NormalizeAngle(pathHeading - car.VelocityHeadingRadians);
        float turnIn = Gains.CurvaturePreviewSeconds ?? _turnLag.Seconds(speed);
        float previewStation = station + MathF.Max(0f, speed) * turnIn / _lanes.Spacing;
        float moveCurvature = _moveCurvature = -MoveAt((int)MathF.Round(previewStation), turnIn).Acceleration /
                              MathF.Max(speed * speed, 25f);
        float feedforward = BlendedCurvature(previewStation) + BlendedSteer(previewStation) + moveCurvature;
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
        float available = Gains.CommandLateralShare * MathF.Max(1f, LaneEnvelope.WithLoad(lateralLimit, turn.A, load, turn.LoadExponent));

        // Better than any estimate: what the axles have this instant, as the
        // physics reports it, with the brakes' share of each circle taken out
        // and the corner split between them the way the physics splits it.
        // The physics winds the wheel off the moment either axle is asked for
        // more than it has, so this is a cliff, not a slope, and the only
        // number accurate enough to stand at the edge of it is its own.
        AxleTelemetry axles = _axles;
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
        _moveBound = (wanted > upper && _moveCurvature > 0f) || (wanted < lower && _moveCurvature < 0f);

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
        AnchorBlendWindow(station);
        float hereSpeed = SpeedAt(station);
        // The speed wanted is the plan's where the car is, as Simplix reads
        // it: the pedals take hold within the step (CarPhysics applies the
        // command as it is given), so there is no lag to read ahead for, and
        // read 0.3 s ahead the car drove the plan a third of a second early:
        // braked twenty or thirty metres before every marker. The margin a
        // corner needs is the plan's and what the learner finds, not a
        // look ahead that moves every braking point at once.
        float slowing = SpeedAt(station + 5f / _lanes.Spacing) < hereSpeed ? 1f : 0f;
        float preview = MathF.Max(dt, 0f) + slowing * (_performance?.ExtraBrakePreviewSeconds ?? 0f);
        float target = SpeedAt(station + MathF.Max(0f, speed) * preview / _lanes.Spacing);
        float ahead = SpeedAt(station + (MathF.Max(0f, speed) * preview + 5f) / _lanes.Spacing);
        target *= speedScale;
        ahead *= speedScale;
        float slope = (ahead - target) / 5f;
        // The road's part, the planner's way round (LaneSpeedPlanner.DriveFrom):
        // the axles are scaled onto the slope and gravity comes on top. Left
        // out, the pedals asked a climb for what the air took and no more,
        // and up Raidillon, power-limited and losing speed, the car ran on
        // eight tenths of a throttle it had to give.
        float speedChange = target * slope + Gains.SpeedGain * (target - speed);
        float accel = (speedChange - att.AlongTrackGravity(StandardGravity, speed)) / att.LongitudinalDemandScale +
                      _envelope.Loss(speed);
        // And brakes only over it, as Simplix does: under the plan the car
        // has the braking curve still to reach.
        if (accel < 0f && speed < target)
            accel = 0f;
        // Flat out where the plan is. Where the speed wanted a moment ahead
        // is no less than full drive would bring, the plan is the motor's
        // own curve and the throttle is on the floor; read as a speed to
        // track, a plan falling away up a power-limited climb looked like
        // one to slow for, and the car lifted on it. The traction cap below
        // still holds the pedal to what the tyres have.
        int at = (int)MathF.Round(station);
        float flatOut = speedScale * LaneSpeedPlanner.FlatOutSpeed(
            _envelope, speed, here, MathF.Max(0f, speed) * preview, FactorFor(at, Range, Side),
            RoadAt(at, car.TrackD), _planShares);
        // Flat out is the pedal on the floor, not the drive the car reported
        // a step ago: the motor gives what it has this step. Asked for the
        // last step's figure, the step a push began asked for the power
        // without it, a tenth less, and the throttle trace dipped.
        if (target >= flatOut - FlatOutToleranceMps)
            accel = MathF.Max(accel, _car.MaxDriveAcceleration);

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
        AxleTelemetry tyres = _axles;
        // The pedal's whole travel: the motor takes what it can of it.
        float driveLimit = _car.MaxDriveAcceleration;
        // What the tyres will carry of the coming corner on this road: a bank
        // takes its share off them, so the throttle is not held back on
        // banking by load the banking is carrying. The corner is the one the
        // line wants, not the one the cap above lets the wheel ask for: read
        // off the capped one, a car whose drive was already eating the rear's
        // circle found room for more drive in the corner the drive had cut,
        // and out of a corner on full power it ran two metres wide and off.
        float lineCurvature = Math.Clamp(wanted, -maxRequest, maxRequest);
        float tyreCorner = MathF.Abs(att.CurvatureDemandScale * speed * speed * lineCurvature +
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
            float drive = _car.MaxDriveAcceleration;
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
        // The wheels turned for that curvature as a driver who knew the car
        // would turn them (SteeringEstimate); the hands are this driver's.
        return new DriverInput(car.Steering.AngleFor(curvature), accel, FrontBrakeBiasOffset: _performance?.FrontBrakeBiasOffset ?? 0f);
    }

    // What the car's own instruments say of its two axles this step.
    private AxleTelemetry _axles;

    /// <summary>
    /// The axles as the car reports them, by its instruments' names (the
    /// single-track chassis's: SingleTrackCar.TelemetryChannels). A car
    /// without them reads as no grip measured, and every use of them falls
    /// back to the envelope.
    /// </summary>
    private static AxleTelemetry AxlesOf(RaceCarSnapshot car) => new(
        car.Instrument("front_grip_mps2") ?? 0f,
        car.Instrument("rear_grip_mps2") ?? 0f,
        car.Instrument("front_lateral_use") ?? 0f,
        car.Instrument("rear_lateral_use") ?? 0f,
        car.Instrument("front_longitudinal_use") ?? 0f,
        car.Instrument("rear_longitudinal_use") ?? 0f,
        car.Instrument("grip_limiter_cut_mps2") ?? 0f,
        car.Instrument("induced_drag_mps2") ?? 0f,
        car.Instrument("rear_slide_severity") ?? 0f);

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
        float rearSeverity = Math.Clamp((_axles.RearSlideSeverity - 0.15f) / 0.6f, 0f, 1f);
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

    /// <summary>
    /// The lap this lane would take on the grip the driver last read, planned
    /// whole for the asking. A report, not something the driver drives on.
    /// </summary>
    public float PlannedLapSeconds(LaneId lane)
    {
        int count = _lanes.Count;
        var corner = new float[count];
        var brake = new float[count];
        var speed = new float[count];
        _file.FactorsAtGrip(lane, Grip, corner, brake);
        float pace = _performance?.PaceShare ?? 1f;
        for (int i = 0; i < count; i++)
        {
            corner[i] *= pace;
            brake[i] *= pace;
        }
        int l = (int)lane;
        LaneSpeedPlanner.PlanClosed(_envelope, _curvature[l], _stepLength[l], corner, brake, _road[l], _planShares, speed);
        float seconds = 0f;
        for (int i = 0; i < count; i++)
            seconds += _stepLength[l][i] / MathF.Max(speed[i], 1f);
        return seconds;
    }

    private bool Moving =>
        _rangeVelocity != 0f || _sideVelocity != 0f ||
        MathF.Abs(Math.Clamp(TargetRange, 0f, 1f) - Range) > 1e-4f ||
        (Range > 1e-4f && MathF.Abs(Math.Clamp(TargetSide, -1f, 1f) - Side) > 1e-4f);

    /// <summary>
    /// The quickest share of the full rate (MoveScales) whose S the plan can
    /// still slow the car for from where it is: the window planned along the
    /// move must not want the car slower at its start than it is, beyond
    /// what the line without the move already wants.
    /// </summary>
    private void ScaleMove(RaceCarSnapshot car, float dt)
    {
        if (!Gains.FollowLaneMoves || !Moving)
        {
            _moveScale = 1f;
            _moveCheckIn = 0f;
            _boundScale = 1f;
            _boundClearFor = 0f;
            return;
        }
        _boundStepIn -= dt;
        if (_moveBound)
        {
            _boundClearFor = 0f;
            if (_boundStepIn <= 0f)
            {
                _boundScale = NextScale(MathF.Min(_boundScale, _moveScale), down: true);
                _boundStepIn = MoveCheckSeconds;
                _moveCheckIn = 0f;
            }
        }
        else if ((_boundClearFor += dt) >= BoundClearSeconds && _boundScale < 1f)
        {
            _boundScale = NextScale(_boundScale, down: false);
            _boundClearFor = 0f;
            _moveCheckIn = 0f;
        }
        _moveCheckIn -= dt;
        if (_moveCheckIn > 0f)
            return;
        _moveCheckIn = MoveCheckSeconds;
        int start = (int)MathF.Floor(_lanes.StationOf(car.TrackS));
        float speed = car.SpeedMetersPerSecond;
        foreach (float scale in MoveScales)
        {
            if (scale > _boundScale)
                continue;
            _moveScale = scale;
            PlanBlendWindow(start);
            if (scale == 0f || _windowSpeed[0] >= MathF.Min(speed, _windowStillSpeed) - MoveSpeedToleranceMps)
                break;
        }
    }

    private static float NextScale(float scale, bool down)
    {
        if (down)
        {
            foreach (float s in MoveScales)
                if (s < scale - 1e-4f)
                    return s;
            return 0f;
        }
        for (int i = MoveScales.Length - 1; i >= 0; i--)
            if (MoveScales[i] > scale + 1e-4f)
                return MoveScales[i];
        return 1f;
    }

    private void MoveBlend(float dt)
    {
        float range = Range, side = Side;
        Ease(ref range, ref _rangeVelocity, Math.Clamp(TargetRange, 0f, 1f), RangeRate * _moveScale, Gains.EaseLaneMoves ? RangeAcceleration : 0f, dt);
        Ease(ref side, ref _sideVelocity, Math.Clamp(TargetSide, -1f, 1f), SideRate * _moveScale, Gains.EaseLaneMoves ? SideAcceleration : 0f, dt);
        Range = range;
        Side = side;
    }

    /// <summary>
    /// One step of a move toward its target at no more than the rate, and
    /// with the rate changing by no more than the acceleration: the quickest
    /// way there that arrives at rest, so the move neither starts nor stops
    /// with a jump.
    /// </summary>
    private static void Ease(ref float value, ref float velocity, float target, float rate, float acceleration, float dt)
    {
        if (dt <= 0f)
            return;
        float error = target - value;
        if (acceleration <= 0f)
        {
            float jump = Math.Clamp(error, -rate * dt, rate * dt);
            value += jump;
            velocity = jump / dt;
            return;
        }
        // Fast as the stop allows: what can still be shed over what is left.
        float wanted = MathF.Sign(error) * MathF.Min(rate, MathF.Sqrt(2f * acceleration * MathF.Abs(error)));
        velocity += Math.Clamp(wanted - velocity, -acceleration * dt, acceleration * dt);
        float next = value + velocity * dt;
        // Arrived, or would pass it: at the target and at rest.
        if (MathF.Abs(error) < 1e-4f || (target - next) * error <= 0f)
        {
            value = target;
            velocity = 0f;
            return;
        }
        value = next;
    }

    /// <summary>
    /// How the move under way carries the line sideways at a station, after
    /// the given seconds: its lateral speed and acceleration in m/s and m/s²,
    /// positive along the track's normal, which is to the right (LaneSet). The blend's lanes are drawn where the blend
    /// stands, so a move's own sideways motion is in none of them: without
    /// it the car was steered to a line that stood still and found it gone,
    /// late into every switch and late out of it.
    /// </summary>
    private (float Speed, float Acceleration) MoveAt(int station, float seconds)
    {
        if (!Gains.FollowLaneMoves || _rangeVelocity == 0f && _sideVelocity == 0f &&
            MathF.Abs(TargetRange - Range) < 1e-4f && MathF.Abs(TargetSide - Side) < 1e-4f)
            return (0f, 0f);
        const float h = 0.05f;
        float range = Range, side = Side, rangeVelocity = _rangeVelocity, sideVelocity = _sideVelocity;
        float targetRange = Math.Clamp(TargetRange, 0f, 1f), targetSide = Math.Clamp(TargetSide, -1f, 1f);
        void Step()
        {
            Ease(ref range, ref rangeVelocity, targetRange, RangeRate * _moveScale, Gains.EaseLaneMoves ? RangeAcceleration : 0f, h);
            Ease(ref side, ref sideVelocity, targetSide, SideRate * _moveScale, Gains.EaseLaneMoves ? SideAcceleration : 0f, h);
        }
        for (float t = h; t < seconds - h; t += h)
            Step();
        float before = _lanes.BlendedOffset(station, range, side);
        Step();
        float now = _lanes.BlendedOffset(station, range, side);
        Step();
        float after = _lanes.BlendedOffset(station, range, side);
        return ((after - before) / (2f * h), (after - 2f * now + before) / (h * h));
    }

    /// <summary>The road ahead of the car on every lane, on the grip it has now.</summary>
    private void PlanAhead(RaceCarSnapshot car, float roadGrip, float dt)
    {
        int start = (int)MathF.Floor(_lanes.StationOf(car.TrackS));
        int n = _aheadSpeed.Length;
        // On the free lane with no move asked for, the side lanes are read by
        // nothing; they are planned again the step a move begins.
        bool sides = Range > 1e-3f || TargetRange > 1e-3f;
        ReadGrip(car, roadGrip, dt, start, n + 1, sides);
        for (int lane = 0; lane < 3; lane++)
        {
            if (lane != (int)LaneId.Free && !sides)
            {
                _planned[lane] = false;
                continue;
            }
            for (int k = 0; k < n; k++)
            {
                int i = _lanes.Wrap(start + k);
                _aheadCurvature[k] = _curvature[lane][i];
                _aheadStep[k] = _stepLength[lane][i];
                _aheadFactor[k] = _corner[lane][i];
                _aheadBrake[k] = _brake[lane][i];
                _aheadRoad[k] = _road[lane][i];
                _leadGuess[k] = _speed[lane][i] > 1f ? _speed[lane][i] : car.SpeedMetersPerSecond;
            }
            LeadCurvatures(_aheadCurvature, _leadGuess, n);
            float[] speed = _speed[lane];
            // Driving on from what the last step planned here, so a corner's
            // exit stays the one its apex allows; the first step has none.
            // Or from the car, where it is already faster: the envelope's
            // drive is the motor without push-to-pass, and on a push the car
            // ran a km/h or two ahead of a plan drawn on less power, read
            // that as too fast, and lifted for a step every few, the length
            // of the straight. The corners ahead still cap it as before.
            LaneSpeedPlanner.PlanAhead(
                _envelope, _aheadCurvature, _aheadStep, _aheadFactor, _aheadBrake, _aheadRoad, _planShares,
                startSpeed: _planned[lane] ? MathF.Max(speed[_lanes.Wrap(start)], car.SpeedMetersPerSecond) : float.PositiveInfinity,
                terminalSpeed: float.PositiveInfinity,
                _aheadSpeed);
            for (int k = 0; k < n; k++)
                speed[_lanes.Wrap(start + k)] = _aheadSpeed[k];
            _planned[lane] = true;
        }
        _windowStart = -1;
    }

    /// <summary>The live envelope and the learned factors at the grip the driver believes in.</summary>
    private void ReadGrip(RaceCarSnapshot car, float roadGrip, float sinceLastRead, int start, int count, bool sides)
    {
        PlanState.Hydrate(_plan, car);
        _tyreAuthority = Gains.TyreRungAuthority
            ? CarPhysics.GripUsage(car.Strategy) / MathF.Max(CarPhysics.GripUsage(CarStrategy.Default), 0.5f)
            : 1f;
        _style = Gains.TyreRungStyles
            ? Gains.TyreStyleTable[Math.Clamp((int)car.Strategy.TireMode - 1, 0, Gains.TyreStyleTable.Length - 1)]
            : null;
        _planShares = _style is { } style
            ? Shares with { Lateral = Shares.Lateral * style.Corner, Brake = Shares.Brake * style.Brake }
            : Shares;
        // The road is the day's surface on this circuit's asphalt. Grip is
        // read against the circuit at its best, so full grip means new tyres
        // in their window on a clean track here, wherever here is: the
        // learned levels are about the tyres and the day, and the asphalt is
        // already in the envelope they multiply.
        float asphalt = _trackView?.AsphaltGrip ?? 1f;
        float road = roadGrip * asphalt;
        LaneEnvelope truth = LaneEnvelope.From(_plan, _car, _tires, car.Strategy, Gains.TurnAwareEnvelope).WithRoadGrip(road);
        ActualGrip = truth.Lateral(40f) / (_referenceLateral * MathF.Max(asphalt, 0.05f));
        // Adaptability: the plan is made on the grip the driver believes in.
        // The hands still feel the axles as they are; only the plan is late.
        Grip = _performance?.PerceiveGrip(ActualGrip, sinceLastRead) ?? ActualGrip;
        float belief = Math.Clamp(Grip / MathF.Max(ActualGrip, 1e-3f), 0.9f, 1.1f);
        _envelope = belief == 1f ? truth : LaneEnvelope.From(_plan, _car, _tires, car.Strategy, Gains.TurnAwareEnvelope).WithRoadGrip(road * belief);
        for (int lane = 0; lane < (sides ? 3 : 1); lane++)
        {
            _file.FactorsAtGrip((LaneId)lane, Grip, start, count, _corner[lane], _brake[lane]);
            // Pace: the share of the plan this driver takes this lap.
            float pace = _performance?.PaceShare ?? 1f;
            if (pace < 1f)
            {
                for (int k = 0; k < count; k++)
                {
                    int i = _lanes.Wrap(start + k);
                    _corner[lane][i] *= pace;
                    _brake[lane][i] *= pace;
                }
            }
        }
    }

    private float SpeedAt(float station)
    {
        if (Range <= 1e-3f && TargetRange <= 1e-3f)
            return Interpolate(_speed[0], station);

        // Off the free lane: plan the blend actually in force over a window,
        // ending on the slower of the pure lanes it lies between. The window
        // starts at the car (AnchorBlendWindow) and every look ahead reads
        // the same one. Started at whichever station was asked about, it was
        // planned again for every look in a step, three or four times over,
        // and a lap held on a side lane took four times as long to drive.
        int start = (int)MathF.Floor(station);
        if (start < _windowStart || start > _windowStart + _windowSpeed.Length - 2 ||
            MathF.Abs(Range - _windowRange) > 0.02f || MathF.Abs(Side - _windowSide) > 0.02f)
            PlanBlendWindow(start);
        float local = station - _windowStart;
        int i0 = Math.Clamp((int)MathF.Floor(local), 0, _windowSpeed.Length - 2);
        float u = Math.Clamp(local - i0, 0f, 1f);
        float planned = _windowSpeed[i0] + (_windowSpeed[i0 + 1] - _windowSpeed[i0]) * u;
        // A blend that is still moving is planned at both ends of its travel.
        return MathF.Min(planned, BlendTargetCeiling(station));
    }

    /// <summary>The blend's window, planned from the car's own station once a station.</summary>
    private void AnchorBlendWindow(float station)
    {
        if (Range <= 1e-3f && TargetRange <= 1e-3f)
            return;
        int start = (int)MathF.Floor(station);
        if (start != _windowStart || MathF.Abs(Range - _windowRange) > 0.02f || MathF.Abs(Side - _windowSide) > 0.02f)
            PlanBlendWindow(start);
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
            _leadGuess[k] = WindowGuess(i);
        }
        LeadCurvatures(_windowCurvature, _leadGuess, n);

        int end = _lanes.Wrap(start + n - 1);
        int sideLane = Side >= 0f ? (int)LaneId.Left : (int)LaneId.Right;
        float terminal = MathF.Min(_speed[0][end], _speed[sideLane][end]);
        LaneSpeedPlanner.PlanWindow(_envelope, _windowCurvature, _windowStep, _windowFactor, _windowBrake, _windowRoad, _planShares, terminal, _windowSpeed);
        _windowStillSpeed = _windowSpeed[0];
        if (Gains.FollowLaneMoves && PlanMoveWindow(start))
            LaneSpeedPlanner.PlanWindow(_envelope, _windowCurvature, _windowStep, _windowFactor, _windowBrake, _windowRoad, _planShares, terminal, _windowSpeed);
        _windowStart = start;
        _windowRange = Range;
        _windowSide = Side;
    }

    /// <summary>
    /// The window as the move under way will drive it: the blend carried on
    /// toward its target at the speeds just planned, station by station, so
    /// the window's line is the S the car will trace and its corners are
    /// that S's. Planned on the blend where it stood, a move begun in a
    /// corner was driven at the speed of a line with none of the move's
    /// own bend in it, and at Monaco's swimming pool the car, its tyres
    /// already full, ran two metres wide of the S. False if nothing moves.
    /// </summary>
    private bool PlanMoveWindow(int start)
    {
        float targetRange = Math.Clamp(TargetRange, 0f, 1f), targetSide = Math.Clamp(TargetSide, -1f, 1f);
        if (_rangeVelocity == 0f && _sideVelocity == 0f &&
            MathF.Abs(targetRange - Range) < 1e-4f && MathF.Abs(targetSide - Side) < 1e-4f)
            return false;
        int n = _windowSpeed.Length;
        int chord = CurvatureChordStations;
        if (_moveOffset.Length != n + 2 * chord)
            _moveOffset = new float[n + 2 * chord];
        float range = Range, side = Side, rangeVelocity = _rangeVelocity, sideVelocity = _sideVelocity;
        float rangeAcceleration = Gains.EaseLaneMoves ? RangeAcceleration : 0f;
        float sideAcceleration = Gains.EaseLaneMoves ? SideAcceleration : 0f;
        // Behind the car the line is where the blend stands.
        for (int k = 0; k < chord; k++)
            _moveOffset[k] = _lanes.BlendedOffset(start - chord + k, range, side);
        for (int k = 0; k < n + chord; k++)
        {
            int i = start + k;
            _moveOffset[chord + k] = _lanes.BlendedOffset(i, range, side);
            float speed = MathF.Max(_windowSpeed[Math.Min(k, n - 1)], 5f);
            float dt = _lanes.Spacing / speed;
            Ease(ref range, ref rangeVelocity, targetRange, RangeRate * _moveScale, rangeAcceleration, dt);
            Ease(ref side, ref sideVelocity, targetSide, SideRate * _moveScale, sideAcceleration, dt);
            if (k < n)
            {
                _windowFactor[k] = FactorFor(i, range, side);
                _windowBrake[k] = BlendOf(lane => _brake[(int)lane], i, range, side);
            }
        }
        for (int k = 0; k < n; k++)
        {
            int i = start + k;
            int at = chord + k;
            Vector2 behind = _lanes.Position(i - chord, _moveOffset[at - chord]);
            Vector2 here = _lanes.Position(i, _moveOffset[at]);
            Vector2 ahead = _lanes.Position(i + chord, _moveOffset[at + chord]);
            _windowCurvature[k] = LaneSet.Curvature(behind, here, ahead);
            _windowRoad[k] = RoadAt(i, _moveOffset[at]);
            _windowStep[k] = (_lanes.Position(i + 1, _moveOffset[at + 1]) - here).Length();
            _leadGuess[k] = MathF.Max(_windowSpeed[k], 1f);
        }
        LeadCurvatures(_windowCurvature, _leadGuess, n);
        return true;
    }

    private float[] _moveOffset = [];

    private float[] _leadGuess = [];
    private float[] _leadBuffer = [];

    private float WindowGuess(int station)
    {
        int i = _lanes.Wrap(station);
        float free = _speed[0][i];
        return free > 1f ? free : 40f;
    }

    /// <summary>
    /// The curvature the plan corners on: at each station the sharpest of the
    /// road from there to as far as the car covers in its turn lag, the time
    /// the hands steer ahead by (TurnLag). The wheel is turned now for the
    /// road that far on, at the speed the car has now, so into a corner that
    /// tightens the tyres are asked for that much more than the road under
    /// the car alone would ask. Planned on the road under the car, the plan
    /// had nothing over for it: braking into Zandvoort's Scheivlak on the
    /// left lane the plan left the tyres 26 m/s² of corner and the hands
    /// asked 32, the wheel stood at the axles' limit, and the car ran wide
    /// and off. Out of a corner the road ahead is straighter and nothing
    /// changes; reading the speed a moment ahead instead, as the pedals
    /// once did, braked earlier for every corner alike.
    /// </summary>
    private void LeadCurvatures(float[] curvature, float[] speedGuess, int n)
    {
        if (!Gains.PlanTurnLead)
            return;
        Array.Copy(curvature, _leadBuffer, n);
        for (int k = 0; k < n; k++)
        {
            float v = speedGuess[k];
            float seconds = Gains.CurvaturePreviewSeconds ?? _turnLag.Seconds(v);
            int lead = (int)MathF.Ceiling(MathF.Max(0f, v) * seconds / _lanes.Spacing);
            float sharpest = _leadBuffer[k];
            for (int j = 1; j <= lead && k + j < n; j++)
                if (MathF.Abs(_leadBuffer[k + j]) > MathF.Abs(sharpest))
                    sharpest = _leadBuffer[k + j];
            curvature[k] = sharpest;
        }
    }

    /// <summary>The window's speed at its start with the blend held where it stands.</summary>
    private float _windowStillSpeed;

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

    /// <summary>How close to full drive's speed the plan must ask for to be driven flat out.</summary>
    private const float FlatOutToleranceMps = 0.3f;

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
        // The plan is for the racing surface, all lap. What the wheels are
        // standing on this instant (a kerb, the day's rubber, the asphalt)
        // is not the road ahead; the road's grip comes in once, in ReadGrip.
        target.SurfaceGrip = 1f;
    }
}
