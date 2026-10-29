using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Saves;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>Follow gains. Steering is in wheel-angle terms, then turned back into curvature.</summary>
public sealed record LaneDriverGains
{
    /// <summary>
    /// What the notes learned of the circuit beforehand: the steering added
    /// station by station, the corner, braking and traction factors on the
    /// envelope, and the circuit's own tyre and energy rungs. Off, the plan
    /// is the envelope's alone and the hands the line's: the notes were
    /// learned on another car and other hands, the steering drove it into
    /// the inside walls and the corner factors asked for more than the
    /// tyres had. What a driver learns as it drives is apart from these.
    /// </summary>
    public bool UseNotes { get; init; }

    /// <summary>
    /// Feedback is pure pursuit in the path's frame: aim at a point this many
    /// seconds of travel ahead, so the correction's lateral acceleration does
    /// not grow with the square of the speed.
    /// </summary>
    public float PursuitSeconds { get; init; } = 0.3f;
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

    /// <summary>
    /// The rear going: the share of full throttle kept at the worst of a
    /// slide, by a driver with no feel and by one with all of it. Lifting
    /// off all at once throws the weight forward and the rear further out;
    /// a driver who can feel it keeps a little on to hold the rear down.
    /// </summary>
    public float CatchThrottleKeepNoFeel { get; init; } = 0f;
    public float CatchThrottleKeepFullFeel { get; init; } = 0.35f;

    /// <summary>The rear going under braking: the share of the brakes let off at the worst of it, so a locked rear rolls again.</summary>
    public float CatchBrakeRelease { get; init; } = 0.6f;

    /// <summary>After a catch, back to full throttle over this long, s: squeezed by a driver with feel, crept by one without.</summary>
    public float ThrottleReturnSecondsNoFeel { get; init; } = 1.0f;
    public float ThrottleReturnSecondsFullFeel { get; init; } = 0.35f;

    /// <summary>
    /// The front going: how far the wheel is unwound toward the turn the
    /// car is actually making, and how much of the throttle comes off, at
    /// the worst of it. More lock on a front that has let go only scrubs.
    /// </summary>
    public float UnwindShare { get; init; } = 0.6f;
    public float UnderLift { get; init; } = 0.8f;

    /// <summary>
    /// After a spin, handed back by the physics: crept back onto the lane at
    /// no more than this, m/s, and at a walking pace while still facing more
    /// than about thirty degrees off the road; back on it within these.
    /// </summary>
    public float RejoinSpeed { get; init; } = 25f;
    public float RejoinTurnSpeed { get; init; } = 6f;
    public float RejoinFacingRadians { get; init; } = 0.5f;
    public float RejoinLateralMeters { get; init; } = 1.5f;
    public float RejoinHeadingRadians { get; init; } = 0.15f;

    /// <summary>
    /// Off the road or facing well off it, whether or not the physics called
    /// a spin: the angle the road is crossed back onto at most, and the
    /// distance over which the lane is closed at it; the speed the edge is
    /// crossed at, which one side on grass and one on tarmac will take; and
    /// the gap in time to the next car coming that must be there first.
    /// </summary>
    public float RejoinAngleRadians { get; init; } = 0.2f;
    public float RejoinCloseMeters { get; init; } = 15f;
    public float RejoinEdgeSpeed { get; init; } = 12f;
    public float RejoinEdgeMeters { get; init; } = 5f;
    public float RejoinGapSeconds { get; init; } = 3f;
    public float RejoinLookBehindMeters { get; init; } = 300f;
    public float RejoinWrongWayRadians { get; init; } = 1f;
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
    private TyreStyle[]? _tyreStyles;

    /// <summary>The tyre rungs' styles: this circuit's where it has them, the general table where not.</summary>
    private TyreStyle[] TyreStyles => _tyreStyles ??= (Gains.UseNotes ? _file.Tyres?.Table() : null) ?? Gains.TyreStyleTable;
    // The car, known only by what it says it has: an envelope to plan
    // against, tyres, the feel of its grip, and stores and buttons where it
    // has them (Needs).
    private readonly ICar _model;
    private readonly ICarEnvelope _carEnvelope;
    private readonly ICarTires _tyres;
    private readonly ICarGripFeel _feel;
    private readonly ICarStores? _stores;
    private readonly ICarButtons? _buttons;
    private readonly AxleLayout _layout;
    private readonly ICarMotion _motion;
    private readonly float _maxDrive;
    private readonly float _maxBrake;
    private readonly TurnLag _turnLag;
    private readonly CarState _plan = new();
    private readonly float[][] _curvature = new float[LaneSet.LaneCount][];
    private readonly LaneRoad[][] _road = new LaneRoad[LaneSet.LaneCount][];
    private DriverTrackView? _trackView;
    private readonly float[][] _stepLength = new float[LaneSet.LaneCount][];
    private readonly float[][] _speed = new float[LaneSet.LaneCount][];
    private LaneSet _lanes = null!;
    private LaneEnvelope _envelope;
    private readonly Dictionary<string, float> _referenceLateral = new();
    private LanePerformance? _performance;
    private readonly float[][] _corner = new float[LaneSet.LaneCount][];
    private readonly float[][] _brake = new float[LaneSet.LaneCount][];
    private readonly bool[] _planned = new bool[LaneSet.LaneCount];
    private float _sincePlan;

    /// <summary>
    /// How often the speeds ahead are planned again. The tyres, the pack and
    /// the road change over seconds, not steps, and planning five hundred
    /// metres of up to three lanes every step was most of what a car cost:
    /// eight cars took ten milliseconds a step. Each car keeps its own beat,
    /// offset from the others by its name, so they do not all plan in one
    /// frame; a lane needed before it has been planned is planned at once.
    /// </summary>
    public float PlanIntervalSeconds { get; init; } = 0.1f;
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

    /// <summary>What the driver cannot drive a car without, and what it uses when the car has it.</summary>
    public static readonly CarNeeds CarNeeds = new(
        [typeof(ICarEnvelope), typeof(ICarTires), typeof(ICarGripFeel), typeof(ICarMotion)],
        [typeof(ICarStores), typeof(ICarButtons), typeof(ICarAxles)]);

    public CarNeeds Needs => CarNeeds;

    /// <param name="car">
    /// The kind of car it drives, the driver's own copy to plan with. It
    /// must have what <see cref="CarNeeds"/> requires.
    /// </param>
    public LaneDriver(LaneTrackFile file, ICar car)
    {
        _file = file ?? throw new ArgumentNullException(nameof(file));
        _model = car ?? throw new ArgumentNullException(nameof(car));
        IReadOnlyList<Type> missing = CarNeeds.MissingFrom(_model);
        if (missing.Count > 0)
            throw new ArgumentException(
                $"The lane driver cannot drive a {_model.GetType().Name}: it has no {string.Join(", ", missing.Select(t => t.Name))}.",
                nameof(car));
        _carEnvelope = (ICarEnvelope)_model;
        _tyres = (ICarTires)_model;
        _feel = (ICarGripFeel)_model;
        _stores = _model as ICarStores;
        _buttons = _model as ICarButtons;
        _motion = (ICarMotion)_model;
        // Without two axles to share them between, the brakes and the drive
        // are taken as split evenly, and the driver never moves them.
        _layout = (_model as ICarAxles)?.Layout ?? new AxleLayout(0.5f, 0.5f, false, null);
        _maxDrive = _carEnvelope.Capabilities(new CarState(), CarStrategy.Default).MaxDriveAcceleration;
        _maxBrake = _model.Spec.MaximumBrakeDeceleration;
        _turnLag = TurnLag.Measure(_model);
    }

    public LaneDriverGains Gains { get; init; } = new();

    /// <summary>Whether the driver's ratings shape the drive. Off only for probes of the plan itself.</summary>
    public bool UseAbilities { get; init; } = true;

    public LanePerformance? Performance => _performance;

    /// <summary>
    /// What this driver has learned of this circuit in this car, on top of
    /// the car's notes. Read from <see cref="Save"/> when the session
    /// starts, where the game keeps it; left unset, or learned on another
    /// layout, the driver starts as on a circuit never driven. Only a driver
    /// with ratings learns as it drives, and only when LearnsAsItDrives.
    /// </summary>
    public LaneFamiliarity? Familiarity { get; set; }

    /// <summary>
    /// Where this person's experience of the car here is kept: read when the
    /// session starts, written at the end of every lap. None, and what is
    /// learned lasts the session.
    /// </summary>
    public ISaveSlot? Save { get; init; }

    // What the driver keeps in its save slot.
    private const string FamiliarityFile = "familiarity.json";

    /// <summary>Off for the offline learner: the car's notes are learned on the car, not a driver's nerves.</summary>
    public bool LearnsAsItDrives { get; init; } = true;

    // The familiarity in force: null when the driver does not learn as it drives.
    private LaneFamiliarity? _familiar;
    private float _adaptability = 1f;
    private float _tyreHeed = 1f;
    private int _familiarSegment = -1;
    private bool _scaredInSegment;
    private float _segmentPeakUse;
    private bool _segmentHeldBack;
    private int _familiarLap = int.MinValue;

    /// <summary>How long before each braking zone the driver lifts, seconds: the energy rung's.</summary>
    public float CoastSeconds { get; private set; }

    /// <summary>Holds the lift at this instead of the rung's, for probes that sweep it.</summary>
    public float? CoastOverride { get; set; }

    /// <summary>Most the driver will lift early: past it the car looks like it is cruising.</summary>
    public float MaximumCoastSeconds { get; init; } = 1.5f;

    private void ManageEnergy(RaceCarSnapshot car)
    {
        float[] table = _energyTable ??= (Gains.UseNotes ? _file.Energy?.LiftSeconds() : null) ?? Gains.EnergyRungLiftSeconds;
        float asked = (_model.Ladders.Count > 0 ? _model.Ladders[0].RungsUp(car.Strategy.Setting(0, _model.Ladders[0])) : 0f) + (_performance?.EnergyRungDrift ?? 0f);
        float lift = CoastOverride ?? (table.Length == 0 ? 0f : Rung(table, asked));
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

    /// <summary>
    /// A speed the driver is not to exceed, m/s, or null for none: set by
    /// whatever drives this driver from outside (traffic, a pit lane, a
    /// safety car). The plan is otherwise untouched; the brakes come on only
    /// above it, as they do above the plan.
    /// </summary>
    public float? SpeedCap { get; set; }

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
        for (int lane = 0; lane < LaneSet.LaneCount; lane++)
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
        _referenceLateral.Clear();
        _performance = UseAbilities ? new LanePerformance(context.Profile) : null;
        _performance?.Update(0, context.Car.TrackS, 0f);
        _adaptability = Math.Clamp(context.Profile.Abilities.Adaptability * 0.01f, 0f, 1f);
        _tyreHeed = Math.Clamp(context.Profile.Abilities.TireManagement * 0.01f, 0f, 1f);
        _familiar = null;
        if (_performance is not null && LearnsAsItDrives)
        {
            if (Familiarity is null && Save?.Read(FamiliarityFile) is { } kept)
                Familiarity = LaneFamiliarity.FromJson(kept);
            if (Familiarity is null || !Familiarity.Fits(context.Track.LengthMeters))
                Familiarity = LaneFamiliarity.NewCircuit(context.Track.LengthMeters, _adaptability);
            _familiar = Familiarity;
        }
        _familiarSegment = -1;
        _scaredInSegment = false;
        _familiarLap = int.MinValue;
        PlanAhead(context.Car, context.Environment.SurfaceGripScalar, 0f);
        _sincePlan = PlanIntervalSeconds * Phase(context.Car.Id);
    }

    /// <summary>
    /// What this car corners at 40 m/s on new tyres of this compound at
    /// their best temperature on a clean road: the full grip every grip
    /// level is a share of. Each compound's own best is its full grip, so a
    /// harder tyre at its best drives on the notes as the softer one did at
    /// its best, and nothing is learned again for it.
    /// </summary>
    private float ReferenceLateral(string? compound)
    {
        string key = compound ?? "";
        if (_referenceLateral.TryGetValue(key, out float known))
            return known;
        var ideal = new CarState();
        TireReading fresh = _tyres.Tire(ideal, WheelId.FrontLeft);
        _tyres.Assume(ideal, compound, [fresh, fresh, fresh, fresh]);
        TireWindow window = _tyres.Tire(ideal, WheelId.FrontLeft).Window;
        float temp = 0.5f * (window.IdealSurfaceLowC + window.IdealSurfaceHighC);
        var best = new TireReading(temp, temp, 0f, 0f, 1f);
        _tyres.Assume(ideal, compound, [best, best, best, best]);
        float reference = MathF.Max(1f, LaneEnvelope.From(ideal, _carEnvelope, _feel, CarStrategy.Default).Lateral(40f));
        _referenceLateral[key] = reference;
        return reference;
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
            _performance.Rung = DrivenRung(car);
            _performance.Update(lap, car.TrackS, dt);
            Familiarise(car, lap);
        }
        _sincePlan += dt;
        if (_sincePlan >= PlanIntervalSeconds || Enumerable.Range(0, LaneSet.LaneCount).Any(lane => Reads(lane) && !_planned[lane]))
        {
            PlanAhead(car, context.Environment.SurfaceGripScalar, _sincePlan);
            _sincePlan = 0f;
        }
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
        // Where the body points, against the road: after a spin the car may
        // be crawling or facing across it, and the way it moves says little.
        float facing = MathHelper.NormalizeAngle(pathHeading - car.HeadingRadians);
        UpdateRejoin(context, car, leftError, facing);
        float turnIn = _turnLag.Seconds(speed);
        if (WindsOn(station, speed))
            turnIn = MathF.Max(turnIn, _turnLag.Seconds(speed, PlannedBraking(station, speed) / MathF.Max(_envelope.Brake(speed), 1f)));
        float previewStation = station + MathF.Max(0f, speed) * turnIn / _lanes.Spacing;
        float moveCurvature = _moveCurvature = -MoveAt((int)MathF.Round(previewStation), turnIn).Acceleration /
                              MathF.Max(speed * speed, 25f);
        float feedforward = BlendedCurvature(previewStation) + BlendedSteer(previewStation) + moveCurvature;
        float here = BlendedCurvature(station);
        float pursuit = MathF.Max(Gains.MinimumPursuitMeters, MathF.Max(speed, 0f) * Gains.PursuitSeconds);
        // What the axles will hold at the braking being done right now.
        float braking = Math.Clamp(-MotionOf(car).StepLongitudinalAccel / MathF.Max(_envelope.Brake(speed), 1f), 0f, 1f);
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
        GripUse axles = _axles;
        if (Gains.CapFromAxles && axles.FrontGripAccel > 0.5f && axles.RearGripAccel > 0.5f && speed > 8f)
        {
            // Each axle's whole circle, not what the brakes or the motor leave
            // of it this instant: the combined-grip limiter trims either of
            // them the moment the wheel asks that axle for corner, so the
            // corner is there to be had. Taking this step's braking out of it
            // held the wheel short through every corner braked into, and the
            // car ran wide with the grip unasked for.
            float frontShare = Math.Clamp(_layout.FrontWeightShare, 0.05f, 0.95f);
            float front = axles.FrontGripAccel / frontShare;
            float rear = axles.RearGripAccel / (1f - frontShare);
            // A driver with less feel leaves the tyre nearer its peak.
            // A mistake is a misreading of the grip, so the hands believe it too.
            float misread = _performance?.CornerOverstep ?? 0f;
            float capShare = _style is { } capStyle
                ? (_feel.SustainablePeakShare * capStyle.Cap + (_performance?.LimitOveruse ?? 0f)) * (1f + 2f * misread)
                : ((_feel.SustainablePeakShare) + (_performance?.LimitOveruse ?? 0f)) * _tyreAuthority * (1f + 2f * misread);
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
        float frontGone = 0f;
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
            (slide, frontGone) = Recover(car, ref curvature, dt);
        }
        // Facing well off the road after a spin: round on full lock at a
        // walk, whichever way is shorter; the line's own steering reads the
        // way the car moves, and a car crawling backwards reads it wrong.
        if (_phase == SlidePhase.Rejoin)
        {
            // And the corner the throttle leaves room for is this one: the
            // lane's own, twenty metres off it, wanted the whole tyre and
            // left the car crawling on no throttle at all.
            curvature = RejoinCurvature(car, pathHeading, facing, leftError, feedforward);
            wanted = curvature;
        }

        // Longitudinal: follow the table with its own slope as feedforward,
        // and pay the drag the axles will be charged for.
        // Consistency: the brake marker, read a few metres early or late,
        // and the speed this segment, a little over or under.
        float marker = (_performance?.BrakeMarkerMeters ?? 0f) / _lanes.Spacing;
        // Focus: a segment the driver has got wrong is taken faster
        // than the plan, not merely allowed more wheel.
        // And the share of the limit it is using, margin and wander, taken
        // as speed: the grip a corner asks goes with its square.
        float speedScale = (1f + (_performance?.SpeedError ?? 0f) + (_performance?.CornerOverstep ?? 0f)) *
                           MathF.Sqrt(MathF.Max(0.5f, 1f + (_performance?.LimitOveruse ?? 0f)));
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
        // The foot's beat onto the brakes is started half of it early.
        float footLead = _performance is null ? 0f : 0.5f * _maxBrake / FootBrakeRate;
        float preview = MathF.Max(dt, 0f) + slowing * ((_performance?.ExtraBrakePreviewSeconds ?? 0f) + footLead);
        float target = SpeedAt(station + MathF.Max(0f, speed) * preview / _lanes.Spacing);
        float ahead = SpeedAt(station + (MathF.Max(0f, speed) * preview + 5f) / _lanes.Spacing);
        target *= speedScale;
        ahead *= speedScale;
        float uncapped = target;
        if (SpeedCap is { } cap)
        {
            target = MathF.Min(target, cap);
            ahead = MathF.Min(ahead, cap);
        }
        if (_phase == SlidePhase.Rejoin)
        {
            float creep = RejoinSpeedNow(context, car, facing);
            target = MathF.Min(target, creep);
            ahead = MathF.Min(ahead, creep);
        }
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
        // Held under the plan by a cap (a car in the way, a spin to come back
        // from), the speed is tracked, not floored: stopped and waiting, the
        // flat-out speed a step ahead is next to nothing, and every step that
        // read as the plan being flat out the pedal went to the floor.
        bool capped = target < uncapped - 0.1f;
        if (!capped && target >= flatOut - FlatOutToleranceMps)
            accel = MathF.Max(accel, _maxDrive);

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
        GripUse tyres = _axles;
        // The pedal's whole travel: the motor takes what it can of it.
        float driveLimit = _maxDrive;
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
            * (1f + 2f * (_performance?.CornerOverstep ?? 0f))
            * TractionHere((int)MathF.Round(station));
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
        float motorShare = Math.Clamp(_layout.FrontDriveShare, 0f, 1f);
        float rearRoom = Room(tyres.RearGripAccel, 1f - _layout.FrontWeightShare, tyres.RearLateral);
        float frontRoom = Room(tyres.FrontGripAccel, _layout.FrontWeightShare, tyres.FrontLateral);
        float tractionRoom;
        if (_layout.DriveFollowsGrip)
        {
            // The car shares drive between its motors by the grip each axle
            // has left, so the room is the two together, each held to what
            // its own motor can give.
            float drive = _maxDrive;
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
        accel = Math.Clamp(accel, -_maxBrake, MathF.Max(0f, driveLimit));
        // Coming back, the pedals ask no more than the ground under the
        // driven wheels will take: on grass a fraction of the tarmac's.
        if (_phase == SlidePhase.Rejoin && Ground(car) < OnTarmac)
        {
            float pedals = RejoinPedalShare * RejoinGripAccel * Ground(car);
            accel = Math.Clamp(accel, -pedals, pedals);
        }

        // Brakes: eased the moment either axle is near the edge of its circle,
        // which is what the combined-grip limiter does at the Normal rung and
        // stops doing at Attack.
        if (accel < 0f)
        {
            float front = Combined(tyres.FrontLateral, tyres.FrontLongitudinal);
            float rear = Combined(tyres.RearLateral, tyres.RearLongitudinal);
            float feel = Gains.BrakeFeelUse * (_style?.Cap ?? _tyreAuthority);
            float over = MathF.Max(front, rear) - MathF.Min(0.995f, feel) - 2f * (_performance?.CornerOverstep ?? 0f);
            if (over > 0f)
                accel *= Math.Clamp(1f - over / Gains.BrakeFeelBand, Gains.BrakeFeelFloor, 1f);
        }
        if (_performance is null)
        {
            if (slide > 0f)
                accel = MathF.Min(accel, accel * (1f - slide));
        }
        else
        {
            accel = SlidePedals(accel, slide, frontGone, dt);
            accel = ExitThrottle(accel, dt);
            // A foot takes a beat to press a pedal down, the same beat for
            // every driver: the brakes to full in about a tenth, the
            // throttle in about a quarter. Coming off either is at once.
            if (accel < 0f && accel < _lastAccel)
                accel = MathF.Max(accel, MathF.Min(_lastAccel, 0f) - FootBrakeRate * MathF.Max(dt, 0f));
            else if (accel > 0f && accel > _lastAccel)
                accel = MathF.Min(accel, MathF.Max(_lastAccel, 0f) + FootThrottleRate * MathF.Max(dt, 0f));
        }
        _lastAccel = accel;

        LastTelemetry = new LaneDriverTelemetry(
            -leftError, headingError, target, feedforward, curvature, accel, Range, Side, wanted, curvature >= 0f ? upper : lower);
        // The wheels turned for that curvature as a driver who knew the car
        // would turn them (SteeringEstimate); the hands are this driver's.
        // The lock for that curvature as a driver who knew the car exactly
        // would put on (SteeringEstimate), misjudged by this driver's hands;
        // the line takes up what is left.
        float steer = LockFor(car, curvature) * (1f + (_performance?.SteeringRead ?? 0f));
        return new DriverInput(steer, accel)
        {
            Axles = new AxleBias(BrakeBias(tyres, accel), 0f),
            TyreWearExcess = _performance?.TyreWearExcess ?? 0f
        };
    }

    // What the driver feels of the two axles this step.
    private GripUse _axles;

    /// <summary>
    /// The estimate's lock (SteeringEstimate.AngleFor), with the slip angles'
    /// part faded in by how much of the corner asked for the car is already
    /// cornering, not by how much of all its grip: read against all of it, a
    /// gentle bend taken fast on a car pressed down by its wings never
    /// counted as loaded, and half the lock it wanted was left off.
    /// </summary>
    private float LockFor(RaceCarSnapshot car, float curvature)
    {
        const float epsilon = 1e-4f;
        SteeringEstimate estimate = car.Steering;
        float wheelBase = MathF.Max(estimate.WheelBaseMeters, epsilon);
        float frontArm = MathF.Max(estimate.FrontArmMeters, epsilon);
        float rearArm = MathF.Max(estimate.RearArmMeters, epsilon);
        float frontCapacity = MathF.Max(estimate.FrontGripAccel, epsilon);
        float rearCapacity = MathF.Max(estimate.RearGripAccel, epsilon);
        float holdable = SteeringEstimate.InversionShare * MathF.Min(frontCapacity * wheelBase / rearArm, rearCapacity * wheelBase / frontArm);
        float asked = estimate.LateralDemand(curvature);
        float corner = Math.Clamp(asked, -holdable, holdable);
        float front = rearArm / wheelBase * corner;
        float rear = frontArm / wheelBase * corner;
        float slip = estimate.FrontPeakSlipAngleRatio * _feel.SlipAngleAtShare(front / frontCapacity) -
                     _feel.SlipAngleAtShare(rear / rearCapacity);
        float loaded = MathF.Max(MathF.Abs(asked), epsilon);
        float load = Math.Clamp(MathF.Abs(estimate.ActualLateralAccel) / loaded, 0f, 1f);
        float angle = MathF.Atan(wheelBase * curvature) + slip * load +
                      estimate.CurvatureFeedbackGain * wheelBase * (curvature - estimate.ActualCurvature);
        float maximum = MathF.Max(estimate.MaxSteerAngleRadians, 0f);
        return Math.Clamp(angle, -maximum, maximum);
    }


    /// <summary>
    /// The braking the plan has over the turn-in to come, m/s², not the foot's
    /// now: into a slow corner the brakes come off half their pressure in a
    /// tenth of a second as the wheel goes on, and read off the pedal at the
    /// turn-in the hands took the car for one still braking hard, read the
    /// corner too early and ran wide out of it.
    /// </summary>
    private float PlannedBraking(float station, float speed)
    {
        float reach = MathF.Max(1f, MathF.Max(0f, speed) * _turnLag.Seconds(speed, 1f));
        float ahead = SpeedAt(station + reach / _lanes.Spacing);
        return MathF.Max(0f, (speed * speed - ahead * ahead) / (2f * reach));
    }

    /// <summary>
    /// Whether the wheel is being wound on towards the corner ahead, within
    /// the longest turn-in the car has: from the straight, from less of it,
    /// or from the other way. Unwinding, the tyres come off their peak onto
    /// the steep part of their curve and the car answers as quickly as the
    /// small step has it; read as far ahead as a turn-in, the wheel came off
    /// before the corner opened and the car ran wide onto the wall out of it.
    /// </summary>
    private bool WindsOn(float station, float speed)
    {
        float here = BlendedCurvature(station);
        float reach = MathF.Max(0f, speed) * _turnLag.Seconds(speed, 1f) / _lanes.Spacing;
        for (float d = 0.5f; d <= reach + 1e-3f; d += 0.5f)
        {
            float k = BlendedCurvature(station + d);
            if (k * (k - here) > 0f)
                return true;
        }
        return false;
    }

    /// <summary>
    /// The axles as the driver feels them (<see cref="ICarGripFeel"/>). A car
    /// that does not say reads as no grip measured, and every use of them
    /// falls back to the envelope.
    /// </summary>
    private static GripUse AxlesOf(RaceCarSnapshot car) => car.GripUse ?? default;

    private static CarMotion MotionOf(RaceCarSnapshot car) => car.Motion ?? default;

    private enum SlidePhase { None, Catch, Spun, Rejoin }

    private SlidePhase _phase;
    private float _perceivedSlip;
    private float _perceivedYawError;
    private float _perceivedUnder;
    private float _pushingFor;
    private const float FeltSeconds = 0.04f;
    private float _slidingFor;
    private float _throttleReturn = 1f;

    /// <summary>Whether the driver is catching a slide right now.</summary>
    public bool IsRecovering => _phase == SlidePhase.Catch;

    /// <summary>Whether the car is spun, the physics playing it out, or crawling back onto its lane after.</summary>
    public bool IsSpun => _phase == SlidePhase.Spun;
    public bool IsRejoining => _phase == SlidePhase.Rejoin;

    /// <summary>Slides caught (or not) since the start, for probes.</summary>
    public int RecoveryEvents { get; private set; }

    /// <summary>
    /// A spin is the physics' from the moment it calls one: the wheel and
    /// pedals do nothing until it hands the car back. Off the road, or
    /// stopped facing well off it, is the driver's to sort out whether or
    /// not the physics called a spin, once any slide being caught is over.
    /// Either way the driver comes back slowly and is back when on the lane
    /// and pointing down it.
    /// </summary>
    private void UpdateRejoin(in DriverContext context, RaceCarSnapshot car, float leftError, float facing)
    {
        _outside = OutsideEdgeMeters(context, car);
        if (MotionOf(car).Spinning)
        {
            if (_phase != SlidePhase.Spun)
                Scared(car.TrackS, big: true);
            _phase = SlidePhase.Spun;
            _slidingFor = 0f;
            return;
        }
        if (_phase == SlidePhase.Spun)
            _phase = SlidePhase.Rejoin;
        if (_phase == SlidePhase.None &&
            (_outside > 0.5f ||
             (MathF.Abs(facing) > Gains.RejoinWrongWayRadians && car.SpeedMetersPerSecond < Gains.RejoinSpeed)))
        {
            _phase = SlidePhase.Rejoin;
            Scared(car.TrackS, big: true);
        }
        // Back on the road and pointing down it, the driving is the
        // ordinary kind again, whatever the speed: holding a car that has
        // come back on at forty metres a second to a crawl only threw it
        // round under the brakes.
        if (_phase == SlidePhase.Rejoin &&
            _outside < 0f &&
            (MathF.Abs(facing) < Gains.RejoinHeadingRadians ||
             (MathF.Abs(facing) < Gains.RejoinFacingRadians && car.SpeedMetersPerSecond > Gains.RejoinSpeed) ||
             MathF.Abs(leftError) < Gains.RejoinLateralMeters && MathF.Abs(facing) < Gains.RejoinFacingRadians))
        {
            _phase = SlidePhase.None;
            _throttleReturn = 0f;
        }
    }

    /// <summary>How far the car is outside the road and its kerb, metres; negative on it.</summary>
    private static float OutsideEdgeMeters(in DriverContext context, RaceCarSnapshot car) =>
        MathF.Abs(car.TrackD) - context.Track.Sample(car.TrackS).HalfWidth - SurfaceGrip.KerbWidthMeters;

    /// <summary>
    /// The wheel while coming back. Facing well off the road: round on full
    /// lock at a walk, the shorter way unless that sweeps the nose away from
    /// the road, toward whatever is out there; off the road the other way
    /// round is the one with room. Facing down it: onto the lane at a
    /// shallow angle, not straight at it, and no more lock than the ground
    /// under the tyres will take.
    /// </summary>
    private float RejoinCurvature(RaceCarSnapshot car, float pathHeading, float facing, float leftError, float feedforward)
    {
        float maxLock = car.Capabilities.MaxCurvatureRequest;
        // Round on full lock only at a crawl: still going quickly, the car
        // is slowed first and turned no more than the ground takes.
        if (MathF.Abs(facing) > Gains.RejoinFacingRadians && car.SpeedMetersPerSecond < RejoinCatchSpeed)
        {
            float turn = MathF.Sign(facing);
            if (_outside > 0f)
            {
                // The way off the road, as an angle from the nose: if the
                // shorter way round reaches it first, go the long way.
                // The road's positive side is clockwise of its heading.
                float awayHeading = pathHeading - MathF.Sign(car.TrackD) * MathF.PI * 0.5f;
                float away = MathHelper.NormalizeAngle(awayHeading - car.HeadingRadians);
                if (MathF.Sign(away) == turn && MathF.Abs(away) < MathF.Abs(facing))
                    turn = -turn;
            }
            return turn * maxLock;
        }
        float across = Math.Clamp(MathF.Atan(leftError / Gains.RejoinCloseMeters), -Gains.RejoinAngleRadians, Gains.RejoinAngleRadians);
        float heading = car.SpeedMetersPerSecond > 3f ? car.VelocityHeadingRadians : car.HeadingRadians;
        float pursuit = MathF.Max(Gains.MinimumPursuitMeters, car.SpeedMetersPerSecond * Gains.PursuitSeconds);
        float wanted = feedforward + 2f * MathF.Sin(MathHelper.NormalizeAngle(pathHeading - across - heading)) / pursuit;
        float speed = MathF.Max(car.SpeedMetersPerSecond, 3f);
        float ground = Ground(car);
        float held = ground < OnTarmac ? RejoinGripAccel * ground / (speed * speed) : maxLock;
        return Math.Clamp(wanted, -MathF.Min(maxLock, held), MathF.Min(maxLock, held));
    }

    /// <summary>The grip of the ground under the worst-placed wheel, 1 on clean tarmac.</summary>
    private static float Ground(RaceCarSnapshot car) =>
        car.Tires.IsDefaultOrEmpty ? 1f : car.Tires.Min(t => t.SurfaceGrip);

    // A wheel on less than this is off the tarmac.
    private const float OnTarmac = 0.95f;

    // What the tyres give at a crawl on ground of full grip, m/s², and the
    // share of it a driver coming back off it asks: all of it to turn, half
    // of it on the pedals, since one wheel on grass and one on tarmac pull
    // the car round under power or brakes.
    private const float RejoinGripAccel = 12f;
    private const float RejoinPedalShare = 0.5f;
    private const float RejoinCatchSpeed = 10f;
    private float _outside;

    /// <summary>
    /// The speed while coming back: a walk while turning round; off the
    /// road, the speed that ground will take, and slow enough near the edge
    /// for one side on grass and one on tarmac; at the edge, a stop while a
    /// car is coming within the gap. Cars a long way back are no reason to
    /// wait: they are gone by the time they matter, or not yet there.
    /// </summary>
    private float RejoinSpeedNow(in DriverContext context, RaceCarSnapshot car, float facing)
    {
        if (MathF.Abs(facing) > Gains.RejoinFacingRadians)
            return Gains.RejoinTurnSpeed;
        if (_outside <= 0f)
            return Gains.RejoinSpeed;
        if (_outside > Gains.RejoinEdgeMeters)
            return Gains.RejoinSpeed;
        return TrafficComing(context, car) ? 0f : Gains.RejoinEdgeSpeed;
    }

    /// <summary>A car within the gap in time behind, or still alongside, on the road.</summary>
    private bool TrafficComing(in DriverContext context, RaceCarSnapshot car)
    {
        float length = MathF.Max(car.TrackLengthMeters, 1f);
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            RaceCarSnapshot other = context.Frame[i];
            float behind = ((car.TrackS - other.TrackS) % length + length) % length;
            if (behind > length * 0.5f)
                behind -= length;
            if (behind < -10f || behind > Gains.RejoinLookBehindMeters)
                continue;
            if (behind <= 0f || behind / MathF.Max(other.SpeedMetersPerSecond, 1f) < Gains.RejoinGapSeconds)
                return true;
        }
        return false;
    }

    /// <summary>
    /// Catching a slide, the historical driver's way: felt through the seat,
    /// acted on only after the driver's reaction time, then countersteer on the perceived sideslip and
    /// on the yaw running past the one asked for, with a firmness drawn for
    /// this one catch (Recovery). A front that has let go instead is unwound,
    /// after the same reaction time, toward the turn the car is making. Returns how severe each is, 0..1: the rear
    /// sliding and the front.
    /// </summary>
    private (float Over, float Under) Recover(RaceCarSnapshot car, ref float curvature, float dt)
    {
        // Coming back at a crawl is no slide to catch; coming back at speed,
        // a slide is caught as any other.
        if (_phase == SlidePhase.Spun || (_phase == SlidePhase.Rejoin && car.SpeedMetersPerSecond < RejoinCatchSpeed))
            return (0f, 0f);
        LanePerformance performance = _performance!;
        float speed = car.SpeedMetersPerSecond;
        float sideslip = car.SideslipAngleRadians;
        float yaw = car.YawRateRadiansPerSecond;
        float asked = speed * curvature;
        float yawError = yaw - asked;
        // What the seat says, smoothed as any body smooths it, alike for
        // every driver: how late the hands move after it is Reactions alone.
        float filter = 1f - MathF.Exp(-MathF.Max(0f, dt) / FeltSeconds);
        _perceivedSlip += (sideslip - _perceivedSlip) * filter;
        _perceivedYawError += (yawError - _perceivedYawError) * filter;

        // In this physics two to four degrees of body slip is an ordinary
        // corner; what says the rear is going is the rear sliding, a slip
        // past that, or the car turning faster than it was asked to. Turning
        // slower than asked is the front, not the rear.
        bool overRotating = yawError * MathF.Sign(yaw) > 0f;
        float slipSeverity = Math.Clamp((MathF.Abs(sideslip) - 0.07f) / 0.08f, 0f, 1f);
        float yawSeverity = overRotating
            ? Math.Clamp((MathF.Abs(yawError) - 0.10f) / 0.6f, 0f, 1f) * Math.Clamp(MathF.Abs(sideslip) / 0.06f, 0f, 1f)
            : 0f;
        float rearSeverity = Math.Clamp((_axles.RearSlideSeverity - 0.15f) / 0.6f, 0f, 1f);
        float severity = MathF.Max(slipSeverity, MathF.Max(yawSeverity, rearSeverity));

        // The front gone: asked for a turn, the car makes a good deal less
        // of it, with the front tyres at the end of what they hold and the
        // rear not the reason.
        float under = 0f;
        if (speed > 15f && MathF.Abs(asked) > 0.05f && severity < 0.05f)
        {
            float made = yaw * MathF.Sign(asked);
            float deficit = 1f - made / MathF.Abs(asked);
            under = Math.Clamp((deficit - 0.12f) / 0.3f, 0f, 1f) *
                    Math.Clamp((_axles.FrontLateral - 0.9f) / 0.08f, 0f, 1f);
        }
        _pushingFor = under > 0f ? _pushingFor + dt : 0f;
        _perceivedUnder += ((_pushingFor >= performance.ReactionSeconds ? under : 0f) - _perceivedUnder) * filter;

        if (_phase != SlidePhase.Catch)
        {
            _slidingFor = severity > 0.15f ? _slidingFor + dt : 0f;
            if (_slidingFor >= performance.ReactionSeconds)
            {
                _phase = SlidePhase.Catch;
                RecoveryEvents++;
                performance.BeginRecovery();
                Scared(car.TrackS, big: false);
            }
        }
        else if (severity < 0.05f)
        {
            _phase = SlidePhase.None;
            _slidingFor = 0f;
        }

        if (_phase == SlidePhase.Catch)
        {
            float wheelBase = MathF.Max(car.WheelBaseMeters, 1.5f);
            float yawTerm = overRotating ? _perceivedYawError : 0f;
            float correction = (0.35f * _perceivedSlip / wheelBase -
                                0.25f * yawTerm / MathF.Max(speed, 5f)) * performance.RecoveryGain;
            float limit = car.Capabilities.MaxCurvatureRequest;
            curvature = Math.Clamp(curvature + correction, -limit, limit);
            return (severity, 0f);
        }

        if (_perceivedUnder > 0.01f && yaw * curvature > 0f)
        {
            // Back toward the turn it is making, a little inside it, so the
            // front tyres come back under their peak and bite again.
            float making = yaw / MathF.Max(speed, 5f);
            curvature += (making * 1.05f - curvature) * Gains.UnwindShare * Math.Clamp(_perceivedUnder, 0f, 1f);
        }
        return (0f, Math.Clamp(_perceivedUnder, 0f, 1f));
    }

    /// <summary>
    /// The pedals through a slide. The rear going: off the throttle by how
    /// bad it is, a driver with feel keeping some on, and off the brakes if
    /// it went under braking; then back on the throttle progressively, not
    /// all at once. The front going: off the throttle by how bad it is.
    /// </summary>
    private float SlidePedals(float accel, float over, float under, float dt)
    {
        float feel = _performance?.Catching ?? 1f;
        if (over > 0f)
        {
            float keepAtWorst = Gains.CatchThrottleKeepNoFeel + (Gains.CatchThrottleKeepFullFeel - Gains.CatchThrottleKeepNoFeel) * feel;
            float keep = 1f - over * (1f - keepAtWorst);
            if (accel > 0f)
                accel = MathF.Min(accel, _maxDrive * keep);
            else
                accel *= 1f - Gains.CatchBrakeRelease * over;
            _throttleReturn = MathF.Min(_throttleReturn, keep);
            return accel;
        }
        if (under > 0f && accel > 0f)
            accel *= 1f - Gains.UnderLift * under;
        if (_throttleReturn < 1f)
        {
            float seconds = Gains.ThrottleReturnSecondsNoFeel + (Gains.ThrottleReturnSecondsFullFeel - Gains.ThrottleReturnSecondsNoFeel) * feel;
            _throttleReturn = MathF.Min(1f, _throttleReturn + MathF.Max(0f, dt) / MathF.Max(seconds, 0.05f));
            if (accel > 0f)
                accel = MathF.Min(accel, _maxDrive * _throttleReturn);
        }
        return accel;
    }

    private float _brakeDial = float.NaN;
    private float _lastAccel;
    private bool _brakedSinceThrottle;
    private float _sinceThrottle = float.PositiveInfinity;

    /// <summary>The corner and braking factors the notes give this stretch at the grip believed in; one, the envelope alone, without them.</summary>
    private void NotedFactors(LaneId lane, int start, int count, Span<float> corner, Span<float> brake)
    {
        if (Gains.UseNotes)
        {
            _file.FactorsAtGrip(lane, Grip, start, count, corner, brake);
            return;
        }
        // Station by station from start, round the lap, as the notes are read.
        int n = corner.Length;
        for (int k = 0, i = ((start % n) + n) % n; k < Math.Min(count, n); k++, i = i + 1 == n ? 0 : i + 1)
        {
            corner[i] = 1f;
            brake[i] = 1f;
        }
    }

    /// <summary>
    /// What the notes say of this corner's exit, at the grip the driver
    /// believes in: the share of the general traction to take here, blended
    /// across the lanes as the line is.
    /// </summary>
    private float TractionHere(int station)
    {
        float At(LaneId lane) => Gains.UseNotes ? _file.TractionAt(lane, Grip, station) : 1f;
        float noted = LaneSet.Blend(At(LaneId.Free), At(LaneId.Left), At(LaneId.Right), At(LaneId.Middle), Range, Side);
        return _familiar is { } familiar ? noted * familiar.Traction[familiar.SegmentOf(station * _lanes.Spacing)] : noted;
    }

    // How fast a foot presses the pedals down, m/s² a second.
    private const float FootBrakeRate = 600f, FootThrottleRate = 240f;

    // Braking harder than this is a braking zone, so the throttle after it is a corner's exit.
    private const float ExitAfterBrakingAccel = -5f;

    /// <summary>
    /// Something went wrong here: the driver takes a little less of the
    /// notes into it from now on, of whatever they were doing on the way.
    /// </summary>
    private void Scared(float s, bool big)
    {
        if (_familiar is not { } familiar)
            return;
        LaneFamiliarity.Pedal pedal = _lastAccel < -3f ? LaneFamiliarity.Pedal.Braking
            : _lastAccel > 1f ? LaneFamiliarity.Pedal.Throttle
            : LaneFamiliarity.Pedal.Corner;
        familiar.Scare(s, pedal, big);
        _scaredInSegment = true;
    }

    // A segment is a corner taken at the limit when either axle used this
    // much of its circle in it (the driver's own cap is about nine tenths);
    // and taken as the driver meant to when never this far under the plan.
    private const float LimitSegmentUse = 0.8f;
    private const float UnderPlanShare = 0.97f;

    /// <summary>
    /// Only a corner taken at the limit teaches anything: one taken at what
    /// the driver dares now, on a rung that asks for it, and come through
    /// clean, gives back some of what is held there. A straight, a corner
    /// taken gently (held up, lifting, nursing the tyres on a gentle rung)
    /// or one gone wrong gives nothing back. And a lap's count.
    /// </summary>
    private void Familiarise(RaceCarSnapshot car, int lap)
    {
        if (_familiar is not { } familiar)
            return;
        if (lap != _familiarLap)
        {
            if (_familiarLap != int.MinValue)
            {
                familiar.Laps++;
                Save?.Write(FamiliarityFile, familiar.ToJson());
            }
            _familiarLap = lap;
        }
        int segment = familiar.SegmentOf(car.TrackS);
        if (segment != _familiarSegment)
        {
            if (_familiarSegment >= 0 && !_scaredInSegment && !_segmentHeldBack && _segmentPeakUse >= LimitSegmentUse)
                familiar.Clean(_familiarSegment, _adaptability);
            _familiarSegment = segment;
            _scaredInSegment = false;
            _segmentPeakUse = 0f;
            _segmentHeldBack = false;
        }
        GripUse axles = AxlesOf(car);
        _segmentPeakUse = MathF.Max(_segmentPeakUse, MathF.Max(
            Combined(axles.FrontLateral, axles.FrontLongitudinal),
            Combined(axles.RearLateral, axles.RearLongitudinal)));
        bool gentleRung = DrivenRung(car) < 2f;
        if (gentleRung || _phase != SlidePhase.None || SpeedCap is not null ||
            car.SpeedMetersPerSecond < UnderPlanShare * PlannedSpeedAt(car.TrackS))
            _segmentHeldBack = true;
    }

    /// <summary>
    /// Pace out of a corner: the first throttle after a braking zone. A
    /// driver short of pace is not sure the car will take the power, so
    /// waits a beat, and now and then gives part of it first; the quick one
    /// is on it the moment the car will have it. The pedals themselves move
    /// as fast for one as for the other.
    /// </summary>
    private float ExitThrottle(float accel, float dt)
    {
        LanePerformance performance = _performance!;
        if (accel < ExitAfterBrakingAccel)
            _brakedSinceThrottle = true;
        if (accel > 0f && _lastAccel <= 0f && _brakedSinceThrottle)
        {
            _brakedSinceThrottle = false;
            _sinceThrottle = 0f;
            performance.BeginExit();
        }
        else
            _sinceThrottle += MathF.Max(0f, dt);
        if (accel <= 0f)
            return accel;
        if (_sinceThrottle < performance.ExitHesitationSeconds)
            return 0f;
        if (_sinceThrottle < performance.ExitHesitationSeconds + performance.PartThrottleSeconds)
            return MathF.Min(accel, _maxDrive * performance.PartThrottleShare);
        return accel;
    }

    /// <summary>
    /// The brake balance dial, on a car whose balance is a fixed split. The
    /// right setting can be worked out: the share at the front that each
    /// axle's grip left, after the corner it is carrying, would take. Under
    /// any braking the dial is there, straight-line stop or trail into a
    /// corner, and a driver is off it only by its own error, which knowing
    /// the car (Technical) sets: a good one nearly never, a poor one by enough now and
    /// then to lock an axle. Held at the last stop's setting through a
    /// trail into a corner, it was the straight-line stop's, forward, and
    /// the front, turning and braking at once, ran wide. Off the brakes the
    /// dial does nothing and is left where it was.
    /// </summary>
    private float BrakeBias(GripUse tyres, float accel)
    {
        float error = _performance?.FrontBrakeBiasOffset ?? 0f;
        if (_layout.FixedFrontBrakeShare is not { } fixedShare)
            return error;
        float front = tyres.FrontGripAccel * MathF.Sqrt(MathF.Max(0f, 1f - tyres.FrontLateral * tyres.FrontLateral));
        float rear = tyres.RearGripAccel * MathF.Sqrt(MathF.Max(0f, 1f - tyres.RearLateral * tyres.RearLateral));
        if (float.IsNaN(_brakeDial))
            _brakeDial = fixedShare;
        if (accel < 0f && front + rear > 1f)
            _brakeDial = front / (front + rear) + error;
        return Math.Clamp(_brakeDial - fixedShare, -0.25f, 0.25f);
    }

    /// <summary>A rung between two of a table's, by the fraction past the lower one; clamped to its ends.</summary>
    // Tyres this far under their window, or this far past it, and the
    // driver backs off by as much as it allows: at most one rung, all of it
    // for a driver who manages the tyres, a third for one who does not.
    private const float ColdSpanC = 20f;
    private const float HotSpanC = 15f;
    private const float MostBackOffRungs = 1f;

    /// <summary>
    /// The tyre rung being driven, 0 (Protect) to 4 (Attack), fractional
    /// between two: how much of what the driver knows is used right now.
    /// The pit wall's rung, this lap's drift off it (mostly harder, for a
    /// driver who manages the tyres badly), and the driver's own backing off
    /// while the tyres are out of their window, cold or cooking. The one
    /// definition: the margin, the mistakes, the shares of the limit and
    /// whether a corner teaches anything all read it.
    /// </summary>
    private float DrivenRung(RaceCarSnapshot car)
    {
        float asked = TireLadder.Usage.RungsUp(car.Strategy.Tires) + (_performance?.TyreRungDrift ?? 0f);
        if (_performance is null)
            return asked;
        if (car.Tires.IsDefaultOrEmpty)
            return asked;
        System.Collections.Immutable.ImmutableArray<TireReading> tyres = car.Tires;
        TireWindow window = tyres[0].Window;
        float outside = 0f;
        if (window.IdealSurfaceHighC > window.IdealSurfaceLowC)
        {
            float surface = tyres.Average(t => t.SurfaceTempC);
            float cold = (window.IdealSurfaceLowC - surface) / ColdSpanC;
            float hot = MathF.Max((tyres.Max(t => t.SurfaceTempC) - window.IdealSurfaceHighC) / HotSpanC,
                                  window.CoreOverheatC > 0f ? (tyres.Max(t => t.CoreTempC) - window.CoreOverheatC) / HotSpanC : 0f);
            outside = Math.Clamp(MathF.Max(cold, hot), 0f, 1f);
        }
        float heed = 1f / 3f + 2f / 3f * Math.Clamp(_tyreHeed, 0f, 1f);
        return Math.Clamp(asked - MostBackOffRungs * outside * heed, 0f, 4f);
    }

    private static TyreStyle Rung(TyreStyle[] table, float at)
    {
        float x = Math.Clamp(at, 0f, table.Length - 1);
        int i = Math.Min((int)MathF.Floor(x), table.Length - 2);
        if (i < 0)
            return table[0];
        float u = x - i;
        TyreStyle a = table[i], b = table[i + 1];
        return new TyreStyle(a.Corner + (b.Corner - a.Corner) * u, a.Brake + (b.Brake - a.Brake) * u,
            a.Traction + (b.Traction - a.Traction) * u, a.Cap + (b.Cap - a.Cap) * u);
    }

    private static float Rung(float[] table, float at)
    {
        float x = Math.Clamp(at, 0f, table.Length - 1);
        int i = Math.Min((int)MathF.Floor(x), table.Length - 2);
        if (i < 0)
            return table[0];
        return table[i] + (table[i + 1] - table[i]) * (x - i);
    }

    private static float Combined(float lateral, float longitudinal) =>
        MathF.Sqrt(lateral * lateral + longitudinal * longitudinal);

    /// <summary>The planned speed at the car, for probes.</summary>
    public float PlannedSpeedAt(float s) => SpeedAt(_lanes.StationOf(s));

    /// <summary>Where in the planning beat a car starts, from its name: the same every run.</summary>
    private static float Phase(string id)
    {
        uint hash = 2166136261;
        foreach (char c in id)
            hash = (hash ^ c) * 16777619;
        return (hash % 1000) / 1000f;
    }

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
        NotedFactors(lane, 0, count, corner, brake);
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
        bool sides = Range > 1e-3f || TargetRange > 1e-3f;
        ReadGrip(car, roadGrip, dt, start, n + 1, sides);
        for (int lane = 0; lane < LaneSet.LaneCount; lane++)
        {
            if (!Reads(lane))
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

    /// <summary>
    /// Whether anything reads this lane's plan now. On the free lane with no
    /// move asked for, the other lanes are read by nothing; they are planned
    /// again the step a move begins. Settled on one lane off the free one,
    /// the others are read by nothing either.
    /// </summary>
    private bool Reads(int lane)
    {
        if (lane == (int)LaneId.Free)
            return true;
        if (Range <= 1e-3f && TargetRange <= 1e-3f)
            return false;
        int settled = SettledSideLane();
        return settled < 0 || lane == settled;
    }

    /// <summary>The live envelope and the learned factors at the grip the driver believes in.</summary>
    private void ReadGrip(RaceCarSnapshot car, float roadGrip, float sinceLastRead, int start, int count, bool sides)
    {
        PlanState.Hydrate(_plan, car, _motion, _tyres, _stores, _buttons);
        _tyreAuthority = Gains.TyreRungAuthority
            ? _carEnvelope.GripUsage(car.Strategy) / MathF.Max(_carEnvelope.GripUsage(CarStrategy.Default), 0.5f)
            : 1f;
        // The rung asked for, as this driver carries it out this lap: a
        // driver who manages the tyres drives the rung, one who does not
        // drifts off it, mostly harder.
        _style = Gains.TyreRungStyles
            ? Rung(TyreStyles, DrivenRung(car))
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
        LaneEnvelope truth = LaneEnvelope.From(_plan, _carEnvelope, _feel, car.Strategy, Gains.TurnAwareEnvelope).WithRoadGrip(road);
        ActualGrip = truth.Lateral(40f) / (ReferenceLateral(car.TireCompound) * MathF.Max(asphalt, 0.05f));
        // Adaptability: the plan is made on the grip the driver believes in.
        // The hands still feel the axles as they are; only the plan is late.
        Grip = _performance?.PerceiveGrip(ActualGrip, sinceLastRead) ?? ActualGrip;
        float belief = Math.Clamp(Grip / MathF.Max(ActualGrip, 1e-3f), 0.9f, 1.1f);
        _envelope = belief == 1f ? truth : LaneEnvelope.From(_plan, _carEnvelope, _feel, car.Strategy, Gains.TurnAwareEnvelope).WithRoadGrip(road * belief);
        for (int lane = 0; lane < (sides ? LaneSet.LaneCount : 1); lane++)
        {
            NotedFactors((LaneId)lane, start, count, _corner[lane], _brake[lane]);
            // The car's notes, as much of them as this driver takes here.
            if (_familiar is { } familiar)
            {
                for (int k = 0, n = _lanes.Count; k < Math.Min(count, n); k++)
                {
                    int i = _lanes.Wrap(start + k);
                    int segment = familiar.SegmentOf(i * _lanes.Spacing);
                    _corner[lane][i] *= familiar.Corner[segment];
                    _brake[lane][i] *= familiar.Brake[segment];
                }
            }
        }
    }

    private float SpeedAt(float station)
    {
        if (Range <= 1e-3f && TargetRange <= 1e-3f)
            return Interpolate(_speed[0], station);
        // Settled on a side lane or the middle one, the blend is that lane,
        // and its own plan is the blend's plan.
        int settled = SettledSideLane();
        if (settled >= 0)
            return Interpolate(_speed[settled], station);

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

    /// <summary>
    /// The lane off the free one the car is settled on, all the way out and
    /// not moving: a side lane or the middle one, or -1. There the blend is
    /// that lane, and planning a window for it again every step made a lap
    /// held on a side lane six times as long to drive as one on the free lane.
    /// </summary>
    private int SettledSideLane()
    {
        if (Range < 1f || TargetRange < 1f || TargetSide != Side || _rangeVelocity != 0f || _sideVelocity != 0f)
            return -1;
        if (Side == 0f)
            return (int)LaneId.Middle;
        if (MathF.Abs(Side) < 1f)
            return -1;
        return Side > 0f ? (int)LaneId.Left : (int)LaneId.Right;
    }

    /// <summary>The blend's window, planned from the car's own station once a station.</summary>
    private void AnchorBlendWindow(float station)
    {
        if (Range <= 1e-3f && TargetRange <= 1e-3f || SettledSideLane() >= 0)
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
        // Ending on the slowest of the lanes the blend lies between: the
        // free one, the middle one and the side lane it is towards.
        int sideLane = Side >= 0f ? (int)LaneId.Left : (int)LaneId.Right;
        float terminal = MathF.Min(_speed[0][end], MathF.Min(_speed[sideLane][end], _speed[(int)LaneId.Middle][end]));
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
            float seconds = _turnLag.Seconds(v);
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
        return LaneSet.Blend(table(LaneId.Free)[i], table(LaneId.Left)[i], table(LaneId.Right)[i], table(LaneId.Middle)[i], range, side);
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
        if (!Gains.UseNotes)
            return 0f;
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
        // Each station's own heading, through its neighbours, turned between
        // them as the car goes. Held along each two-metre chord instead, the
        // heading error jumped at every station round a bend and the wheel
        // chattered with it.
        Vector2 before = Vector2.Normalize(b - BlendedPosition(i0 - 1, Range, Side));
        Vector2 after = Vector2.Normalize(BlendedPosition(i0 + 2, Range, Side) - a);
        Vector2 tangent = before + (after - before) * u;
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
    public static void Hydrate(CarState plan, RaceCarSnapshot car, ICarMotion motion, ICarTires tyres, ICarStores? stores, ICarButtons? buttons)
    {
        plan.Position = car.Position;
        plan.Heading = car.HeadingRadians;
        plan.SideslipAngleRadians = car.SideslipAngleRadians;
        plan.YawRateRadiansPerSecond = car.YawRateRadiansPerSecond;
        plan.Speed = car.SpeedMetersPerSecond;
        // The wheels where the car's are, in the air it is in; what the seat
        // feels is the plan's own, from its last look ahead.
        motion.Assume(plan, motion.Motion(plan) with { WheelAngleRadians = (car.Motion ?? default).WheelAngleRadians },
            new CarAir(car.AirVelocityDeficit, car.DownforceVelocityDeficit, car.WakeDownforceLoss));
        if (buttons is not null)
            for (int i = 0; i < buttons.Buttons.Count && i < car.Buttons.Length; i++)
                buttons.Assume(plan, i, car.Buttons[i].Running);
        if (stores is not null && car.Resources.Length > 0)
        {
            float primary = car.Resources[0].Fraction;
            float secondary = car.Resources.Length > 1 ? car.Resources[1].Fraction : primary;
            stores.Fill(plan, new PowertrainState(primary, secondary));
        }
        if (car.Tires.Length == 4)
        {
            // The plan is for the racing surface, all lap. What the wheels are
            // standing on this instant (a kerb, the day's rubber, the asphalt)
            // is not the road ahead; the road's grip comes in once, in ReadGrip.
            Span<TireReading> read = [car.Tires[0], car.Tires[1], car.Tires[2], car.Tires[3]];
            for (int i = 0; i < 4; i++)
                read[i] = read[i] with { SurfaceGrip = 1f };
            tyres.Assume(plan, car.TireCompound, read);
        }
    }
}
