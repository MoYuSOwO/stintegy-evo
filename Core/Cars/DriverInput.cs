namespace StintegyEVO.Core.Cars;

/// <summary>
/// What a driver does with the car this step: where it turns the road
/// wheels and what it asks of the pedals.
/// </summary>
/// <param name="SteerAngleRadians">The angle to turn the front road wheels
/// to, signed as the curvature it turns the car onto. The physics turns
/// them there as fast as the rack moves and no further than full lock, and
/// does not second-guess it: the lock a corner wants is the driver's to judge
/// (<see cref="SteeringEstimate"/> is what a driver who knew the car
/// exactly would choose).</param>
public readonly record struct DriverInput(
    float SteerAngleRadians,
    float DesiredAccel,
    // Signed fraction of total braking demand. Positive values move braking
    // toward the front axle; zero retains the dynamically optimal split.
    float FrontBrakeBiasOffset = 0f,
    // Signed fraction of total drive moved toward the front axle, off the
    // split the car makes by grip; zero keeps that split.
    float FrontDriveBiasOffset = 0f,
    // How hard this driver's hands are on the tyres beyond what the single
    // track resolves (the small slides and corrections, the kerbs, the
    // wheelspin it averages away), as a share more wear than the driver the
    // tyre was calibrated with: -0.1 wears a tenth less. Zero is that
    // driver, so an input nobody filled in wears as it always did.
    float TyreWearExcess = 0f
);

/// <summary>
/// Everything the strategist sets, as opposed to everything the driver does.
///
/// The tyre ladder is the game's own: every car that has ever raced has had
/// tyres to look after, and the driver reads it to know how hard to lean on
/// them. Every other ladder is the car's (<see cref="ICar.Ladders"/>): an
/// output setting on a pack, an engine map, a hybrid's deployment beside it.
/// How many there are, how long each is and what its rungs are called are
/// the car's to say, and this holds where each is set, in the car's order.
/// </summary>
public readonly record struct CarStrategy
{
    /// <summary>The most ladders of its own a car may declare.</summary>
    public const int MaximumLadders = 4;

    public TireUsageMode TireMode { get; init; }

    // Where each of the car's ladders is set, counted from one; zero is a
    // ladder nobody has touched, which sits where the ladder itself says
    // (ModeLadder.DefaultRung). Held inline rather than in an array so a
    // strategy compares by what it says and costs nothing to pass about.
    private int R1 { get; init; }
    private int R2 { get; init; }
    private int R3 { get; init; }
    private int R4 { get; init; }

    // Which of the car's buttons are held, one bit each in the car's order
    // (ICarButtons.Buttons).
    private int Held { get; init; }

    /// <summary>The most buttons a car can have.</summary>
    public const int MaximumButtons = 8;

    /// <summary>
    /// Whether the car's <paramref name="button"/>th button is held. What
    /// holding it does, and whether it does anything now, is the car's to
    /// say (<see cref="ICarButtons"/>).
    /// </summary>
    public bool Holding(int button) => (Held & Bit(button)) != 0;

    /// <summary>The same strategy with the car's <paramref name="button"/>th button held, or let go.</summary>
    public CarStrategy WithButton(int button, bool held) =>
        this with { Held = held ? Held | Bit(button) : Held & ~Bit(button) };

    private static int Bit(int button) =>
        button is >= 0 and < MaximumButtons ? 1 << button : throw new ArgumentOutOfRangeException(nameof(button));

    /// <summary>A share of the tyres' grip to use in place of what the tyre rung allows.</summary>
    public float? TireGripUsageOverride { get; init; }

    /// <param name="rungs">Where each of the car's ladders is set, in its order, counted from one.</param>
    public CarStrategy(TireUsageMode tireMode, params int[] rungs)
    {
        TireMode = tireMode;
        if (rungs.Length > MaximumLadders)
            throw new ArgumentException($"A car has at most {MaximumLadders} ladders of its own.", nameof(rungs));
        R1 = rungs.Length > 0 ? Math.Max(1, rungs[0]) : 0;
        R2 = rungs.Length > 1 ? Math.Max(1, rungs[1]) : 0;
        R3 = rungs.Length > 2 ? Math.Max(1, rungs[2]) : 0;
        R4 = rungs.Length > 3 ? Math.Max(1, rungs[3]) : 0;
        TireGripUsageOverride = null;
    }

    /// <summary>
    /// The readable form, for a car whose first ladder is a five-rung
    /// output ladder like the one the game's own car has.
    /// </summary>
    public CarStrategy(TireUsageMode tireMode, PowerOutputMode powerMode)
        : this(tireMode, (int)powerMode)
    {
    }

    /// <summary>
    /// Where the car's <paramref name="index"/>th ladder is set, as that
    /// ladder has it. A rung past its end is not an error worth throwing
    /// over: a strategy outlives the car it was written for, and one saved
    /// against an engine with seven maps loads against one with five.
    /// </summary>
    public int Rung(int index, ModeLadder ladder)
    {
        int set = Raw(index);
        return set == 0 ? ladder.DefaultRung : ladder.Clamp(set);
    }

    /// <summary>The same strategy with the car's <paramref name="index"/>th ladder set to <paramref name="ordinal"/>.</summary>
    public CarStrategy WithRung(int index, int ordinal)
    {
        int rung = Math.Max(1, ordinal);
        return index switch
        {
            0 => this with { R1 = rung },
            1 => this with { R2 = rung },
            2 => this with { R3 = rung },
            3 => this with { R4 = rung },
            _ => throw new ArgumentOutOfRangeException(nameof(index))
        };
    }

    /// <summary>The car's <paramref name="index"/>th ladder moved <paramref name="delta"/> rungs, stopping at its ends.</summary>
    public CarStrategy Nudged(int index, ModeLadder ladder, int delta) =>
        WithRung(index, ladder.Clamp(Rung(index, ladder) + delta));

    /// <summary>The tyre ladder moved <paramref name="delta"/> rungs, stopping at its ends.</summary>
    public CarStrategy NudgedTires(int delta) =>
        this with { TireMode = (TireUsageMode)TireLadder.Usage.Clamp((int)TireMode + delta) };

    private int Raw(int index) => index switch
    {
        0 => R1,
        1 => R2,
        2 => R3,
        3 => R4,
        _ => throw new ArgumentOutOfRangeException(nameof(index))
    };

    /// <summary>
    /// Where a car sets off when nobody has told it anything: the tyres on
    /// their middle rung and every ladder of the car's where it says.
    /// </summary>
    public static readonly CarStrategy Default = new(TireUsageMode.Normal);

    public CarStrategy WithTireGripUsage(float usage)
    {
        if (!float.IsFinite(usage) || usage <= 0f || usage > 1f)
            throw new ArgumentOutOfRangeException(nameof(usage));
        return this with { TireGripUsageOverride = usage };
    }
}

public readonly record struct CarPhysicsStepInput(
    DriverInput DriverInput,
    CarStrategy Strategy,
    float AirTempC,
    float TrackTempC = 35f
)
{
    /// <summary>
    /// How the road lies under the car this step. Defaults to flat, so a
    /// caller that has no elevation data gets exactly the old behaviour.
    /// </summary>
    public RoadAttitude RoadAttitude { get; init; } = RoadAttitude.Flat;

    /// <summary>
    /// What the road is worth under each wheel. Defaults to clean racing
    /// surface all round, so a caller with no track under it - a bench
    /// test, a predictor - gets exactly the old behaviour.
    /// </summary>
    public WheelSurfaceGrip SurfaceGrip { get; init; } = WheelSurfaceGrip.Clean;

    /// <summary>
    /// How hard the asphalt is on rubber, as a multiplier on tyre wear: the
    /// circuit's <c>AsphaltAbrasion</c>. One, an average circuit, unless a
    /// track says otherwise.
    /// </summary>
    public float SurfaceAbrasion { get; init; } = 1f;
}

/// <summary>
/// The road's grip multiplier under each of the four wheels, sampled where
/// each of them actually is.
/// </summary>
public readonly record struct WheelSurfaceGrip(
    float FrontLeft,
    float FrontRight,
    float RearLeft,
    float RearRight
)
{
    public static readonly WheelSurfaceGrip Clean = new(1f, 1f, 1f, 1f);

    public float this[WheelId wheel] => wheel switch
    {
        WheelId.FrontLeft => FrontLeft,
        WheelId.FrontRight => FrontRight,
        WheelId.RearLeft => RearLeft,
        WheelId.RearRight => RearRight,
        _ => 1f
    };
}
