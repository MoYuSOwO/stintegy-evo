namespace StintegyEVO.Core.Cars;

public readonly record struct DriverInput(
    float DesiredCurvature,
    float DesiredAccel,
    // Signed fraction of total braking demand. Positive values move braking
    // toward the front axle; zero retains the dynamically optimal split.
    float FrontBrakeBiasOffset = 0f
);

/// <summary>
/// Everything the strategist sets, as opposed to everything the driver does.
///
/// Two ladders, because a car has two things it spends: its tyres and
/// whatever turns its wheels. The tyre ladder is universal - every car that
/// has ever raced has had tyres to look after - while what the power ladder's
/// rungs are called belongs to the powertrain fitted to the car. A third
/// ladder belongs here the day a hybrid arrives, which needs an engine map
/// and a deployment setting at the same time.
/// </summary>
public readonly record struct CarStrategy
{
    public TireUsageMode TireMode { get; init; }

    /// <summary>
    /// Which rung of the fitted powertrain's output ladder, counted from one.
    ///
    /// A number rather than a named setting, and the asymmetry with the tyre
    /// mode beside it is the point. The tyre ladder is the game's own: every
    /// car has tyres, they are spent the same way, and the five names mean
    /// the same thing on all of them. The output ladder belongs to whatever
    /// is fitted, so how long it is and what its rungs are called are the
    /// machinery's to say - five on the car that ships, seven on an engine
    /// with seven maps, and nothing here counts on either.
    ///
    /// A rung past the end of the fitted ladder is not an error worth
    /// throwing over. It is clamped where it is read, because a strategy
    /// outlives the car it was written for: a setup saved against one engine
    /// should load against another rather than refuse to start.
    /// </summary>
    public int PowerRung { get; init; }

    public float? TireGripUsageOverride { get; init; }
    public float? DrivePowerLimitWattsOverride { get; init; }

    public CarStrategy(TireUsageMode tireMode, int powerRung)
    {
        TireMode = tireMode;
        PowerRung = Math.Max(1, powerRung);
        TireGripUsageOverride = null;
        DrivePowerLimitWattsOverride = null;
    }

    /// <summary>
    /// The readable form, for the five-rung ladder the shipped car has.
    /// </summary>
    public CarStrategy(TireUsageMode tireMode, PowerOutputMode powerMode)
        : this(tireMode, (int)powerMode)
    {
    }

    public static readonly CarStrategy Default = new(
        TireUsageMode.Normal,
        PowerOutputMode.Normal
    );

    /// <summary>
    /// Where a car sets off when nobody has told it anything: the middle of
    /// whatever ladder it happens to have, rather than the middle of ours.
    /// </summary>
    public static CarStrategy DefaultFor(IPowertrain powertrain)
    {
        ArgumentNullException.ThrowIfNull(powertrain);
        return new CarStrategy(
            TireUsageMode.Normal,
            powertrain.OutputLadder.DefaultRung
        );
    }

    public CarStrategy WithTireGripUsage(float usage)
    {
        if (!float.IsFinite(usage) || usage <= 0f || usage > 1f)
            throw new ArgumentOutOfRangeException(nameof(usage));
        return this with { TireGripUsageOverride = usage };
    }

    public CarStrategy WithDrivePowerLimitWatts(float powerWatts)
    {
        if (!float.IsFinite(powerWatts) || powerWatts <= 0f)
            throw new ArgumentOutOfRangeException(nameof(powerWatts));
        return this with { DrivePowerLimitWattsOverride = powerWatts };
    }
}

public readonly record struct CarPhysicsStepInput(
    DriverInput DriverInput,
    CarStrategy Strategy,
    float AirTempC,
    float TrackTempC = 35f,
    float TireEnergyEfficiency = 1f,
    float CorneringEfficiency = 1f,
    float LimitSettleUse = float.PositiveInfinity
)
{
    /// <summary>
    /// How the road lies under the car this step. Defaults to flat, so a
    /// caller that has no elevation data gets exactly the old behaviour.
    /// </summary>
    public RoadAttitude RoadAttitude { get; init; } = RoadAttitude.Flat;
}
