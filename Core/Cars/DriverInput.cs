namespace TheStint.Core.Cars;

public readonly record struct DriverInput(
    float DesiredCurvature,
    float DesiredAccel
);

public readonly record struct CarStrategy
{
    public TireUsageMode TireMode { get; init; }
    public BatteryOutputMode BatteryMode { get; init; }
    public float? TireGripUsageOverride { get; init; }
    public float? DrivePowerLimitWattsOverride { get; init; }

    public CarStrategy(
        TireUsageMode tireMode,
        BatteryOutputMode batteryMode
    )
    {
        TireMode = tireMode;
        BatteryMode = batteryMode;
        TireGripUsageOverride = null;
        DrivePowerLimitWattsOverride = null;
    }

    public static readonly CarStrategy Default = new(
        TireUsageMode.Normal,
        BatteryOutputMode.Normal
    );

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
    float TireEnergyEfficiency = 1f
);
