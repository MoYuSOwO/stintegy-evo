namespace TheStint.Core.Cars;

public readonly record struct DriverInput(
    float DesiredCurvature,
    float DesiredAccel
);

public readonly record struct CarStrategy(
    TireUsageMode TireMode,
    BatteryOutputMode BatteryMode
)
{
    public static readonly CarStrategy Default = new(
        TireUsageMode.Normal,
        BatteryOutputMode.Normal
    );
}

public readonly record struct CarPhysicsStepInput(
    DriverInput DriverInput,
    CarStrategy Strategy,
    float AirTempC
);
