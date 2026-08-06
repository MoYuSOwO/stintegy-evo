namespace StintegyEVO.Core.Cars;

public sealed class CarConfig
{
    public float MassKg { get; init; } = 820f;
    public float WheelBaseMeters { get; init; } = 3.1f;
    public float TrackWidthMeters { get; init; } = 1.65f;
    public float CenterOfGravityHeightMeters { get; init; } = 0.32f;
    public float FrontStaticLoadShare { get; init; } = 0.47f;
    public float FrontDriveShare { get; init; } = 0f;
    public float YawInertiaKgM2 { get; init; } = 1450f;
    public float YawResponseTimeSeconds { get; init; } = 0.15f;
    public float SideslipRecoveryTimeSeconds { get; init; } = 0.15f;

    public float MaxCurvatureRequest { get; init; } = 0.32f;
    public float MaxDriveAcceleration { get; init; } = 12f;
    /// <summary>
    /// What the brakes themselves can do, before the tyres are asked whether
    /// they will take it.
    ///
    /// Raised from 1.2g when downforce arrived, because it had quietly become
    /// the thing that stopped the car. With wings the tyres will bear five g at
    /// speed, and a cap below that means the model brakes like a road car at
    /// the one moment a racing car does not. Calibrated with the wings and the
    /// drag together against published lap times: the three test circuits land
    /// within one per cent, cornering settles at 3.5g, braking at 4.1g and top
    /// speed at 301 km/h, all of which is where a Formula 2 car lives.
    /// </summary>
    public float MaxBrakeAccel { get; init; } = 40f;
    public float TractionControlActivationUse { get; init; } = 0.99f;
    public float TractionControlStrength { get; init; } = 0.65f;

    /// <summary>
    /// Where the anti-lock starts holding the brakes back, and how much of the
    /// excess it takes away, in the same terms as the traction control beside
    /// it. Both axles, because both of them lock.
    ///
    /// Strength deliberately short of one: what it removes is braking the car
    /// does not get back, so an anti-lock that held the tyre exactly at its
    /// limit would be leaving the corner entry to a system that cannot see the
    /// corner. Taking most of the excess and no more lets the driver keep
    /// asking for slightly too much, which is what they do.
    ///
    /// Zero switches it off.
    /// </summary>
    public float AntiLockActivationUse { get; init; } = 0.99f;
    public float AntiLockStrength { get; init; } = 0.65f;
    public float MinPowerSpeed { get; init; } = 8f;
    public float BatteryCapacityJoules { get; init; } = 830000000f;
    public float BatteryDriveEfficiency { get; init; } = 0.92f;
    public float LowSocPowerLimitStart { get; init; } = 0.08f;
    public float RegenEfficiency { get; init; } = 0.56f;
    public float RegenPowerCapWatts { get; init; } = 260000f;
    public float SaveDrivePowerLimitWatts { get; init; } = 372000f;
    public float EcoDrivePowerLimitWatts { get; init; } = 381000f;
    public float NormalDrivePowerLimitWatts { get; init; } = 390000f;
    public float PushDrivePowerLimitWatts { get; init; } = 400000f;
    public float AttackDrivePowerLimitWatts { get; init; } = 409000f;

    public float RollingDragAccel { get; init; } = 0.18f;
    public float AeroDragAccelPerSpeedSquared { get; init; } = 0.0009f;

    /// <summary>
    /// Downforce, as the acceleration it would add to gravity, per squared
    /// metre per second.
    ///
    /// Written this way because that is how it is used: the wheels are pressed
    /// down by weight and by air, and only their sum matters to the tyre. It
    /// also makes the number readable - multiplied by the square of a speed it
    /// gives an acceleration to set beside 9.81, so a car making its own weight
    /// in downforce at fifty metres a second is carrying a coefficient of about
    /// four thousandths.
    ///
    /// Zero is a car with no wings at all, which is what the model had before
    /// this existed: grip that does not care how fast the car is going, so a
    /// fast corner is worth no more than a slow one of the same radius.
    ///
    /// The default is a Formula 2 car, and the drag beside it was raised to
    /// suit: a lift to drag ratio of about four, which is where real
    /// single-seaters sit, and 301 km/h down the longest straight.
    /// </summary>
    public float DownforceAccelPerSpeedSquared { get; init; } = 0.0035f;

    /// <summary>
    /// How much of its own speed the air is still carrying directly behind
    /// this car, as a fraction, before any of it has been left behind.
    ///
    /// This is the size of the hole the car punches, and it is written as a
    /// speed rather than as a share of drag because that is what it physically
    /// is. Drag is fought against the air the car is moving through, not
    /// against the ground: a car in air already travelling at a fifth of its
    /// speed meets four fifths of the wind, and pays the square of that.
    ///
    /// Zero leaves the car alone on the circuit however close it gets.
    ///
    /// The default is set from the one figure about a tow that can be checked
    /// without a wind tunnel: what it is worth at the end of a long straight,
    /// which for these cars is ten to fifteen km/h. Top speed goes as the cube
    /// root of the drag, so that is a tenth of it escaped at a couple of car
    /// lengths and no more.
    ///
    /// Deliberately not the far larger reductions quoted for drafting, which
    /// belong to cars with their wheels covered and no wings to lose. An open
    /// wheeled car following closely gives up downforce as it gains slipstream,
    /// and the tow that survives is the modest one.
    /// </summary>
    public float WakeVelocityDeficit { get; init; } = 0.11f;
    public float CorneringScrubAccel { get; init; } = 1.15f;
    public float OverLimitMinGripEfficiency { get; init; } = 0.8f;
    public float OverLimitCostCap { get; init; } = 0.2f;

    public float LoadTransferResponse { get; init; } = 8f;
    public float MinimumWheelLoadShare { get; init; } = 0.08f;

    public float GetDrivePowerLimitWatts(BatteryOutputMode mode)
    {
        return mode switch
        {
            BatteryOutputMode.Save => SaveDrivePowerLimitWatts,
            BatteryOutputMode.Eco => EcoDrivePowerLimitWatts,
            BatteryOutputMode.Normal => NormalDrivePowerLimitWatts,
            BatteryOutputMode.Push => PushDrivePowerLimitWatts,
            BatteryOutputMode.Attack => AttackDrivePowerLimitWatts,
            _ => NormalDrivePowerLimitWatts
        };
    }

    public float GetDrivePowerLimitWatts(CarStrategy strategy)
    {
        if (!strategy.DrivePowerLimitWattsOverride.HasValue)
            return GetDrivePowerLimitWatts(strategy.BatteryMode);

        float minimum = Math.Min(
            SaveDrivePowerLimitWatts,
            AttackDrivePowerLimitWatts
        );
        float maximum = Math.Max(
            SaveDrivePowerLimitWatts,
            AttackDrivePowerLimitWatts
        );
        return Math.Clamp(
            strategy.DrivePowerLimitWattsOverride.Value,
            minimum,
            maximum
        );
    }

    public float GetDrivePowerLimitWatts(float sliderPosition)
    {
        float scaled = Math.Clamp(sliderPosition, 0f, 1f) * 4f;
        int segment = Math.Min((int)scaled, 3);
        float t = scaled - segment;
        return segment switch
        {
            0 => Lerp(SaveDrivePowerLimitWatts, EcoDrivePowerLimitWatts, t),
            1 => Lerp(EcoDrivePowerLimitWatts, NormalDrivePowerLimitWatts, t),
            2 => Lerp(NormalDrivePowerLimitWatts, PushDrivePowerLimitWatts, t),
            _ => Lerp(PushDrivePowerLimitWatts, AttackDrivePowerLimitWatts, t)
        };
    }

    private static float Lerp(float from, float to, float t) =>
        from + (to - from) * t;
}
