using System.Collections.Generic;
using Official.SingleTrack;
using StintegyEVO.Core.Cars;

namespace Official.FormulaAMixed;

/// <summary>
/// Formula A, mixed: the game's own car. A near-future single-seater on the
/// game's single-track chassis -- a range-extended electric powertrain, a
/// floor that makes downforce far more cheaply than any built yet, on
/// Formula A's tyres -- set to corner like a Formula One car.
///
/// Every number is written here, none taken from a default: the chassis's
/// defaults are the chassis's, and this car is this car. The tyres' numbers
/// are their pack's, not the car's.
/// </summary>
public sealed class FormulaAMixedCar : ICarDefinition
{
    /// <summary>On the tyres its car.json declares: Formula A's (official:formula_a).</summary>
    public ICar Build(IReadOnlyList<TireCompound> compounds) => new SingleTrackCar(Chassis(), compounds);

    private static CarConfig Chassis() => new()
    {
        MassKg = 900f,
        Powertrain = Powertrain(),
        Aero = new StandardAero
        {
            // Downforce equal to its weight at about 145 km/h; the drag of a
            // high-downforce single-seater. Lift to drag about 6.9, twice a
            // Formula One car's: the near-future floor.
            DownforcePerSpeedSquared = 5.6f,
            DragPerSpeedSquared = 0.81f,
            ActiveWakeDownforceRecovery = 0.5f,
            FrontDownforceShare = null,
        },
        WheelBaseMeters = 3.1f,
        TrackWidthMeters = 1.65f,
        Body = new CarBody
        {
            LengthMeters = 4.8f,
            WidthMeters = 1.9f,
            Restitution = 0.05f,
            Friction = 0.35f,
            WallRestitution = 0.1f,
            WallFriction = 0.5f,
        },
        CenterOfGravityHeightMeters = 0.32f,
        FrontStaticLoadShare = 0.47f,
        FrontLateralLoadTransferShare = 0.55f,
        FrontLateralComplianceRatio = 1.4f,
        FrontDriveShare = 0.42f,
        AdaptiveDriveSplit = true,
        PushToPass = new PushToPassRules
        {
            TotalSeconds = 200f,
            MaxActivationSeconds = 15f,
            CooldownSeconds = 30f,
        },
        YawInertiaKgM2 = 1450f,
        YawResponseTimeSeconds = 0.15f,
        SideslipRecoveryTimeSeconds = 0.15f,
        FrontPeakSlipAngleRatio = 1.35f,
        MaxSteerAngleRadians = 0.8f,
        SteerRateLimitRadiansPerSecond = 1.047f,
        SteerCurvatureFeedbackGain = 0.5f,
        MaxCurvatureRequest = 0.32f,
        MaxDriveAcceleration = 12f,
        MaxBrakeAccel = 57f,
        // No traction control: the driver's right foot holds the throttle to
        // what the tyres will take, and a driver without the feel for it
        // spins the rear. The device stays in the physics for a car that has one.
        CombinedGripLimiterStrength = 0f,
        // The brake balance is the driver's dial on a fixed split: 57 per
        // cent front is about the load at the front braking hard from
        // racing speed, downforce and weight transfer together.
        FixedFrontBrakeShare = 0.57f,
        CombinedGripLimiterPastPeakMinimumSpeed = 10f,
        RollingDragAccel = 0.18f,
        WakeVelocityDeficit = 0.11f,
        WakeDownforceDisruption = 0.08f,
        WakeTowHalfDistanceMeters = 11f,
        WakeDownforceDecayLengthMeters = 12f,
        WakeDirtyAirDecayLengthMeters = 16f,
        DirtyAirSensitivity = 1f,
        OverLimitMinGripEfficiency = 0.8f,
        OverLimitCostCap = 0.2f,
        LoadTransferResponse = 8f,
        MinimumWheelLoadShare = 0.08f,
    };

    private static RangeExtenderPowertrain Powertrain() => new()
    {
        BatteryCapacityJoules = 108000000f,
        BatteryDriveEfficiency = 0.92f,
        RegenEfficiency = 0.85f,
        RegenPowerCapWatts = 600000f,
        MinPowerSpeed = 8f,
        RaceDrivePowerLimitWatts = 600000f,
        GeneratorPowerWatts = 320000f,
        PushToPassDrivePowerWatts = 672000f,
        PushToPassGeneratorShare = 0.5f,
        GeneratorEfficiency = 0.36f,
        GeneratorStopCharge = 0.95f,
        FuelAllowanceKg = 120f,
        LowVoltageStart = 0.05f,
        LowVoltageOutputFraction = 0.85f,
        EmptyBand = 0.005f,
        LowSocPowerFloor = 0.08f,
    };
}
