using StintegyEVO.Core.Cars;

namespace Official.FormulaAMixed;

/// <summary>
/// Formula A, mixed: the game's own car. A near-future single-seater on the
/// game's single-track chassis -- a range-extended electric powertrain, a
/// floor that makes downforce far more cheaply than any built yet, and tyres
/// whose grip falls with load -- set to corner like a Formula One car.
///
/// Every number is written here, none taken from a default: the chassis's
/// defaults are the chassis's, and this car is this car.
/// </summary>
public sealed class FormulaAMixed : ICarDefinition
{
    public ICar Build() => new SingleTrackCar(Chassis(), TireSet.Of(Tyre()));

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
        CombinedGripLimiterStrength = 1f,
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

    private static TireConfig Tyre() => new()
    {
        CompoundId = "default",
        StartingSurfaceTempC = 70f,
        StartingCoreTempC = 70f,
        AccelerationUsage = 0.977f,
        BaseMu = 2.4f,
        LoadSensitivity = 0.3f,
        IdealSurfaceTempLowC = 85f,
        IdealSurfaceTempHighC = 115f,
        ColdGripLossPerCSquared = 8E-05f,
        HotGripLossPerCSquared = 0.0001f,
        CoreOverheatTempC = 112f,
        CoreOverheatGripLossPerC = 0.03f,
        WearLinearGripLoss = 0.04f,
        WearCliffStart = 0.7f,
        WearCliffGripLoss = 0.12f,
        RollingSurfaceHeatRate = 0.64f,
        RollingCoreHeatRate = 0.24f,
        BrakeCoreHeatRate = 1.5f,
        LateralHeatRate = 2.4f,
        LongitudinalHeatRate = 0.33f,
        NearLimitHeatRate = 1.2f,
        NearLimitHeatStartShare = 0.97f,
        LateralWearRate = 0.00048f,
        LongitudinalWearRate = 5.46E-05f,
        WearFromSlipWork = true,
        SlipWorkWearRate = 0.0002064f,
        NearLimitWearRate = 5.16E-05f,
        OverLimitWearRate = 0.000473f,
        ColdWearPerCSquared = 0.0015f,
        HotWearPerCSquared = 0.0035f,
        WearReferenceTempC = 100f,
        WearTempScaleC = 10f,
        MaximumHotWearFactor = 6f,
    };
}
