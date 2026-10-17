using System;
using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// The chassis the game offers every car: a single-track model with load
/// transfer, slip-angle tyres that heat and wear, a combined-grip limiter and
/// whatever powertrain is fitted (<see cref="CarPhysics"/>). A car built on
/// it is this plus its numbers; a car that wants something else implements
/// <see cref="ICar"/> itself.
/// </summary>
public class SingleTrackCar : ICar
{
    public SingleTrackCar(CarConfig config, TireConfig tires)
    {
        Config = config ?? throw new ArgumentNullException(nameof(config));
        Tires = tires ?? throw new ArgumentNullException(nameof(tires));
        Spec = new CarSpec
        {
            WheelBaseMeters = config.WheelBaseMeters,
            FrontAxleOffsetMeters = config.FrontAxleOffsetMeters,
            RearAxleOffsetMeters = config.RearAxleOffsetMeters,
            TrackWidthMeters = config.TrackWidthMeters,
            YawInertiaKgM2 = config.YawInertiaKgM2,
            MaximumBrakeDeceleration = config.MaxBrakeAccel,
            KinematicBelowSpeed = CarPhysics.DynamicYawMinimumSpeed,
            Wake = new CarWake
            {
                VelocityDeficit = config.WakeVelocityDeficit,
                TowHalfDistanceMeters = config.WakeTowHalfDistanceMeters,
                DownforceDecayLengthMeters = config.WakeDownforceDecayLengthMeters,
                DownforceDisruption = config.WakeDownforceDisruption,
                DirtyAirDecayLengthMeters = config.WakeDirtyAirDecayLengthMeters
            },
            DirtyAirSensitivity = config.DirtyAirSensitivity
        };
    }

    public CarConfig Config { get; }
    public TireConfig Tires { get; }
    public CarSpec Spec { get; }
    public IPowertrain Powertrain => Config.Powertrain;

    public float TotalMassKg(CarState state) => CarPhysics.TotalMassKg(Config, state.Energy);

    public void Step(CarState state, CarPhysicsStepInput input, float dt) =>
        CarPhysics.Step(state, Config, Tires, input, dt);

    public CarPerformanceLimits EstimatePerformanceLimits(
        CarState state,
        CarStrategy strategy,
        float speed,
        float curvature,
        float gripUsage = 1f,
        float assumedLongitudinalAcceleration = 0f,
        float frontBrakeBiasOffset = 0f
    ) => CarPhysics.EstimatePerformanceLimits(
        state, Config, Tires, strategy, speed, curvature, gripUsage,
        assumedLongitudinalAcceleration, frontBrakeBiasOffset);

    public float EstimateCorneringLimit(CarState state, CarStrategy strategy, float speed, float gripUsage = 1f) =>
        CarPhysics.EstimateCorneringLimit(state, Config, Tires, strategy, speed, gripUsage);

    public float GripUsage(CarStrategy strategy) => Tires.GetAccelerationUsage(strategy);

    public TireWindow TireWindow(WheelId wheel) =>
        new(Tires.CompoundId, Tires.IdealSurfaceTempLowC, Tires.IdealSurfaceTempHighC, Tires.CoreOverheatTempC);

    public void InstallFreshTires(CarState state) => state.InstallFreshTires(Tires);

    public CarCapabilities Capabilities(CarState state, CarStrategy strategy)
    {
        float mass = TotalMassKg(state);
        return new CarCapabilities(
            mass,
            Config.MaxCurvatureRequest,
            Config.MaxSteerAngleRadians,
            Config.SteerRateLimitRadiansPerSecond,
            Config.MaxDriveAcceleration,
            Powertrain.DriveAccelerationLimit(
                state.Energy, strategy, state.Speed, mass, Config.MaxDriveAcceleration, state.PushToPass.Active),
            Powertrain.OutputAvailability(state.Energy),
            Tires.CompoundId);
    }

    public float PushToPassRemainingSeconds(CarState state) => state.PushToPass.RemainingSeconds(Config.PushToPass);

    public void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy)
    {
        PowertrainState energy = state.Energy;
        IReadOnlyList<PowertrainResourceInfo> declared = Powertrain.Resources;
        for (int i = 0; i < declared.Count && i < PowertrainState.Capacity; i++)
        {
            PowertrainResourceInfo info = declared[i];
            dashboard.AddResource(new DashboardResource(info.Id, info.Label, energy[i], info.FullMassKg * energy[i]));
        }

        AddTire(dashboard, "FL", state.FrontLeft, TireWindow(WheelId.FrontLeft));
        AddTire(dashboard, "FR", state.FrontRight, TireWindow(WheelId.FrontRight));
        AddTire(dashboard, "RL", state.RearLeft, TireWindow(WheelId.RearLeft));
        AddTire(dashboard, "RR", state.RearRight, TireWindow(WheelId.RearRight));

        dashboard.AddMode(TireLadder.Usage, (int)strategy.TireMode);
        dashboard.AddMode(Powertrain.OutputLadder, strategy.PowerRung);
        dashboard.SetOutputAvailability(Powertrain.OutputAvailability(energy));
        dashboard.SetMass(TotalMassKg(state));
    }

    private static readonly TelemetryChannel[] Channels =
    [
        new("front_lateral_use", "Front lateral use", "share"),
        new("rear_lateral_use", "Rear lateral use", "share"),
        new("front_longitudinal_use", "Front longitudinal use", "share"),
        new("rear_longitudinal_use", "Rear longitudinal use", "share"),
        new("front_grip_mps2", "Front grip", "m/s2"),
        new("rear_grip_mps2", "Rear grip", "m/s2"),
        new("grip_limiter_cut_mps2", "Grip limiter cut", "m/s2"),
        new("induced_drag_mps2", "Induced drag", "m/s2"),
        new("rear_slide_severity", "Rear slide severity", "share"),
    ];

    public IReadOnlyList<TelemetryChannel> TelemetryChannels => Channels;

    public void ReadTelemetry(CarState state, Span<float> values)
    {
        CarTelemetry t = state.Telemetry;
        ReadOnlySpan<float> readings =
        [
            t.FrontLateralUse, t.RearLateralUse, t.FrontLongitudinalUse, t.RearLongitudinalUse,
            t.FrontGripAccel, t.RearGripAccel, t.CombinedGripLimiterCutAccel, t.InducedDragAccel,
            t.RearSlideSeverity,
        ];
        readings[..Math.Min(readings.Length, values.Length)].CopyTo(values);
    }

    private static void AddTire(CarDashboard dashboard, string label, TireState tire, TireWindow window) =>
        dashboard.AddTire(new DashboardTire(label, 1f - tire.Wear, tire.SurfaceTempC, tire.CoreTempC, tire.LoadN, window));
}
