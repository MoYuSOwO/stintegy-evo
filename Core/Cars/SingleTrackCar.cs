using System;
using System.Collections.Generic;
using System.Linq;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// The chassis the game offers every car: a single-track model with load
/// transfer, slip-angle tyres that heat and wear, a combined-grip limiter and
/// whatever powertrain is fitted (<see cref="CarPhysics"/>). A car built on
/// it is this plus its numbers, the parts it fits -- aero
/// (<see cref="IAero"/>), powertrain (<see cref="IPowertrain"/>) -- and the
/// compounds its pack declares (<see cref="TireCompound"/>), and may
/// override any of it; a car that wants something else entirely implements
/// <see cref="ICar"/> itself. The tyres are the game's either way
/// (<see cref="Tire"/>).
/// </summary>
public class SingleTrackCar : ICar
{
    /// <summary>One compound in no pack, built to <paramref name="tires"/>: for tests and probes.</summary>
    public SingleTrackCar(CarConfig config, TireConfig tires)
        : this(config, [TireCompound.Of(tires ?? throw new ArgumentNullException(nameof(tires)))])
    {
    }

    public SingleTrackCar(CarConfig config, IReadOnlyList<TireCompound> compounds)
    {
        Config = config ?? throw new ArgumentNullException(nameof(config));
        if (compounds is null || compounds.Count == 0)
            throw new ArgumentException("A car runs at least one compound.", nameof(compounds));
        Compounds = compounds;
        _sets = new Dictionary<string, TireSet>(StringComparer.Ordinal);
        foreach (TireCompound compound in compounds)
            if (!_sets.TryAdd(compound.Id, compound.Set))
                throw new ArgumentException($"Compound '{compound.Id}' is listed twice.", nameof(compounds));
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

    private readonly Dictionary<string, TireSet> _sets;

    public CarConfig Config { get; }
    public IReadOnlyList<TireCompound> Compounds { get; }

    /// <summary>The tyres on the car: the compound fitted, or the first one if none is.</summary>
    public TireSet TiresOn(CarState state) =>
        state.Compound is { } id && _sets.TryGetValue(id, out TireSet? set) ? set : Compounds[0].Set;
    public CarSpec Spec { get; }
    public IPowertrain Powertrain => Config.Powertrain;

    /// <summary>The powertrain's output ladder, and nothing else.</summary>
    public virtual IReadOnlyList<ModeLadder> Ladders => [Config.Powertrain.OutputLadder];

    public virtual float TotalMassKg(CarState state) => CarPhysics.TotalMassKg(Config, state.Energy);

    public virtual void Step(CarState state, CarPhysicsStepInput input, float dt) =>
        CarPhysics.Step(state, Config, TiresOn(state), input, dt);

    public virtual CarPerformanceLimits EstimatePerformanceLimits(
        CarState state,
        CarStrategy strategy,
        float speed,
        float curvature,
        float gripUsage = 1f,
        float assumedLongitudinalAcceleration = 0f,
        float frontBrakeBiasOffset = 0f
    ) => CarPhysics.EstimatePerformanceLimits(
        state, Config, TiresOn(state), strategy, speed, curvature, gripUsage,
        assumedLongitudinalAcceleration, frontBrakeBiasOffset);

    public virtual float EstimateCorneringLimit(CarState state, CarStrategy strategy, float speed, float gripUsage = 1f) =>
        CarPhysics.EstimateCorneringLimit(state, Config, TiresOn(state), strategy, speed, gripUsage);

    public virtual SteeringEstimate EstimateSteering(CarState state, RoadAttitude road) =>
        CarPhysics.EstimateSteering(state, Config, TiresOn(state), road);

    public virtual float GripUsage(CarStrategy strategy) => CarPhysics.GripUsage(strategy);

    public virtual TireWindow TireWindow(CarState state, WheelId wheel) => TiresOn(state).For(wheel).Window;

    public virtual void FitTires(CarState state, string compound)
    {
        if (!_sets.TryGetValue(compound, out TireSet? set))
            throw new ArgumentException(
                $"This car does not run '{compound}'; it runs {string.Join(", ", Compounds.Select(c => c.Id))}.",
                nameof(compound));
        state.Compound = compound;
        set.Install(state);
    }

    public virtual void InstallFreshTires(CarState state) =>
        FitTires(state, state.Compound is { } id && _sets.ContainsKey(id) ? id : Compounds[0].Id);

    public virtual CarCapabilities Capabilities(CarState state, CarStrategy strategy)
    {
        float mass = TotalMassKg(state);
        return new CarCapabilities(
            mass,
            Config.MaxCurvatureRequest,
            Config.MaxSteerAngleRadians,
            Config.SteerRateLimitRadiansPerSecond,
            Config.MaxDriveAcceleration,
            Powertrain.DriveAccelerationLimit(
                state.Energy, strategy.Rung(CarPhysics.OutputLadderIndex, Powertrain.OutputLadder), state.Speed, mass,
                Config.MaxDriveAcceleration, state.PushToPass.Active),
            Powertrain.OutputAvailability(state.Energy),
            TiresOn(state).Front.Window.Compound);
    }

    public virtual float PushToPassRemainingSeconds(CarState state) => state.PushToPass.RemainingSeconds(Config.PushToPass);

    public virtual void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy)
    {
        PowertrainState energy = state.Energy;
        IReadOnlyList<PowertrainResourceInfo> declared = Powertrain.Resources;
        for (int i = 0; i < declared.Count && i < PowertrainState.Capacity; i++)
        {
            PowertrainResourceInfo info = declared[i];
            dashboard.AddResource(new DashboardResource(info.Id, info.Label, energy[i], info.FullMassKg * energy[i]));
        }

        AddTire(dashboard, "FL", state.FrontLeft, TireWindow(state, WheelId.FrontLeft));
        AddTire(dashboard, "FR", state.FrontRight, TireWindow(state, WheelId.FrontRight));
        AddTire(dashboard, "RL", state.RearLeft, TireWindow(state, WheelId.RearLeft));
        AddTire(dashboard, "RR", state.RearRight, TireWindow(state, WheelId.RearRight));

        dashboard.AddMode(TireLadder.Usage, (int)strategy.TireMode);
        IReadOnlyList<ModeLadder> ladders = Ladders;
        for (int i = 0; i < ladders.Count; i++)
            dashboard.AddMode(ladders[i], strategy.Rung(i, ladders[i]));
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

    public virtual IReadOnlyList<TelemetryChannel> TelemetryChannels => Channels;

    public virtual void ReadTelemetry(CarState state, Span<float> values)
    {
        AxleTelemetry t = state.Axles();
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
