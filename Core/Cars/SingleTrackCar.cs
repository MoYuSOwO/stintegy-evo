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
public class SingleTrackCar : ICar, ICarTires, ICarStores, ICarButtons, ICarEnvelope, ICarGripFeel
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
        state.Chassis().Compound is { } id && _sets.TryGetValue(id, out TireSet? set) ? set : Compounds[0].Set;
    public CarSpec Spec { get; }
    public IPowertrain Powertrain => Config.Powertrain;

    /// <summary>The powertrain's output ladder, and nothing else.</summary>
    public virtual IReadOnlyList<ModeLadder> Ladders => [Config.Powertrain.OutputLadder];

    public virtual float TotalMassKg(CarState state) => CarPhysics.TotalMassKg(Config, state.Chassis().Energy);

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

    public virtual string? Compound(CarState state) => state.Chassis().Compound;

    public virtual TireReading Tire(CarState state, WheelId wheel)
    {
        TireState tire = state.Chassis().GetTire(wheel);
        return new TireReading(tire.SurfaceTempC, tire.CoreTempC, tire.Wear, tire.LoadN, tire.SurfaceGrip)
        {
            Window = TireWindow(state, wheel)
        };
    }

    public virtual void Assume(CarState plan, string? compound, ReadOnlySpan<TireReading> tires)
    {
        if (tires.Length != 4)
            throw new ArgumentException("Four tyres: front left, front right, rear left, rear right.", nameof(tires));
        SingleTrackState own = plan.Chassis();
        own.Compound = compound is not null && _sets.ContainsKey(compound) ? compound : null;
        ReadOnlySpan<TireState> corners = [own.FrontLeft, own.FrontRight, own.RearLeft, own.RearRight];
        for (int i = 0; i < 4; i++)
        {
            corners[i].SurfaceTempC = tires[i].SurfaceTempC;
            corners[i].CoreTempC = tires[i].CoreTempC;
            corners[i].Wear = tires[i].Wear;
            corners[i].LoadN = tires[i].LoadN;
            corners[i].SurfaceGrip = tires[i].SurfaceGrip;
        }
    }

    public virtual void WarmTires(CarState state, float surfaceTempC, float coreTempC) =>
        state.Chassis().WarmTires(surfaceTempC, coreTempC);

    public virtual PowertrainState Stores(CarState state) => state.Chassis().Energy;

    public virtual void Fill(CarState state, PowertrainState stores)
    {
        stores.Clamp();
        state.Chassis().Energy = stores;
    }

    public virtual void FitTires(CarState state, string compound)
    {
        if (!_sets.TryGetValue(compound, out TireSet? set))
            throw new ArgumentException(
                $"This car does not run '{compound}'; it runs {string.Join(", ", Compounds.Select(c => c.Id))}.",
                nameof(compound));
        state.Chassis().Compound = compound;
        set.Install(state);
    }

    public virtual void InstallFreshTires(CarState state) =>
        FitTires(state, state.Chassis().Compound is { } id && _sets.ContainsKey(id) ? id : Compounds[0].Id);

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
                state.Chassis().Energy, strategy.Rung(CarPhysics.OutputLadderIndex, Powertrain.OutputLadder), state.Speed, mass,
                Config.MaxDriveAcceleration, state.Chassis().PushToPass.Active),
            Powertrain.OutputAvailability(state.Chassis().Energy));
    }

    private static readonly CarButton[] ButtonList = [new("push_to_pass", "Push")];

    /// <summary>Push-to-pass, the chassis's one button.</summary>
    public virtual IReadOnlyList<CarButton> Buttons => ButtonList;

    public virtual CarButtonReading Read(CarState state, int button)
    {
        if (button != CarPhysics.PushToPassButton)
            throw new ArgumentOutOfRangeException(nameof(button));
        PushToPassState push = state.Chassis().PushToPass;
        PushToPassRules rules = Config.PushToPass;
        float left = push.RemainingSeconds(rules);
        bool ready = !push.Locked && !push.Active && push.CooldownSeconds <= 0f && left > 0f;
        string status = push.Locked ? "LOCKED"
            : push.Active ? $"ON  {rules.MaxActivationSeconds - push.ActivationSeconds:0}s"
            : push.CooldownSeconds > 0f ? $"WAIT  {push.CooldownSeconds:0}s"
            : left > 0f ? "READY" : "USED";
        return new CarButtonReading(ready, push.Active, push.Locked, left, push.CooldownSeconds, $"{status}   {left:0}s LEFT");
    }

    public virtual void HoldUntilLine(CarState state)
    {
        PushToPassState push = state.Chassis().PushToPass;
        push.Locked = true;
        state.Chassis().PushToPass = push;
    }

    public virtual void CrossedLine(CarState state)
    {
        PushToPassState push = state.Chassis().PushToPass;
        if (!push.Locked)
            return;
        push.Locked = false;
        state.Chassis().PushToPass = push;
    }

    public virtual void Assume(CarState plan, int button, bool running)
    {
        if (button != CarPhysics.PushToPassButton)
            throw new ArgumentOutOfRangeException(nameof(button));
        SingleTrackState own = plan.Chassis();
        PushToPassState push = own.PushToPass;
        push.Active = running;
        own.PushToPass = push;
        // The chassis trims its aero to the wake while a push runs.
        own.ActiveAero = running ? 1f : 0f;
    }

    public virtual AxleLayout Layout => new(
        Config.FrontStaticLoadShare, Config.FrontDriveShare, Config.AdaptiveDriveSplit, Config.FixedFrontBrakeShare);

    public virtual GripUse GripUse(CarState state)
    {
        AxleTelemetry axles = state.Axles();
        return new GripUse(
            axles.FrontGripAccel, axles.RearGripAccel,
            axles.FrontLateralUse, axles.RearLateralUse,
            axles.FrontLongitudinalUse, axles.RearLongitudinalUse,
            axles.RearSlideSeverity);
    }

    /// <summary>The slip curve's own figure (<see cref="TireSlipCurve.SustainablePeakShare"/>).</summary>
    public virtual float SustainablePeakShare => TireSlipCurve.SustainablePeakShare;

    public virtual float PeakSlipAngleRadians => TireSlipCurve.PeakSlipAngleRadians;

    public virtual float SlipAngleAtShare(float share) => TireSlipCurve.InverseEvaluate(share);

    public virtual float LoadSensitivity(CarState state) => TiresOn(state).Front.Config.LoadSensitivity;

    public virtual void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy)
    {
        PowertrainState energy = state.Chassis().Energy;
        IReadOnlyList<PowertrainResourceInfo> declared = Powertrain.Resources;
        for (int i = 0; i < declared.Count && i < PowertrainState.Capacity; i++)
        {
            PowertrainResourceInfo info = declared[i];
            dashboard.AddResource(new DashboardResource(info.Id, info.Label, energy[i], info.FullMassKg * energy[i]));
        }

        SingleTrackState own = state.Chassis();
        AddTire(dashboard, "FL", own.FrontLeft, TireWindow(state, WheelId.FrontLeft));
        AddTire(dashboard, "FR", own.FrontRight, TireWindow(state, WheelId.FrontRight));
        AddTire(dashboard, "RL", own.RearLeft, TireWindow(state, WheelId.RearLeft));
        AddTire(dashboard, "RR", own.RearRight, TireWindow(state, WheelId.RearRight));

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
