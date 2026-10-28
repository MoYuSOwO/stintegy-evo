using System;
using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Mods;

namespace Alpha;

/// <summary>What alpha offers other mods' code.</summary>
public static class AlphaParts
{
    public const string DriverName = "steady";
}

public sealed class AlphaMod : IMod
{
    public void Register(ModContext context)
    {
        context.Registry.Drivers.Register(AlphaParts.DriverName, new SteadyDriver());
        context.Log.Info("steady is in");
    }
}

/// <summary>A mod that tries to register in another's namespace.</summary>
public sealed class Squatter : IMod
{
    public void Register(ModContext context) =>
        context.Registry.Drivers.Register($"alpha:{AlphaParts.DriverName}", new SteadyDriver());
}

public sealed class SteadyDriver : IDriverDefinition
{
    public CarNeeds Needs => CarNeeds.None;

    public IDriverController Seat(DriverSeat seat) => throw new NotSupportedException("A registry test's driver drives nothing.");
}

/// <summary>A mod with a chassis and a car built on it, on whatever tyres its car.json names.</summary>
public sealed class AlphaCars : IMod
{
    public void Register(ModContext context)
    {
        context.Registry.Chassis.Register("frame", new ChassisDeclaration("Frame", 1));
        context.Registry.Cars.Register("box", new BoxDefinition());
    }
}

/// <summary>A mod whose code registers a car its mod.json does not list.</summary>
public sealed class AlphaStrayCar : IMod
{
    public void Register(ModContext context) => context.Registry.Cars.Register("stray", new BoxDefinition());
}

public sealed class BoxDefinition : ICarDefinition
{
    public ICar Build(IReadOnlyList<TireCompound> compounds) => new Box(compounds);
}

/// <summary>A box that rolls on at its speed, on the compounds it is given.</summary>
public sealed class Box(IReadOnlyList<TireCompound> compounds) : ICar, ICarTires
{
    private sealed class Fitted : ICarStateExtension
    {
        public string? Compound { get; set; }
        public ICarStateExtension Clone() => new Fitted { Compound = Compound };
        public void CopyFrom(ICarStateExtension other) => Compound = ((Fitted)other).Compound;
    }

    private static Fitted Own(CarState state) => (Fitted)(state.Own ??= new Fitted());

    public CarSpec Spec { get; } = new()
    {
        WheelBaseMeters = 2.5f,
        FrontAxleOffsetMeters = 1.2f,
        RearAxleOffsetMeters = 1.3f,
        TrackWidthMeters = 1.5f,
        YawInertiaKgM2 = 1200f,
        MaximumBrakeDeceleration = 9f,
        KinematicBelowSpeed = 1f,
        Body = new CarBody { LengthMeters = 4f, WidthMeters = 1.8f, Restitution = 0.1f, Friction = 0.3f, WallRestitution = 0.1f, WallFriction = 0.5f },
        Wake = new CarWake { VelocityDeficit = 0f, TowHalfDistanceMeters = 10f, DownforceDecayLengthMeters = 10f, DownforceDisruption = 0f, DirtyAirDecayLengthMeters = 10f },
        DirtyAirSensitivity = 0f,
    };

    public IReadOnlyList<ModeLadder> Ladders => [];
    public float TotalMassKg(CarState state) => 800f;
    public void Step(CarState state, CarStepInput input, float dt) =>
        state.Position += new Vector2(MathF.Cos(state.Heading), MathF.Sin(state.Heading)) * state.Speed * dt;
    public void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy) => dashboard.SetMass(800f);
    public IReadOnlyList<TelemetryChannel> TelemetryChannels => [];
    public void ReadTelemetry(CarState state, Span<float> values) { }

    public IReadOnlyList<TireCompound> Compounds { get; } = compounds;
    public void FitTires(CarState state, string compound) => Own(state).Compound = compound;
    public void InstallFreshTires(CarState state) => Own(state).Compound ??= Compounds.Count > 0 ? Compounds[0].Id : null;
    public void WarmTires(CarState state, float surfaceTempC, float coreTempC) { }
    public string? Compound(CarState state) => Own(state).Compound;
    public TireReading Tire(CarState state, WheelId wheel) => new(70f, 70f, 0f, 2000f, 1f);
    public void Assume(CarState plan, string? compound, ReadOnlySpan<TireReading> tires) => Own(plan).Compound = compound;
}
