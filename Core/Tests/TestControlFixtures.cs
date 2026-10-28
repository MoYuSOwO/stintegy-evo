using System;
using Official.SingleTrack;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Deterministic physical commands for tests that exercise the simulation
/// without depending on a production driving policy.
/// </summary>
internal static class TestControlFixtures
{
    public static RaceCar ExternalCar(
        string id,
        CarState state,
        DriverInput input = default,
        CarConfig? config = null,
        TireConfig? tires = null
    )
    {
        RaceCar car = new(
            id,
            new SingleTrackCar(config ?? new CarConfig(), tires ?? WarmTires()),
            state: state
        );
        car.ExternalInput = input;
        return car;
    }

    public static TestTires WarmTires() => new(new TireConfig
    {
    }, 90f, 90f);

    /// <summary>
    /// A car driven by <paramref name="command"/>: a curvature and an
    /// acceleration each step, the wheels turned for the curvature as a
    /// driver who knew the car would turn them (<see cref="SteeringEstimate"/>).
    /// </summary>
    public static RaceCar CurvatureCar(
        string id,
        CarState state,
        Func<RaceCarSnapshot, (float Curvature, float Accel)> command,
        CarConfig? config = null,
        TireConfig? tires = null
    ) => new(
        id,
        new SingleTrackCar(config ?? new CarConfig(), tires ?? WarmTires()),
        new Driver(new DriverProfile(id, new(), 42), new CurvatureDriver(command)),
        state
    );
}

/// <summary>
/// Steering for a test that asks for a curvature: the road-wheel angle a
/// driver who knew the car exactly would turn to, read off the car as it is.
/// </summary>
internal static class TestSteering
{
    public static float For(CarState state, CarConfig car, TireConfig tires, float curvature, RoadAttitude? road = null) =>
        CarPhysics.EstimateSteering(state, car, tires, road ?? RoadAttitude.Flat).AngleFor(curvature);

    /// <summary>For a car in a simulation, on a level road.</summary>
    public static DriverInput For(RaceCar car, Ask ask) =>
        new(car.Car.As<ICarEnvelope>().EstimateSteering(car.State, RoadAttitude.Flat).AngleFor(ask.Curvature), ask.Accel)
        {
            Axles = new AxleBias(ask.FrontBrakeBiasOffset, 0f)
        };

    /// <summary>The pedals as given, the wheels turned for <paramref name="curvature"/>.</summary>
    public static DriverInput Holding(CarState state, CarConfig car, TireConfig tires, float curvature, float accel, RoadAttitude? road = null) =>
        new(For(state, car, tires, curvature, road), accel);
}

/// <summary>
/// What a test asks a car for: a curvature and the pedals. The car is given
/// a wheel angle, so this is steered each step (<see cref="For"/>) as a
/// driver who knew the car would steer it.
/// </summary>
internal readonly record struct Ask(float Curvature, float Accel, float FrontBrakeBiasOffset = 0f)
{
    public DriverInput For(CarState state, CarConfig car, TireConfig tires, RoadAttitude? road = null) =>
        new(TestSteering.For(state, car, tires, Curvature, road), Accel) { Axles = new AxleBias(FrontBrakeBiasOffset, 0f) };
}

/// <summary>Holds what <paramref name="command"/> says each step, steering for its curvature.</summary>
internal sealed class CurvatureDriver(Func<RaceCarSnapshot, (float Curvature, float Accel)> command) : IDriverController
{
    public DriverInput GetControl(in DriverContext context, float dt)
    {
        RaceCarSnapshot car = context.Car;
        (float curvature, float accel) = command(car);
        return new DriverInput(car.Steering.AngleFor(curvature), accel);
    }
}

/// <summary>
/// A compound for a test and how warm a fresh set of it starts. How warm
/// tyres start is the paddock's (the blankets), not the compound's, so a
/// test that wants them warmer or colder says so here and the set is
/// brought there when it is put on (<see cref="TestTireExtensions"/>).
/// Passes for its compound wherever a <see cref="TireConfig"/> is asked for.
/// </summary>
public sealed record TestTires(TireConfig Config, float SurfaceTempC = Tire.BlanketTempC, float CoreTempC = Tire.BlanketTempC)
{
    public static implicit operator TireConfig(TestTires tires) => tires.Config;
    public static implicit operator TestTires(TireConfig config) => new(config);
}

public static class TestTireExtensions
{
    /// <summary>A fresh set on every corner, at the test's temperatures.</summary>
    public static void InstallFreshTires(this CarState state, TestTires tires)
    {
        state.Chassis().InstallFreshTires();
        state.Chassis().WarmTires(tires.SurfaceTempC, tires.CoreTempC);
    }
}

/// <summary>Short ways for tests to reach a car's features and the chassis's own state.</summary>
internal static class TestFeatureExtensions
{
    /// <summary>The car's <typeparamref name="T"/> feature; the test fails if the car has none.</summary>
    public static T As<T>(this ICar car) where T : class =>
        car as T ?? throw new InvalidOperationException($"{car.GetType().Name} has no {typeof(T).Name}.");

    /// <summary>The state with the chassis's stores set to <paramref name="energy"/>.</summary>
    public static CarState WithEnergy(this CarState state, PowertrainState energy)
    {
        state.Chassis().Energy = energy;
        return state;
    }
}
