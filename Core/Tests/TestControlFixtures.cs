using StintegyEVO.Core.Cars;
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
        TireConfig? tires = null,
        CarCollisionConfig? collision = null
    )
    {
        RaceCar car = new(
            id,
            config ?? new CarConfig(),
            tires ?? WarmTires(),
            state: state,
            collision: collision
        );
        car.ExternalInput = input;
        return car;
    }

    public static TestTires WarmTires() => new(new TireConfig
    {
    }, 90f, 90f);
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
        state.InstallFreshTires();
        state.WarmTires(tires.SurfaceTempC, tires.CoreTempC);
    }
}
