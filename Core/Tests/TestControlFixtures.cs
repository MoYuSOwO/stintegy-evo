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

    public static TireConfig WarmTires() => new()
    {
        StartingSurfaceTempC = 90f,
        StartingCoreTempC = 90f
    };
}
