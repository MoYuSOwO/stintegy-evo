using System;
using TheStint.Core.Cars;
using TheStint.Core.Drivers;

namespace TheStint.Core.Racing;

public sealed class RaceCar
{
    public RaceCar(
        string id,
        CarConfig carConfig,
        TireConfig tireConfig,
        IRaceDriver driver,
        CarState? state = null,
        CarCollisionConfig? collision = null,
        bool installFreshTires = true
    )
    {
        Id = string.IsNullOrWhiteSpace(id) ? throw new ArgumentException("Car id is required.", nameof(id)) : id;
        CarConfig = carConfig ?? throw new ArgumentNullException(nameof(carConfig));
        TireConfig = tireConfig ?? throw new ArgumentNullException(nameof(tireConfig));
        Driver = driver ?? throw new ArgumentNullException(nameof(driver));
        State = state ?? new CarState();
        Collision = collision ?? new CarCollisionConfig();

        if (installFreshTires)
            State.InstallFreshTires(TireConfig);
    }

    public string Id { get; }
    public CarState State { get; }
    public CarConfig CarConfig { get; }
    public TireConfig TireConfig { get; private set; }
    public CarStrategy Strategy { get; set; } = CarStrategy.Default;
    public IRaceDriver Driver { get; set; }
    public RaceProgress Progress { get; } = new();
    public CarCollisionConfig Collision { get; }
    public DriverInput LastInput { get; internal set; }
    public TrackBoundaryContact? LastBoundaryContact { get; internal set; }

    public void InstallFreshTires(TireConfig tireConfig)
    {
        TireConfig = tireConfig ?? throw new ArgumentNullException(nameof(tireConfig));
        State.InstallFreshTires(TireConfig);
    }
}

public sealed class CarCollisionConfig
{
    public float LengthMeters { get; init; } = 4.8f;
    public float WidthMeters { get; init; } = 1.9f;
    public float Restitution { get; init; } = 0.05f;
    public float Friction { get; init; } = 0.35f;
    public float WallRestitution { get; init; } = 0.02f;
    public float WallFriction { get; init; } = 0.5f;
    public float ReferenceImpactSpeed { get; init; } = 25f;
    public int SolverIterations { get; init; } = 6;

    public float HalfLengthMeters => LengthMeters * 0.5f;
    public float HalfWidthMeters => WidthMeters * 0.5f;
}
