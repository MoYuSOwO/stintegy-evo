using System;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;

namespace StintegyEVO.Core.Racing;

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

    /// <summary>
    /// How long this car spent against a barrier during the last call to
    /// <see cref="RaceSimulation.Step"/>, in seconds.
    ///
    /// Whether a car touched something is a different question from how long
    /// it leant on it, and a caller stepping the simulation a tenth of a
    /// second at a time can only see the first. Scoring a race on that alone
    /// prices a glance off the wall the same as scraping it the whole way,
    /// which leaves a car that has already touched with no reason to get off
    /// until the step ends.
    /// </summary>
    public float BoundaryContactSeconds { get; internal set; }

    internal bool TouchedBoundaryThisSubstep { get; set; }
    public bool HitCarThisStep { get; internal set; }

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
