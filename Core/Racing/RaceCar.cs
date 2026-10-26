using System;
using System.Collections.Generic;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;

namespace StintegyEVO.Core.Racing;

public sealed class RaceCar
{
    public RaceCar(
        string id,
        ICar car,
        Driver? driver = null,
        CarState? state = null,
        CarCollisionConfig? collision = null,
        bool installFreshTires = true
    )
    {
        Id = string.IsNullOrWhiteSpace(id) ? throw new ArgumentException("Car id is required.", nameof(id)) : id;
        Car = car ?? throw new ArgumentNullException(nameof(car));
        if (driver is not null)
            CheckFits(driver, car);
        Driver = driver;
        State = state ?? new CarState();
        Collision = collision ?? new CarCollisionConfig();

        if (installFreshTires && Car is ICarTires tires)
            tires.InstallFreshTires(State);
    }

    /// <summary>
    /// A driver is only put in a car that has what the driver cannot drive
    /// without (<see cref="IDriverController.Needs"/>).
    /// </summary>
    private static void CheckFits(Driver driver, ICar car)
    {
        IReadOnlyList<Type> missing = driver.Controller.Needs.MissingFrom(car);
        if (missing.Count > 0)
            throw new CarUnfitForDriverException(driver.Profile.Id, car, missing);
    }

    /// <summary>A car on the game's own chassis, made of these numbers.</summary>
    public RaceCar(
        string id,
        CarConfig carConfig,
        TireConfig tireConfig,
        Driver? driver = null,
        CarState? state = null,
        CarCollisionConfig? collision = null,
        bool installFreshTires = true
    ) : this(id, new SingleTrackCar(carConfig, tireConfig), driver, state, collision, installFreshTires)
    {
    }

    public string Id { get; }
    public CarState State { get; }

    /// <summary>The kind of car this is: everything the race knows about how it moves.</summary>
    public ICar Car { get; }
    public CarStrategy Strategy { get; set; } = CarStrategy.Default;
    public Driver? Driver { get; }

    /// <summary>
    /// A held command supplied by an external host when no controller is attached.
    /// Defaults to zero; Core does not invent an autonomous fallback driver.
    /// Controller-driven entries ignore this input.
    /// </summary>
    public DriverInput ExternalInput { get; set; }
    public RaceProgress Progress { get; } = new();

    /// <summary>
    /// Race control's hold on the car's buttons (<see cref="ICarButtons"/>),
    /// for the start and every restart: none does anything until this car
    /// next crosses the line, which lifts it. Nothing on a car without buttons.
    /// </summary>
    public void HoldButtonsUntilLine()
    {
        if (Car is ICarButtons buttons)
            buttons.HoldUntilLine(State);
    }
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

    /// <summary>A new set of what is on, on a car with changeable tyres; nothing on one without.</summary>
    public void InstallFreshTires() => (Car as ICarTires)?.InstallFreshTires(State);
}

public sealed class CarCollisionConfig
{
    public float LengthMeters { get; init; } = 4.8f;
    public float WidthMeters { get; init; } = 1.9f;
    public float Restitution { get; init; } = 0.05f;
    public float Friction { get; init; } = 0.35f;
    /// <summary>
    /// How much of the speed into the barrier comes back out of it. Full-scale
    /// crash tests into rigid barriers measure about a tenth: the car
    /// crumples and the barrier hardly gives.
    /// </summary>
    public float WallRestitution { get; init; } = 0.1f;

    /// <summary>
    /// Sliding friction between the car and the barrier: carbon, aluminium
    /// and tyre sidewall on steel and concrete.
    /// </summary>
    public float WallFriction { get; init; } = 0.5f;
    public int SolverIterations { get; init; } = 6;

    public float HalfLengthMeters => LengthMeters * 0.5f;
    public float HalfWidthMeters => WidthMeters * 0.5f;
}

/// <summary>A driver put in a car without something the driver cannot drive without.</summary>
public sealed class CarUnfitForDriverException(string who, ICar car, IReadOnlyList<Type> missing)
    : InvalidOperationException(
        $"{who} cannot drive a {car.GetType().Name}: it has no {string.Join(", ", System.Linq.Enumerable.Select(missing, t => t.Name))}.")
{
    public IReadOnlyList<Type> Missing { get; } = missing;
}
