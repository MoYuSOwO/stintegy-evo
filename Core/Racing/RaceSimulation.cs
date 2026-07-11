using System;
using System.Collections.Generic;
using TheStint.Core.Cars;
using TheStint.Core.Drivers;
using TheStint.Core.Track;

namespace TheStint.Core.Racing;

public sealed class RaceSimulation
{
    private const float MaxSubstepSeconds = 1f / 120f;

    private readonly List<RaceCar> _cars = [];

    public RaceSimulation(TrackData track, RaceEnvironment? environment = null)
    {
        Track = track ?? throw new ArgumentNullException(nameof(track));
        Environment = environment ?? new RaceEnvironment();
    }

    public TrackData Track { get; }
    public RaceEnvironment Environment { get; }
    public IReadOnlyList<RaceCar> Cars => _cars;
    public float RaceTimeSeconds { get; private set; }

    public void AddCar(RaceCar car)
    {
        ArgumentNullException.ThrowIfNull(car);

        TrackBoundaryContact? contact = TrackBoundaryResolver.ResolveCurrent(Track, car.State, car.Collision);
        TrackPose pose = Track.Project(car.State.Position);
        car.Progress.Reset(pose, TrackBoundaryResolver.Classify(pose), contact.HasValue);
        car.LastBoundaryContact = contact;
        RaceDriverInitContext context = new(car, Track, pose, Environment, RaceTimeSeconds);
        car.Driver.Initialize(in context);
        _cars.Add(car);
    }

    public void Step(float dt)
    {
        if (dt <= 0f)
            return;

        foreach (RaceCar car in _cars)
            car.LastBoundaryContact = null;

        float remaining = dt;
        while (remaining > 0f)
        {
            float substep = MathF.Min(remaining, MaxSubstepSeconds);
            StepSubstep(substep);
            RaceTimeSeconds += substep;
            remaining -= substep;
        }
    }

    private void StepSubstep(float dt)
    {
        foreach (RaceCar car in _cars)
        {
            TrackBoundaryContact? preStepContact = TrackBoundaryResolver.ResolveCurrent(Track, car.State, car.Collision);
            TrackPose pose = Track.Project(car.State.Position);
            RaceDriverFrameContext context = new(car, Track, pose, Environment, RaceTimeSeconds);
            DriverInput input = car.Driver.GetControl(in context, dt);
            CarPhysicsStepInput physicsInput = new(
                input,
                car.Strategy,
                Environment.AirTempC,
                Environment.TrackTempC,
                car.Driver.TireEnergyEfficiency
            );

            car.LastInput = input;
            if (preStepContact.HasValue)
                car.LastBoundaryContact = preStepContact;

            CarState startState = car.State.Clone();
            CarState predictedState = car.State.Clone();
            CarPhysics.Step(
                predictedState,
                car.CarConfig,
                car.TireConfig,
                physicsInput,
                dt
            );

            TrackBoundaryContact? sweepContact = TrackBoundaryResolver.ResolveSweep(
                Track,
                startState,
                predictedState,
                car.Collision
            );
            car.State.CopyFrom(predictedState);
            if (sweepContact.HasValue)
                car.LastBoundaryContact = sweepContact;
        }

        ResolveContactsAndWalls();

        foreach (RaceCar car in _cars)
        {
            TrackPose finalPose = Track.Project(car.State.Position);
            TrackRegion region = TrackBoundaryResolver.Classify(finalPose);
            car.Progress.Update(Track, finalPose, region, car.LastBoundaryContact.HasValue);
        }
    }

    private void ResolveContactsAndWalls()
    {
        int iterations = 1;
        foreach (RaceCar car in _cars)
            iterations = Math.Max(iterations, car.Collision.SolverIterations);

        for (int i = 0; i < iterations; i++)
        {
            ResolveCurrentWalls();
            CarContactResolver.Resolve(_cars);
            ResolveCurrentWalls();
        }
    }

    private void ResolveCurrentWalls()
    {
        foreach (RaceCar car in _cars)
        {
            TrackBoundaryContact? contact = TrackBoundaryResolver.ResolveCurrent(Track, car.State, car.Collision);
            if (contact.HasValue)
                car.LastBoundaryContact = contact;
        }
    }
}

public sealed class RaceEnvironment
{
    public float AirTempC { get; set; } = 25f;
    public float TrackTempC { get; set; } = 35f;
}
