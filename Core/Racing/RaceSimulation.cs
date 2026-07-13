using System;
using System.Collections.Generic;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Racing;

public sealed class RaceSimulation
{
    private const float MaxSubstepSeconds = 1f / 120f;

    private readonly List<RaceCar> _cars = [];
    private DriverInput[] _stepInputs = [];
    private float[] _stepTireEnergyEfficiencies = [];
    private CarStrategy[] _stepStrategies = [];
    private TrackPose[] _stepPoses = [];
    private TrackBoundaryContact?[] _preStepContacts = [];
    private TrackBoundaryContact?[] _sweepContacts = [];
    private CarState[] _startStates = [];
    private CarState[] _predictedStates = [];

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
        car.Progress.Reset(
            Track,
            pose,
            TrackBoundaryResolver.Classify(pose),
            contact.HasValue
        );
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
        int carCount = _cars.Count;
        if (carCount == 0)
            return;

        EnsureStepCapacity(carCount);

        // Wall correction is independent per car, but must finish for every car
        // before the shared driver snapshot is captured.
        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            _preStepContacts[i] = TrackBoundaryResolver.ResolveCurrent(
                Track,
                car.State,
                car.Collision
            );
        }

        RaceCarSnapshot[] carSnapshots = new RaceCarSnapshot[carCount];
        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            TrackPose pose = Track.Project(car.State.Position);
            _stepPoses[i] = pose;
            _startStates[i].CopyFrom(car.State);
            carSnapshots[i] = RaceCarSnapshot.Capture(car, pose);
        }

        RaceFrameSnapshot frame = new(RaceTimeSeconds, carSnapshots);
        float airTempC = Environment.AirTempC;
        float trackTempC = Environment.TrackTempC;

        // Driver evaluation is a read phase: every driver receives the exact same
        // pre-physics vehicle snapshot regardless of car insertion order.
        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            RaceDriverFrameContext context = new(
                car,
                Track,
                _stepPoses[i],
                Environment,
                RaceTimeSeconds,
                frame,
                i
            );
            DriverInput input = car.Driver.GetControl(in context, dt);

            _stepInputs[i] = input;
            _stepStrategies[i] = car.Strategy;
            _stepTireEnergyEfficiencies[i] = car.Driver.TireEnergyEfficiency;
            car.LastInput = input;
            if (_preStepContacts[i].HasValue)
                car.LastBoundaryContact = _preStepContacts[i];
        }

        // Predict every car from its frozen start state before committing any of
        // the results to the live race state.
        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            CarPhysicsStepInput physicsInput = new(
                _stepInputs[i],
                _stepStrategies[i],
                airTempC,
                trackTempC,
                _stepTireEnergyEfficiencies[i]
            );

            CarState startState = _startStates[i];
            CarState predictedState = _predictedStates[i];
            predictedState.CopyFrom(startState);
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
            _sweepContacts[i] = sweepContact;
        }

        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            car.State.CopyFrom(_predictedStates[i]);
            if (_sweepContacts[i].HasValue)
                car.LastBoundaryContact = _sweepContacts[i];
        }

        ResolveContactsAndWalls();

        foreach (RaceCar car in _cars)
        {
            TrackPose finalPose = Track.Project(car.State.Position);
            TrackRegion region = TrackBoundaryResolver.Classify(finalPose);
            car.Progress.Update(Track, finalPose, region, car.LastBoundaryContact.HasValue);
        }
    }

    private void EnsureStepCapacity(int required)
    {
        if (_stepInputs.Length >= required)
            return;

        int previousCapacity = _stepInputs.Length;
        int capacity = Math.Max(required, Math.Max(4, previousCapacity * 2));
        Array.Resize(ref _stepInputs, capacity);
        Array.Resize(ref _stepTireEnergyEfficiencies, capacity);
        Array.Resize(ref _stepStrategies, capacity);
        Array.Resize(ref _stepPoses, capacity);
        Array.Resize(ref _preStepContacts, capacity);
        Array.Resize(ref _sweepContacts, capacity);
        Array.Resize(ref _startStates, capacity);
        Array.Resize(ref _predictedStates, capacity);

        for (int i = previousCapacity; i < capacity; i++)
        {
            _startStates[i] = new CarState();
            _predictedStates[i] = new CarState();
        }
    }

    private void ResolveContactsAndWalls()
    {
        int iterations = 1;
        foreach (RaceCar car in _cars)
            iterations = Math.Max(iterations, car.Collision.SolverIterations);

        for (int i = 0; i < iterations; i++)
        {
            bool changed = ResolveCurrentWalls();
            changed |= CarContactResolver.ResolveUntilSeparated(_cars);
            changed |= ResolveCurrentWalls();
            if (!changed)
                break;
        }
    }

    private bool ResolveCurrentWalls()
    {
        bool resolvedAny = false;
        foreach (RaceCar car in _cars)
        {
            TrackBoundaryContact? contact = TrackBoundaryResolver.ResolveCurrent(Track, car.State, car.Collision);
            if (contact.HasValue)
            {
                car.LastBoundaryContact = contact;
                resolvedAny = true;
            }
        }
        return resolvedAny;
    }
}

public sealed class RaceEnvironment
{
    public float AirTempC { get; set; } = 25f;
    public float TrackTempC { get; set; } = 35f;
}
