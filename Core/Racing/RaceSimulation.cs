using System;
using System.Collections.Generic;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Racing;

public sealed class RaceSimulation
{
    private const float MaxDriverStepSeconds = 1f / 60f;
    private const float MaxSubstepSeconds = 1f / 120f;
    private const float MinimumStepSeconds = 1e-7f;

    private readonly List<RaceCar> _cars = [];
    private DriverInput[] _stepInputs = [];
    private float[] _stepTireEnergyEfficiencies = [];
    private float[] _stepCorneringEfficiencies = [];
    private float[] _stepLimitSettleUses = [];
    private CarStrategy[] _stepStrategies = [];
    private TrackPose[] _stepPoses = [];
    private TrackBoundaryContact?[] _preStepContacts = [];
    private TrackBoundaryContact?[] _sweepContacts = [];
    private CarState[] _startStates = [];
    private CarState[] _predictedStates = [];
    private TrafficMotionPlan?[] _stepTrafficMotionPlans = [];
    private TrafficMotionPlan?[] _previousTrafficMotionPlans = [];
    private readonly RacingRoomCoordinator _racingRoomCoordinator = new();

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

        float remainingDriverTime = dt;
        while (remainingDriverTime > MinimumStepSeconds)
        {
            float driverStep = MathF.Min(
                remainingDriverTime,
                MaxDriverStepSeconds
            );
            EvaluateDrivers(driverStep);

            float remainingPhysicsTime = driverStep;
            while (remainingPhysicsTime > MinimumStepSeconds)
            {
                float physicsStep = MathF.Min(
                    remainingPhysicsTime,
                    MaxSubstepSeconds
                );
                StepPhysicsSubstep(physicsStep);
                RaceTimeSeconds += physicsStep;
                remainingPhysicsTime -= physicsStep;
            }
            remainingDriverTime -= driverStep;
        }
    }

    private void EvaluateDrivers(float dt)
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
            carSnapshots[i] = RaceCarSnapshot.Capture(
                car,
                pose,
                Track.LengthMeters
            );
            _stepTrafficMotionPlans[i] = null;
        }

        ApplyWakeEffects(carSnapshots);
        RacingRoomSnapshot racingRoom = _racingRoomCoordinator.Update(
            carSnapshots
        );

        // Planning is a write-only phase over one frozen physical snapshot.
        // No driver can read another driver's partially prepared plan. The
        // separate previous-plan buffers are stable snapshots captured after
        // the preceding evaluation phase.
        RaceFrameSnapshot planningFrame = new(
            RaceTimeSeconds,
            carSnapshots,
            _stepTrafficMotionPlans,
            _previousTrafficMotionPlans,
            racingRoom
        );
        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            if (car.Driver is not ITrafficMotionPlanSource source)
                continue;

            RaceDriverFrameContext planningContext = new(
                car,
                Track,
                _stepPoses[i],
                Environment,
                RaceTimeSeconds,
                planningFrame,
                i
            );
            source.PrepareTrafficMotionPlan(in planningContext, dt);
        }

        // Barrier: only after every source has prepared may any submitted plan
        // become visible to the shared decision frame.
        for (int i = 0; i < carCount; i++)
        {
            _stepTrafficMotionPlans[i] = _cars[i].Driver is
                ITrafficMotionPlanSource source
                    ? source.FreezeTrafficMotionPlan()
                    : null;
        }

        RaceFrameSnapshot frame = new(
            RaceTimeSeconds,
            carSnapshots,
            _stepTrafficMotionPlans,
            racingRoom
        );

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
            _stepCorneringEfficiencies[i] = car.Driver.CorneringEfficiency;
            _stepLimitSettleUses[i] = car.Driver.LimitSettleUse;
            car.LastInput = input;
            if (_preStepContacts[i].HasValue)
                car.LastBoundaryContact = _preStepContacts[i];
        }

        CapturePreviousTrafficMotionPlans(carCount);
    }

    /// <summary>
    /// Distance behind a car at which the air it has dragged along has given
    /// back half the speed it had.
    ///
    /// A wake does not fade evenly. It is torn apart quickest right behind the
    /// car where the shear is fiercest, so most of a tow is gone within a few
    /// car lengths and what is left trails off slowly. Straight-line fading
    /// gets this backwards at both ends: it hands a car fifty metres back half
    /// the tow, when the truth there is nearer a seventh of it, and racecraft
    /// reading that believes it can close on a straight where it cannot.
    /// </summary>
    private const float WakeHalfDistanceMeters = 11f;

    /// <summary>
    /// The rotating, unsteady part of the wake outlives more of its close-range
    /// strength than the useful tow. This is deliberately still finite and
    /// shares the same cutoff: it is a second response to one wake, not a
    /// second invisible object trailing the car.
    /// </summary>
    private const float DirtyAirHalfDistanceMeters = 24f;

    /// <summary>
    /// Downforce recovers quickly once a car moves sideways out of the wake;
    /// experiments find it recovers faster laterally than drag.
    /// </summary>
    private const float DirtyAirLateralRecovery = 2f;

    /// <summary>
    /// How fast the wake widens with distance, as a half angle. A turbulent
    /// wake spreads into a cone of a few degrees, which is why sitting exactly
    /// behind matters when close and matters much less far back: the hole is
    /// narrow and strong at a car length and broad and weak at fifty metres.
    /// </summary>
    private const float WakeSpreadPerMeter = 0.123f;

    /// <summary>Beyond this there is nothing left worth computing.</summary>
    private const float WakeReachMeters = 80f;

    /// <summary>
    /// How much of its own speed each car's air is already carrying, because
    /// somebody in front has dragged it along.
    ///
    /// Read from one finished picture of the grid and written back before any
    /// car plans anything, so a car's tow does not depend on where it sits in
    /// the list. A car takes the strongest wake on offer rather than adding up
    /// several: two cars in line ahead punch one hole in the air, not two.
    /// </summary>
    private void ApplyWakeEffects(RaceCarSnapshot[] snapshots)
    {
        int carCount = snapshots.Length;
        for (int i = 0; i < carCount; i++)
        {
            RaceCarSnapshot ego = snapshots[i];
            float strongestTow = 0f;
            float strongestDirtyAir = 0f;
            for (int j = 0; j < carCount; j++)
            {
                if (j == i)
                    continue;

                RaceCarSnapshot other = snapshots[j];
                float along = Track.WrapS(other.TrackS - ego.TrackS);
                if (along > Track.LengthMeters * 0.5f)
                    along -= Track.LengthMeters;

                float gap = along - (ego.LengthMeters + other.LengthMeters) * 0.5f;
                if (gap <= 0f || gap >= WakeReachMeters)
                    continue;

                // A car pointed the other way punches its hole the other way.
                if (MathF.Cos(other.VelocityHeadingRadians -
                              ego.VelocityHeadingRadians) <= 0.5f)
                {
                    continue;
                }

                CarConfig wakeCar = _cars[j].CarConfig;
                float deficit = wakeCar.WakeVelocityDeficit /
                                (1f + gap / WakeHalfDistanceMeters);
                float downforceLoss = wakeCar.WakeDownforceDisruption /
                                      (1f + gap / DirtyAirHalfDistanceMeters);

                // Across the wake the deficit falls away from the middle, and
                // the middle is wider the further back it is read.
                float halfWidth = other.WidthMeters * 0.5f +
                                  gap * WakeSpreadPerMeter;
                float sideways = MathF.Abs(other.TrackD - ego.TrackD) /
                                 MathF.Max(halfWidth, 0.1f);
                deficit *= MathF.Exp(-sideways * sideways);
                downforceLoss *= MathF.Exp(
                    -DirtyAirLateralRecovery * sideways * sideways
                );

                strongestTow = MathF.Max(strongestTow, deficit);
                strongestDirtyAir = MathF.Max(
                    strongestDirtyAir,
                    downforceLoss
                );
            }
            _cars[i].State.AirVelocityDeficit = strongestTow;
            _cars[i].State.WakeDownforceLoss = strongestDirtyAir;
        }
    }

    private void StepPhysicsSubstep(float dt)
    {
        int carCount = _cars.Count;
        if (carCount == 0)
            return;

        for (int i = 0; i < carCount; i++)
            _startStates[i].CopyFrom(_cars[i].State);

        // Predict every car from its frozen start state before committing any of
        // the results to the live race state.
        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            CarPhysicsStepInput physicsInput = new(
                _stepInputs[i],
                _stepStrategies[i],
                Environment.AirTempC,
                Environment.TrackTempC,
                _stepTireEnergyEfficiencies[i],
                _stepCorneringEfficiencies[i],
                _stepLimitSettleUses[i]
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
        Array.Resize(ref _stepCorneringEfficiencies, capacity);
        Array.Resize(ref _stepLimitSettleUses, capacity);
        Array.Resize(ref _stepStrategies, capacity);
        Array.Resize(ref _stepPoses, capacity);
        Array.Resize(ref _preStepContacts, capacity);
        Array.Resize(ref _sweepContacts, capacity);
        Array.Resize(ref _startStates, capacity);
        Array.Resize(ref _predictedStates, capacity);
        Array.Resize(ref _stepTrafficMotionPlans, capacity);
        Array.Resize(ref _previousTrafficMotionPlans, capacity);

        for (int i = previousCapacity; i < capacity; i++)
        {
            _startStates[i] = new CarState();
            _predictedStates[i] = new CarState();
        }
    }

    private void CapturePreviousTrafficMotionPlans(int carCount)
    {
        for (int i = 0; i < carCount; i++)
        {
            TrafficMotionPlan? current = _stepTrafficMotionPlans[i];
            if (current is null)
            {
                _previousTrafficMotionPlans[i]?.Clear();
                continue;
            }

            TrafficMotionPlan snapshot =
                _previousTrafficMotionPlans[i] ??= new TrafficMotionPlan();
            snapshot.CopyFrom(current);
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
