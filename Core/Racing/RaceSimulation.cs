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
    private int[] _overtakeAssistCrossings = [];
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
    /// How close a car must be to the one ahead, as it crosses the line, to
    /// earn overtake mode for the lap that follows.
    /// </summary>
    private const float OvertakeAssistGapSeconds = 1f;

    private static float GaussianFalloff(float gap, float decayLengthMeters)
    {
        float scale = MathF.Max(decayLengthMeters, 1e-3f);
        float normalized = gap / scale;
        return MathF.Exp(-normalized * normalized);
    }

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
            float strongestDownforceDeficit = 0f;
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
                // The wake's two faces part company with distance: it rises
                // off the road as it ages, so the follower's body stays sunk
                // in slowed air - the drag relief keeps a long hyperbolic
                // tail - while wings and floor climb out of it within a
                // couple of car lengths, so everything the downforce model
                // reads decays on the short Gaussians.
                float towDeficit = wakeCar.WakeVelocityDeficit /
                                   (1f + gap / MathF.Max(
                                       wakeCar.WakeTowHalfDistanceMeters,
                                       1e-3f
                                   ));
                float downforceDeficit = wakeCar.WakeVelocityDeficit *
                                         GaussianFalloff(
                                             gap,
                                             wakeCar.WakeDownforceDecayLengthMeters
                                         );
                float downforceLoss = wakeCar.WakeDownforceDisruption *
                                      GaussianFalloff(
                                          gap,
                                          wakeCar.WakeDirtyAirDecayLengthMeters
                                      );

                // Across the wake the deficit falls away from the middle, and
                // the middle is wider the further back it is read. Downforce
                // recovers faster sideways than the tow does.
                float halfWidth = other.WidthMeters * 0.5f +
                                  gap * WakeSpreadPerMeter;
                float sideways = MathF.Abs(other.TrackD - ego.TrackD) /
                                 MathF.Max(halfWidth, 0.1f);
                towDeficit *= MathF.Exp(-sideways * sideways);
                float lateralRecovery = MathF.Exp(
                    -DirtyAirLateralRecovery * sideways * sideways
                );
                downforceDeficit *= lateralRecovery;
                downforceLoss *= lateralRecovery;

                strongestTow = MathF.Max(strongestTow, towDeficit);
                strongestDownforceDeficit = MathF.Max(
                    strongestDownforceDeficit,
                    downforceDeficit
                );
                strongestDirtyAir = MathF.Max(
                    strongestDirtyAir,
                    downforceLoss
                );
            }

            // The drag relief of a tow is universal; how much of the
            // disturbed air reaches the working surfaces is the follower's
            // own trait.
            float sensitivity = MathF.Max(
                0f,
                _cars[i].CarConfig.DirtyAirSensitivity
            );
            _cars[i].State.AirVelocityDeficit = strongestTow;
            _cars[i].State.DownforceVelocityDeficit = MathF.Min(
                1f,
                strongestDownforceDeficit * sensitivity
            );
            _cars[i].State.WakeDownforceLoss = MathF.Min(
                1f,
                strongestDirtyAir * sensitivity
            );
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
            )
            {
                RoadAttitude = SampleRoadAttitude(car)
            };

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

        UpdateOvertakeAssist();
    }

    /// <summary>
    /// Overtake mode is settled once a lap, as the car crosses the line: a
    /// car within one second of whoever is directly ahead of it on the road
    /// carries the mode for the whole of the next lap. Deciding it at a
    /// single point, for a whole lap, keeps the right from flickering with
    /// every metre of gap, and keeps the rule something a driver could be
    /// told rather than something only a solver could follow. The right
    /// swaps sides on its own: whoever completes a pass stops qualifying at
    /// the next crossing, and the car just passed starts to. Nobody carries
    /// the mode off the grid - the first decision waits for the first real
    /// crossing, the way the real aids sit disabled on an opening lap.
    /// </summary>
    private void UpdateOvertakeAssist()
    {
        if (_overtakeAssistCrossings.Length < _cars.Count)
        {
            int previous = _overtakeAssistCrossings.Length;
            Array.Resize(ref _overtakeAssistCrossings, _cars.Count);
            for (int i = previous; i < _overtakeAssistCrossings.Length; i++)
                _overtakeAssistCrossings[i] = int.MinValue;
        }

        for (int i = 0; i < _cars.Count; i++)
        {
            RaceCar car = _cars[i];
            // Continuous race distance over lap length is a crossing counter
            // that, unlike the clamped lap number, still ticks for a car
            // gridded behind the line - and only forward progress advances
            // it, so sliding backwards across the line cannot re-roll the
            // decision at a hundred and twenty hertz.
            int crossings = (int)MathF.Floor(
                car.Progress.RaceDistanceMeters / Track.LengthMeters
            );
            if (_overtakeAssistCrossings[i] == int.MinValue)
            {
                _overtakeAssistCrossings[i] = crossings;
                continue;
            }
            if (crossings <= _overtakeAssistCrossings[i])
                continue;
            _overtakeAssistCrossings[i] = crossings;

            float ownDistance = car.Progress.RaceDistanceMeters;
            float nearestAhead = float.PositiveInfinity;
            for (int j = 0; j < _cars.Count; j++)
            {
                if (j == i)
                    continue;
                float delta = OnTrackDistanceAhead(
                    _cars[j].Progress.RaceDistanceMeters,
                    ownDistance,
                    Track.LengthMeters
                );
                if (delta > 0f && delta < nearestAhead)
                    nearestAhead = delta;
            }

            float speed = MathF.Max(car.State.Speed, 1f);
            car.State.OvertakeAssist =
                nearestAhead / speed <= OvertakeAssistGapSeconds ? 1f : 0f;
        }
    }

    /// <summary>
    /// Distance up the road from one car to another, whole laps removed.
    /// The wake the mode answers sits on the road, so a backmarker five
    /// metres ahead counts and a rival a lap up in the standings does not.
    /// </summary>
    internal static float OnTrackDistanceAhead(
        float otherRaceDistanceMeters,
        float ownRaceDistanceMeters,
        float trackLengthMeters
    )
    {
        float delta = (otherRaceDistanceMeters - ownRaceDistanceMeters) %
                      trackLengthMeters;
        if (delta < 0f)
            delta += trackLengthMeters;
        return delta;
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

    /// <summary>
    /// Reads the road under a car. The bank is taken at the car's own
    /// lateral offset rather than at the centreline, so a progressively
    /// banked corner really does reward the car that runs high.
    ///
    /// Where the car is round the lap and across it was worked out at the
    /// end of the last substep and kept. Projecting again would be a third
    /// search per car per substep to learn something that has not moved
    /// half a metre, and every other reading this step is taken from the
    /// same instant.
    /// </summary>
    private RoadAttitude SampleRoadAttitude(RaceCar car)
    {
        TrackSample sample = Track.Sample(car.Progress.CurrentS);
        if (sample.Grade == 0f &&
            sample.BankSlope == 0f &&
            sample.BankCurvature == 0f &&
            sample.VerticalCurvature == 0f)
        {
            return RoadAttitude.Flat;
        }
        return new RoadAttitude(
            sample.Grade,
            sample.BankSlopeAt(car.Progress.CurrentD),
            sample.VerticalCurvature
        );
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
