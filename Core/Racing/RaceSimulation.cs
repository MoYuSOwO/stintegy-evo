using System;
using System.Numerics;
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
    private readonly IReadOnlyList<RaceCar> _carsView;
    private readonly DriverTrackView _driverTrack;
    private RaceEnvironmentSnapshot _stepEnvironment;
    private bool _stepping;
    private DriverInput[] _stepInputs = [];
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
        _driverTrack = new DriverTrackView(Track);
        _carsView = _cars.AsReadOnly();
    }

    public TrackData Track { get; }
    public RaceEnvironment Environment { get; }
    public IReadOnlyList<RaceCar> Cars => _carsView;
    public float RaceTimeSeconds { get; private set; }

    public void AddCar(RaceCar car)
    {
        ArgumentNullException.ThrowIfNull(car);
        if (_stepping)
            throw new InvalidOperationException("Cannot change the entry list during a step.");
        foreach (RaceCar existing in _cars)
        {
            if (existing.Id == car.Id)
                throw new ArgumentException("Car ids must be unique within a race.", nameof(car));
            if (car.Driver is not null && existing.Driver is not null &&
                (existing.Driver.Profile.Id == car.Driver.Profile.Id ||
                 ReferenceEquals(existing.Driver.Controller, car.Driver.Controller)))
                throw new ArgumentException("A driver and controller can belong to only one entry.", nameof(car));
        }

        TrackBoundaryContact? contact = TrackBoundaryResolver.ResolveCurrent(Track, car.State, car.Collision, car.CarConfig);
        TrackPose pose = Track.Project(car.State.Position);
        car.Progress.Reset(
            Track,
            pose,
            TrackBoundaryResolver.Classify(pose),
            contact.HasValue
        );
        car.LastBoundaryContact = contact;
        _cars.Add(car);
        try
        {
            if (car.Driver is not null)
            {
                DriverContext context = CaptureFrameContext(car);
                _stepping = true;
                car.Driver.Controller.Initialize(in context);
            }
        }
        catch
        {
            _cars.Remove(car);
            throw;
        }
        finally { _stepping = false; }
    }

    public void Step(float dt)
    {
        if (!float.IsFinite(dt) || dt < 0f)
            throw new ArgumentOutOfRangeException(nameof(dt));
        if (_stepping)
            throw new InvalidOperationException("Race steps cannot be nested.");
        if (dt == 0f)
            return;
        _stepping = true;
        try { StepCore(dt); }
        finally { _stepping = false; }
    }

    private void StepCore(float dt)
    {
        bool firstDriverStep = true;
        float remainingDriverTime = dt;
        while (remainingDriverTime > MinimumStepSeconds)
        {
            float driverStep = MathF.Min(
                remainingDriverTime,
                MaxDriverStepSeconds
            );
            EvaluateDrivers(driverStep, firstDriverStep);
            firstDriverStep = false;

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

    /// <summary>Observes the race without advancing time, controllers, or physical state.</summary>
    public RaceFrameSnapshot CaptureFrame()
    {
        RaceCarSnapshot[] snapshots = new RaceCarSnapshot[_cars.Count];
        for (int i = 0; i < _cars.Count; i++)
        {
            RaceCar car = _cars[i];
            snapshots[i] = RaceCarSnapshot.Capture(car, Track.Project(car.State.Position), Track.LengthMeters);
        }
        CalculateWakeEffects(snapshots);
        return new RaceFrameSnapshot(RaceTimeSeconds,
            new(Environment.AirTempC, Environment.TrackTempC, Environment.SurfaceGripScalar), snapshots);
    }

    public DriverContext CaptureFrameContext(RaceCar car)
    {
        ArgumentNullException.ThrowIfNull(car);
        int index = _cars.IndexOf(car);
        if (index < 0)
            throw new ArgumentException("That car is not in this race.", nameof(car));
        if (car.Driver is null)
            throw new InvalidOperationException("This entry has no driver. Use CaptureFrame for externally controlled entries.");
        return new DriverContext(car.Driver.Profile, _driverTrack, CaptureFrame(), index);
    }

    private void EvaluateDrivers(float dt, bool resetContacts)
    {
        int carCount = _cars.Count;
        if (carCount == 0)
            return;
        EnsureStepCapacity(carCount);
        for (int i = 0; i < carCount; i++)
            _startStates[i].CopyFrom(_cars[i].State);
        RaceFrameSnapshot frame;
        try
        {
            for (int i = 0; i < carCount; i++)
            {
                RaceCar car = _cars[i];
                _preStepContacts[i] = TrackBoundaryResolver.ResolveCurrent(Track, car.State, car.Collision, car.CarConfig);
                _stepPoses[i] = Track.Project(car.State.Position);
                // Freeze all host inputs before any controller callback is invoked.
                _stepInputs[i] = car.ExternalInput;
            }

            frame = CaptureFrame();
            _stepEnvironment = frame.Environment;
            for (int i = 0; i < carCount; i++)
            {
                RaceCarSnapshot snapshot = frame[i];
                _stepStrategies[i] = snapshot.Strategy;
            }
            for (int i = 0; i < carCount; i++)
            {
                RaceCar car = _cars[i];
                if (car.Driver is not null)
                {
                    DriverContext context = new(car.Driver.Profile, _driverTrack, frame, i);
                    _stepInputs[i] = car.Driver.Controller.GetControl(in context, dt);
                }
                DriverInput input = _stepInputs[i];
                if (!float.IsFinite(input.DesiredCurvature) || !float.IsFinite(input.DesiredAccel) ||
                    !float.IsFinite(input.FrontBrakeBiasOffset))
                    throw new InvalidOperationException($"Non-finite control for car '{car.Id}'.");
            }
        }
        catch
        {
            for (int i = 0; i < carCount; i++)
                _cars[i].State.CopyFrom(_startStates[i]);
            throw;
        }
        // Publish world fields and commands only after every control validates.
        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            if (resetContacts)
            {
                car.LastBoundaryContact = null;
                car.BoundaryContactSeconds = 0f;
                car.FourWheelsOffSeconds = 0f;
                car.HitCarThisStep = false;
            }
            car.State.AirVelocityDeficit = frame[i].AirVelocityDeficit;
            car.State.DownforceVelocityDeficit = frame[i].DownforceVelocityDeficit;
            car.State.WakeDownforceLoss = frame[i].WakeDownforceLoss;
            _cars[i].LastInput = _stepInputs[i];
            if (_preStepContacts[i].HasValue)
                _cars[i].LastBoundaryContact = _preStepContacts[i];
        }
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
    private void CalculateWakeEffects(RaceCarSnapshot[] snapshots)
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
            snapshots[i] = ego with
            {
                AirVelocityDeficit = strongestTow,
                DownforceVelocityDeficit = MathF.Min(1f, strongestDownforceDeficit * sensitivity),
                WakeDownforceLoss = MathF.Min(1f, strongestDirtyAir * sensitivity)
            };
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
                _stepEnvironment.AirTempC,
                _stepEnvironment.TrackTempC
            )
            {
                RoadAttitude = SampleRoadAttitude(car),
                SurfaceGrip = SampleWheelSurfaceGrip(car)
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
                car.Collision,
                car.CarConfig
            );
            _sweepContacts[i] = sweepContact;
        }

        for (int i = 0; i < carCount; i++)
        {
            RaceCar car = _cars[i];
            car.State.CopyFrom(_predictedStates[i]);
            car.TouchedBoundaryThisSubstep = false;
            if (_sweepContacts[i].HasValue)
            {
                car.LastBoundaryContact = _sweepContacts[i];
                car.TouchedBoundaryThisSubstep = true;
            }
        }

        ResolveContactsAndWalls();

        foreach (RaceCar car in _cars)
        {
            if (car.TouchedBoundaryThisSubstep)
                car.BoundaryContactSeconds += dt;
            TrackPose finalPose = Track.Project(car.State.Position);
            TrackRegion region = TrackBoundaryResolver.Classify(finalPose);
            car.Progress.Update(Track, finalPose, region, car.LastBoundaryContact.HasValue);
            WheelOffsets wheels = MeasureWheelOffsets(car);
            if (TrackLimits.AllFourWheelsBeyondTheLine(
                    wheels.Sample.HalfWidth,
                    wheels.FrontLeft,
                    wheels.FrontRight,
                    wheels.RearLeft,
                    wheels.RearRight))
            {
                car.FourWheelsOffSeconds += dt;
            }
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
    /// <summary>
    /// What the road is worth under each of this car's four wheels.
    ///
    /// One projection, and it is one the car has already paid for: the
    /// progress tracker keeps where the centre is, and where a wheel is
    /// follows from that plus the car's heading and its own dimensions.
    /// Projecting four times would buy a slightly better answer at four
    /// times the price, and the difference is smaller than the width of
    /// the smoothing at the road's edge.
    ///
    /// This is also the socket the rubber-and-water grid plugs into when it
    /// arrives: four points already being asked what they are standing on,
    /// with a second layer waiting behind the first.
    /// </summary>
    private WheelSurfaceGrip SampleWheelSurfaceGrip(RaceCar car)
    {
        WheelOffsets wheels = MeasureWheelOffsets(car);
        // The day's grip goes in through the dynamic layer, which is
        // exactly the slot it was reserved for.
        float today = _stepEnvironment.SurfaceGripScalar;
        return new WheelSurfaceGrip(
            SurfaceGrip.At(wheels.Sample, wheels.FrontLeft, today),
            SurfaceGrip.At(wheels.Sample, wheels.FrontRight, today),
            SurfaceGrip.At(wheels.Sample, wheels.RearLeft, today),
            SurfaceGrip.At(wheels.Sample, wheels.RearRight, today)
        );
    }

    /// <summary>
    /// Where each of a car's four wheels is across the road, from the one
    /// projection its progress tracker already holds plus its heading and
    /// dimensions. Shared by the grip under each wheel and by the
    /// four-wheel track-limits reading, so the two can never disagree about
    /// where a wheel is.
    /// </summary>
    private WheelOffsets MeasureWheelOffsets(RaceCar car)
    {
        TrackSample sample = Track.Sample(car.Progress.CurrentS);
        float centre = car.Progress.CurrentD;
        float heading = car.State.Heading;
        Vector2 forward = new(MathF.Cos(heading), MathF.Sin(heading));
        Vector2 left = new(-forward.Y, forward.X);
        Vector2 normal = sample.Normal;
        CarConfig config = car.CarConfig;
        float halfTrack = MathF.Max(config.TrackWidthMeters, 0f) * 0.5f;

        float OffsetOf(Vector2 fromCentre) =>
            centre + Vector2.Dot(fromCentre, normal);

        Vector2 front = forward * config.FrontAxleOffsetMeters;
        Vector2 rear = -forward * config.RearAxleOffsetMeters;
        Vector2 side = left * halfTrack;

        return new WheelOffsets(
            sample,
            OffsetOf(front + side),
            OffsetOf(front - side),
            OffsetOf(rear + side),
            OffsetOf(rear - side)
        );
    }

    private readonly record struct WheelOffsets(
        TrackSample Sample,
        float FrontLeft,
        float FrontRight,
        float RearLeft,
        float RearRight
    );

    private RoadAttitude SampleRoadAttitude(RaceCar car)
    {
        TrackSample sample = Track.Sample(car.Progress.CurrentS);
        if (sample.Grade == 0f &&
            sample.BankSlope == 0f &&
            sample.BankCurvature == 0f &&
            sample.VerticalRate == 0f)
        {
            return RoadAttitude.Flat;
        }
        return new RoadAttitude(
            sample.Grade,
            sample.BankSlopeAt(car.Progress.CurrentD),
            sample.VerticalRate
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
            TrackBoundaryContact? contact = TrackBoundaryResolver.ResolveCurrent(Track, car.State, car.Collision, car.CarConfig);
            if (contact.HasValue)
            {
                car.LastBoundaryContact = contact;
                car.TouchedBoundaryThisSubstep = true;
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

    /// <summary>
    /// What the whole road is worth today, as a multiplier on the static
    /// surface layer.
    ///
    /// A circuit is not the same circuit on every day of a weekend. Green
    /// tarmac on a Friday morning and a dusty one after a support race give
    /// away five to ten per cent; by Sunday the racing line is rubbered in
    /// and gives a couple back. Nothing about the geometry changes, and
    /// nothing about where the kerbs are — only what the surface is worth.
    ///
    /// It is deliberately the <i>dynamic</i> layer of the grip model rather
    /// than a new concept. That layer was reserved when the layered grip
    /// landed, defined as "what gets laid down on the road and washed off
    /// it", with a note that it defaults to one until there is a grid
    /// behind it. This is its first tenant: a grid of one cell. When the
    /// rubber-and-water grid arrives it takes the same slot and everything
    /// written against the signature keeps working.
    ///
    /// Rain is not in this range. Water does not exist in this world yet,
    /// and pretending a dry circuit at 0.9 is a wet one would teach a
    /// policy that wet means "the same road, slightly worse", which is the
    /// opposite of what wet means.
    /// </summary>
    public float SurfaceGripScalar { get; set; } = 1f;
}
