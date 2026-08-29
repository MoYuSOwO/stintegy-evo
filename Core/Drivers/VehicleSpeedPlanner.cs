using System;
using System.Buffers;
using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Builds car-specific rolling speed lookaheads with lateral limits plus
/// forward acceleration and backward braking integration. Work is proportional
/// to the configured local horizon rather than total track length.
/// </summary>
public sealed class VehicleSpeedPlanner
{
    private const float PerformanceCacheSpeedStepMetersPerSecond = 0.25f;
    private const float PerformanceCacheCurvatureStep = 0.001f;
    private const float WakeStateStep = 0.0025f;
    private const float OvertakeAssistStateStep = 0.5f;
    private const float InitialMaximumSpeedSearchMetersPerSecond = 50f;
    private const float NumericalMaximumSpeedMetersPerSecond = 1000f;
    private const float GravityMetersPerSecondSquared = 9.80665f;
    private const int MaximumSpeedSolveIterations = 12;
    private readonly Dictionary<long, float> _driveAccelerationCache = new(16_384);
    private readonly Dictionary<long, float> _brakeDecelerationCache = new(16_384);
    private readonly CarState _optimisticState = new();
    private readonly PreparedPathSpeedPlan _preparedFreePathPlan = new();
    private RaceCar? _planningCar;
    private CarState? _planningState;
    private TireConfig? _planningTireConfig;
    private CarStrategy _planningStrategy;
    private PerformanceStateKey _planningStateKey;
    private long _planningSnapshotGeneration;
    private bool _hasPlanningStateKey;
    private float _planningMaximumSpeedMetersPerSecond = NumericalMaximumSpeedMetersPerSecond;
    private float _planningDownforceAccelPerSpeedSquared;
    private const int LoadTransferPasses = 3;

    private DriverPlanningModifiers _driverModifiers = DriverPlanningModifiers.Neutral;

    public VehicleSpeedPlanner(VehicleSpeedPlanningConfig? config = null)
    {
        Config = config ?? new VehicleSpeedPlanningConfig();
        Validate(Config);
    }

    public VehicleSpeedPlanningConfig Config { get; }
    public float EstimateLateralSpeedLimit(RaceCar car, float curvature)
    {
        ArgumentNullException.ThrowIfNull(car);
        float downforceAccelPerSpeedSquared =
            CarPhysics.EffectiveDownforceAccelPerSpeedSquared(
                car.State,
                car.CarConfig
            );
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            car.State,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            speed: 0f,
            curvature: 0f,
            gripUsage: Config.GetAccelerationUsage(car.Strategy)
        );
        return LateralSpeedLimit(
            curvature,
            limits.LateralAccelerationLimit,
            EstimateMaximumSpeedMetersPerSecond(car),
            downforceAccelPerSpeedSquared
        );
    }

    public float EstimateMaximumSpeedMetersPerSecond(RaceCar car)
    {
        ArgumentNullException.ThrowIfNull(car);

        _optimisticState.CopyFrom(car.State);
        _optimisticState.BatterySoc = 1f;
        float currentSpeed = Math.Max(0f, car.State.Speed);
        float upper = Math.Max(
            InitialMaximumSpeedSearchMetersPerSecond,
            currentSpeed
        );
        upper = Math.Min(upper, NumericalMaximumSpeedMetersPerSecond);

        float upperNetAcceleration = EstimateStraightNetAcceleration(
            car,
            _optimisticState,
            upper
        );
        while (upperNetAcceleration > 0f &&
               upper < NumericalMaximumSpeedMetersPerSecond)
        {
            upper = Math.Min(
                upper * 2f,
                NumericalMaximumSpeedMetersPerSecond
            );
            upperNetAcceleration = EstimateStraightNetAcceleration(
                car,
                _optimisticState,
                upper
            );
        }

        float equilibriumSpeed;
        if (upperNetAcceleration > 0f)
        {
            // A vehicle configured without enough resistance has no finite
            // equilibrium speed in this model. Keep only a numerical guard.
            equilibriumSpeed = NumericalMaximumSpeedMetersPerSecond;
        }
        else
        {
            float lower = 0f;
            for (int i = 0; i < MaximumSpeedSolveIterations; i++)
            {
                float midpoint = (lower + upper) * 0.5f;
                if (EstimateStraightNetAcceleration(car, _optimisticState, midpoint) > 0f)
                    lower = midpoint;
                else
                    upper = midpoint;
            }
            equilibriumSpeed = (lower + upper) * 0.5f;
        }

        float optimisticSpeed = Math.Max(currentSpeed, equilibriumSpeed) *
                                Config.MaximumSpeedEstimateMultiplier;
        return Math.Min(optimisticSpeed, NumericalMaximumSpeedMetersPerSecond);
    }

    public VehicleSpeedLookahead PlanReferenceLookahead(
        VehicleSpeedLookahead destination,
        RaceCar car,
        TrackData track,
        float startS,
        float horizonMeters,
        float stepMeters,
        DriverPlanningModifiers driverModifiers
    )
    {
        ArgumentNullException.ThrowIfNull(destination);
        ArgumentNullException.ThrowIfNull(car);
        ArgumentNullException.ThrowIfNull(track);
        BeginPlanningSnapshot(car, driverModifiers);

        float horizon = MathF.Max(horizonMeters, 0.25f);
        float requestedStep = MathF.Max(stepMeters, 0.25f);
        int count = Math.Max(
            2,
            (int)MathF.Ceiling(horizon / requestedStep) + 1
        );
        float planningStep = horizon / (count - 1);

        float[] curvatures = ArrayPool<float>.Shared.Rent(count);
        float[] segmentLengths = ArrayPool<float>.Shared.Rent(count);
        float[] speedLimits = ArrayPool<float>.Shared.Rent(count);
        float[] speeds = ArrayPool<float>.Shared.Rent(count);
        float[] alongTrackGravity = ArrayPool<float>.Shared.Rent(count);
        try
        {
            CarPerformanceLimits baseLimits = BasePlanningLimits(car);
            float lateralAccelerationLimit =
                baseLimits.LateralAccelerationLimit *
                _driverModifiers.EstimatedGripScale;
            for (int i = 0; i < count; i++)
            {
                float distance = i * planningStep;
                TrackSample surface = track.Sample(startS + distance);
                float curvature = BankedEquivalentCurvature(
                    SamplePeakCurvature(
                        track,
                        startS + distance,
                        planningStep * 0.5f
                    ),
                    surface,
                    lateralAccelerationLimit,
                    surface.RefOffset
                );
                curvatures[i] = curvature;
                speedLimits[i] = LateralSpeedLimit(
                    curvature,
                    lateralAccelerationLimit,
                    _planningMaximumSpeedMetersPerSecond,
                    _planningDownforceAccelPerSpeedSquared
                );
                speeds[i] = speedLimits[i];
                // Worked out after the limit, because how much of gravity
                // pulls along the road depends on how fast the car is going
                // over the road's own vertical bend.
                alongTrackGravity[i] = RoadAt(in surface, surface.RefOffset)
                    .AlongTrackGravity(
                        GravityMetersPerSecondSquared,
                        speedLimits[i]
                    );
                if (i > 0)
                    segmentLengths[i - 1] = planningStep;
            }

            PlanOpenChain(
                car,
                curvatures,
                count,
                segmentLengths,
                speedLimits,
                speeds,
                lateralAccelerationLimit,
                alongTrackGravity
            );
            FillLookahead(
                destination,
                count,
                planningStep,
                horizon,
                segmentLengths,
                speeds
            );
            return destination;
        }
        finally
        {
            ArrayPool<float>.Shared.Return(curvatures);
            ArrayPool<float>.Shared.Return(segmentLengths);
            ArrayPool<float>.Shared.Return(speedLimits);
            ArrayPool<float>.Shared.Return(speeds);
            ArrayPool<float>.Shared.Return(alongTrackGravity);
        }
    }

    internal ReferenceLineHandoverConstraints CreateHandoverConstraints(
        RaceCar car,
        VehicleSpeedLookahead baselineSpeedPlan,
        float latestCompletionDistanceMeters
    )
    {
        ArgumentNullException.ThrowIfNull(car);
        ArgumentNullException.ThrowIfNull(baselineSpeedPlan);
        EnsurePlanningSnapshot(car);
        float standingLateralAccelerationLimit =
            BasePlanningLimits(car).LateralAccelerationLimit *
            _driverModifiers.EstimatedGripScale;
        return new ReferenceLineHandoverConstraints(
            baselineSpeedPlan,
            standingLateralAccelerationLimit,
            _planningDownforceAccelPerSpeedSquared,
            latestCompletionDistanceMeters
        );
    }

    internal ReferenceLineHandoverConstraints CreateHandoverConstraints(
        RaceCar car,
        VehicleSpeedLookahead baselineSpeedPlan,
        DriverPlanningModifiers driverModifiers,
        float latestCompletionDistanceMeters
    )
    {
        ArgumentNullException.ThrowIfNull(car);
        ArgumentNullException.ThrowIfNull(baselineSpeedPlan);
        BeginPlanningSnapshot(car, driverModifiers);
        return CreateHandoverConstraints(
            car,
            baselineSpeedPlan,
            latestCompletionDistanceMeters
        );
    }

    /// <summary>
    /// Plans a path with nobody else on it. The track is optional only
    /// because a caller may not have one; hand it over whenever there is
    /// one, or the plan will be built for a road that is level and flat
    /// wherever the real one climbs or leans.
    /// </summary>
    public DynamicPathSpeedPlan PlanPredictedPath(
        VehicleSpeedLookahead destination,
        RaceCar car,
        VehiclePathPrediction path,
        TrackData? track = null
    )
    {
        TrafficConstraintMemory noTrafficMemory = default;
        return PlanPredictedPathCore(
            car,
            path,
            destination,
            capturePreparedFreePlan: false,
            usePreparedFreePlan: false,
            track,
            frame: default,
            egoSnapshotIndex: -1,
            in noTrafficMemory,
            out _,
            out _,
            out _
        );
    }

    /// <summary>
    /// Builds the traffic-free plan that the traffic-aware pass then reuses.
    /// It takes the track because that cached plan is what the car actually
    /// drives: leaving the road out here would have the reference lookahead
    /// know about a bank that the plan under the wheels never saw.
    /// </summary>
    internal DynamicPathSpeedPlan PreparePredictedPathForTraffic(
        VehicleSpeedLookahead destination,
        RaceCar car,
        VehiclePathPrediction path,
        TrackData? track = null
    )
    {
        TrafficConstraintMemory noTrafficMemory = default;
        return PlanPredictedPathCore(
            car,
            path,
            destination,
            capturePreparedFreePlan: true,
            usePreparedFreePlan: false,
            track,
            frame: default,
            egoSnapshotIndex: -1,
            in noTrafficMemory,
            out _,
            out _,
            out _
        );
    }

    internal TrafficAwareSpeedPlan PlanPredictedPath(
        VehicleSpeedLookahead destination,
        RaceCar car,
        VehiclePathPrediction path,
        TrackData track,
        in RaceFrameSnapshot frame,
        int egoSnapshotIndex,
        in TrafficConstraintMemory committedTrafficMemory
    )
    {
        ArgumentNullException.ThrowIfNull(track);
        DynamicPathSpeedPlan speedPlan = PlanPredictedPathCore(
            car,
            path,
            destination,
            capturePreparedFreePlan: false,
            usePreparedFreePlan: true,
            track,
            frame,
            egoSnapshotIndex,
            in committedTrafficMemory,
            out TrafficConstraintMemory nextTrafficMemory,
            out TrafficSpeedConstraint trafficConstraint,
            out TrafficConflictReport conflictReport
        );
        return new TrafficAwareSpeedPlan(
            speedPlan,
            trafficConstraint,
            nextTrafficMemory,
            conflictReport
        );
    }

    private DynamicPathSpeedPlan PlanPredictedPathCore(
        RaceCar car,
        VehiclePathPrediction path,
        VehicleSpeedLookahead destination,
        bool capturePreparedFreePlan,
        bool usePreparedFreePlan,
        TrackData? track,
        RaceFrameSnapshot frame,
        int egoSnapshotIndex,
        in TrafficConstraintMemory committedTrafficMemory,
        out TrafficConstraintMemory nextTrafficMemory,
        out TrafficSpeedConstraint trafficConstraint,
        out TrafficConflictReport conflictReport
    )
    {
        ArgumentNullException.ThrowIfNull(destination);
        ArgumentNullException.ThrowIfNull(car);
        ArgumentNullException.ThrowIfNull(path);
        if (path.Count < 2)
            throw new ArgumentException("Predicted path requires at least two points.", nameof(path));
        EnsurePlanningSnapshot(car);

        int count = path.Count;
        bool trafficEnabled = track != null &&
                              Config.EnableTrafficAvoidance &&
                              egoSnapshotIndex >= 0 &&
                              egoSnapshotIndex < frame.Count &&
                              frame.Count > 1;
        TrafficConstraintMemory evaluationMemory = committedTrafficMemory;
        trafficConstraint = default;
        if (!trafficEnabled)
            evaluationMemory.Clear();
        nextTrafficMemory = evaluationMemory;
        conflictReport = default;
        float[] curvatures = ArrayPool<float>.Shared.Rent(count);
        float[] segmentLengths = ArrayPool<float>.Shared.Rent(count);
        float[] speedLimits = ArrayPool<float>.Shared.Rent(count);
        float[] speeds = ArrayPool<float>.Shared.Rent(count);
        float[] arrivalTimes = trafficEnabled
            ? ArrayPool<float>.Shared.Rent(count)
            : Array.Empty<float>();
        float[] alongTrackGravity = ArrayPool<float>.Shared.Rent(count);
        try
        {
            CarPerformanceLimits baseLimits = BasePlanningLimits(car);
            float lateralAccelerationLimit =
                baseLimits.LateralAccelerationLimit *
                _driverModifiers.EstimatedGripScale;

            float maximumAbsoluteCurvature = 0f;
            bool restoredPreparedFreePlan = usePreparedFreePlan &&
                _preparedFreePathPlan.TryRestore(
                    path,
                    _planningSnapshotGeneration,
                    curvatures,
                    segmentLengths,
                    speedLimits,
                    speeds,
                    alongTrackGravity,
                    out maximumAbsoluteCurvature
                );
            if (!restoredPreparedFreePlan)
            {
                maximumAbsoluteCurvature = 0f;
                for (int i = 0; i < count; i++)
                {
                    VehiclePathPredictionPoint point = path[i];
                    float planningCurvature = point.CommandedCurvature;
                    alongTrackGravity[i] = 0f;
                    if (track is not null)
                    {
                        TrackSample surface = track.Sample(point.ReferenceS);
                        // Where this path runs across the road, not where the
                        // reference line runs. On a progressively banked corner
                        // those are two different surfaces, and the plan has to
                        // price the one that will be under the wheels.
                        float pathOffset = Vector2.Dot(
                            point.Position - surface.Center,
                            surface.Normal
                        );
                        planningCurvature = BankedEquivalentCurvature(
                            planningCurvature,
                            surface,
                            lateralAccelerationLimit,
                            pathOffset
                        );
                        alongTrackGravity[i] = RoadAt(in surface, pathOffset)
                            .AlongTrackGravity(
                                GravityMetersPerSecondSquared,
                                point.EstimatedSpeed
                            );
                    }
                    if (i == 0)
                    {
                        planningCurvature = GreaterMagnitude(
                            planningCurvature,
                            car.State.Telemetry.ActualCurvature
                        );
                    }
                    maximumAbsoluteCurvature = MathF.Max(
                        maximumAbsoluteCurvature,
                        MathF.Abs(planningCurvature)
                    );
                    curvatures[i] = planningCurvature;
                    float lateralLimit = LateralSpeedLimit(
                        planningCurvature,
                        lateralAccelerationLimit,
                        _planningMaximumSpeedMetersPerSecond,
                        _planningDownforceAccelPerSpeedSquared
                    );
                    speedLimits[i] = lateralLimit;
                    speeds[i] = speedLimits[i];
                    if (i == 0)
                        continue;

                    segmentLengths[i - 1] = MathF.Max(
                        0f,
                        point.DistanceMeters - path[i - 1].DistanceMeters
                    );
                }

                // This is an acyclic/open chain. One backward pass propagates all future
                // braking constraints to the start, then one forward pass propagates the
                // actual entry speed and acceleration reachability to the end.
                PlanOpenChain(
                    car,
                    curvatures,
                    count,
                    segmentLengths,
                    speedLimits,
                    speeds,
                    lateralAccelerationLimit,
                    alongTrackGravity
                );
                if (capturePreparedFreePlan)
                {
                    _preparedFreePathPlan.Capture(
                        path,
                        _planningSnapshotGeneration,
                        curvatures,
                        segmentLengths,
                        speedLimits,
                        speeds,
                        alongTrackGravity,
                        maximumAbsoluteCurvature
                    );
                }
            }
            if (trafficEnabled)
            {
                TrafficConflictEvaluator.FillArrivalTimes(
                    count,
                    segmentLengths,
                    speeds,
                    arrivalTimes
                );
                int reportIndex = TrafficReportEvaluationIndex(
                    count,
                    arrivalTimes,
                    Config.TrafficPredictionHorizonSeconds
                );
                float freeArrivalTime = arrivalTimes[reportIndex];
                for (int iteration = 0;
                     iteration < Config.TrafficConstraintIterations;
                     iteration++)
                {
                    bool changed = TrafficConflictEvaluator.ApplyConstraints(
                        Config,
                        track!,
                        path,
                        in frame,
                        egoSnapshotIndex,
                        segmentLengths,
                        speeds,
                        speedLimits,
                        arrivalTimes,
                        ref evaluationMemory,
                        ref trafficConstraint,
                        out bool requiresReevaluation
                    );
                    if (!changed)
                        break;

                    Array.Copy(speedLimits, speeds, count);
                    PlanOpenChain(
                        car,
                        curvatures,
                        count,
                        segmentLengths,
                        speedLimits,
                        speeds,
                        lateralAccelerationLimit,
                        alongTrackGravity
                    );
                    if (!requiresReevaluation)
                        break;
                }
                TrafficConflictEvaluator.FillArrivalTimes(
                    count,
                    segmentLengths,
                    speeds,
                    arrivalTimes
                );
                conflictReport = new TrafficConflictReport(
                    trafficConstraint,
                    path[reportIndex].DistanceMeters,
                    freeArrivalTime,
                    arrivalTimes[reportIndex]
                );
            }
            float firstForwardReachable = segmentLengths[0] <=
                                          Config.MinimumSegmentLengthMeters
                ? speeds[0]
                : IntegrateForward(
                    car,
                    curvatures[0],
                    curvatures[1],
                    speeds[0],
                    segmentLengths[0],
                    lateralAccelerationLimit,
                    SegmentGravity(alongTrackGravity, 0)
                );

            float referenceAcceleration = segmentLengths[0] <=
                                          Config.MinimumSegmentLengthMeters
                ? 0f
                : (speeds[1] * speeds[1] - speeds[0] * speeds[0]) /
                  (2f * segmentLengths[0]);
            bool firstSegmentUsesFullDrive = speeds[1] >
                                             speeds[0] +
                                             Config.ConvergenceToleranceMetersPerSecond &&
                                             speeds[1] >=
                                             firstForwardReachable -
                                             Config.ConvergenceToleranceMetersPerSecond;
            if (firstSegmentUsesFullDrive)
            {
                // The kinematic quotient is the mean acceleration across the
                // segment. Feed the controller the instantaneous acceleration at
                // its start; the substep propagation above still determines v[1].
                // The road is part of that instant: the profile was settled
                // with the gradient in it, so dropping it here would hand the
                // controller a flat-road number on a hill.
                referenceAcceleration =
                    MaximumNetDriveAcceleration(
                        car,
                        speeds[0],
                        curvatures[0]
                    ) + SegmentGravity(alongTrackGravity, 0);
            }
            float stepLength = path.LengthMeters / Math.Max(count - 1, 1);
            FillLookahead(
                destination,
                count,
                stepLength,
                path.LengthMeters,
                segmentLengths,
                speeds
            );
            destination.Set(
                0,
                new VehicleSpeedPlanPoint(
                    speeds[0],
                    referenceAcceleration
                )
            );
            nextTrafficMemory = evaluationMemory;
            return new DynamicPathSpeedPlan(
                new VehicleSpeedPlanPoint(speeds[0], referenceAcceleration),
                speeds[1],
                segmentLengths[0],
                path.LengthMeters,
                maximumAbsoluteCurvature,
                count
            );
        }
        finally
        {
            ArrayPool<float>.Shared.Return(curvatures);
            ArrayPool<float>.Shared.Return(segmentLengths);
            ArrayPool<float>.Shared.Return(speedLimits);
            ArrayPool<float>.Shared.Return(speeds);
            ArrayPool<float>.Shared.Return(alongTrackGravity);
            if (arrivalTimes.Length > 0)
                ArrayPool<float>.Shared.Return(arrivalTimes);
        }
    }

    /// <summary>
    /// The flat corner that would be as hard as this banked one.
    ///
    /// The whole planner already reasons in curvature — the speed limit,
    /// the braking integration, and how much grip a corner leaves over for
    /// accelerating all read the same array — so folding the road into that
    /// number lets every one of them account for the bank without being
    /// told about it. The conversion equates the banked balance
    ///
    ///   k*v^2 - g*B = L*cos + (L/g)*(k*B + downforce)*v^2
    ///
    /// with the flat one it replaces, and solves for the curvature that
    /// would produce the same speed. It carries the demand the bank takes
    /// off the tyres, which also correctly leaves more grip for braking and
    /// acceleration; it does not carry the extra load the bank presses on
    /// beyond that, so a steeply banked corner is modelled slightly
    /// conservatively rather than optimistically.
    /// </summary>
    /// <summary>
    /// The road under the racing line, which is where the free plan assumes
    /// the car will be. A car that then runs wider on a progressively
    /// banked corner finds more grip than this planned for, which is the
    /// right way round: the plan stays honest and the extra is upside.
    /// </summary>
    /// <summary>
    /// The road as it lies at one place across its width. A cross-section
    /// that curves — a crown, a bowl, a progressively banked corner — leans
    /// by a different amount at every offset, so the caller has to say where
    /// on the road the car will be.
    /// </summary>
    private static RoadAttitude RoadAt(in TrackSample sample, float offsetMeters)
    {
        if (sample.Grade == 0f &&
            sample.BankSlope == 0f &&
            sample.BankCurvature == 0f &&
            sample.VerticalRate == 0f)
        {
            return RoadAttitude.Flat;
        }
        return new RoadAttitude(
            sample.Grade,
            sample.BankSlopeAt(offsetMeters),
            sample.VerticalRate
        );
    }

    /// <summary>
    /// The flat-road corner that would be worth the same speed as this one.
    ///
    /// Setting the tyres' duty equal to the grip they have, on the real road,
    ///
    ///     (G/L) v^2 k  -  b (g + v^2 r) / (L G)
    ///         = a0/g * ( [g + v^2 (k b + r)] / L  +  C v^2 )
    ///
    /// and on a flat one, v^2 = a0 / (k_eq - a0 C / g), and asking for the
    /// same v^2 out of both, gives a k_eq that has no speed left in it. So
    /// the whole of the road's effect on a corner folds into the curvature
    /// array the planner already carries, and the braking integration and the
    /// leftover longitudinal grip inherit it for nothing.
    ///
    /// b is the bank taken in the corner's own sense, positive where it leans
    /// into the turn; r is the rate the gradient changes at, positive into a
    /// compression.
    /// </summary>
    private float BankedEquivalentCurvature(
        float curvature,
        in TrackSample sample,
        float lateralAccelerationLimit,
        float offsetMeters
    )
    {
        if (sample.BankSlope == 0f &&
            sample.BankCurvature == 0f &&
            sample.VerticalRate == 0f &&
            sample.Grade == 0f)
        {
            return curvature;
        }

        float absoluteCurvature = MathF.Abs(curvature);
        if (absoluteCurvature <= Config.CurvatureEpsilon)
            return curvature;

        RoadAttitude road = RoadAt(in sample, offsetMeters);
        float alongShape = MathF.Sqrt(
            1f + Sanitize(sample.Grade) * Sanitize(sample.Grade)
        );
        float surfaceShape = 1f / MathF.Max(road.NormalCosine, 1e-6f);
        float bank = curvature >= 0f
            ? Sanitize(road.BankTangent)
            : -Sanitize(road.BankTangent);
        float rate = Sanitize(sample.VerticalRate);

        float standingLimit = MathF.Max(lateralAccelerationLimit, 1e-3f);
        float gripPerGravity = standingLimit / GravityMetersPerSecondSquared;
        float airPaid = gripPerGravity * _planningDownforceAccelPerSpeedSquared;

        // What the corner is worth before the tyres are asked for anything:
        // an adverse bank steeper than the grip makes it simply impossible.
        float carried = standingLimit + bank * GravityMetersPerSecondSquared /
                                        alongShape;
        if (carried <= 0f)
            return MathF.CopySign(MaximumEquivalentCurvature, curvature);

        float asked = alongShape * absoluteCurvature -
                      bank * rate / alongShape -
                      gripPerGravity * (absoluteCurvature * bank + rate +
                                        _planningDownforceAccelPerSpeedSquared *
                                        surfaceShape);
        if (asked <= 0f)
            return MathF.CopySign(MathF.Max(airPaid, 0f), curvature);

        float equivalent = airPaid + standingLimit * asked / carried;
        return MathF.CopySign(
            MathF.Min(MathF.Max(equivalent, 0f), MaximumEquivalentCurvature),
            curvature
        );
    }

    private const float MaximumEquivalentCurvature = 1f;

    private static float Sanitize(float value) =>
        float.IsFinite(value) ? value : 0f;

    private static int TrafficReportEvaluationIndex(
        int count,
        float[] arrivalTimes,
        float horizonSeconds
    )
    {
        int index = 0;
        float horizon = MathF.Max(0f, horizonSeconds);
        while (index + 1 < count &&
               float.IsFinite(arrivalTimes[index + 1]) &&
               arrivalTimes[index + 1] <= horizon)
        {
            index++;
        }

        if (index == 0 &&
            count > 1 &&
            float.IsFinite(arrivalTimes[1]))
        {
            return 1;
        }
        return index;
    }

    private float IntegrateForward(
        RaceCar car,
        float startCurvature,
        float endCurvature,
        float startSpeed,
        float distance,
        float lateralAccelerationLimit,
        float alongTrackGravity = 0f
    )
    {
        int substeps = Math.Max(1, Config.IntegrationSubsteps);
        float stepDistance = distance / substeps;
        float speed = Math.Max(0f, startSpeed);

        for (int substep = 0; substep < substeps; substep++)
        {
            float t = (substep + 0.5f) / substeps;
            float curvature = Lerp(startCurvature, endCurvature, t);
            // A climb takes from what the engine can add, a descent gives
            // to it. Applied outside the cached lookup because it is a
            // simple sum and does not belong in the cache key.
            float acceleration =
                MaximumNetDriveAcceleration(car, speed, curvature) +
                alongTrackGravity;
            float predicted = MathF.Sqrt(MathF.Max(0f, speed * speed + 2f * acceleration * stepDistance));
            float midpointSpeed = (speed + predicted) * 0.5f;
            float midpointAcceleration =
                MaximumNetDriveAcceleration(car, midpointSpeed, curvature) +
                alongTrackGravity;
            speed = MathF.Min(
                LateralSpeedLimit(
                    curvature,
                    lateralAccelerationLimit,
                    _planningMaximumSpeedMetersPerSecond,
                    _planningDownforceAccelPerSpeedSquared
                ),
                MathF.Sqrt(MathF.Max(0f, speed * speed + 2f * midpointAcceleration * stepDistance))
            );
        }

        return speed;
    }

    private float IntegrateBackward(
        RaceCar car,
        float startCurvature,
        float endCurvature,
        float endSpeed,
        float distance,
        float lateralAccelerationLimit,
        float alongTrackGravity = 0f
    )
    {
        int substeps = Math.Max(1, Config.IntegrationSubsteps);
        float stepDistance = distance / substeps;
        float speed = Math.Max(0f, endSpeed);

        for (int substep = substeps - 1; substep >= 0; substep--)
        {
            float t = (substep + 0.5f) / substeps;
            float curvature = Lerp(startCurvature, endCurvature, t);
            // Braking downhill has gravity working against it, which is
            // why a plan that ignores the road brakes too late into a
            // descending corner.
            float deceleration =
                MaximumNetBrakeDeceleration(car, speed, curvature) -
                alongTrackGravity;
            float predicted = MathF.Sqrt(MathF.Max(0f, speed * speed + 2f * deceleration * stepDistance));
            float midpointSpeed = (speed + predicted) * 0.5f;
            float midpointDeceleration =
                MaximumNetBrakeDeceleration(car, midpointSpeed, curvature) -
                alongTrackGravity;
            speed = MathF.Min(
                LateralSpeedLimit(
                    curvature,
                    lateralAccelerationLimit,
                    _planningMaximumSpeedMetersPerSecond,
                    _planningDownforceAccelPerSpeedSquared
                ),
                MathF.Sqrt(MathF.Max(0f, speed * speed + 2f * midpointDeceleration * stepDistance))
            );
        }

        return speed;
    }

    private float MaximumNetDriveAcceleration(RaceCar car, float speed, float curvature)
    {
        long cacheKey = PerformanceCacheKey(speed, curvature, out float quantizedSpeed, out float quantizedCurvature);
        if (_driveAccelerationCache.TryGetValue(cacheKey, out float cached))
            return cached;

        float acceleration = 0f;
        for (int i = 0; i < Config.AccelerationSolveIterations; i++)
        {
            CarPerformanceLimits limits = EstimateLimits(
                car,
                quantizedSpeed,
                quantizedCurvature,
                assumedLongitudinalAcceleration: acceleration
            );
            float drive = limits.MaximumDriveAcceleration *
                          Config.DriveAccelerationUsage *
                          _driverModifiers.EstimatedGripScale;
            float next = Math.Max(0f, drive - limits.LossAcceleration);
            if (MathF.Abs(next - acceleration) < 1e-3f)
            {
                _driveAccelerationCache[cacheKey] = next;
                return next;
            }
            acceleration = next;
        }
        _driveAccelerationCache[cacheKey] = acceleration;
        return acceleration;
    }

    private float MaximumNetBrakeDeceleration(RaceCar car, float speed, float curvature)
    {
        long cacheKey = PerformanceCacheKey(speed, curvature, out float quantizedSpeed, out float quantizedCurvature);
        if (_brakeDecelerationCache.TryGetValue(cacheKey, out float cached))
            return cached;

        float deceleration = 0f;
        for (int i = 0; i < Config.AccelerationSolveIterations; i++)
        {
            CarPerformanceLimits limits = EstimateLimits(
                car,
                quantizedSpeed,
                quantizedCurvature,
                assumedLongitudinalAcceleration: -deceleration
            );
            float brake = limits.MaximumBrakeDeceleration *
                          Config.BrakeDecelerationUsage *
                          _driverModifiers.EstimatedGripScale;
            float next = Math.Max(0f, brake + limits.LossAcceleration);
            if (MathF.Abs(next - deceleration) < 1e-3f)
            {
                _brakeDecelerationCache[cacheKey] = next;
                return next;
            }
            deceleration = next;
        }
        _brakeDecelerationCache[cacheKey] = deceleration;
        return deceleration;
    }

    private CarPerformanceLimits EstimateLimits(
        RaceCar car,
        float speed,
        float curvature,
        float assumedLongitudinalAcceleration = 0f
    )
    {
        return CarPhysics.EstimatePerformanceLimits(
            _planningState ?? car.State,
            car.CarConfig,
            car.TireConfig,
            _planningState == null ? car.Strategy : _planningStrategy,
            speed,
            curvature,
            Config.GetAccelerationUsage(_planningStrategy),
            assumedLongitudinalAcceleration,
            _driverModifiers.FrontBrakeBiasOffset,
            _driverModifiers.PaceEfficiency
        );
    }

    private static float EstimateStraightNetAcceleration(
        RaceCar car,
        CarState optimisticState,
        float speed
    )
    {
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            optimisticState,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            speed,
            curvature: 0f,
            gripUsage: 1f
        );
        return limits.MaximumDriveAcceleration - limits.LossAcceleration;
    }

    /// <summary>
    /// How fast a corner of this curvature can be taken.
    ///
    /// The limit handed in is what the tyres will bear standing still, and it
    /// is not what they will bear at speed: the air presses the car down, the
    /// load rises with the square of the speed, and grip rises with the load.
    /// Tyre grip in this model does not care how hard it is being pressed, so
    /// the limit is exactly proportional to the load and the whole thing has a
    /// closed form. Cornering asks for v*v*k and the tyres offer
    /// a0 * (1 + C*v*v/g), so
    ///
    ///     v*v * (k - a0*C/g) = a0
    ///
    /// and where the bracket runs out the corner is flat: the wings are
    /// gaining grip at least as fast as the corner is asking for it, and the
    /// only thing left holding the car back is how fast it can go at all. That
    /// is a real car's behaviour and it is the whole character of a quick
    /// circuit. Without it every corner is worth the same as a slow one of the
    /// same radius, and a lap of Silverstone came out slower than a lap of
    /// Shanghai, which is the wrong way round by some distance.
    /// </summary>
    private float LateralSpeedLimit(
        float curvature,
        float lateralAccelerationLimit,
        float maximumSpeedMetersPerSecond,
        float downforceAccelPerSpeedSquared
    )
    {
        float absoluteCurvature = MathF.Abs(curvature);
        if (absoluteCurvature <= Config.CurvatureEpsilon)
            return maximumSpeedMetersPerSecond;

        float standingLimit = Math.Max(0f, lateralAccelerationLimit);
        float curvatureTheAirPaysFor = standingLimit *
                                       downforceAccelPerSpeedSquared /
                                       GravityMetersPerSecondSquared;
        float curvatureLeft = absoluteCurvature - curvatureTheAirPaysFor;
        if (curvatureLeft <= 0f)
            return maximumSpeedMetersPerSecond;

        return MathF.Min(
            maximumSpeedMetersPerSecond,
            MathF.Sqrt(standingLimit / curvatureLeft)
        );
    }

    private static float Lerp(float from, float to, float t)
    {
        return from + (to - from) * Math.Clamp(t, 0f, 1f);
    }

    private static float SamplePeakCurvature(TrackData track, float s, float radius)
    {
        float best = track.Sample(s).RefCurvature;
        float before = track.Sample(s - radius).RefCurvature;
        float after = track.Sample(s + radius).RefCurvature;
        if (MathF.Abs(before) > MathF.Abs(best))
            best = before;
        if (MathF.Abs(after) > MathF.Abs(best))
            best = after;
        return best;
    }

    private static float GreaterMagnitude(float first, float second) =>
        MathF.Abs(second) > MathF.Abs(first) ? second : first;

    private CarPerformanceLimits BasePlanningLimits(RaceCar car)
    {
        return CarPhysics.EstimatePerformanceLimits(
            _planningState!,
            car.CarConfig,
            car.TireConfig,
            _planningStrategy,
            speed: 0f,
            curvature: 0f,
            gripUsage: Config.GetAccelerationUsage(_planningStrategy),
            corneringEfficiency: _driverModifiers.PaceEfficiency
        );
    }

    /// <summary>
    /// Settles the speed profile over an open chain of points. The optional
    /// per-point pull along the track is what lets the braking integration
    /// know the road descends: without it a plan brakes for a corner as
    /// though the approach were level, and arrives at it too fast.
    /// </summary>
    private void PlanOpenChain(
        RaceCar car,
        float[] curvatures,
        int count,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float lateralAccelerationLimit,
        float[]? alongTrackGravity = null
    )
    {
        for (int pass = 0; pass < LoadTransferPasses; pass++)
        {
            ApplyBackwardOpenPass(
                car,
                curvatures,
                count,
                segmentLengths,
                speedLimits,
                speeds,
                lateralAccelerationLimit,
                alongTrackGravity
            );
            speeds[0] = MathF.Min(speeds[0], MathF.Max(0f, car.State.Speed));
            // Kept for the last time round only. Tightening a corner and
            // running the braking back from it can only lower speeds, and what
            // the forward pass settles - how fast the car can be going here
            // having accelerated from where it actually is - does not change
            // when a point further along is lowered. Running it every time
            // round was most of what the extra passes cost.
            bool settled = pass == LoadTransferPasses - 1 ||
                           !TightenCornersForTheBrakingIntoThem(
                               car,
                               curvatures,
                               count,
                               segmentLengths,
                               speeds,
                               speedLimits
                           );
            if (!settled)
                continue;

            ApplyForwardOpenPass(
                car,
                curvatures,
                count,
                segmentLengths,
                speedLimits,
                speeds,
                lateralAccelerationLimit,
                alongTrackGravity
            );
            break;
        }
    }

    /// <summary>
    /// Brings each corner speed down to what the car can still hold while it is
    /// braking for that corner.
    ///
    /// The friction budget was being charged one way only. Cornering took grip
    /// away from the brakes, correctly; braking took nothing away from the
    /// cornering, which is wrong, and wrong in the place it costs most. Braking
    /// moves weight off the rear, and the rear is the axle that runs out of
    /// cornering first, because the share of the corner it has to carry is
    /// fixed by where the mass sits and cannot move with the load - the car
    /// would turn on its own if it did.
    ///
    /// So the corner speed and the braking that reaches it decide each other,
    /// and the chain has to be run more than once to settle them. Only ever
    /// lowers a limit, so the passes converge downwards instead of chasing each
    /// other, and it stops early once a pass changes nothing.
    ///
    /// This is what a driver does by feel and calls trail braking: on the
    /// brakes earlier, off them as the corner arrives, because there is not
    /// enough of the rear axle for all of both.
    /// </summary>
    private bool TightenCornersForTheBrakingIntoThem(
        RaceCar car,
        float[] curvatures,
        int count,
        float[] segmentLengths,
        float[] speeds,
        float[] speedLimits
    )
    {
        CarState state = _planningState ?? car.State;
        float gripUsage = Config.GetAccelerationUsage(_planningStrategy);
        bool changed = false;
        for (int i = 0; i < count - 1; i++)
        {
            float absoluteCurvature = MathF.Abs(curvatures[i]);
            if (absoluteCurvature <= 1e-4f || segmentLengths[i] <= 1e-5f)
                continue;

            float longitudinal = (
                speeds[i + 1] * speeds[i + 1] - speeds[i] * speeds[i]
            ) / (2f * segmentLengths[i]);
            if (longitudinal >= 0f)
                continue;

            float lateral = CarPhysics.EstimateLateralAccelerationLimit(
                state,
                car.CarConfig,
                car.TireConfig,
                speeds[i],
                curvatures[i],
                gripUsage,
                longitudinal,
                _driverModifiers.PaceEfficiency
            ) * _driverModifiers.EstimatedGripScale;
            if (lateral <= 0f)
                continue;

            // Turned straight into a speed rather than through the standing
            // form, because this limit already carries the downforce of the
            // speed it was asked at and the standing form would add the wings
            // a second time.
            float holdable = MathF.Sqrt(lateral / absoluteCurvature);
            if (holdable >=
                speedLimits[i] - Config.ConvergenceToleranceMetersPerSecond)
            {
                continue;
            }

            speedLimits[i] = holdable;
            changed = true;
        }
        return changed;
    }

    private static void FillLookahead(
        VehicleSpeedLookahead lookahead,
        int count,
        float stepLength,
        float lengthMeters,
        float[] segmentLengths,
        float[] speeds
    )
    {
        lookahead.Reset(count, stepLength, lengthMeters);
        for (int i = 0; i < count; i++)
        {
            float referenceAcceleration = 0f;
            if (i < count - 1 && segmentLengths[i] > 1e-5f)
            {
                referenceAcceleration = (
                    speeds[i + 1] * speeds[i + 1] -
                    speeds[i] * speeds[i]
                ) / (2f * segmentLengths[i]);
            }
            lookahead.Set(
                i,
                new VehicleSpeedPlanPoint(
                    speeds[i],
                    referenceAcceleration
                )
            );
        }
    }

    private void BeginPlanningSnapshot(
        RaceCar car,
        DriverPlanningModifiers driverModifiers
    )
    {
        _planningSnapshotGeneration = unchecked(
            _planningSnapshotGeneration + 1
        );
        if (_planningSnapshotGeneration == 0)
            _planningSnapshotGeneration = 1;

        DriverPlanningModifiers normalizedModifiers = new(
            Math.Clamp(driverModifiers.PaceEfficiency, 0.8f, 1f),
            Math.Clamp(driverModifiers.EstimatedGripScale, 0.88f, 1.12f),
            float.IsFinite(driverModifiers.FrontBrakeBiasOffset)
                ? Math.Clamp(driverModifiers.FrontBrakeBiasOffset, -0.25f, 0.25f)
                : 0f
        );
        PerformanceStateKey stateKey = BuildPerformanceStateKey(
            car,
            normalizedModifiers
        );
        if (ReferenceEquals(_planningCar, car) &&
            ReferenceEquals(_planningTireConfig, car.TireConfig) &&
            _planningStrategy == car.Strategy &&
            _hasPlanningStateKey &&
            _planningStateKey == stateKey)
        {
            return;
        }

        _planningCar = car;
        _planningDownforceAccelPerSpeedSquared =
            CarPhysics.EffectiveDownforceAccelPerSpeedSquared(
                car.State,
                car.CarConfig
            );
        _planningTireConfig = car.TireConfig;
        _planningState ??= new CarState();
        _planningState.CopyFrom(car.State);
        _planningStrategy = car.Strategy;
        _driverModifiers = normalizedModifiers;
        _planningStateKey = stateKey;
        _hasPlanningStateKey = true;
        _planningMaximumSpeedMetersPerSecond = EstimateMaximumSpeedMetersPerSecond(car);
        _driveAccelerationCache.Clear();
        _brakeDecelerationCache.Clear();
    }

    private static PerformanceStateKey BuildPerformanceStateKey(
        RaceCar car,
        DriverPlanningModifiers modifiers
    )
    {
        return new PerformanceStateKey(
            car.Strategy,
            Quantize(car.State.BatterySoc, 0.005f),
            Quantize(car.State.AirVelocityDeficit, WakeStateStep),
            Quantize(car.State.WakeDownforceLoss, WakeStateStep),
            Quantize(car.State.DownforceVelocityDeficit, WakeStateStep),
            Quantize(car.State.OvertakeAssist, OvertakeAssistStateStep),
            Quantize(modifiers.PaceEfficiency, 0.005f),
            Quantize(modifiers.EstimatedGripScale, 0.005f),
            Quantize(modifiers.FrontBrakeBiasOffset, 0.0025f),
            BuildTireStateKey(car.State.FrontLeft),
            BuildTireStateKey(car.State.FrontRight),
            BuildTireStateKey(car.State.RearLeft),
            BuildTireStateKey(car.State.RearRight)
        );
    }

    private static TireStateKey BuildTireStateKey(TireState tire)
    {
        return new TireStateKey(
            Quantize(tire.SurfaceTempC, 1f),
            Quantize(tire.CoreTempC, 1f),
            Quantize(tire.Wear, 0.005f)
        );
    }

    private static int Quantize(float value, float step) =>
        (int)MathF.Round(value / step);

    private readonly record struct TireStateKey(
        int SurfaceTemperature,
        int CoreTemperature,
        int Wear
    );

    private readonly record struct PerformanceStateKey(
        CarStrategy Strategy,
        int BatterySoc,
        int AirVelocityDeficit,
        int WakeDownforceLoss,
        int DownforceVelocityDeficit,
        int OvertakeAssist,
        int PaceEfficiency,
        int EstimatedGripScale,
        int FrontBrakeBiasOffset,
        TireStateKey FrontLeft,
        TireStateKey FrontRight,
        TireStateKey RearLeft,
        TireStateKey RearRight
    );

    private void EnsurePlanningSnapshot(RaceCar car)
    {
        if (!ReferenceEquals(_planningCar, car) || _planningState == null || _planningStrategy != car.Strategy)
            BeginPlanningSnapshot(car, _driverModifiers);
    }

    private static long PerformanceCacheKey(
        float speed,
        float curvature,
        out float quantizedSpeed,
        out float quantizedCurvature
    )
    {
        int speedBin = (int)MathF.Round(
            MathF.Max(0f, speed) / PerformanceCacheSpeedStepMetersPerSecond
        );
        int curvatureBin = (int)MathF.Round(
            curvature / PerformanceCacheCurvatureStep
        );
        quantizedSpeed = speedBin * PerformanceCacheSpeedStepMetersPerSecond;
        quantizedCurvature = curvatureBin * PerformanceCacheCurvatureStep;
        return ((long)speedBin << 32) | (uint)curvatureBin;
    }

    private static void Validate(VehicleSpeedPlanningConfig config)
    {
        if (config.MaximumSpeedEstimateMultiplier < 1f ||
            !float.IsFinite(config.MaximumSpeedEstimateMultiplier))
        {
            throw new ArgumentOutOfRangeException(
                nameof(config),
                "Maximum speed estimate multiplier must be finite and at least one."
            );
        }
        float protectUsage = config.ProtectAccelerationUsage;
        float lightUsage = config.LightAccelerationUsage;
        float normalUsage = config.NormalAccelerationUsage;
        float pushUsage = config.PushAccelerationUsage;
        float attackUsage = config.AttackAccelerationUsage;
        ValidateAccelerationUsage(protectUsage);
        ValidateAccelerationUsage(lightUsage);
        ValidateAccelerationUsage(normalUsage);
        ValidateAccelerationUsage(pushUsage);
        ValidateAccelerationUsage(attackUsage);
        if (!(protectUsage < lightUsage &&
              lightUsage < normalUsage &&
              normalUsage < pushUsage &&
              pushUsage < attackUsage))
        {
            throw new ArgumentOutOfRangeException(
                nameof(config),
                "Tire-mode acceleration usages must increase from Protect to Attack."
            );
        }
        if (config.DriveAccelerationUsage <= 0f || config.DriveAccelerationUsage > 1f)
            throw new ArgumentOutOfRangeException(nameof(config), "Drive acceleration usage must be in (0, 1].");
        if (config.BrakeDecelerationUsage <= 0f || config.BrakeDecelerationUsage > 1f)
            throw new ArgumentOutOfRangeException(nameof(config), "Brake deceleration usage must be in (0, 1].");
        if (config.IntegrationSubsteps < 1)
            throw new ArgumentOutOfRangeException(nameof(config), "Planning pass counts must be positive.");
        if (config.AccelerationSolveIterations < 1)
            throw new ArgumentOutOfRangeException(nameof(config), "Acceleration iterations must be positive.");
        if (!float.IsFinite(config.TrafficPredictionHorizonSeconds) ||
            config.TrafficPredictionHorizonSeconds <= 0f ||
            !float.IsFinite(config.TrafficMinimumGapMeters) ||
            config.TrafficMinimumGapMeters < 0f ||
            !float.IsFinite(config.TrafficFollowingTimeHeadwaySeconds) ||
            config.TrafficFollowingTimeHeadwaySeconds < 0f ||
            !float.IsFinite(config.TrafficFollowingControlResponseSeconds) ||
            config.TrafficFollowingControlResponseSeconds < 0f ||
            !float.IsFinite(
                config.TrafficApproachDecelerationMetersPerSecondSquared
            ) ||
            config.TrafficApproachDecelerationMetersPerSecondSquared <= 0f ||
            !float.IsFinite(config.TrafficTimeHeadwaySeconds) ||
            config.TrafficTimeHeadwaySeconds < 0f ||
            !float.IsFinite(config.TrafficLateralMergePredictionSeconds) ||
            config.TrafficLateralMergePredictionSeconds < 0f ||
            !float.IsFinite(config.TrafficLateralSafetyMarginMeters) ||
            config.TrafficLateralSafetyMarginMeters < 0f ||
            !float.IsFinite(config.TrafficLongitudinalSafetyMarginMeters) ||
            config.TrafficLongitudinalSafetyMarginMeters < 0f ||
            !float.IsFinite(config.TrafficConstraintHoldSeconds) ||
            config.TrafficConstraintHoldSeconds < 0f ||
            config.TrafficConstraintIterations < 1 ||
            config.TrafficArrivalSolveIterations < 1)
        {
            throw new ArgumentOutOfRangeException(
                nameof(config),
                "Traffic prediction settings must be finite and non-negative."
            );
        }
        if (config.SpeedPlanningHorizonMeters <= 0f ||
            config.PathPredictionStepMeters <= 0f ||
            config.MinimumDynamicPredictionMeters < 0f ||
            config.PredictionConvergenceHoldMeters < 0f ||
            config.PredictionConvergenceLateralErrorMeters < 0f ||
            config.PredictionConvergenceHeadingErrorRadians < 0f ||
            config.PredictionConvergenceCurvatureError < 0f)
        {
            throw new ArgumentOutOfRangeException(nameof(config), "Path prediction distances must be positive.");
        }
    }

    private static void ValidateAccelerationUsage(float usage)
    {
        if (!float.IsFinite(usage) || usage <= 0f || usage > 1f)
        {
            throw new ArgumentOutOfRangeException(
                nameof(usage),
                "Acceleration usage must be finite and in (0, 1]."
            );
        }
    }

    /// <summary>
    /// Gravity's pull along one segment, averaged over its ends, or none
    /// when the caller has no road to speak of.
    /// </summary>
    private static float SegmentGravity(float[]? alongTrackGravity, int index)
    {
        return alongTrackGravity is null
            ? 0f
            : 0.5f * (alongTrackGravity[index] + alongTrackGravity[index + 1]);
    }

    private bool ApplyForwardOpenPass(
        RaceCar car,
        float[] curvatures,
        int count,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float lateralAccelerationLimit,
        float[]? alongTrackGravity
    )
    {
        bool changed = false;
        for (int i = 0; i < count - 1; i++)
        {
            float distance = segmentLengths[i];
            float reachable = distance <= Config.MinimumSegmentLengthMeters
                ? speeds[i]
                : IntegrateForward(
                    car,
                    curvatures[i],
                    curvatures[i + 1],
                    speeds[i],
                    distance,
                    lateralAccelerationLimit,
                    SegmentGravity(alongTrackGravity, i)
                );
            float limited = MathF.Min(speedLimits[i + 1], reachable);
            if (limited < speeds[i + 1] - Config.ConvergenceToleranceMetersPerSecond)
            {
                speeds[i + 1] = limited;
                changed = true;
            }
        }
        return changed;
    }

    private bool ApplyBackwardOpenPass(
        RaceCar car,
        float[] curvatures,
        int count,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float lateralAccelerationLimit,
        float[]? alongTrackGravity
    )
    {
        bool changed = false;
        for (int i = count - 2; i >= 0; i--)
        {
            float distance = segmentLengths[i];
            float reachable = distance <= Config.MinimumSegmentLengthMeters
                ? speeds[i + 1]
                : IntegrateBackward(
                    car,
                    curvatures[i],
                    curvatures[i + 1],
                    speeds[i + 1],
                    distance,
                    lateralAccelerationLimit,
                    SegmentGravity(alongTrackGravity, i)
                );
            float limited = MathF.Min(speedLimits[i], reachable);
            if (limited < speeds[i] - Config.ConvergenceToleranceMetersPerSecond)
            {
                speeds[i] = limited;
                changed = true;
            }
        }
        return changed;
    }

}
