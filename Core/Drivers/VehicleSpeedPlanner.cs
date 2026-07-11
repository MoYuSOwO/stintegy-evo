using System;
using System.Buffers;
using System.Collections.Generic;
using System.Numerics;
using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;

namespace TheStint.Core.Drivers;

/// <summary>
/// Builds a closed-loop, car-specific speed profile with a lateral limit pass,
/// forward acceleration integration, and backward braking integration.
/// Tire, battery, drag, and strategy state are frozen at planning time and are
/// refreshed by the driver on a short interval.
/// </summary>
public sealed class VehicleSpeedPlanner
{
    private const float PerformanceCacheSpeedStepMetersPerSecond = 0.25f;
    private const float PerformanceCacheCurvatureStep = 0.001f;
    private const float InitialMaximumSpeedSearchMetersPerSecond = 50f;
    private const float NumericalMaximumSpeedMetersPerSecond = 1000f;
    private const int MaximumSpeedSolveIterations = 12;
    private readonly Dictionary<long, float> _driveAccelerationCache = new(16_384);
    private readonly Dictionary<long, float> _brakeDecelerationCache = new(16_384);
    private RaceCar? _planningCar;
    private CarState? _planningState;
    private CarStrategy _planningStrategy;
    private float _planningMaximumSpeedMetersPerSecond = NumericalMaximumSpeedMetersPerSecond;
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
            EstimateMaximumSpeedMetersPerSecond(car)
        );
    }

    public float EstimateMaximumSpeedMetersPerSecond(RaceCar car)
    {
        ArgumentNullException.ThrowIfNull(car);

        CarState optimisticState = car.State.Clone();
        optimisticState.BatterySoc = 1f;
        float currentSpeed = Math.Max(0f, car.State.Speed);
        float upper = Math.Max(
            InitialMaximumSpeedSearchMetersPerSecond,
            currentSpeed
        );
        upper = Math.Min(upper, NumericalMaximumSpeedMetersPerSecond);

        float upperNetAcceleration = EstimateStraightNetAcceleration(
            car,
            optimisticState,
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
                optimisticState,
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
                if (EstimateStraightNetAcceleration(car, optimisticState, midpoint) > 0f)
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

    public VehicleSpeedProfile Plan(RaceCar car, TrackData track)
    {
        return Plan(car, track, DriverPlanningModifiers.Neutral);
    }

    public VehicleSpeedProfile Plan(
        RaceCar car,
        TrackData track,
        DriverPlanningModifiers driverModifiers
    )
    {
        ArgumentNullException.ThrowIfNull(car);
        ArgumentNullException.ThrowIfNull(track);
        BeginPlanningSnapshot(car, driverModifiers);

        int count = Math.Max(1, (int)MathF.Ceiling(track.LengthMeters / Config.PlanningStepMeters));
        float planningStep = track.LengthMeters / count;
        TrackSample[] samples = new TrackSample[count];
        float[] planningCurvatures = new float[count];
        float[] segmentLengths = new float[count];
        float[] speedLimits = new float[count];
        float[] speeds = new float[count];

        CarPerformanceLimits baseLimits = CarPhysics.EstimatePerformanceLimits(
            _planningState!,
            car.CarConfig,
            car.TireConfig,
            _planningStrategy,
            speed: 0f,
            curvature: 0f,
            gripUsage: Config.GetAccelerationUsage(_planningStrategy)
        );

        for (int i = 0; i < count; i++)
        {
            samples[i] = track.Sample(i * planningStep);
            planningCurvatures[i] = SamplePeakCurvature(
                track,
                i * planningStep,
                planningStep * 0.5f
            );
            speedLimits[i] = LateralSpeedLimit(
                planningCurvatures[i],
                baseLimits.LateralAccelerationLimit * _driverModifiers.CombinedConfidence,
                _planningMaximumSpeedMetersPerSecond
            );
            speeds[i] = speedLimits[i];
        }

        for (int i = 0; i < count; i++)
        {
            int next = (i + 1) % count;
            segmentLengths[i] = Vector2.Distance(
                samples[i].RefPosition,
                samples[next].RefPosition
            );
        }

        for (int pass = 0; pass < Config.ClosedLoopPasses; pass++)
        {
            bool changed = ApplyForwardPass(
                car,
                planningCurvatures,
                segmentLengths,
                speedLimits,
                speeds,
                baseLimits.LateralAccelerationLimit * _driverModifiers.CombinedConfidence
            );
            changed |= ApplyBackwardPass(
                car,
                planningCurvatures,
                segmentLengths,
                speedLimits,
                speeds,
                baseLimits.LateralAccelerationLimit * _driverModifiers.CombinedConfidence
            );
            if (!changed)
                break;
        }

        VehicleSpeedProfilePoint[] points = new VehicleSpeedProfilePoint[count];
        for (int i = 0; i < count; i++)
        {
            int next = (i + 1) % count;
            float distance = segmentLengths[i];
            float desiredNetAcceleration = distance <= Config.MinimumSegmentLengthMeters
                ? 0f
                : (speeds[next] * speeds[next] - speeds[i] * speeds[i]) / (2f * distance);
            points[i] = new VehicleSpeedProfilePoint(speeds[i], desiredNetAcceleration);
        }

        return new VehicleSpeedProfile(points, planningStep, track.LengthMeters);
    }

    public CurvatureCorrectionSpeedPlan PlanCurvatureCorrection(
        RaceCar car,
        TrackData track,
        VehicleSpeedProfile globalProfile,
        float startS,
        float curvatureCorrection,
        float commandedCurvature
    )
    {
        ArgumentNullException.ThrowIfNull(car);
        ArgumentNullException.ThrowIfNull(track);
        ArgumentNullException.ThrowIfNull(globalProfile);
        EnsurePlanningSnapshot(car);

        float step = MathF.Max(Config.PlanningStepMeters, 0.5f);
        float horizon = MathF.Max(Config.CurvatureCorrectionHorizonMeters, step);
        float decayDistance = MathF.Max(
            Config.MinimumCurvatureCorrectionDecayMeters,
            MathF.Max(0f, car.State.Speed) * Config.CurvatureCorrectionDecayTimeSeconds
        );
        int count = Math.Max(
            2,
            (int)MathF.Ceiling(horizon / step) + 1
        );
        CorrectionGeometrySample[] geometry = ArrayPool<CorrectionGeometrySample>.Shared.Rent(count);
        float[] segmentLengths = ArrayPool<float>.Shared.Rent(count);
        float[] speedLimits = ArrayPool<float>.Shared.Rent(count);
        float[] speeds = ArrayPool<float>.Shared.Rent(count);
        try
        {
            CarPerformanceLimits baseLimits = CarPhysics.EstimatePerformanceLimits(
                _planningState!,
                car.CarConfig,
                car.TireConfig,
                _planningStrategy,
                speed: 0f,
                curvature: 0f,
                gripUsage: Config.GetAccelerationUsage(_planningStrategy)
            );

            float maximumAbsoluteCurvature = 0f;
            for (int i = 0; i < count; i++)
            {
                float distance = MathF.Min(i * step, horizon);
                TrackSample sample = track.Sample(startS + distance);
                float referenceCurvature = SamplePeakCurvature(
                    track,
                    startS + distance,
                    step * 0.5f
                );
                float correctionWeight = MathF.Exp(-distance / decayDistance);
                float virtualCurvature = referenceCurvature +
                                         curvatureCorrection * correctionWeight;
                if (i == 0)
                {
                    virtualCurvature = GreaterMagnitude(
                        virtualCurvature,
                        commandedCurvature
                    );
                    virtualCurvature = GreaterMagnitude(
                        virtualCurvature,
                        car.State.Telemetry.ActualCurvature
                    );
                }
                maximumAbsoluteCurvature = MathF.Max(
                    maximumAbsoluteCurvature,
                    MathF.Abs(virtualCurvature)
                );
                geometry[i] = new CorrectionGeometrySample(
                    sample.S,
                    sample.RefPosition,
                    virtualCurvature
                );
                float lateralLimit = LateralSpeedLimit(
                    virtualCurvature,
                    baseLimits.LateralAccelerationLimit * _driverModifiers.CombinedConfidence,
                    _planningMaximumSpeedMetersPerSecond
                );
                float globalLimit = globalProfile.Sample(sample.S).TargetSpeed;
                speedLimits[i] = MathF.Min(lateralLimit, globalLimit);
                speeds[i] = speedLimits[i];
                if (i == 0)
                    continue;

                float segmentLength = Vector2.Distance(
                    geometry[i - 1].Position,
                    geometry[i].Position
                );
                segmentLengths[i - 1] = segmentLength;
            }

            // This is an acyclic/open chain. One backward pass propagates all future
            // braking constraints to the start, then one forward pass propagates the
            // actual entry speed and acceleration reachability to the end.
            ApplyBackwardOpenPass(
                car,
                geometry,
                count,
                segmentLengths,
                speedLimits,
                speeds,
                baseLimits.LateralAccelerationLimit * _driverModifiers.CombinedConfidence
            );
            speeds[0] = MathF.Min(speeds[0], MathF.Max(0f, car.State.Speed));
            float firstForwardReachable = segmentLengths[0] <=
                                          Config.MinimumSegmentLengthMeters
                ? speeds[0]
                : IntegrateForward(
                    car,
                    geometry[0].Curvature,
                    geometry[1].Curvature,
                    speeds[0],
                    segmentLengths[0],
                    baseLimits.LateralAccelerationLimit * _driverModifiers.CombinedConfidence
                );
            ApplyForwardOpenPass(
                car,
                geometry,
                count,
                segmentLengths,
                speedLimits,
                speeds,
                baseLimits.LateralAccelerationLimit * _driverModifiers.CombinedConfidence
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
                referenceAcceleration = MaximumNetDriveAcceleration(
                    car,
                    speeds[0],
                    geometry[0].Curvature
                );
            }
            return new CurvatureCorrectionSpeedPlan(
                new VehicleSpeedProfilePoint(speeds[0], referenceAcceleration),
                speeds[1],
                segmentLengths[0],
                decayDistance,
                maximumAbsoluteCurvature,
                count
            );
        }
        finally
        {
            ArrayPool<CorrectionGeometrySample>.Shared.Return(geometry);
            ArrayPool<float>.Shared.Return(segmentLengths);
            ArrayPool<float>.Shared.Return(speedLimits);
            ArrayPool<float>.Shared.Return(speeds);
        }
    }

    private bool ApplyForwardPass(
        RaceCar car,
        float[] curvatures,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float lateralAccelerationLimit
    )
    {
        bool changed = false;
        for (int i = 0; i < speeds.Length; i++)
        {
            int next = (i + 1) % speeds.Length;
            float distance = segmentLengths[i];
            float reachable = distance <= Config.MinimumSegmentLengthMeters
                ? speeds[i]
                : IntegrateForward(
                    car,
                    curvatures[i],
                    curvatures[next],
                    speeds[i],
                    distance,
                    lateralAccelerationLimit
                );
            float limited = MathF.Min(speedLimits[next], reachable);
            if (limited < speeds[next] - Config.ConvergenceToleranceMetersPerSecond)
            {
                speeds[next] = limited;
                changed = true;
            }
        }
        return changed;
    }

    private bool ApplyBackwardPass(
        RaceCar car,
        float[] curvatures,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float lateralAccelerationLimit
    )
    {
        bool changed = false;
        for (int i = speeds.Length - 1; i >= 0; i--)
        {
            int next = (i + 1) % speeds.Length;
            float distance = segmentLengths[i];
            float reachable = distance <= Config.MinimumSegmentLengthMeters
                ? speeds[next]
                : IntegrateBackward(
                    car,
                    curvatures[i],
                    curvatures[next],
                    speeds[next],
                    distance,
                    lateralAccelerationLimit
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

    private float IntegrateForward(
        RaceCar car,
        float startCurvature,
        float endCurvature,
        float startSpeed,
        float distance,
        float lateralAccelerationLimit
    )
    {
        int substeps = Math.Max(1, Config.IntegrationSubsteps);
        float stepDistance = distance / substeps;
        float speed = Math.Max(0f, startSpeed);

        for (int substep = 0; substep < substeps; substep++)
        {
            float t = (substep + 0.5f) / substeps;
            float curvature = Lerp(startCurvature, endCurvature, t);
            float acceleration = MaximumNetDriveAcceleration(car, speed, curvature);
            float predicted = MathF.Sqrt(MathF.Max(0f, speed * speed + 2f * acceleration * stepDistance));
            float midpointSpeed = (speed + predicted) * 0.5f;
            float midpointAcceleration = MaximumNetDriveAcceleration(car, midpointSpeed, curvature);
            speed = MathF.Min(
                LateralSpeedLimit(
                    curvature,
                    lateralAccelerationLimit,
                    _planningMaximumSpeedMetersPerSecond
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
        float lateralAccelerationLimit
    )
    {
        int substeps = Math.Max(1, Config.IntegrationSubsteps);
        float stepDistance = distance / substeps;
        float speed = Math.Max(0f, endSpeed);

        for (int substep = substeps - 1; substep >= 0; substep--)
        {
            float t = (substep + 0.5f) / substeps;
            float curvature = Lerp(startCurvature, endCurvature, t);
            float deceleration = MaximumNetBrakeDeceleration(car, speed, curvature);
            float predicted = MathF.Sqrt(MathF.Max(0f, speed * speed + 2f * deceleration * stepDistance));
            float midpointSpeed = (speed + predicted) * 0.5f;
            float midpointDeceleration = MaximumNetBrakeDeceleration(car, midpointSpeed, curvature);
            speed = MathF.Min(
                LateralSpeedLimit(
                    curvature,
                    lateralAccelerationLimit,
                    _planningMaximumSpeedMetersPerSecond
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
                          _driverModifiers.CombinedConfidence;
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
                          _driverModifiers.CombinedConfidence;
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
            assumedLongitudinalAcceleration
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

    private float LateralSpeedLimit(
        float curvature,
        float lateralAccelerationLimit,
        float maximumSpeedMetersPerSecond
    )
    {
        float absoluteCurvature = MathF.Abs(curvature);
        if (absoluteCurvature <= Config.CurvatureEpsilon)
            return maximumSpeedMetersPerSecond;
        return MathF.Min(
            maximumSpeedMetersPerSecond,
            MathF.Sqrt(Math.Max(0f, lateralAccelerationLimit) / absoluteCurvature)
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

    private void BeginPlanningSnapshot(
        RaceCar car,
        DriverPlanningModifiers driverModifiers
    )
    {
        _planningCar = car;
        _planningState = car.State.Clone();
        _planningStrategy = car.Strategy;
        _driverModifiers = new DriverPlanningModifiers(
            Math.Clamp(driverModifiers.PaceEfficiency, 0.8f, 1f),
            Math.Clamp(driverModifiers.EstimatedGripScale, 0.9f, 1.1f)
        );
        _planningMaximumSpeedMetersPerSecond = EstimateMaximumSpeedMetersPerSecond(car);
        _driveAccelerationCache.Clear();
        _brakeDecelerationCache.Clear();
    }

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
        if (config.PlanningStepMeters <= 0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Planning step must be positive.");
        if (config.IntegrationSubsteps < 1 || config.ClosedLoopPasses < 1)
            throw new ArgumentOutOfRangeException(nameof(config), "Planning pass counts must be positive.");
        if (config.AccelerationSolveIterations < 1)
            throw new ArgumentOutOfRangeException(nameof(config), "Acceleration iterations must be positive.");
        if (config.CurvatureCorrectionHorizonMeters <= 0f ||
            config.CurvatureCorrectionDecayTimeSeconds <= 0f ||
            config.MinimumCurvatureCorrectionDecayMeters <= 0f ||
            config.CurvatureCorrectionActivationThreshold < 0f)
        {
            throw new ArgumentOutOfRangeException(nameof(config), "Curvature correction distances must be positive.");
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

    private bool ApplyForwardOpenPass(
        RaceCar car,
        CorrectionGeometrySample[] geometry,
        int count,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float lateralAccelerationLimit
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
                    geometry[i].Curvature,
                    geometry[i + 1].Curvature,
                    speeds[i],
                    distance,
                    lateralAccelerationLimit
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
        CorrectionGeometrySample[] geometry,
        int count,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float lateralAccelerationLimit
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
                    geometry[i].Curvature,
                    geometry[i + 1].Curvature,
                    speeds[i + 1],
                    distance,
                    lateralAccelerationLimit
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

    private readonly record struct CorrectionGeometrySample(
        float TrackS,
        Vector2 Position,
        float Curvature
    );

}
