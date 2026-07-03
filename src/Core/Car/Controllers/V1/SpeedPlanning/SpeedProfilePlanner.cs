using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Online;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;

public sealed class SpeedProfilePlanner
{
    private readonly SpeedPlanningConfig _config;

    public SpeedProfilePlanner(SpeedPlanningConfig? config = null)
    {
        _config = config ?? new SpeedPlanningConfig();
        ValidateConfig(_config);
    }

    public SpeedProfile Plan(DynamicPathOnlinePath path, CarConfig carConfig, float initialSpeedMetersPerSecond = 0.0f)
    {
        return Plan(path.Samples, carConfig, initialSpeedMetersPerSecond);
    }

    public SpeedProfile Plan(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarConfig carConfig,
        float initialSpeedMetersPerSecond = 0.0f
    )
    {
        return Plan(samples, carConfig, SpeedPlanningState.FromConfig(carConfig), initialSpeedMetersPerSecond);
    }

    public SpeedProfile Plan(
        DynamicPathOnlinePath path,
        CarConfig carConfig,
        SpeedPlanningState state,
        float initialSpeedMetersPerSecond = 0.0f
    )
    {
        return Plan(path.Samples, carConfig, state, initialSpeedMetersPerSecond);
    }

    public SpeedProfile PlanCurrentFrame(
        DynamicPathOnlinePath path,
        CarSensor carSensor,
        CarLogic carLogic,
        TrackData track,
        float dirtyAirFactor = 0.0f
    )
    {
        return PlanCurrentFrame(path.Samples, carSensor, carLogic, track, dirtyAirFactor);
    }

    public SpeedProfile PlanCurrentFrame(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarSensor carSensor,
        CarLogic carLogic,
        TrackData track,
        float dirtyAirFactor = 0.0f
    )
    {
        SpeedPlanningState state = SpeedPlanningState.FromCurrentFrame(carSensor, carLogic, track, dirtyAirFactor);
        return Plan(samples, carLogic.Config, state, carSensor.LinearVelocity.Length());
    }

    public SpeedProfile Plan(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarConfig carConfig,
        SpeedPlanningState state,
        float initialSpeedMetersPerSecond = 0.0f
    )
    {
        ArgumentNullException.ThrowIfNull(samples);
        ArgumentNullException.ThrowIfNull(carConfig);

        if (samples.Count == 0)
            return new SpeedProfile([]);

        float[] distances = CalculateDistances(samples);
        float[] speedLimits = CalculateSpeedLimits(samples, carConfig, state);
        float[] speeds = new float[samples.Count];
        for (int i = 0; i < speeds.Length; i++)
            speeds[i] = speedLimits[i];

        speeds[0] = MathF.Min(MathF.Max(0.0f, initialSpeedMetersPerSecond), speedLimits[0]);
        ApplyForwardAccelerationPass(samples, carConfig, state, speeds, speedLimits);
        ApplyBackwardBrakingPass(samples, carConfig, state, speeds);

        return BuildProfile(samples, carConfig, state, distances, speedLimits, speeds);
    }

    private void ApplyForwardAccelerationPass(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarConfig carConfig,
        SpeedPlanningState state,
        float[] speeds,
        float[] speedLimits
    )
    {
        for (int i = 0; i < samples.Count - 1; i++)
        {
            float segmentLength = MathF.Max(0.0f, samples[i].LengthToNext);
            if (segmentLength <= _config.MinimumSegmentLengthMeters)
            {
                speeds[i + 1] = MathF.Min(speeds[i + 1], speeds[i]);
                continue;
            }

            float reachableSpeed = IntegrateForwardSegment(
                samples,
                carConfig,
                state,
                i,
                speeds[i],
                segmentLength
            );
            speeds[i + 1] = MathF.Min(speedLimits[i + 1], MathF.Min(speeds[i + 1], reachableSpeed));
        }
    }

    private void ApplyBackwardBrakingPass(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarConfig carConfig,
        SpeedPlanningState state,
        float[] speeds
    )
    {
        if (float.IsFinite(_config.TerminalSpeedMetersPerSecond))
            speeds[^1] = MathF.Min(speeds[^1], MathF.Max(0.0f, _config.TerminalSpeedMetersPerSecond));

        for (int i = samples.Count - 2; i >= 0; i--)
        {
            float segmentLength = MathF.Max(0.0f, samples[i].LengthToNext);
            if (segmentLength <= _config.MinimumSegmentLengthMeters)
            {
                speeds[i] = MathF.Min(speeds[i], speeds[i + 1]);
                continue;
            }

            float reachableSpeed = IntegrateBackwardSegment(
                samples,
                carConfig,
                state,
                i,
                speeds[i + 1],
                segmentLength
            );
            speeds[i] = MathF.Min(speeds[i], reachableSpeed);
        }
    }

    private float IntegrateForwardSegment(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarConfig carConfig,
        SpeedPlanningState state,
        int segmentIndex,
        float startSpeed,
        float segmentLength
    )
    {
        int substeps = Math.Max(1, _config.IntegrationSubsteps);
        float substepLength = segmentLength / substeps;
        float speed = MathF.Max(0.0f, startSpeed);

        for (int substep = 0; substep < substeps; substep++)
        {
            float segmentT = (substep + 0.5f) / substeps;
            float curvature = InterpolateCurvature(samples, segmentIndex, segmentT);
            float acceleration = SpeedPlanningDynamics.SolveMaxAcceleration(
                carConfig,
                state,
                speed,
                curvature,
                _config
            );
            float predictedSpeed = MathF.Sqrt(MathF.Max(0.0f, speed * speed + 2.0f * acceleration * substepLength));
            float midpointSpeed = (speed + predictedSpeed) * 0.5f;
            float midpointAcceleration = SpeedPlanningDynamics.SolveMaxAcceleration(
                carConfig,
                state,
                midpointSpeed,
                curvature,
                _config
            );
            float substepLimit = SpeedPlanningDynamics.CalculateLateralSpeedLimit(
                carConfig,
                state,
                curvature,
                _config
            );
            speed = MathF.Min(
                substepLimit,
                MathF.Sqrt(MathF.Max(0.0f, speed * speed + 2.0f * midpointAcceleration * substepLength))
            );
        }

        return speed;
    }

    private float IntegrateBackwardSegment(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarConfig carConfig,
        SpeedPlanningState state,
        int segmentIndex,
        float endSpeed,
        float segmentLength
    )
    {
        int substeps = Math.Max(1, _config.IntegrationSubsteps);
        float substepLength = segmentLength / substeps;
        float speed = MathF.Max(0.0f, endSpeed);

        for (int substep = substeps - 1; substep >= 0; substep--)
        {
            float segmentT = (substep + 0.5f) / substeps;
            float curvature = InterpolateCurvature(samples, segmentIndex, segmentT);
            float deceleration = SpeedPlanningDynamics.SolveMaxDeceleration(
                carConfig,
                state,
                speed,
                curvature,
                _config
            );
            float predictedSpeed = MathF.Sqrt(MathF.Max(0.0f, speed * speed + 2.0f * deceleration * substepLength));
            float midpointSpeed = (speed + predictedSpeed) * 0.5f;
            float midpointDeceleration = SpeedPlanningDynamics.SolveMaxDeceleration(
                carConfig,
                state,
                midpointSpeed,
                curvature,
                _config
            );
            float substepLimit = SpeedPlanningDynamics.CalculateLateralSpeedLimit(
                carConfig,
                state,
                curvature,
                _config
            );
            speed = MathF.Min(
                substepLimit,
                MathF.Sqrt(MathF.Max(0.0f, speed * speed + 2.0f * midpointDeceleration * substepLength))
            );
        }

        return speed;
    }

    private SpeedProfile BuildProfile(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarConfig carConfig,
        SpeedPlanningState state,
        float[] distances,
        float[] speedLimits,
        float[] speeds
    )
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[samples.Count];
        float[] accelerationToNext = new float[samples.Count];
        float[] timeFromStart = new float[samples.Count];

        for (int i = 0; i < samples.Count - 1; i++)
        {
            float segmentLength = MathF.Max(0.0f, samples[i].LengthToNext);
            if (segmentLength <= _config.MinimumSegmentLengthMeters)
            {
                timeFromStart[i + 1] = timeFromStart[i];
                continue;
            }

            accelerationToNext[i] = (speeds[i + 1] * speeds[i + 1] - speeds[i] * speeds[i]) / (2.0f * segmentLength);
            float averageSpeedTerm = speeds[i] + speeds[i + 1];
            float segmentTime = averageSpeedTerm > 1e-3f ? 2.0f * segmentLength / averageSpeedTerm : 0.0f;
            timeFromStart[i + 1] = timeFromStart[i] + segmentTime;
        }

        for (int i = 0; i < samples.Count; i++)
        {
            DynamicPathEdgeSample sample = samples[i];
            float speed = speeds[i];
            float curvature = sample.Curvature;
            float maxAcceleration = SpeedPlanningDynamics.SolveMaxAcceleration(carConfig, state, speed, curvature, _config);
            float maxDeceleration = SpeedPlanningDynamics.SolveMaxDeceleration(carConfig, state, speed, curvature, _config);

            points[i] = new SpeedProfilePoint(
                SampleIndex: i,
                Position: sample.Position,
                Heading: sample.Heading,
                Curvature: curvature,
                Distance: distances[i],
                Speed: speed,
                AccelerationToNext: accelerationToNext[i],
                TimeFromStart: timeFromStart[i],
                MaxSpeed: speedLimits[i],
                MaxAcceleration: maxAcceleration,
                MaxDeceleration: maxDeceleration,
                LateralAcceleration: speed * speed * MathF.Abs(curvature)
            );
        }

        return new SpeedProfile(points);
    }

    private float[] CalculateDistances(IReadOnlyList<DynamicPathEdgeSample> samples)
    {
        float[] distances = new float[samples.Count];
        for (int i = 1; i < samples.Count; i++)
            distances[i] = distances[i - 1] + MathF.Max(0.0f, samples[i - 1].LengthToNext);
        return distances;
    }

    private float[] CalculateSpeedLimits(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        CarConfig carConfig,
        SpeedPlanningState state
    )
    {
        float[] speedLimits = new float[samples.Count];
        for (int i = 0; i < samples.Count; i++)
        {
            speedLimits[i] = SpeedPlanningDynamics.CalculateLateralSpeedLimit(
                carConfig,
                state,
                samples[i].Curvature,
                _config
            );
        }

        return speedLimits;
    }

    private static void ValidateConfig(SpeedPlanningConfig config)
    {
        if (config.MaximumSpeedMetersPerSecond <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Maximum speed must be positive.");
        if (config.FrictionUsage <= 0.0f || config.FrictionUsage > 1.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Friction usage must be in (0, 1].");
        if (config.TrackFrictionMultiplier <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Track friction multiplier must be positive.");
        if (config.LoadTransferIterations < 1)
            throw new ArgumentOutOfRangeException(nameof(config), "Load transfer iterations must be at least one.");
        if (config.IntegrationSubsteps < 1)
            throw new ArgumentOutOfRangeException(nameof(config), "Integration substeps must be at least one.");
        if (config.LateralSpeedSearchIterations < 1)
            throw new ArgumentOutOfRangeException(nameof(config), "Lateral speed search iterations must be at least one.");
    }

    private static float InterpolateCurvature(
        IReadOnlyList<DynamicPathEdgeSample> samples,
        int segmentIndex,
        float t
    )
    {
        float start = samples[segmentIndex].Curvature;
        if (segmentIndex + 1 >= samples.Count)
            return start;

        float end = samples[segmentIndex + 1].Curvature;
        return Mathf.Lerp(start, end, Mathf.Clamp(t, 0.0f, 1.0f));
    }
}
