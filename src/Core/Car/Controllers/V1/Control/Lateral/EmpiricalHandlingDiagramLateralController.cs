using System;
using Godot;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;

public sealed class EmpiricalHandlingDiagramLateralController
{
    private readonly EmpiricalHandlingDiagramControlConfig _config;
    private EmpiricalHandlingDiagramModel _model;
    private float _lastSteeringAngle;

    public EmpiricalHandlingDiagramLateralController(
        EmpiricalHandlingDiagramControlConfig? config = null,
        EmpiricalHandlingDiagramModel? model = null
    )
    {
        _config = config ?? new EmpiricalHandlingDiagramControlConfig();
        _model = model ?? EmpiricalHandlingDiagramModel.Zero;
        Validate(_config);
    }

    public EmpiricalHandlingDiagramModel Model => _model;

    public void Initialize(CarConfig carConfig, TrackData track)
    {
        ArgumentNullException.ThrowIfNull(carConfig);
        ArgumentNullException.ThrowIfNull(track);
        _model = EmpiricalHandlingDiagramCalibrator.Calibrate(carConfig, track, _config.Calibration);
        Reset();
    }

    public void Reset()
    {
        _lastSteeringAngle = 0.0f;
    }

    public LateralControlOutput Control(
        SpeedProfile profile,
        CarSensor sensor,
        CarConfig carConfig,
        TrackData track,
        float dt
    )
    {
        ArgumentNullException.ThrowIfNull(profile);
        ArgumentNullException.ThrowIfNull(carConfig);
        ArgumentNullException.ThrowIfNull(track);
        if (profile.Count == 0)
            return default;

        SpeedProfileSampler.Projection projection = SpeedProfileSampler.ProjectPosition(profile, sensor.Position);
        SpeedProfilePoint anchor = projection.Point;
        int nearestIndex = projection.Index;

        float currentSpeed = sensor.LinearVelocity.Length();
        float controlSpeed = MathF.Max(currentSpeed, _config.MinimumControlSpeedMetersPerSecond);
        float lookaheadDistance = CalculateLookaheadDistance(controlSpeed);
        SpeedProfilePoint lookahead = SpeedProfileSampler.SampleAtDistance(profile, anchor.Distance + lookaheadDistance);
        float trackingSpeed = MathF.Max(
            MathF.Min(currentSpeed, lookahead.Speed),
            _config.MinimumControlSpeedMetersPerSecond
        );

        float targetLateralAcceleration = trackingSpeed * trackingSpeed * lookahead.Curvature;
        float feedForwardSteering = _model.PredictSteeringAngle(
            carConfig,
            targetLateralAcceleration,
            trackingSpeed
        );

        Vector2 localVelocity = sensor.LinearVelocity.Rotated(-sensor.Rotation);
        float beta = MathF.Atan2(localVelocity.Y, MathF.Max(MathF.Abs(localVelocity.X), 1.0f));
        float velocityHeading = sensor.Rotation + beta;
        Vector2 pathNormal = new(-MathF.Sin(anchor.Heading), MathF.Cos(anchor.Heading));
        float lateralError = Mathf.Clamp(
            (sensor.Position - anchor.Position).Dot(pathNormal),
            -_config.MaximumLateralErrorMeters,
            _config.MaximumLateralErrorMeters
        );
        float headingError = Mathf.Clamp(
            SpeedProfileSampler.WrapAngle(velocityHeading - lookahead.Heading),
            -_config.MaximumHeadingErrorRadians,
            _config.MaximumHeadingErrorRadians
        );
        float geometricSteering = CalculatePurePursuitSteering(
            carConfig,
            sensor.Position,
            sensor.Rotation,
            lookahead.Position
        );
        float feedbackSteering = Mathf.Clamp(
            geometricSteering - feedForwardSteering,
            -_config.MaximumFeedbackSteeringRadians,
            _config.MaximumFeedbackSteeringRadians
        );

        float yawRateReference = trackingSpeed * lookahead.Curvature;
        float betaReference = Mathf.Clamp(
            _config.BetaReferenceGain * carConfig.Chassis.WheelBase * lookahead.Curvature,
            -_config.MaximumBetaReferenceRadians,
            _config.MaximumBetaReferenceRadians
        );
        float betaError = beta - betaReference;
        float yawRateError = sensor.AngularVelocity - yawRateReference;
        float stabilityRisk = CalculateStabilityRisk(betaError, yawRateError);

        float yawDampingSteering = Mathf.Clamp(
            -_config.YawRateDampingGain * yawRateError,
            -_config.MaximumYawDampingSteeringRadians,
            _config.MaximumYawDampingSteeringRadians
        );
        float betaDampingSteering = Mathf.Clamp(
            -_config.BetaDampingGain * betaError,
            -_config.MaximumBetaDampingSteeringRadians,
            _config.MaximumBetaDampingSteeringRadians
        );
        betaDampingSteering = ApplyYawPriorityToConflictingBetaDamping(
            yawDampingSteering,
            betaDampingSteering,
            yawRateError
        );
        feedbackSteering = ApplyYawPriorityToConflictingPathFeedback(
            feedbackSteering,
            yawDampingSteering,
            yawRateError
        );
        feedbackSteering *= CalculateFeedbackScale(stabilityRisk);
        TrackBoundaryState trackBoundary = CalculateTrackBoundaryState(sensor.Position, carConfig, track);
        float stabilitySteering = Mathf.Clamp(
            yawDampingSteering + betaDampingSteering,
            -_config.MaximumStabilitySteeringRadians,
            _config.MaximumStabilitySteeringRadians
        );

        float targetSteeringAngle = Mathf.Clamp(
            feedForwardSteering + feedbackSteering + stabilitySteering,
            -carConfig.Chassis.MaxSteerAngle,
            carConfig.Chassis.MaxSteerAngle
        );
        float steeringAngle = ApplySteeringRateLimit(targetSteeringAngle, dt);
        float steeringInput = carConfig.Chassis.MaxSteerAngle <= 1e-5f
            ? 0.0f
            : steeringAngle / carConfig.Chassis.MaxSteerAngle;

        return new LateralControlOutput(
            SteeringInput: Mathf.Clamp(steeringInput, -1.0f, 1.0f),
            SteeringAngle: steeringAngle,
            FeedForwardSteeringAngle: feedForwardSteering,
            FeedbackSteeringAngle: feedbackSteering,
            StabilitySteeringAngle: stabilitySteering,
            YawDampingSteeringAngle: yawDampingSteering,
            BetaDampingSteeringAngle: betaDampingSteering,
            LookaheadDistance: lookaheadDistance,
            LookaheadPoint: lookahead.Position,
            TargetSpeed: trackingSpeed,
            TargetLateralAcceleration: targetLateralAcceleration,
            NearestProfileIndex: nearestIndex,
            ProfileDistance: anchor.Distance,
            LateralError: lateralError,
            HeadingError: headingError,
            Beta: beta,
            BetaReference: betaReference,
            YawRate: sensor.AngularVelocity,
            YawRateReference: yawRateReference,
            YawRateError: yawRateError,
            StabilityRisk: stabilityRisk,
            TrackOffset: trackBoundary.Offset,
            TrackUsableHalfWidth: trackBoundary.UsableHalfWidth,
            TrackBoundaryExcess: trackBoundary.AsphaltExcess,
            TrackBufferBoundaryExcess: trackBoundary.BufferExcess
        );
    }

    private float CalculateLookaheadDistance(float targetSpeed)
    {
        return Mathf.Clamp(
            _config.LookaheadBaseMeters + _config.LookaheadSpeedGainSeconds * targetSpeed,
            _config.MinimumLookaheadMeters,
            _config.MaximumLookaheadMeters
        );
    }

    private static float CalculatePurePursuitSteering(
        CarConfig carConfig,
        Vector2 position,
        float heading,
        Vector2 lookaheadPosition
    )
    {
        Vector2 localLookahead = (lookaheadPosition - position).Rotated(-heading);
        float distance = MathF.Max(localLookahead.Length(), 1e-3f);
        float alpha = MathF.Atan2(localLookahead.Y, localLookahead.X);
        float curvature = 2.0f * MathF.Sin(alpha) / distance;
        return Mathf.Clamp(
            MathF.Atan(carConfig.Chassis.WheelBase * curvature),
            -carConfig.Chassis.MaxSteerAngle,
            carConfig.Chassis.MaxSteerAngle
        );
    }

    private float CalculateStabilityRisk(float betaError, float yawRateError)
    {
        float betaRisk = Mathf.InverseLerp(_config.VscBetaStartRadians, _config.VscBetaFullRadians, MathF.Abs(betaError));
        float yawRisk = Mathf.InverseLerp(
            _config.VscYawRateErrorStartRadiansPerSecond,
            _config.VscYawRateErrorFullRadiansPerSecond,
            MathF.Abs(yawRateError)
        );
        return Mathf.Clamp(MathF.Max(betaRisk, yawRisk), 0.0f, 1.0f);
    }

    private float CalculateFeedbackScale(float stabilityRisk)
    {
        return Mathf.Clamp(1.0f - stabilityRisk * _config.StabilityFeedbackCutFactor, 0.0f, 1.0f);
    }

    private float ApplyYawPriorityToConflictingBetaDamping(
        float yawDampingSteering,
        float betaDampingSteering,
        float yawRateError
    )
    {
        if (MathF.Abs(yawDampingSteering) <= 1e-5f || MathF.Abs(betaDampingSteering) <= 1e-5f)
            return betaDampingSteering;
        if (MathF.Sign(yawDampingSteering) == MathF.Sign(betaDampingSteering))
            return betaDampingSteering;

        float yawPriority = Mathf.Clamp(
            Mathf.InverseLerp(
                _config.VscYawRateErrorStartRadiansPerSecond,
                _config.VscYawRateErrorFullRadiansPerSecond,
                MathF.Abs(yawRateError)
            ),
            0.0f,
            1.0f
        );
        float betaScale = Mathf.Lerp(1.0f, _config.ConflictingBetaDampingScale, yawPriority);
        return betaDampingSteering * betaScale;
    }

    private float ApplyYawPriorityToConflictingPathFeedback(
        float feedbackSteering,
        float yawDampingSteering,
        float yawRateError
    )
    {
        if (MathF.Abs(feedbackSteering) <= 1e-5f || MathF.Abs(yawDampingSteering) <= 1e-5f)
            return feedbackSteering;
        if (MathF.Sign(feedbackSteering) == MathF.Sign(yawDampingSteering))
            return feedbackSteering;

        float yawPriority = Mathf.Clamp(
            Mathf.InverseLerp(
                _config.PathFeedbackYawConflictStartRadiansPerSecond,
                _config.PathFeedbackYawConflictFullRadiansPerSecond,
                MathF.Abs(yawRateError)
            ),
            0.0f,
            1.0f
        );
        float feedbackScale = Mathf.Lerp(1.0f, _config.ConflictingPathFeedbackScale, yawPriority);
        return feedbackSteering * feedbackScale;
    }

    private static TrackBoundaryState CalculateTrackBoundaryState(
        Vector2 position,
        CarConfig carConfig,
        TrackData track
    )
    {
        int trackIndex = track.FindNearestIndex(position);
        TrackPoint point = track[trackIndex];
        float offset = (position - point.Center).Dot(point.Normal);
        float usableHalfWidth = MathF.Max(0.0f, point.HalfWidth - carConfig.Chassis.Width * 0.5f);
        float bufferHalfWidth = offset >= 0.0f
            ? point.HalfWidth + point.LeftBufferWidth
            : point.HalfWidth + point.RightBufferWidth;

        return new TrackBoundaryState(
            offset,
            usableHalfWidth,
            MathF.Max(0.0f, MathF.Abs(offset) - usableHalfWidth),
            MathF.Max(0.0f, MathF.Abs(offset) - MathF.Max(0.0f, bufferHalfWidth))
        );
    }

    private float ApplySteeringRateLimit(float steeringAngle, float dt)
    {
        if (!float.IsFinite(_config.MaximumSteeringRateRadiansPerSecond) || dt <= 0.0f)
        {
            _lastSteeringAngle = steeringAngle;
            return steeringAngle;
        }

        float maxDelta = _config.MaximumSteeringRateRadiansPerSecond * dt;
        float limited = Mathf.Clamp(steeringAngle, _lastSteeringAngle - maxDelta, _lastSteeringAngle + maxDelta);
        _lastSteeringAngle = limited;
        return limited;
    }

    private static void Validate(EmpiricalHandlingDiagramControlConfig config)
    {
        if (config.MinimumLookaheadMeters <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Minimum lookahead must be positive.");
        if (config.MaximumLookaheadMeters < config.MinimumLookaheadMeters)
            throw new ArgumentOutOfRangeException(nameof(config), "Maximum lookahead must be at least the minimum lookahead.");
        if (config.MinimumControlSpeedMetersPerSecond <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Minimum control speed must be positive.");
        if (config.VscBetaFullRadians < config.VscBetaStartRadians)
            throw new ArgumentOutOfRangeException(nameof(config), "Full beta risk must be at least the start beta risk.");
        if (config.VscYawRateErrorFullRadiansPerSecond < config.VscYawRateErrorStartRadiansPerSecond)
            throw new ArgumentOutOfRangeException(nameof(config), "Full yaw risk must be at least the start yaw risk.");
        if (config.StabilityFeedbackCutFactor is < 0.0f or > 1.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Stability feedback cut factor must be between zero and one.");
        if (config.ConflictingBetaDampingScale is < 0.0f or > 1.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Conflicting beta damping scale must be between zero and one.");
        if (config.PathFeedbackYawConflictFullRadiansPerSecond < config.PathFeedbackYawConflictStartRadiansPerSecond)
            throw new ArgumentOutOfRangeException(nameof(config), "Full path-feedback yaw conflict must be at least the start.");
        if (config.ConflictingPathFeedbackScale is < 0.0f or > 1.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Conflicting path feedback scale must be between zero and one.");
    }

    private readonly record struct TrackBoundaryState(
        float Offset,
        float UsableHalfWidth,
        float AsphaltExcess,
        float BufferExcess
    );
}
