using System;
using Godot;
using StintegyEVO.Core.Car.Components;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;
using StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.Control.Longitudinal;

public sealed class StabilityAwareSpeedController
{
    private readonly StabilityAwareSpeedControlConfig _config;

    public StabilityAwareSpeedController(StabilityAwareSpeedControlConfig? config = null)
    {
        _config = config ?? new StabilityAwareSpeedControlConfig();
        Validate(_config);
    }

    public StabilityAwareSpeedControlOutput Control(
        SpeedProfile profile,
        LateralControlOutput lateralOutput,
        CarSensor sensor,
        CarLogic carLogic,
        TrackData track
    )
    {
        ArgumentNullException.ThrowIfNull(profile);
        ArgumentNullException.ThrowIfNull(carLogic);
        ArgumentNullException.ThrowIfNull(track);
        if (profile.Count == 0 || lateralOutput.NearestProfileIndex < 0)
            return default;

        float currentSpeed = sensor.LinearVelocity.Length();
        float profileDistance = Mathf.Clamp(lateralOutput.ProfileDistance, 0.0f, profile.PhysicalLength);
        SpeedProfilePoint nearest = SpeedProfileSampler.SampleAtDistance(profile, profileDistance);
        float lookaheadDistance = _config.SpeedLookaheadBaseMeters + _config.SpeedLookaheadTimeSeconds * currentSpeed;
        float targetDistance = nearest.Distance + lookaheadDistance;
        SpeedProfilePoint target = SpeedProfileSampler.SampleAtDistance(profile, targetDistance);
        float speedError = target.Speed - currentSpeed;
        float speedErrorAcceleration = Mathf.Clamp(
            _config.SpeedErrorGain * speedError,
            -_config.MaximumSpeedErrorAccelerationMetersPerSecondSquared,
            _config.MaximumSpeedErrorAccelerationMetersPerSecondSquared
        );
        float requestedAcceleration =
            _config.AccelerationFeedForwardGain * target.AccelerationToNext +
            speedErrorAcceleration;
        requestedAcceleration = MathF.Min(
            requestedAcceleration,
            CalculateFutureSpeedBrakingAccelerationCap(profile, nearest.Distance, currentSpeed)
        );

        SpeedPlanningState state = SpeedPlanningState.FromCurrentFrame(sensor, carLogic, track);
        float pathCurvature = SelectDominantCurvature(
            SelectDominantCurvature(nearest.Curvature, target.Curvature),
            target.Curvature
        );
        float demandCurvature = SelectDemandCurvature(pathCurvature, lateralOutput, currentSpeed);
        float lateralDemandAcceleration = currentSpeed * currentSpeed * MathF.Abs(demandCurvature);
        float lateralSpeedLimit = SpeedPlanningDynamics.CalculateLateralSpeedLimit(
            carLogic.Config,
            state,
            demandCurvature,
            _config.EnvelopeConfig
        );
        float pathLateralSpeedLimit = SpeedPlanningDynamics.CalculateLateralSpeedLimit(
            carLogic.Config,
            state,
            pathCurvature,
            _config.EnvelopeConfig
        );
        if (currentSpeed > lateralSpeedLimit + 0.1f)
        {
            float brakingDistance = MathF.Max(lookaheadDistance, 1.0f);
            float correctiveAcceleration = (lateralSpeedLimit * lateralSpeedLimit - currentSpeed * currentSpeed)
                / (2.0f * brakingDistance);
            requestedAcceleration = MathF.Min(
                requestedAcceleration,
                MathF.Max(correctiveAcceleration, -_config.MaximumCorrectiveBrakeMetersPerSecondSquared)
            );
        }

        float pathTrackingRisk = CalculateTrackingRisk(lateralOutput, currentSpeed);
        float trackBoundaryRisk = CalculateTrackBoundaryRisk(lateralOutput);
        if (trackBoundaryRisk > 0.0f)
        {
            requestedAcceleration = MathF.Min(
                requestedAcceleration,
                CalculateTrackBoundaryRecoveryAccelerationCap(trackBoundaryRisk, currentSpeed)
            );
        }
        float lateralStabilityRisk = lateralOutput.StabilityRisk * CalculateStabilityBrakeSpeedScale(currentSpeed);
        float dynamicStabilityRisk = MathF.Max(
            lateralStabilityRisk,
            CalculateSlideRisk(sensor.Params, currentSpeed)
        );
        if (dynamicStabilityRisk > 0.0f)
        {
            requestedAcceleration = MathF.Min(
                requestedAcceleration,
                -_config.StabilityRiskBrakeMetersPerSecondSquared * dynamicStabilityRisk
            );
        }
        float trackingRisk = MathF.Max(pathTrackingRisk, trackBoundaryRisk);
        float controlRisk = MathF.Max(dynamicStabilityRisk, trackBoundaryRisk);
        float throttleRisk = MathF.Max(dynamicStabilityRisk, trackBoundaryRisk);

        float maxAcceleration = SpeedPlanningDynamics.SolveMaxAcceleration(
            carLogic.Config,
            state,
            currentSpeed,
            demandCurvature,
            _config.EnvelopeConfig
        ) * CalculateThrottleFactor(throttleRisk);
        float maxDeceleration = SpeedPlanningDynamics.SolveMaxDeceleration(
            carLogic.Config,
            state,
            currentSpeed,
            curvature: 0.0f,
            _config.EnvelopeConfig
        );
        float limitedAcceleration = Mathf.Clamp(requestedAcceleration, -maxDeceleration, maxAcceleration);
        float input = AccelerationToInput(carLogic.Config, sensor.Mass, currentSpeed, limitedAcceleration);

        return new StabilityAwareSpeedControlOutput(
            Input: input,
            TargetSpeed: target.Speed,
            TargetAcceleration: target.AccelerationToNext,
            RequestedAcceleration: requestedAcceleration,
            LimitedAcceleration: limitedAcceleration,
            MaximumAcceleration: maxAcceleration,
            MaximumDeceleration: maxDeceleration,
            LateralDemandAcceleration: lateralDemandAcceleration,
            LateralSpeedLimit: lateralSpeedLimit,
            TrackingRisk: trackingRisk,
            StabilityRisk: controlRisk
        );
    }

    private static float SelectDominantCurvature(float a, float b)
    {
        return MathF.Abs(a) >= MathF.Abs(b) ? a : b;
    }

    private static float SelectDemandCurvature(
        float pathCurvature,
        LateralControlOutput lateralOutput,
        float currentSpeed
    )
    {
        float speedSquared = currentSpeed * currentSpeed;
        if (speedSquared <= 1e-4f)
            return pathCurvature;

        float yawDemand = lateralOutput.YawRate * currentSpeed / speedSquared;
        float referenceDemand = lateralOutput.YawRateReference * currentSpeed / speedSquared;
        float dominant = SelectDominantCurvature(pathCurvature, referenceDemand);
        return SelectDominantCurvature(dominant, yawDemand);
    }

    private float CalculateThrottleFactor(float stabilityRisk)
    {
        float cut = Mathf.InverseLerp(
            _config.StabilityRiskThrottleCutStart,
            _config.FullThrottleCutRisk,
            stabilityRisk
        );
        return 1.0f - Mathf.Clamp(cut, 0.0f, 1.0f);
    }

    private float CalculateStabilityBrakeSpeedScale(float currentSpeed)
    {
        return Mathf.Clamp(
            Mathf.InverseLerp(
                _config.StabilityBrakeStartSpeedMetersPerSecond,
                _config.StabilityBrakeFullSpeedMetersPerSecond,
                currentSpeed
            ),
            0.0f,
            1.0f
        );
    }

    private float CalculateFutureSpeedBrakingAccelerationCap(
        SpeedProfile profile,
        float startDistance,
        float currentSpeed
    )
    {
        if (currentSpeed <= 1e-3f)
            return float.PositiveInfinity;

        float cap = float.PositiveInfinity;
        for (int i = 0; i < profile.Count; i++)
        {
            SpeedProfilePoint point = profile[i];
            float distance = point.Distance - startDistance;
            if (distance <= 1.0f || point.Speed >= currentSpeed)
                continue;

            float requiredAcceleration = (point.Speed * point.Speed - currentSpeed * currentSpeed)
                / (2.0f * distance);
            if (requiredAcceleration > -_config.FutureBrakingActivationMetersPerSecondSquared)
                continue;

            cap = MathF.Min(cap, requiredAcceleration);
        }

        return float.IsFinite(cap)
            ? MathF.Max(cap, -_config.MaximumCorrectiveBrakeMetersPerSecondSquared)
            : float.PositiveInfinity;
    }

    private float CalculateSlideRisk(IntermediateParams parameters, float currentSpeed)
    {
        if (currentSpeed < _config.MinimumSpeedForSlideRiskMetersPerSecond)
            return 0.0f;

        int slidingTires = 0;
        if (parameters.FrontLeft.IsSliding) slidingTires++;
        if (parameters.FrontRight.IsSliding) slidingTires++;
        if (parameters.RearLeft.IsSliding) slidingTires++;
        if (parameters.RearRight.IsSliding) slidingTires++;

        if (slidingTires == 0)
            return 0.0f;
        return Mathf.Clamp(slidingTires * _config.SlideThrottleCutFactor, 0.0f, 1.0f);
    }

    private float CalculateTrackingRisk(LateralControlOutput lateralOutput, float currentSpeed)
    {
        float lateralRisk = Mathf.Clamp(
            Mathf.InverseLerp(
                _config.TrackingLateralErrorStartMeters,
                _config.TrackingLateralErrorFullMeters,
                MathF.Abs(lateralOutput.LateralError)
            ),
            0.0f,
            1.0f
        );
        float headingRisk = Mathf.Clamp(
            Mathf.InverseLerp(
                _config.TrackingHeadingErrorStartRadians,
                _config.TrackingHeadingErrorFullRadians,
                MathF.Abs(lateralOutput.HeadingError)
            ),
            0.0f,
            1.0f
        );
        float speedScale = Mathf.Clamp(
            Mathf.InverseLerp(
                _config.TrackingRiskStartSpeedMetersPerSecond,
                _config.TrackingRiskFullSpeedMetersPerSecond,
                currentSpeed
            ),
            0.0f,
            1.0f
        );
        return Mathf.Clamp(MathF.Max(lateralRisk, headingRisk) * speedScale, 0.0f, 1.0f);
    }

    private float CalculateTrackBoundaryRisk(LateralControlOutput lateralOutput)
    {
        return Mathf.Clamp(
            Mathf.InverseLerp(
                _config.TrackBoundaryExcessStartMeters,
                _config.TrackBoundaryExcessFullMeters,
                lateralOutput.TrackBoundaryExcess
            ),
            0.0f,
            1.0f
        );
    }

    private float CalculateTrackBoundaryRecoveryAccelerationCap(float trackBoundaryRisk, float currentSpeed)
    {
        float risk = Mathf.Clamp(trackBoundaryRisk, 0.0f, 1.0f);
        float targetSpeed = _config.TrackBoundaryRecoverySpeedMetersPerSecond;
        if (currentSpeed <= targetSpeed)
            return _config.TrackBoundaryRecoveryAccelerationCapMetersPerSecondSquared;

        float brakingDistance = MathF.Max(_config.TrackBoundaryRecoveryDistanceMeters, 1.0f);
        float correctiveAcceleration = (targetSpeed * targetSpeed - currentSpeed * currentSpeed)
            / (2.0f * brakingDistance);
        correctiveAcceleration = MathF.Max(
            correctiveAcceleration,
            -_config.MaximumCorrectiveBrakeMetersPerSecondSquared
        );
        return Mathf.Lerp(
            _config.TrackBoundaryRecoveryAccelerationCapMetersPerSecondSquared,
            correctiveAcceleration,
            risk
        );
    }

    private static float AccelerationToInput(
        CarConfig carConfig,
        float sensorMass,
        float speedMetersPerSecond,
        float acceleration
    )
    {
        float mass = sensorMass > 0.0f ? sensorMass : carConfig.Chassis.DryMass;
        float dragForce = AeroComponent.CalculateAero(carConfig.Aero, speedMetersPerSecond).DragForce;

        if (acceleration >= 0.0f)
        {
            float requiredDriveForce = mass * acceleration + dragForce;
            float maxDriveForce = MathF.Max(1e-3f, carConfig.Power.CalcMaxDriveForceAtSpeed(speedMetersPerSecond));
            return Mathf.Clamp(requiredDriveForce / maxDriveForce, 0.0f, 1.0f);
        }

        float requiredBrakeForce = MathF.Max(0.0f, mass * -acceleration - dragForce);
        float maxBrakeForce = MathF.Max(1e-3f, carConfig.Power.MaxBrakeForce);
        return -Mathf.Clamp(requiredBrakeForce / maxBrakeForce, 0.0f, 1.0f);
    }

    private static void Validate(StabilityAwareSpeedControlConfig config)
    {
        if (config.SpeedLookaheadBaseMeters < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Speed lookahead base cannot be negative.");
        if (config.SpeedLookaheadTimeSeconds < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Speed lookahead time cannot be negative.");
        if (config.SpeedErrorGain < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Speed error gain cannot be negative.");
        if (config.MaximumSpeedErrorAccelerationMetersPerSecondSquared <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Maximum speed-error acceleration must be positive.");
        if (config.FutureBrakingActivationMetersPerSecondSquared < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Future braking activation cannot be negative.");
        if (config.FullThrottleCutRisk < config.StabilityRiskThrottleCutStart)
            throw new ArgumentOutOfRangeException(nameof(config), "Full throttle cut risk must be at least the cut start.");
        if (config.StabilityBrakeFullSpeedMetersPerSecond < config.StabilityBrakeStartSpeedMetersPerSecond)
            throw new ArgumentOutOfRangeException(nameof(config), "Full stability brake speed must be at least the start speed.");
        if (config.TrackingLateralErrorFullMeters < config.TrackingLateralErrorStartMeters)
            throw new ArgumentOutOfRangeException(nameof(config), "Full tracking lateral error risk must be at least the start risk.");
        if (config.TrackingHeadingErrorFullRadians < config.TrackingHeadingErrorStartRadians)
            throw new ArgumentOutOfRangeException(nameof(config), "Full tracking heading risk must be at least the start risk.");
        if (config.TrackingRiskFullSpeedMetersPerSecond < config.TrackingRiskStartSpeedMetersPerSecond)
            throw new ArgumentOutOfRangeException(nameof(config), "Full tracking risk speed must be at least the start speed.");
        if (config.TrackBoundaryExcessFullMeters < config.TrackBoundaryExcessStartMeters)
            throw new ArgumentOutOfRangeException(nameof(config), "Full track-boundary excess must be at least the start excess.");
        if (config.TrackBoundaryRecoveryAccelerationCapMetersPerSecondSquared < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Track-boundary recovery acceleration cap cannot be negative.");
        if (config.TrackBoundaryRecoverySpeedMetersPerSecond < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Track-boundary recovery speed cannot be negative.");
        if (config.TrackBoundaryRecoveryDistanceMeters <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Track-boundary recovery distance must be positive.");
    }
}
