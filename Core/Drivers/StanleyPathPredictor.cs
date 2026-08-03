using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Rolls the current Stanley control law forward in space. Commanded curvature
/// remains available for speed planning, while motion curvature is clipped by
/// the vehicle's estimated lateral capability and drives the predicted pose.
/// </summary>
public sealed class StanleyPathPredictor
{
    public VehiclePathPrediction Predict(
        VehiclePathPrediction destination,
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead speedEstimate,
        float lateralTargetOffsetMeters,
        float stanleyGain,
        float stanleySofteningSpeed,
        float headingGain,
        float curvaturePreviewTimeSeconds,
        float maximumCurvaturePreviewMeters,
        float horizonMeters,
        float stepMeters,
        float minimumDynamicMeters,
        float convergenceHoldMeters,
        float convergenceLateralErrorMeters,
        float convergenceHeadingErrorRadians,
        float convergenceCurvatureError,
        float gripUsage,
        float initialCommandedCurvature
    )
    {
        ArgumentNullException.ThrowIfNull(car);
        StabilityPredictionSeed stabilitySeed = new(
            new StabilityControlState(
                car.State.SideslipAngleRadians,
                car.State.YawRateRadiansPerSecond
            ),
            IsRecovering: false,
            EffectiveControl: 1f,
            ControlGainScale: 1f
        );
        return Predict(
            destination,
            car,
            track,
            speedEstimate,
            lateralTargetOffsetMeters,
            stanleyGain,
            stanleySofteningSpeed,
            headingGain,
            curvaturePreviewTimeSeconds,
            maximumCurvaturePreviewMeters,
            horizonMeters,
            stepMeters,
            minimumDynamicMeters,
            convergenceHoldMeters,
            convergenceLateralErrorMeters,
            convergenceHeadingErrorRadians,
            convergenceCurvatureError,
            gripUsage,
            initialCommandedCurvature,
            stabilitySeed
        );
    }

    internal VehiclePathPrediction Predict(
        VehiclePathPrediction destination,
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead speedEstimate,
        float tacticalOffsetMeters,
        TrackConstrainedLateralOffset lateralOffsetProfile,
        float executionOffsetMeters,
        float stanleyGain,
        float stanleySofteningSpeed,
        float headingGain,
        float curvaturePreviewTimeSeconds,
        float maximumCurvaturePreviewMeters,
        float horizonMeters,
        float stepMeters,
        float minimumDynamicMeters,
        float convergenceHoldMeters,
        float convergenceLateralErrorMeters,
        float convergenceHeadingErrorRadians,
        float convergenceCurvatureError,
        float gripUsage,
        float initialCommandedCurvature
    )
    {
        ArgumentNullException.ThrowIfNull(car);
        ArgumentNullException.ThrowIfNull(lateralOffsetProfile);
        StabilityPredictionSeed stabilitySeed = new(
            new StabilityControlState(
                car.State.SideslipAngleRadians,
                car.State.YawRateRadiansPerSecond
            ),
            IsRecovering: false,
            EffectiveControl: 1f,
            ControlGainScale: 1f
        );
        return Predict(
            destination,
            car,
            track,
            speedEstimate,
            tacticalOffsetMeters,
            lateralOffsetProfile,
            executionOffsetMeters,
            stanleyGain,
            stanleySofteningSpeed,
            headingGain,
            curvaturePreviewTimeSeconds,
            maximumCurvaturePreviewMeters,
            horizonMeters,
            stepMeters,
            minimumDynamicMeters,
            convergenceHoldMeters,
            convergenceLateralErrorMeters,
            convergenceHeadingErrorRadians,
            convergenceCurvatureError,
            gripUsage,
            initialCommandedCurvature,
            stabilitySeed
        );
    }

    internal VehiclePathPrediction Predict(
        VehiclePathPrediction destination,
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead speedEstimate,
        float lateralTargetOffsetMeters,
        float stanleyGain,
        float stanleySofteningSpeed,
        float headingGain,
        float curvaturePreviewTimeSeconds,
        float maximumCurvaturePreviewMeters,
        float horizonMeters,
        float stepMeters,
        float minimumDynamicMeters,
        float convergenceHoldMeters,
        float convergenceLateralErrorMeters,
        float convergenceHeadingErrorRadians,
        float convergenceCurvatureError,
        float gripUsage,
        float initialCommandedCurvature,
        StabilityPredictionSeed stabilitySeed
    )
    {
        return Predict(
            destination,
            car,
            track,
            speedEstimate,
            tacticalOffsetMeters: 0f,
            lateralOffsetProfile: null,
            executionOffsetMeters: lateralTargetOffsetMeters,
            stanleyGain,
            stanleySofteningSpeed,
            headingGain,
            curvaturePreviewTimeSeconds,
            maximumCurvaturePreviewMeters,
            horizonMeters,
            stepMeters,
            minimumDynamicMeters,
            convergenceHoldMeters,
            convergenceLateralErrorMeters,
            convergenceHeadingErrorRadians,
            convergenceCurvatureError,
            gripUsage,
            initialCommandedCurvature,
            stabilitySeed
        );
    }

    internal VehiclePathPrediction Predict(
        VehiclePathPrediction destination,
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead speedEstimate,
        float tacticalOffsetMeters,
        TrackConstrainedLateralOffset? lateralOffsetProfile,
        float executionOffsetMeters,
        float stanleyGain,
        float stanleySofteningSpeed,
        float headingGain,
        float curvaturePreviewTimeSeconds,
        float maximumCurvaturePreviewMeters,
        float horizonMeters,
        float stepMeters,
        float minimumDynamicMeters,
        float convergenceHoldMeters,
        float convergenceLateralErrorMeters,
        float convergenceHeadingErrorRadians,
        float convergenceCurvatureError,
        float gripUsage,
        float initialCommandedCurvature,
        StabilityPredictionSeed stabilitySeed
    )
    {
        ArgumentNullException.ThrowIfNull(destination);
        ArgumentNullException.ThrowIfNull(car);
        ArgumentNullException.ThrowIfNull(track);
        ArgumentNullException.ThrowIfNull(speedEstimate);

        float horizon = MathF.Max(horizonMeters, 0f);
        float step = MathF.Max(stepMeters, 0.25f);
        int count = Math.Max(2, (int)MathF.Ceiling(horizon / step) + 1);
        destination.Reset(count);

        CarState state = car.State;
        Vector2 position = state.Position;
        float velocityHeading = state.VelocityHeading;
        float wheelBase = MathF.Max(car.CarConfig.WheelBaseMeters, 0.5f);
        float maximumCurvature = MathF.Max(
            car.CarConfig.MaxCurvatureRequest,
            0f
        );
        float predictedSideslip = state.SideslipAngleRadians;
        float predictedYawRate = state.YawRateRadiansPerSecond;
        float predictedRearSlide = MathF.Max(
            state.Telemetry.RearSlideSeverity,
            0f
        );
        StabilityControlState predictedStabilityState =
            stabilitySeed.ObservationState;
        bool predictedRecovery = stabilitySeed.IsRecovering;
        float effectiveControl = Math.Clamp(
            stabilitySeed.EffectiveControl,
            0f,
            1f
        );
        float controlGainScale = stabilitySeed.ControlGainScale;
        float previousSegmentTime = 0f;
        float convergedDistance = 0f;
        bool followsReferenceLine = false;
        float referenceLineCenterS = 0f;

        for (int i = 0; i < count; i++)
        {
            float distance = MathF.Min(i * step, horizon);
            float estimatedSpeed = EstimateSpeed(
                car,
                speedEstimate,
                distance
            );
            if (followsReferenceLine)
            {
                float referenceDistance =
                    distance - destination.ReferenceLineJoinDistanceMeters;
                TrackSample referenceSample = track.Sample(
                    referenceLineCenterS + referenceDistance
                );
                TrackLateralTargetSample target = SampleTargetGeometry(
                    track,
                    referenceSample.S,
                    lateralOffsetProfile,
                    tacticalOffsetMeters,
                    executionOffsetMeters,
                    car.Collision.HalfWidthMeters
                );
                float targetCurvature = SamplePeakTargetCurvature(
                    track,
                    referenceSample.S,
                    step * 0.5f,
                    lateralOffsetProfile,
                    tacticalOffsetMeters,
                    executionOffsetMeters,
                    car.Collision.HalfWidthMeters
                );
                position = target.Position;
                velocityHeading = target.Heading;
                destination.Add(new VehiclePathPredictionPoint(
                    distance,
                    position,
                    velocityHeading,
                    referenceSample.S,
                    0f,
                    targetCurvature,
                    targetCurvature,
                    0f,
                    targetCurvature,
                    estimatedSpeed
                ));
                continue;
            }

            StanleyControlSample control = lateralOffsetProfile is null
                ? StanleyControlLaw.Sample(
                    track,
                    position,
                    velocityHeading,
                    estimatedSpeed,
                    wheelBase,
                    executionOffsetMeters,
                    stanleyGain,
                    stanleySofteningSpeed,
                    headingGain,
                    curvaturePreviewTimeSeconds,
                    maximumCurvaturePreviewMeters,
                    maximumCurvature
                )
                : StanleyControlLaw.Sample(
                    track,
                    position,
                    velocityHeading,
                    estimatedSpeed,
                    wheelBase,
                    car.Collision.HalfWidthMeters,
                    lateralOffsetProfile,
                    tacticalOffsetMeters,
                    executionOffsetMeters,
                    stanleyGain,
                    stanleySofteningSpeed,
                    headingGain,
                    curvaturePreviewTimeSeconds,
                    maximumCurvaturePreviewMeters,
                    maximumCurvature
                );
            float commandedCurvature;
            float stabilityCurvatureCorrection;
            if (i == 0)
            {
                commandedCurvature = Math.Clamp(
                    initialCommandedCurvature,
                    -maximumCurvature,
                    maximumCurvature
                );
                stabilityCurvatureCorrection =
                    commandedCurvature - control.DesiredCurvature;
            }
            else
            {
                float severity = StabilityControlLaw.CalculateSeverity(
                    estimatedSpeed,
                    predictedSideslip,
                    predictedYawRate,
                    predictedRearSlide,
                    control.DesiredCurvature
                );
                bool shouldRecover = StabilityControlLaw.IsUnstable(
                    severity,
                    predictedRecovery
                );
                if (shouldRecover && !predictedRecovery)
                {
                    predictedRecovery = true;
                    controlGainScale =
                        StabilityControlLaw.NominalControlGainScale(
                            effectiveControl
                        );
                }
                else if (!shouldRecover && predictedRecovery)
                {
                    predictedRecovery = false;
                    controlGainScale = 1f;
                }

                StabilityControlResult stabilityResult =
                    StabilityControlLaw.Apply(
                        ref predictedStabilityState,
                        predictedSideslip,
                        predictedYawRate,
                        estimatedSpeed,
                        control.DesiredCurvature,
                        wheelBase,
                        maximumCurvature,
                        effectiveControl,
                        controlGainScale,
                        predictedRecovery,
                        previousSegmentTime
                    );
                commandedCurvature = stabilityResult.CommandedCurvature;
                stabilityCurvatureCorrection =
                    commandedCurvature - control.DesiredCurvature;
            }
            float motionCurvature = EstimateMotionCurvature(
                car,
                estimatedSpeed,
                commandedCurvature,
                gripUsage
            );

            destination.Add(new VehiclePathPredictionPoint(
                distance,
                position,
                velocityHeading,
                control.FrontPose.S,
                control.LateralErrorMeters,
                control.TargetPreviewCurvature,
                commandedCurvature,
                stabilityCurvatureCorrection,
                motionCurvature,
                estimatedSpeed
            ));

            if (i == count - 1)
                break;

            float segmentLength = MathF.Min(step, horizon - distance);
            bool converged =
                distance >= MathF.Max(minimumDynamicMeters, 0f) &&
                !predictedRecovery &&
                MathF.Abs(control.LateralErrorMeters) <=
                MathF.Max(convergenceLateralErrorMeters, 0f) &&
                MathF.Abs(control.HeadingErrorRadians) <=
                MathF.Max(convergenceHeadingErrorRadians, 0f) &&
                MathF.Abs(
                    commandedCurvature -
                    control.TargetPreviewCurvature
                ) <= MathF.Max(convergenceCurvatureError, 0f);
            convergedDistance = converged
                ? convergedDistance + segmentLength
                : 0f;
            if (convergedDistance >= MathF.Max(convergenceHoldMeters, 0f))
            {
                TrackPose centerPose = track.Project(position);
                float nextReferenceCurvature = SamplePeakTargetCurvature(
                    track,
                    centerPose.S + segmentLength,
                    segmentLength * 0.5f,
                    lateralOffsetProfile,
                    tacticalOffsetMeters,
                    executionOffsetMeters,
                    car.Collision.HalfWidthMeters
                );
                destination.MarkReferenceLineJoin(
                    distance,
                    nextReferenceCurvature - commandedCurvature
                );
                referenceLineCenterS = centerPose.S;
                followsReferenceLine = true;
                continue;
            }

            float segmentTime = segmentLength /
                                MathF.Max(estimatedSpeed, 5f);
            IntegrateArc(
                ref position,
                ref velocityHeading,
                motionCurvature,
                segmentLength
            );
            AdvanceLateralState(
                ref predictedSideslip,
                ref predictedYawRate,
                ref predictedRearSlide,
                car.CarConfig,
                estimatedSpeed,
                motionCurvature,
                segmentTime
            );
            previousSegmentTime = segmentTime;
        }

        return destination;
    }

    /// <summary>
    /// Advances only the small beta/yaw state needed by stability control. It
    /// mirrors the response targets and limits of CarPhysics without stepping
    /// tires, energy, load transfer, or the rest of the vehicle simulation.
    /// </summary>
    private static void AdvanceLateralState(
        ref float sideslip,
        ref float yawRate,
        ref float rearSlideSeverity,
        CarConfig config,
        float speed,
        float motionCurvature,
        float dt
    )
    {
        if (dt <= 0f)
            return;

        float trajectoryYawRate = speed * motionCurvature;
        float yawResponseTime = MathF.Max(
            config.YawResponseTimeSeconds,
            0.05f
        );
        float sideslipRecoveryTime = MathF.Max(
            config.SideslipRecoveryTimeSeconds,
            0.05f
        );
        float stabilizedYawRate = trajectoryYawRate +
                                  sideslip / sideslipRecoveryTime;
        float yawAcceleration = Math.Clamp(
            (stabilizedYawRate - yawRate) / yawResponseTime,
            -ReducedOrderDynamicsLimits.MaximumYawAccelerationRadiansPerSecondSquared,
            ReducedOrderDynamicsLimits.MaximumYawAccelerationRadiansPerSecondSquared
        );
        float nextYawRate = Math.Clamp(
            yawRate + yawAcceleration * dt,
            -ReducedOrderDynamicsLimits.MaximumYawRateRadiansPerSecond,
            ReducedOrderDynamicsLimits.MaximumYawRateRadiansPerSecond
        );
        float averageYawRate = (yawRate + nextYawRate) * 0.5f;
        sideslip = Math.Clamp(
            sideslip + (trajectoryYawRate - averageYawRate) * dt,
            -ReducedOrderDynamicsLimits.MaximumBodySideslipRadians,
            ReducedOrderDynamicsLimits.MaximumBodySideslipRadians
        );
        yawRate = nextYawRate;
        rearSlideSeverity *= MathF.Exp(-dt / sideslipRecoveryTime);
    }

    private static float EstimateSpeed(
        RaceCar car,
        VehicleSpeedLookahead speedEstimate,
        float distance
    )
    {
        float currentSpeed = MathF.Max(0f, car.State.Speed);
        float upperReachable = MathF.Sqrt(MathF.Max(
            0f,
            currentSpeed * currentSpeed +
            2f * MathF.Max(0f, car.CarConfig.MaxDriveAcceleration) * distance
        ));
        float lowerReachable = MathF.Sqrt(MathF.Max(
            0f,
            currentSpeed * currentSpeed -
            2f * MathF.Max(0f, car.CarConfig.MaxBrakeAccel) * distance
        ));
        float target = speedEstimate.Sample(distance).TargetSpeed;
        return Math.Clamp(target, lowerReachable, upperReachable);
    }

    private static float SamplePeakTargetCurvature(
        TrackData track,
        float s,
        float radius,
        TrackConstrainedLateralOffset? lateralOffsetProfile,
        float tacticalOffsetMeters,
        float executionOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        float best = SampleTargetGeometry(
            track,
            s,
            lateralOffsetProfile,
            tacticalOffsetMeters,
            executionOffsetMeters,
            vehicleHalfWidthMeters
        ).Curvature;
        float before = SampleTargetGeometry(
            track,
            s - radius,
            lateralOffsetProfile,
            tacticalOffsetMeters,
            executionOffsetMeters,
            vehicleHalfWidthMeters
        ).Curvature;
        float after = SampleTargetGeometry(
            track,
            s + radius,
            lateralOffsetProfile,
            tacticalOffsetMeters,
            executionOffsetMeters,
            vehicleHalfWidthMeters
        ).Curvature;
        if (MathF.Abs(before) > MathF.Abs(best))
            best = before;
        if (MathF.Abs(after) > MathF.Abs(best))
            best = after;
        return best;
    }

    private static TrackLateralTargetSample SampleTargetGeometry(
        TrackData track,
        float s,
        TrackConstrainedLateralOffset? lateralOffsetProfile,
        float tacticalOffsetMeters,
        float executionOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        if (lateralOffsetProfile is not null)
        {
            return lateralOffsetProfile.SampleGeometry(
                track,
                s,
                tacticalOffsetMeters,
                executionOffsetMeters,
                vehicleHalfWidthMeters
            );
        }

        TrackSample sample = track.Sample(s);
        return new TrackLateralTargetSample(
            sample.RefPosition + sample.Normal * executionOffsetMeters,
            sample.RefHeading,
            sample.RefCurvature,
            executionOffsetMeters
        );
    }

    private static float EstimateMotionCurvature(
        RaceCar car,
        float speed,
        float commandedCurvature,
        float gripUsage
    )
    {
        if (speed <= 0.5f || MathF.Abs(commandedCurvature) <= 1e-6f)
            return commandedCurvature;

        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            car.State,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            speed,
            commandedCurvature,
            gripUsage
        );
        float achievableMagnitude = limits.LateralAccelerationLimit /
                                    MathF.Max(speed * speed, 1e-4f);
        return Math.Clamp(
            commandedCurvature,
            -achievableMagnitude,
            achievableMagnitude
        );
    }

    private static void IntegrateArc(
        ref Vector2 position,
        ref float heading,
        float curvature,
        float distance
    )
    {
        if (distance <= 0f)
            return;

        float headingDelta = curvature * distance;
        float midpointHeading = heading + headingDelta * 0.5f;
        float chordLength = MathF.Abs(curvature) <= 1e-6f
            ? distance
            : 2f * MathF.Sin(headingDelta * 0.5f) / curvature;
        position += new Vector2(
            MathF.Cos(midpointHeading),
            MathF.Sin(midpointHeading)
        ) * chordLength;
        heading = MathHelper.NormalizeAngle(heading + headingDelta);
    }
}
