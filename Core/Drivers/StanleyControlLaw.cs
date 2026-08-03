using System;
using System.Numerics;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Drivers;

internal readonly record struct StanleyControlSample(
    TrackPose FrontPose,
    TrackSample FrontSample,
    TrackSample PreviewSample,
    float LateralErrorMeters,
    float HeadingErrorRadians,
    float TargetPreviewCurvature,
    float DesiredCurvature
);

internal static class StanleyControlLaw
{
    public static StanleyControlSample Sample(
        TrackData track,
        Vector2 centerPosition,
        float velocityHeading,
        float speed,
        float wheelBase,
        float lateralTargetOffsetMeters,
        float stanleyGain,
        float stanleySofteningSpeed,
        float headingGain,
        float curvaturePreviewTimeSeconds,
        float maximumCurvaturePreviewMeters,
        float maximumCurvatureRequest
    )
    {
        return SampleCore(
            track,
            centerPosition,
            velocityHeading,
            speed,
            wheelBase,
            vehicleHalfWidthMeters: 0f,
            lateralOffsetProfile: null,
            tacticalOffsetMeters: 0f,
            executionOffsetMeters: lateralTargetOffsetMeters,
            stanleyGain,
            stanleySofteningSpeed,
            headingGain,
            curvaturePreviewTimeSeconds,
            maximumCurvaturePreviewMeters,
            maximumCurvatureRequest
        );
    }

    public static StanleyControlSample Sample(
        TrackData track,
        Vector2 centerPosition,
        float velocityHeading,
        float speed,
        float wheelBase,
        float vehicleHalfWidthMeters,
        TrackConstrainedLateralOffset lateralOffsetProfile,
        float tacticalOffsetMeters,
        float executionOffsetMeters,
        float stanleyGain,
        float stanleySofteningSpeed,
        float headingGain,
        float curvaturePreviewTimeSeconds,
        float maximumCurvaturePreviewMeters,
        float maximumCurvatureRequest
    )
    {
        ArgumentNullException.ThrowIfNull(lateralOffsetProfile);
        return SampleCore(
            track,
            centerPosition,
            velocityHeading,
            speed,
            wheelBase,
            vehicleHalfWidthMeters,
            lateralOffsetProfile,
            tacticalOffsetMeters,
            executionOffsetMeters,
            stanleyGain,
            stanleySofteningSpeed,
            headingGain,
            curvaturePreviewTimeSeconds,
            maximumCurvaturePreviewMeters,
            maximumCurvatureRequest
        );
    }

    private static StanleyControlSample SampleCore(
        TrackData track,
        Vector2 centerPosition,
        float velocityHeading,
        float speed,
        float wheelBase,
        float vehicleHalfWidthMeters,
        TrackConstrainedLateralOffset? lateralOffsetProfile,
        float tacticalOffsetMeters,
        float executionOffsetMeters,
        float stanleyGain,
        float stanleySofteningSpeed,
        float headingGain,
        float curvaturePreviewTimeSeconds,
        float maximumCurvaturePreviewMeters,
        float maximumCurvatureRequest
    )
    {
        Vector2 velocityForward = new(
            MathF.Cos(velocityHeading),
            MathF.Sin(velocityHeading)
        );
        Vector2 frontAxlePosition = centerPosition +
                                    velocityForward * (wheelBase * 0.5f);
        TrackPose frontPose = track.Project(frontAxlePosition);
        TrackSample frontSample = frontPose.Sample;
        TrackLateralTargetSample frontTarget = SampleTarget(
            track,
            frontPose.S,
            lateralOffsetProfile,
            tacticalOffsetMeters,
            executionOffsetMeters,
            vehicleHalfWidthMeters
        );
        float previewDistance = MathF.Min(
            MathF.Max(0f, speed) * curvaturePreviewTimeSeconds,
            maximumCurvaturePreviewMeters
        );
        TrackSample previewSample = track.Sample(frontPose.S + previewDistance);
        TrackLateralTargetSample previewTarget = SampleTarget(
            track,
            previewSample.S,
            lateralOffsetProfile,
            tacticalOffsetMeters,
            executionOffsetMeters,
            vehicleHalfWidthMeters
        );
        float lateralError = frontPose.D -
                             (frontSample.RefOffset + frontTarget.OffsetMeters);
        float headingError = MathHelper.NormalizeAngle(
            frontTarget.Heading - velocityHeading
        );
        float stanleyCorrection = MathF.Atan(
            stanleyGain * lateralError /
            MathF.Max(stanleySofteningSpeed + speed, 0.1f)
        );
        float feedforwardSteer = MathF.Atan(
            wheelBase * previewTarget.Curvature
        );
        float curvatureLimit = MathF.Max(maximumCurvatureRequest, 0f);
        float maximumSteeringAngle = MathF.Atan(wheelBase * curvatureLimit);
        float steeringAngle = Math.Clamp(
            feedforwardSteer + headingGain * headingError + stanleyCorrection,
            -maximumSteeringAngle,
            maximumSteeringAngle
        );
        float desiredCurvature = Math.Clamp(
            MathF.Tan(steeringAngle) / wheelBase,
            -curvatureLimit,
            curvatureLimit
        );

        return new StanleyControlSample(
            frontPose,
            frontSample,
            previewSample,
            lateralError,
            headingError,
            previewTarget.Curvature,
            desiredCurvature
        );
    }

    private static TrackLateralTargetSample SampleTarget(
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
}
