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
        Vector2 velocityForward = new(
            MathF.Cos(velocityHeading),
            MathF.Sin(velocityHeading)
        );
        Vector2 frontAxlePosition = centerPosition +
                                    velocityForward * (wheelBase * 0.5f);
        TrackPose frontPose = track.Project(frontAxlePosition);
        TrackSample frontSample = frontPose.Sample;
        float previewDistance = MathF.Min(
            MathF.Max(0f, speed) * curvaturePreviewTimeSeconds,
            maximumCurvaturePreviewMeters
        );
        TrackSample previewSample = track.Sample(frontPose.S + previewDistance);
        float lateralError = frontPose.D -
                             (frontSample.RefOffset + lateralTargetOffsetMeters);
        float headingError = MathHelper.NormalizeAngle(
            frontSample.RefHeading - velocityHeading
        );
        float stanleyCorrection = MathF.Atan(
            stanleyGain * lateralError /
            MathF.Max(stanleySofteningSpeed + speed, 0.1f)
        );
        float feedforwardSteer = MathF.Atan(
            wheelBase * previewSample.RefCurvature
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
            desiredCurvature
        );
    }
}
