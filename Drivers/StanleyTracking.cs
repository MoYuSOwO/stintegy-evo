using System.Numerics;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers;

/// <summary>
/// Stanley against a baked line: path curvature feedforward plus lateral and
/// heading corrections. Clamping to the car's envelope lives outside this type.
/// </summary>
internal static class StanleyTracking
{
    public const float Gain = 0.90f;
    public const float SofteningSpeed = 5f;
    public const float HeadingGain = 0.85f;
    public const float CurvaturePreviewTimeSeconds = 0.20f;
    public const float MaximumCurvaturePreviewMeters = 12f;

    public readonly record struct Sample(
        float UnconstrainedCurvature,
        float LateralErrorMeters,
        float HeadingErrorRadians,
        float LineCurvature
    );

    public static Sample At(
        RaceCarSnapshot car,
        GeometricRacingLine line,
        DriverTrackView track,
        Vector2 position,
        float velocityHeading,
        float speed
    )
    {
        float wheelBase = MathF.Max(car.WheelBaseMeters, 1.5f);
        Vector2 forward = new(MathF.Cos(velocityHeading), MathF.Sin(velocityHeading));
        Vector2 front = position + forward * (wheelBase * 0.5f);
        TrackPose pose = track.Project(front);
        return AtOffset(car, line, track, pose.S, pose.D, velocityHeading, speed);
    }

    /// <summary>
    /// Stanley against the road centreline. Launch uses this so a standing
    /// start holds the ribbon instead of cutting toward the racing line.
    /// </summary>
    public static Sample AtRoad(
        RaceCarSnapshot car,
        DriverTrackView track,
        float speed
    )
    {
        float wheelBase = MathF.Max(car.WheelBaseMeters, 1.5f);
        float velocityHeading = car.VelocityHeadingRadians;
        Vector2 forward = new(MathF.Cos(velocityHeading), MathF.Sin(velocityHeading));
        Vector2 front = car.Position + forward * (wheelBase * 0.5f);
        TrackPose pose = track.Project(front);
        return AtRoad(car, track, pose.S, pose.D, velocityHeading, speed);
    }

    public static Sample AtRoad(
        RaceCarSnapshot car,
        DriverTrackView track,
        float s,
        float n,
        float velocityHeading,
        float speed
    )
    {
        float wheelBase = MathF.Max(car.WheelBaseMeters, 1.5f);
        TrackSample here = track.Sample(s);
        float previewDistance = MathF.Min(
            MathF.Max(0f, speed) * CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters
        );
        TrackSample preview = track.Sample(track.WrapS(s + previewDistance));
        float headingError = MathHelper.NormalizeAngle(here.Heading - velocityHeading);
        float stanley = MathF.Atan(
            Gain * n / MathF.Max(SofteningSpeed + speed, 0.1f)
        );
        float feedforward = MathF.Atan(wheelBase * preview.Curvature);
        float maxK = MathF.Max(car.Capabilities.MaxCurvatureRequest, 1e-3f);
        float maxSteer = MathF.Atan(wheelBase * maxK);
        float steer = Math.Clamp(
            feedforward + HeadingGain * headingError + stanley,
            -maxSteer,
            maxSteer
        );
        return new Sample(
            MathF.Tan(steer) / wheelBase,
            n,
            headingError,
            preview.Curvature
        );
    }

    public static Sample AtOffset(
        RaceCarSnapshot car,
        GeometricRacingLine line,
        DriverTrackView track,
        float s,
        float n,
        float velocityHeading,
        float speed
    )
    {
        float wheelBase = MathF.Max(car.WheelBaseMeters, 1.5f);
        GeometricRacingLine.Sample target = line.At(track.Sample, track.WrapS, s);
        float previewDistance = MathF.Min(
            MathF.Max(0f, speed) * CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters
        );
        GeometricRacingLine.Sample preview = line.At(
            track.Sample,
            track.WrapS,
            s + previewDistance
        );

        float lateralError = n - target.OffsetMeters;
        float headingError = MathHelper.NormalizeAngle(target.Heading - velocityHeading);
        float stanley = MathF.Atan(
            Gain * lateralError / MathF.Max(SofteningSpeed + speed, 0.1f)
        );
        float feedforward = MathF.Atan(wheelBase * preview.PathCurvature);
        float maxK = MathF.Max(car.Capabilities.MaxCurvatureRequest, 1e-3f);
        float maxSteer = MathF.Atan(wheelBase * maxK);
        float steer = Math.Clamp(
            feedforward + HeadingGain * headingError + stanley,
            -maxSteer,
            maxSteer
        );
        return new Sample(
            MathF.Tan(steer) / wheelBase,
            lateralError,
            headingError,
            preview.PathCurvature
        );
    }

    public static float ClampToEnvelope(
        float curvature,
        float speed,
        float lateralAccelerationLimit,
        float maxCurvatureRequest,
        float usage = 1f
    )
    {
        float used = Math.Clamp(usage, 0.05f, 1f);
        float maxKGrip = used * lateralAccelerationLimit / MathF.Max(speed * speed, 1f);
        float maxK = MathF.Min(MathF.Max(maxCurvatureRequest, 1e-3f), MathF.Max(maxKGrip, 1e-4f));
        return Math.Clamp(curvature, -maxK, maxK);
    }
}
