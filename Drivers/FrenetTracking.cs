using System.Numerics;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers;

/// <summary>
/// Path follow with curvature feedforward, heading P, and a damped lateral
/// term. Stanley has no ė, so it overshoots a line that already sits on the
/// kerb. ė is the kinematic closing rate, not a filtered derivative.
/// </summary>
internal static class FrenetTracking
{
    public const float LateralGain = 0.90f;
    public const float LateralDamping = 0.12f;
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

    public static Sample AtRoad(RaceCarSnapshot car, DriverTrackView track, float speed)
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
        float previewDistance = PreviewDistance(speed);
        TrackSample preview = track.Sample(track.WrapS(s + previewDistance));
        float headingError = MathHelper.NormalizeAngle(here.Heading - velocityHeading);
        return Finish(
            car,
            wheelBase,
            speed,
            n,
            headingError,
            preview.Curvature,
            LateralGain,
            HeadingGain,
            LateralDamping
        );
    }

    public static Sample AtOffset(
        RaceCarSnapshot car,
        GeometricRacingLine line,
        DriverTrackView track,
        float s,
        float n,
        float velocityHeading,
        float speed,
        float lateralBiasMeters = 0f,
        float lateralGain = LateralGain,
        float headingGain = HeadingGain,
        float lateralDamping = LateralDamping,
        float previewSeconds = CurvaturePreviewTimeSeconds,
        float maximumPreviewMeters = MaximumCurvaturePreviewMeters
    )
    {
        float wheelBase = MathF.Max(car.WheelBaseMeters, 1.5f);
        GeometricRacingLine.Sample target = line.At(track.Sample, track.WrapS, s);
        float previewDistance = Math.Clamp(
            MathF.Max(0f, speed) * previewSeconds,
            0f,
            maximumPreviewMeters
        );
        GeometricRacingLine.Sample preview = line.At(
            track.Sample,
            track.WrapS,
            s + previewDistance
        );
        float lateralError = n - (target.OffsetMeters + lateralBiasMeters);
        float headingError = MathHelper.NormalizeAngle(target.Heading - velocityHeading);
        return Finish(
            car,
            wheelBase,
            speed,
            lateralError,
            headingError,
            preview.PathCurvature,
            lateralGain,
            headingGain,
            lateralDamping
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

    private static Sample Finish(
        RaceCarSnapshot car,
        float wheelBase,
        float speed,
        float lateralError,
        float headingError,
        float pathCurvature,
        float lateralGain,
        float headingGain,
        float lateralDamping
    )
    {
        float v = MathF.Max(speed, 0.1f);
        float closing = -v * MathF.Sin(headingError);
        float heading = headingGain * headingError;
        float lateral = MathF.Atan(
            lateralGain * lateralError / MathF.Max(SofteningSpeed + v, 0.1f)
        );
        float damp = lateralDamping * closing / MathF.Max(SofteningSpeed + v, 0.1f);
        float feedforward = MathF.Atan(wheelBase * pathCurvature);
        float maxK = MathF.Max(car.Capabilities.MaxCurvatureRequest, 1e-3f);
        float maxSteer = MathF.Atan(wheelBase * maxK);
        float steer = Math.Clamp(
            feedforward + heading + lateral + damp,
            -maxSteer,
            maxSteer
        );
        return new Sample(
            MathF.Tan(steer) / wheelBase,
            lateralError,
            headingError,
            pathCurvature
        );
    }

    private static float PreviewDistance(float speed) =>
        MathF.Min(
            MathF.Max(0f, speed) * CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters
        );
}
