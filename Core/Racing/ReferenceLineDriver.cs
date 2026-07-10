using System;
using TheStint.Core.Cars;
using TheStint.Core.Util;

namespace TheStint.Core.Racing;

/// <summary>
/// A deliberately small baseline driver for the strategy-first vehicle model.
/// It follows the generated reference line with curvature feedback and plans
/// braking from curvature samples ahead. It does not model steering angle,
/// countersteer, or spin recovery.
/// </summary>
public sealed class ReferenceLineDriver : IRaceDriver
{
    public float MaximumSpeedMetersPerSecond { get; init; } = 42f;
    public float MinimumCornerSpeedMetersPerSecond { get; init; } = 9f;
    public float BaseLateralAccelerationBudget { get; init; } = 9f;
    public float SpeedGain { get; init; } = 2.5f;
    public float MaximumAcceleration { get; init; } = 4f;
    public float MaximumBraking { get; init; } = 9f;
    public float HeadingCorrectionLengthMeters { get; init; } = 14f;
    public float LateralCorrectionLengthMeters { get; init; } = 14f;
    public float SpeedLookaheadMeters { get; init; } = 120f;
    public float SpeedLookaheadStepMeters { get; init; } = 5f;

    public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
    {
        var state = context.Car.State;
        var sample = context.Pose.Sample;

        float headingLength = MathF.Max(HeadingCorrectionLengthMeters, 1f);
        float lateralLength = MathF.Max(LateralCorrectionLengthMeters, 1f);
        float headingError = MathHelper.NormalizeAngle(sample.RefHeading - state.Heading);
        float lateralError = context.Pose.D - sample.RefOffset;
        float desiredCurvature =
            sample.RefCurvature +
            headingError / headingLength +
            lateralError / (lateralLength * lateralLength);

        float lateralBudget = BaseLateralAccelerationBudget * TireModePaceFactor(context.Car.Strategy.TireMode);
        float currentTargetSpeed = SpeedForCurvature(sample.RefCurvature, lateralBudget);
        float desiredAcceleration = Math.Clamp(
            (currentTargetSpeed - state.Speed) * SpeedGain,
            -MaximumBraking,
            MaximumAcceleration
        );

        float lookahead = MathF.Max(0f, SpeedLookaheadMeters);
        float step = MathF.Max(1f, SpeedLookaheadStepMeters);
        for (float distance = step; distance <= lookahead + 1e-4f; distance += step)
        {
            var future = context.Track.Sample(context.Pose.S + distance);
            float futureSpeed = SpeedForCurvature(future.RefCurvature, lateralBudget);
            float requiredAcceleration =
                (futureSpeed * futureSpeed - state.Speed * state.Speed) /
                (2f * distance);
            desiredAcceleration = MathF.Min(desiredAcceleration, requiredAcceleration);
        }

        desiredAcceleration = Math.Clamp(
            desiredAcceleration,
            -MaximumBraking,
            MaximumAcceleration
        );
        return new DriverInput(desiredCurvature, desiredAcceleration);
    }

    private float SpeedForCurvature(float curvature, float lateralBudget)
    {
        float absoluteCurvature = MathF.Abs(curvature);
        if (absoluteCurvature < 1e-4f)
            return MaximumSpeedMetersPerSecond;

        float speed = MathF.Sqrt(MathF.Max(lateralBudget, 0.1f) / absoluteCurvature);
        return Math.Clamp(speed, MinimumCornerSpeedMetersPerSecond, MaximumSpeedMetersPerSecond);
    }

    private static float TireModePaceFactor(TireUsageMode mode)
    {
        return mode switch
        {
            TireUsageMode.Protect => 0.82f,
            TireUsageMode.Light => 0.91f,
            TireUsageMode.Push => 1.06f,
            TireUsageMode.Attack => 1.10f,
            _ => 1f
        };
    }
}
