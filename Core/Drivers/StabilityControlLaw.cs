using System;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Copyable observation memory used by the stability controller. The live
/// driver owns one instance and path prediction advances a value copy.
/// </summary>
internal struct StabilityControlState
{
    public StabilityControlState(
        float perceivedSideslipRadians,
        float perceivedYawRateRadiansPerSecond
    )
    {
        PerceivedSideslipRadians = perceivedSideslipRadians;
        PerceivedYawRateRadiansPerSecond = perceivedYawRateRadiansPerSecond;
    }

    public float PerceivedSideslipRadians { get; private set; }
    public float PerceivedYawRateRadiansPerSecond { get; private set; }

    public void Observe(
        float sideslipRadians,
        float yawRateRadiansPerSecond,
        float effectiveControl,
        float dt
    )
    {
        float observationTime = StabilityControlLaw.Lerp(
            0.25f,
            0.04f,
            effectiveControl
        );
        float response = 1f - MathF.Exp(
            -MathF.Max(0f, dt) / observationTime
        );
        PerceivedSideslipRadians = StabilityControlLaw.Lerp(
            PerceivedSideslipRadians,
            sideslipRadians,
            response
        );
        PerceivedYawRateRadiansPerSecond = StabilityControlLaw.Lerp(
            PerceivedYawRateRadiansPerSecond,
            yawRateRadiansPerSecond,
            response
        );
    }
}

internal readonly record struct StabilityPredictionSeed(
    StabilityControlState ObservationState,
    bool IsRecovering,
    float EffectiveControl,
    float ControlGainScale
);

internal readonly record struct StabilityControlResult(
    float CommandedCurvature,
    float CurvatureCorrection
);

/// <summary>
/// Deterministic stability-control calculations shared by the live driver and
/// spatial path prediction. Random driver-performance events stay outside this
/// type and are sampled only by the live driver.
/// </summary>
internal static class StabilityControlLaw
{
    private const float RecoveryEnterSeverity = 0.15f;
    private const float RecoveryExitSeverity = 0.05f;

    public static float CalculateSeverity(
        float speed,
        float sideslipRadians,
        float yawRateRadiansPerSecond,
        float rearSlideSeverity,
        float desiredCurvature
    )
    {
        float desiredYawRate = speed * desiredCurvature;
        float actualYawError = yawRateRadiansPerSecond - desiredYawRate;
        float sideslipSeverity = Math.Clamp(
            (MathF.Abs(sideslipRadians) - 0.03f) / 0.07f,
            0f,
            1f
        );
        float yawSeverity = Math.Clamp(
            (MathF.Abs(actualYawError) - 0.08f) / 0.6f,
            0f,
            1f
        ) * Math.Clamp(
            MathF.Abs(sideslipRadians) / 0.04f,
            0f,
            1f
        );
        float rearSeverity = Math.Clamp(
            (rearSlideSeverity - 0.15f) / 0.6f,
            0f,
            1f
        );
        return MathF.Max(
            sideslipSeverity,
            MathF.Max(yawSeverity, rearSeverity)
        );
    }

    public static bool IsUnstable(float severity, bool isRecovering) =>
        isRecovering
            ? severity > RecoveryExitSeverity
            : severity > RecoveryEnterSeverity;

    public static float NominalControlGainScale(float effectiveControl) =>
        Lerp(0.72f, 1f, effectiveControl);

    public static StabilityControlResult Apply(
        ref StabilityControlState state,
        float sideslipRadians,
        float yawRateRadiansPerSecond,
        float speed,
        float desiredCurvature,
        float wheelBase,
        float maximumCurvature,
        float effectiveControl,
        float controlGainScale,
        bool isRecovering,
        float dt
    )
    {
        state.Observe(
            sideslipRadians,
            yawRateRadiansPerSecond,
            effectiveControl,
            dt
        );

        if (!isRecovering)
            return new StabilityControlResult(desiredCurvature, 0f);

        float desiredYawRate = speed * desiredCurvature;
        float perceivedYawError =
            state.PerceivedYawRateRadiansPerSecond - desiredYawRate;
        float correction = (
            0.35f * state.PerceivedSideslipRadians /
            MathF.Max(wheelBase, 0.5f) -
            0.25f * perceivedYawError / MathF.Max(speed, 5f)
        ) * controlGainScale;
        float commandedCurvature = Math.Clamp(
            desiredCurvature + correction,
            -MathF.Max(maximumCurvature, 0f),
            MathF.Max(maximumCurvature, 0f)
        );
        return new StabilityControlResult(
            commandedCurvature,
            correction
        );
    }

    internal static float Lerp(float from, float to, float t) =>
        from + (to - from) * Math.Clamp(t, 0f, 1f);
}
