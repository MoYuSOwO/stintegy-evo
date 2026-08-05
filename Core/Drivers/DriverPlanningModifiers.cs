using System;

namespace StintegyEVO.Core.Drivers;

public readonly record struct DriverPlanningModifiers(
    float PaceEfficiency,
    float EstimatedGripScale,
    float FrontBrakeBiasOffset
)
{
    public static readonly DriverPlanningModifiers Neutral = new(1f, 1f, 0f);

    internal float CombinedConfidence => Math.Clamp(
        PaceEfficiency * EstimatedGripScale,
        0.8f,
        1.05f
    );
}
