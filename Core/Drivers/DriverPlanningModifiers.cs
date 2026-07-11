using System;

namespace TheStint.Core.Drivers;

public readonly record struct DriverPlanningModifiers(
    float PaceEfficiency,
    float EstimatedGripScale
)
{
    public static readonly DriverPlanningModifiers Neutral = new(1f, 1f);

    internal float CombinedConfidence => Math.Clamp(
        PaceEfficiency * EstimatedGripScale,
        0.8f,
        1.05f
    );
}
