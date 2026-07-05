using System;

namespace StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Online;

public enum DynamicPathPlanHorizonMode
{
    Distance,
    Layers
}

public sealed class DynamicPathOnlineConfig
{
    public float MinimumPlanHorizonMeters { get; init; } = 300.0f;
    public int MinimumPlanHorizonLayers { get; init; } = 20;
    public DynamicPathPlanHorizonMode PlanHorizonMode { get; init; } = DynamicPathPlanHorizonMode.Distance;
    public int StartLayerLookahead { get; init; } = 2;
    public float MaxInitialHeadingOffsetRadians { get; init; } = 0.8f;
    public float MaxPoseOffsetMeters { get; init; } = 16.0f;
    public float CalculationTimeSafetyFactor { get; init; } = 2.0f;
    public int CalculationTimeBufferLength { get; init; } = 5;
    public float MaxConstantPrefixSeconds { get; init; } = 0.5f;
    public float[] PreviousPathCostFactors { get; init; } = [0.0f, 0.5f, 0.8f];

    public void Validate()
    {
        if (!float.IsFinite(MinimumPlanHorizonMeters) || MinimumPlanHorizonMeters <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(MinimumPlanHorizonMeters));
        if (MinimumPlanHorizonLayers <= 0)
            throw new ArgumentOutOfRangeException(nameof(MinimumPlanHorizonLayers));
        if (StartLayerLookahead < 0)
            throw new ArgumentOutOfRangeException(nameof(StartLayerLookahead));
        if (!float.IsFinite(MaxInitialHeadingOffsetRadians) || MaxInitialHeadingOffsetRadians < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(MaxInitialHeadingOffsetRadians));
        if (!float.IsFinite(MaxPoseOffsetMeters) || MaxPoseOffsetMeters <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(MaxPoseOffsetMeters));
        if (!float.IsFinite(CalculationTimeSafetyFactor) || CalculationTimeSafetyFactor < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(CalculationTimeSafetyFactor));
        if (CalculationTimeBufferLength <= 0)
            throw new ArgumentOutOfRangeException(nameof(CalculationTimeBufferLength));
        if (!float.IsFinite(MaxConstantPrefixSeconds) || MaxConstantPrefixSeconds < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(MaxConstantPrefixSeconds));
        if (PreviousPathCostFactors is null)
            throw new ArgumentNullException(nameof(PreviousPathCostFactors));

        for (int i = 0; i < PreviousPathCostFactors.Length; i++)
        {
            float factor = PreviousPathCostFactors[i];
            if (!float.IsFinite(factor) || factor < 0.0f)
                throw new ArgumentOutOfRangeException(nameof(PreviousPathCostFactors));
        }
    }
}
