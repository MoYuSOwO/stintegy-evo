using StintegyEVO.Core.Car.Controllers.V1.RacingLines;

namespace StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;

public readonly record struct DynamicPathCostWeights(
    float Raceline,
    float RacelineSaturation,
    float Length,
    float CurvatureAverage,
    float CurvaturePeak,
    float VirtualGoal
)
{
    public static DynamicPathCostWeights OfficialDefaults => new(
        Raceline: 1.0f,
        RacelineSaturation: 1.0f,
        Length: 0.0f,
        CurvatureAverage: 7500.0f,
        CurvaturePeak: 2500.0f,
        VirtualGoal: 10000.0f
    );
}

public sealed class DynamicPathOfflineConfig
{
    public float LateralResolutionMeters { get; init; } = 0.5f;
    public float LongitudinalStraightStepMeters { get; init; } = 30.0f;
    public float LongitudinalCurveStepMeters { get; init; } = 10.0f;
    public float CurveThresholdRadPerMeter { get; init; } = 0.008f;
    public float LateralOffsetPerMeter { get; init; } = 0.25f;
    public float EdgeSampleStepMeters { get; init; } = 2.5f;
    public float SafetyMarginMeters { get; init; } = TrackPlanningBounds.EdgeSafetyMarginMeters;
    public bool VariableHeading { get; init; } = true;
    public DynamicPathCostWeights CostWeights { get; init; } = DynamicPathCostWeights.OfficialDefaults;

    public void Validate()
    {
        ValidatePositive(LateralResolutionMeters, nameof(LateralResolutionMeters));
        ValidatePositive(LongitudinalStraightStepMeters, nameof(LongitudinalStraightStepMeters));
        ValidatePositive(LongitudinalCurveStepMeters, nameof(LongitudinalCurveStepMeters));
        ValidatePositive(EdgeSampleStepMeters, nameof(EdgeSampleStepMeters));

        if (CurveThresholdRadPerMeter < 0.0f)
            throw new System.ArgumentOutOfRangeException(nameof(CurveThresholdRadPerMeter));
        if (LateralOffsetPerMeter <= 0.0f)
            throw new System.ArgumentOutOfRangeException(nameof(LateralOffsetPerMeter));
        if (SafetyMarginMeters < 0.0f)
            throw new System.ArgumentOutOfRangeException(nameof(SafetyMarginMeters));
    }

    private static void ValidatePositive(float value, string name)
    {
        if (!float.IsFinite(value) || value <= 0.0f)
            throw new System.ArgumentOutOfRangeException(name);
    }
}
