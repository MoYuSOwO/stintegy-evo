namespace TheStint.Core.Drivers;

public readonly record struct DynamicPathSpeedPlan(
    VehicleSpeedPlanPoint Current,
    float NextTargetSpeed,
    float FirstSegmentLengthMeters,
    float PathLengthMeters,
    float MaximumAbsoluteCurvature,
    int SampleCount
);
