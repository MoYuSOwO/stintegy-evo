namespace TheStint.Core.Racing;

public readonly record struct CurvatureCorrectionSpeedPlan(
    VehicleSpeedProfilePoint Current,
    float NextTargetSpeed,
    float FirstSegmentLengthMeters,
    float DecayDistanceMeters,
    float MaximumAbsoluteCurvature,
    int SampleCount
);
