namespace TheStint.Core.Racing;

public readonly record struct ReferenceLineDriverTelemetry(
    float FrontAxleS,
    float LateralErrorMeters,
    float HeadingErrorRadians,
    float ReferenceCurvature,
    float PreviewCurvature,
    float DesiredCurvature,
    float CurvatureCorrection,
    float CorrectionDecayDistanceMeters,
    float CorrectionEnvelopeMaximumCurvature,
    float CorrectionSpeedPlanningMilliseconds,
    float GlobalProfileTargetSpeed,
    float TargetSpeed,
    float ReferenceAcceleration,
    float LossCompensationAcceleration,
    float SpeedFeedbackAcceleration,
    float DriveAccelerationLimit,
    float DesiredAcceleration
);
