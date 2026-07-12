using System;
using System.Numerics;

namespace StintegyEVO.Core.Drivers;

public readonly record struct VehiclePathPredictionPoint(
    float DistanceMeters,
    Vector2 Position,
    float VelocityHeading,
    float ReferenceS,
    float LateralErrorMeters,
    float ReferenceCurvature,
    float CommandedCurvature,
    float StabilityCurvatureCorrection,
    float MotionCurvature,
    float EstimatedSpeed
);

/// <summary>
/// Spatial prediction of the path that the vehicle controller is expected to
/// produce. The object is reused by its predictor and remains valid only until
/// the owning driver performs its next prediction.
/// </summary>
public sealed class VehiclePathPrediction
{
    private VehiclePathPredictionPoint[] _points = [];

    public int Count { get; private set; }
    public float LengthMeters { get; private set; }
    public float MaximumAbsoluteCommandedCurvature { get; private set; }
    public float MaximumAbsoluteCurvatureCorrection { get; private set; }
    public bool JoinsReferenceLine { get; private set; }
    public float DynamicPredictionLengthMeters =>
        JoinsReferenceLine ? ReferenceLineJoinDistanceMeters : LengthMeters;
    public float ReferenceLineJoinDistanceMeters { get; private set; }
    public float ReferenceLineJoinCurvatureDelta { get; private set; }
    public float TerminalLateralErrorMeters =>
        Count == 0 ? 0f : _points[Count - 1].LateralErrorMeters;

    public VehiclePathPredictionPoint this[int index]
    {
        get
        {
            if ((uint)index >= (uint)Count)
                throw new ArgumentOutOfRangeException(nameof(index));
            return _points[index];
        }
    }

    internal void Reset(int requiredCapacity)
    {
        if (_points.Length < requiredCapacity)
            _points = new VehiclePathPredictionPoint[requiredCapacity];

        Count = 0;
        LengthMeters = 0f;
        MaximumAbsoluteCommandedCurvature = 0f;
        MaximumAbsoluteCurvatureCorrection = 0f;
        JoinsReferenceLine = false;
        ReferenceLineJoinDistanceMeters = 0f;
        ReferenceLineJoinCurvatureDelta = 0f;
    }

    internal void Add(in VehiclePathPredictionPoint point)
    {
        _points[Count++] = point;
        LengthMeters = point.DistanceMeters;
        MaximumAbsoluteCommandedCurvature = MathF.Max(
            MaximumAbsoluteCommandedCurvature,
            MathF.Abs(point.CommandedCurvature)
        );
        MaximumAbsoluteCurvatureCorrection = MathF.Max(
            MaximumAbsoluteCurvatureCorrection,
            MathF.Abs(point.CommandedCurvature - point.ReferenceCurvature)
        );
    }

    internal void MarkReferenceLineJoin(
        float distanceMeters,
        float curvatureDelta
    )
    {
        JoinsReferenceLine = true;
        ReferenceLineJoinDistanceMeters = distanceMeters;
        ReferenceLineJoinCurvatureDelta = MathF.Abs(curvatureDelta);
    }
}
