using StintegyEVO.Core.Track;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;

namespace StintegyEVO.Core.Car.Controllers.V1.GraphBased;

public sealed class GraphPlannerConfig
{
    public float HorizonMeters { get; set; } = 120f;
    public float LayerStepMeters { get; set; } = 4f;
    public float LateralResolutionMeters { get; set; } = 0.5f;
    public float VehicleHalfWidthMeters { get; set; } = TrackPlanningBounds.VehicleHalfWidthMeters;
    public float EdgeSafetyMarginMeters { get; set; } = TrackPlanningBounds.EdgeSafetyMarginMeters;
    public float LateralOffsetPerMeter { get; set; } = 0.25f;

    public float RacingLineWeight { get; set; } = 1.0f;
    public float RacingLineSaturationWeight { get; set; } = 1.0f;
    public float EdgeLengthWeight { get; set; } = 0.0f;
    public float EdgeAverageCurvatureWeight { get; set; } = 7500.0f;
    public float EdgePeakCurvatureWeight { get; set; } = 2500.0f;
    public float BoundaryWeight { get; set; } = 500.0f;
    public float BoundarySoftMarginMeters { get; set; } = 0.75f;
    public float OffsetChangeWeight { get; set; } = 0.5f;
    public float OffsetSecondDiffWeight { get; set; } = 8.0f;
    public float VehicleTurnRadiusMeters { get; set; } = 7.0f;
    public float VirtualGoalWeight { get; set; } = 10000.0f;
    public int EdgeCostSamples { get; set; } = 4;
    public float SplineSampleStepMeters { get; set; } = 2.5f;
    public float PreviousPathWeight { get; set; } = 0.0f;
    public int PreviousPathMaxIndexGapSamples { get; set; } = 6;
    public float PreviousPathReuseMaxDistanceMeters { get; set; } = 3.0f;
    public float[] LastEdgeCostFactors { get; set; } = [0.0f, 0.5f, 0.8f];
    public float StartHeadingWeight { get; set; } = 0.0f;
    public int StartHeadingLayers { get; set; } = 4;
    public int RollingStartLeadSamples { get; set; } = 1;
    public float ConstantPathMeters { get; set; } = 0f;
}
