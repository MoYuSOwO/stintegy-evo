namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Reusable storage for one spatial path candidate and its speed plan. The
/// object and its backing arrays are allocated only as capacities grow.
/// </summary>
internal sealed class PathPlanBuffer
{
    public VehiclePathPrediction Path { get; } = new();
    public VehicleSpeedLookahead SpeedLookahead { get; } = new();
    public DynamicPathSpeedPlan SpeedPlan { get; set; }
    public TrafficSpeedConstraint TrafficConstraint { get; set; }
    public TrafficConstraintMemory NextTrafficMemory { get; set; }
}
