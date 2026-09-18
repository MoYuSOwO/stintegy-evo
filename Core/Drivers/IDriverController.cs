using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Reads a frozen race frame and returns a physical command. Implementations,
/// their internal clocks, planning, and ability-to-behaviour mappings live outside Core.
/// One controller instance belongs to one driver in a race.
/// </summary>
public interface IDriverController
{
    void Initialize(in DriverContext context) { }
    DriverInput GetControl(in DriverContext context, float dt);
}

/// <summary>No live car, mutable environment, policy vector or planning protocol crosses this boundary.</summary>
public readonly record struct DriverContext(
    DriverProfile Profile,
    DriverTrackView Track,
    RaceFrameSnapshot Frame,
    int CarSnapshotIndex
)
{
    public RaceCarSnapshot Car => Frame[CarSnapshotIndex];
    public RaceEnvironmentSnapshot Environment => Frame.Environment;
    public float RaceTimeSeconds => Frame.RaceTimeSeconds;
}
