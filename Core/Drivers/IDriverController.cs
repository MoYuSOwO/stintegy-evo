using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Reads a frozen race frame and returns a physical command. Implementations,
/// their internal clocks, planning, and ability-to-behaviour mappings live outside Core.
/// One controller instance belongs to one driver in a race.
/// </summary>
/// <remarks>
/// Timing is freeze, collect, advance. In every driving substep the simulation
/// captures one <see cref="RaceFrameSnapshot"/>, hands that same frame to every
/// controller, and validates all of their commands before any of them is
/// published or any physics advances, so no controller sees a world another
/// controller has already changed. If a controller throws or returns a
/// non-finite command, that substep's temporary boundary corrections are
/// undone and nothing from it is published; earlier substeps that completed
/// are not rolled back, and neither is the controller's own private state.
/// </remarks>
public interface IDriverController
{
    void Initialize(in DriverContext context) { }
    DriverInput GetControl(in DriverContext context, float dt);
}

/// <summary>
/// No live car, mutable environment, policy vector or planning protocol crosses this boundary:
/// a controller sees the driver's profile, a read-only track view, and the frozen frame.
/// </summary>
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
