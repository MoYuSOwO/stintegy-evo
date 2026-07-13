using System;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

public interface IRaceDriver
{
    /// <summary>
    /// Small proxy for tire sliding energy that is not represented by the
    /// aggregate curvature/acceleration command. One preserves legacy physics.
    /// </summary>
    float TireEnergyEfficiency => 1f;

    void Initialize(in RaceDriverInitContext context)
    {
    }

    DriverInput GetControl(in RaceDriverFrameContext context, float dt);
}

public readonly record struct RaceDriverInitContext(
    RaceCar Car,
    TrackData Track,
    TrackPose Pose,
    RaceEnvironment Environment,
    float RaceTimeSeconds
);

public readonly record struct RaceDriverFrameContext(
    RaceCar Car,
    TrackData Track,
    TrackPose Pose,
    RaceEnvironment Environment,
    float RaceTimeSeconds,
    RaceFrameSnapshot Frame = default,
    int CarSnapshotIndex = -1
)
{
    public bool HasFrameSnapshot => CarSnapshotIndex >= 0 && CarSnapshotIndex < Frame.Count;

    public RaceCarSnapshot CarSnapshot => HasFrameSnapshot
        ? Frame[CarSnapshotIndex]
        : throw new InvalidOperationException("This driver context has no race-frame snapshot.");
}
