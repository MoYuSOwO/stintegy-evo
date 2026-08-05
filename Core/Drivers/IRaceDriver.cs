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

    /// <summary>
    /// How much of the cornering the tyres are being asked for actually
    /// reaches the road, from this driver.
    ///
    /// Not how hard the driver decides to push, which is a strategy setting.
    /// This is what is left over after the way they push: the line taken, the
    /// steadiness of hand, how much of the tyre's work goes into turning the
    /// car and how much goes into scrubbing it sideways. One driver asked for
    /// nine tenths of a tyre gets nine tenths of a corner out of it; another
    /// asked for the same gets less, and the tyre is no less worn for it.
    /// </summary>
    float CorneringEfficiency => 1f;

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
