using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;

namespace TheStint.Core.Drivers;

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
    float RaceTimeSeconds
);
