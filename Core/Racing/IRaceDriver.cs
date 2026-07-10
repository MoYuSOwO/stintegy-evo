using TheStint.Core.Cars;
using TheStint.Core.Track;

namespace TheStint.Core.Racing;

public interface IRaceDriver
{
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
