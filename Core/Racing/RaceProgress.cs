using System;
using TheStint.Core.Track;

namespace TheStint.Core.Racing;

public sealed class RaceProgress
{
    private bool _initialized;

    public float CurrentS { get; private set; }
    public float CurrentD { get; private set; }
    public float TotalDistance { get; private set; }
    public float LastDeltaS { get; private set; }
    public int Lap { get; private set; }
    public TrackRegion Region { get; private set; }
    public bool HitWallThisFrame { get; private set; }

    public void Reset(TrackPose pose, TrackRegion region, bool hitWallThisFrame = false)
    {
        _initialized = true;
        CurrentS = pose.S;
        CurrentD = pose.D;
        TotalDistance = 0f;
        LastDeltaS = 0f;
        Lap = 0;
        Region = region;
        HitWallThisFrame = hitWallThisFrame;
    }

    public void Update(TrackData track, TrackPose pose, TrackRegion region, bool hitWallThisFrame)
    {
        if (!_initialized)
        {
            Reset(pose, region);
            HitWallThisFrame = hitWallThisFrame;
            return;
        }

        float deltaS = pose.S - CurrentS;
        float halfLength = track.LengthMeters * 0.5f;

        if (deltaS > halfLength)
            deltaS -= track.LengthMeters;
        else if (deltaS < -halfLength)
            deltaS += track.LengthMeters;

        CurrentS = pose.S;
        CurrentD = pose.D;
        LastDeltaS = deltaS;
        TotalDistance += deltaS;
        Lap = (int)MathF.Floor(TotalDistance / Math.Max(track.LengthMeters, 1e-5f));
        Region = region;
        HitWallThisFrame = hitWallThisFrame;
    }
}
