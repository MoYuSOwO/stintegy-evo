using System;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Racing;

public sealed class RaceProgress
{
    private bool _initialized;
    private float _initialStartLineOffset;

    public float CurrentS { get; private set; }
    public float CurrentD { get; private set; }
    public float TotalDistance { get; private set; }
    /// <summary>
    /// Continuous longitudinal race position measured from the configured
    /// start line. Unlike CurrentS this does not wrap at the lap seam, and
    /// unlike TotalDistance it includes each car's starting-grid offset.
    /// </summary>
    public float RaceDistanceMeters => _initialStartLineOffset + TotalDistance;
    public float LastDeltaS { get; private set; }
    public int Lap { get; private set; }
    public TrackRegion Region { get; private set; }
    public bool HitWallThisFrame { get; private set; }

    public void Reset(TrackPose pose, TrackRegion region, bool hitWallThisFrame = false)
    {
        _initialized = true;
        _initialStartLineOffset = 0f;
        CurrentS = pose.S;
        CurrentD = pose.D;
        TotalDistance = 0f;
        LastDeltaS = 0f;
        Lap = 0;
        Region = region;
        HitWallThisFrame = hitWallThisFrame;
    }

    public void Reset(
        TrackData track,
        TrackPose pose,
        TrackRegion region,
        bool hitWallThisFrame = false
    )
    {
        Reset(pose, region, hitWallThisFrame);
        float offset = pose.S - track.StartingLineS;
        float halfLength = track.LengthMeters * 0.5f;
        if (offset > halfLength)
            offset -= track.LengthMeters;
        else if (offset < -halfLength)
            offset += track.LengthMeters;
        _initialStartLineOffset = offset;
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
        float startLineDistance = _initialStartLineOffset + TotalDistance;
        Lap = Math.Max(
            0,
            (int)MathF.Floor(
                startLineDistance / Math.Max(track.LengthMeters, 1e-5f)
            )
        );
        Region = region;
        HitWallThisFrame = hitWallThisFrame;
    }
}
