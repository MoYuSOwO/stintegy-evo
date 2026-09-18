using System.Numerics;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

/// <summary>Read-only access to fixed road geometry, not a preferred driving path.</summary>
public sealed class DriverTrackView
{
    private readonly TrackData _track;
    internal DriverTrackView(TrackData track) => _track = track;
    public float LengthMeters => _track.LengthMeters;
    public float StartingLineS => _track.StartingLineS;
    public TrackSample Sample(float distanceMeters) => _track.Sample(distanceMeters);
    public TrackPose Project(Vector2 position) => _track.Project(position);
    public float WrapS(float distanceMeters) => _track.WrapS(distanceMeters);
}
