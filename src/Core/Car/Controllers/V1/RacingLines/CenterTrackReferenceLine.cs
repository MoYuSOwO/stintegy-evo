using Godot;
using StintegyEVO.Core.Track;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public sealed class CenterTrackReferenceLine(TrackData track) : ITrackReferenceLine
{
    public TrackReferencePoint GetPoint(int trackIndex)
    {
        TrackPoint point = track[trackIndex];
        Vector2 prev = track[trackIndex - 1].Center;
        Vector2 next = track[trackIndex + 1].Center;
        Vector2 tangent = (next - prev).LengthSquared() > 1e-6f
            ? (next - prev).Normalized()
            : point.Tangent;

        return new TrackReferencePoint(
            point.Index,
            0f,
            point.Center,
            GeomUtil.NormalizeAngle(tangent.Angle()),
            GeomUtil.Curvature(prev, point.Center, next)
        );
    }
}
