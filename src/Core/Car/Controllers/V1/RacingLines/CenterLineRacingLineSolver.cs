using Godot;
using StintegyEVO.Core.Track;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public sealed class CenterLineRacingLineSolver : IRacingLineSolver
{
    public static readonly CenterLineRacingLineSolver Instance = new();

    private CenterLineRacingLineSolver()
    {
    }

    public RacingLine Generate(TrackData track)
    {
        RacingLinePoint[] points = new RacingLinePoint[track.Length];

        for (int i = 0; i < track.Length; i++)
        {
            Vector2 prev = track[i - 1].Center;
            Vector2 curr = track[i].Center;
            Vector2 next = track[i + 1].Center;
            Vector2 tangent = (next - prev).Normalized();
            float heading = tangent.Angle();
            float curvature = GeomUtil.Curvature(prev, curr, next);

            points[i] = new RacingLinePoint(
                i,
                i * TrackData.StepLength,
                0f,
                curr,
                heading,
                curvature
            );
        }

        return new RacingLine(points);
    }
}
