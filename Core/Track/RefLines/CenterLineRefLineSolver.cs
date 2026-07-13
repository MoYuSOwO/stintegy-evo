using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Track.RefLines;

public sealed class CenterLineRefLineSolver : IRefLineSolver
{
    public static readonly CenterLineRefLineSolver Instance = new();

    private CenterLineRefLineSolver()
    {
    }

    public RefLine Generate(IReadOnlyList<RefLineTrackPoint> track)
    {
        RefLinePoint[] points = new RefLinePoint[track.Count];

        for (int i = 0; i < track.Count; i++)
        {
            Vector2 prev = track[WrapIndex(i - 1, track.Count)].Center;
            Vector2 curr = track[i].Center;
            Vector2 next = track[WrapIndex(i + 1, track.Count)].Center;
            Vector2 tangent = Vector2.Normalize(next - prev);
            float heading = tangent.Angle();
            float curvature = MathHelper.Curvature(prev, curr, next);

            points[i] = new RefLinePoint(
                0f,
                curr,
                heading,
                curvature
            );
        }

        return new RefLine(points);
    }

    private static int WrapIndex(int index, int length) => (index % length + length) % length;
}
