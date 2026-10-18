using System;
using System.Collections.Generic;
using System.Numerics;

namespace StintegyEVO.Circuits;

/// <summary>
/// Local repairs to a source centreline, where the survey has drawn a
/// corner the road cannot have. Each one touches a stretch named by the
/// circuit that needs it and leaves the rest of the lap as it came.
/// </summary>
internal static class SourceRepairs
{
    /// <summary>
    /// Spreads the turn of a bend drawn as a kink over the stretch around
    /// it. Each source point's turn is shared out over the points within
    /// <paramref name="spreadHalfMeters"/> of it (a triangle, clipped to the
    /// stretch), so the bend turns exactly as far as it did, and the
    /// stretch is then bent back onto its own ends: two smooth corrections
    /// that vanish at both ends land the last point where it was, heading
    /// as it did. The widths along the stretch are smoothed the same way,
    /// since the centre the pipeline uses is the middle of the two edges
    /// and an edge that jumps moves it.
    /// </summary>
    public static IReadOnlyList<TrackCenterlinePoint> SpreadBend(
        IReadOnlyList<TrackCenterlinePoint> points,
        float fromMeters,
        float toMeters,
        float spreadHalfMeters
    )
    {
        int count = points.Count;
        var along = new float[count];
        for (int i = 1; i < count; i++)
            along[i] = along[i - 1] + Vector2.Distance(points[i - 1].Center, points[i].Center);
        int first = Nearest(along, fromMeters);
        int last = Nearest(along, toMeters);
        if (first < 1 || last >= count - 1 || last - first < 4)
            throw new ArgumentOutOfRangeException(nameof(toMeters), "The stretch must lie inside the source, away from its seam.");

        int segments = last - first;
        var length = new double[segments];
        var heading = new double[segments];
        for (int j = 0; j < segments; j++)
        {
            Vector2 step = points[first + j + 1].Center - points[first + j].Center;
            length[j] = step.Length();
            heading[j] = Math.Atan2(step.Y, step.X);
            if (j > 0)
                heading[j] = heading[j - 1] + NormalizeAngle(heading[j] - heading[j - 1]);
        }

        // Each interior point's turn, shared over its neighbours.
        var mid = new double[segments - 1];
        for (int j = 0; j < mid.Length; j++)
            mid[j] = along[first + j + 1] - along[first];
        var spread = new double[mid.Length];
        for (int j = 0; j < mid.Length; j++)
        {
            double turn = heading[j + 1] - heading[j];
            double total = 0;
            for (int k = 0; k < mid.Length; k++)
                total += Triangle(mid[k] - mid[j], spreadHalfMeters);
            for (int k = 0; k < mid.Length; k++)
                spread[k] += turn * Triangle(mid[k] - mid[j], spreadHalfMeters) / total;
        }
        var bent = new double[segments];
        bent[0] = heading[0];
        for (int j = 1; j < segments; j++)
            bent[j] = bent[j - 1] + spread[j - 1];

        // Back onto the stretch's own end: two bumps that are zero at both
        // ends, solved for the end point's miss, a few times over because
        // a heading moves the point by its sine and cosine.
        double totalLength = 0;
        foreach (double l in length)
            totalLength += l;
        var bumpA = new double[segments];
        var bumpB = new double[segments];
        double run = 0;
        for (int j = 0; j < segments; j++)
        {
            double u = (run + 0.5 * length[j]) / totalLength;
            run += length[j];
            bumpA[j] = u * u * (1 - u) * (1 - u);
            bumpB[j] = bumpA[j] * (u - 0.5);
        }
        Vector2 target = points[last].Center;
        for (int pass = 0; pass < 5; pass++)
        {
            Vector2 miss = Walk(points[first].Center, length, bent)[segments] - target;
            double ax = 0, ay = 0, bx = 0, by = 0;
            for (int j = 0; j < segments; j++)
            {
                ax += -length[j] * bumpA[j] * Math.Sin(bent[j]);
                ay += length[j] * bumpA[j] * Math.Cos(bent[j]);
                bx += -length[j] * bumpB[j] * Math.Sin(bent[j]);
                by += length[j] * bumpB[j] * Math.Cos(bent[j]);
            }
            double det = ax * by - bx * ay;
            if (Math.Abs(det) < 1e-12)
                break;
            double a = (-miss.X * by + bx * miss.Y) / det;
            double b = (-ay * -miss.X + ax * -miss.Y) / det;
            for (int j = 0; j < segments; j++)
                bent[j] += a * bumpA[j] + b * bumpB[j];
        }

        Vector2[] centres = Walk(points[first].Center, length, bent);
        var repaired = new TrackCenterlinePoint[count];
        for (int i = 0; i < count; i++)
            repaired[i] = points[i];
        for (int j = 1; j < segments; j++)
        {
            double right = 0, left = 0, weight = 0;
            for (int k = 0; k <= segments; k++)
            {
                double w = Triangle(along[first + k] - along[first + j], spreadHalfMeters * 0.5);
                right += w * points[first + k].RightWidth;
                left += w * points[first + k].LeftWidth;
                weight += w;
            }
            // Eased in from the stretch's ends, so the widths meet the
            // untouched ones without a step.
            double u = (along[first + j] - along[first]) / (along[last] - along[first]);
            double ease = Math.Min(1.0, 4.0 * u * (1 - u));
            TrackCenterlinePoint source = points[first + j];
            repaired[first + j] = new TrackCenterlinePoint(
                centres[j],
                (float)(source.RightWidth + ease * (right / weight - source.RightWidth)),
                (float)(source.LeftWidth + ease * (left / weight - source.LeftWidth)));
        }
        return repaired;
    }

    private static Vector2[] Walk(Vector2 start, double[] length, double[] heading)
    {
        var centres = new Vector2[length.Length + 1];
        double x = start.X, y = start.Y;
        centres[0] = start;
        for (int j = 0; j < length.Length; j++)
        {
            x += length[j] * Math.Cos(heading[j]);
            y += length[j] * Math.Sin(heading[j]);
            centres[j + 1] = new Vector2((float)x, (float)y);
        }
        return centres;
    }

    private static double Triangle(double offset, double half) =>
        Math.Max(0.0, 1.0 - Math.Abs(offset) / half);

    private static int Nearest(float[] along, float meters)
    {
        int best = 0;
        for (int i = 1; i < along.Length; i++)
            if (Math.Abs(along[i] - meters) < Math.Abs(along[best] - meters))
                best = i;
        return best;
    }

    private static double NormalizeAngle(double angle)
    {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle < -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }
}
