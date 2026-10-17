using System.Numerics;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Circuits;

/// <summary>
/// Where a circuit's pit lane is, as far as anyone authoring it has to say:
/// how far before the line it leaves the track, how far after it comes
/// back, on which side (+1 left, -1 right, looking along the lap), and
/// the limit. The shape follows from the road; the rest is the same at
/// every circuit.
/// </summary>
internal sealed record PitSpec(int EntryFromLine, int ExitFromLine, int Side, float SpeedLimitKph);

/// <summary>
/// Draws a pit lane beside the road it belongs to: out of the road's edge
/// at the entry, across the run-off and past the barrier, alongside the
/// track behind the pit wall, and back the same way at the exit.
/// </summary>
internal static class Pits
{
    /// <summary>A fast lane and a working lane, six metres each.</summary>
    private const float LaneWidthMeters = 12f;

    /// <summary>Where it leaves and rejoins the road, the lane is the fast lane alone.</summary>
    private const float EndWidthMeters = 6f;

    /// <summary>How far along the track the lane takes to move out from the road to behind the wall, and back.</summary>
    private const float RampMeters = 90f;

    /// <summary>Between the barrier's line and the lane's edge: the pit wall and the space behind it.</summary>
    private const float WallMeters = 2f;

    /// <summary>How far in from each end the limit lines are painted.</summary>
    private const float LimitLineMeters = 45f;

    /// <summary>Box to box: one garage's frontage.</summary>
    private const float BoxSpacingMeters = 14f;

    /// <summary>
    /// The lane and its rows for a road as it is written: the track the game
    /// will load and the heights of its rows.
    /// </summary>
    public static (CircuitPitLane Lane, PitLaneFile.Row[] Rows) Build(PitSpec spec, TrackData track, float[] heights)
    {
        int n = heights.Length;
        int line = (int)MathF.Round(track.StartingLineS / TrackData.StepLength);
        int entry = (line + spec.EntryFromLine + n) % n;
        int exit = (line + spec.ExitFromLine + n) % n;
        int span = spec.ExitFromLine - spec.EntryFromLine;
        if (span <= 2 * RampMeters)
            throw new InvalidOperationException($"A pit lane of {span} m along the track has no room to move off it and back.");

        var shape = new List<PitLaneFile.Row>(span + 1);
        for (int j = 0; j <= span; j++)
        {
            int row = (entry + j) % n;
            TrackSample road = track.Sample(row * TrackData.StepLength);
            float buffer = spec.Side > 0 ? road.LeftBufferWidth : road.RightBufferWidth;
            float away = MathF.Min(Ease(j / RampMeters), Ease((span - j) / RampMeters));
            float atRoad = road.HalfWidth - EndWidthMeters * 0.5f;
            float behindWall = road.HalfWidth + buffer + WallMeters + LaneWidthMeters * 0.5f;
            float d = spec.Side * (atRoad + (behindWall - atRoad) * away);
            shape.Add(new PitLaneFile.Row(
                road.Center + road.Normal * d,
                heights[row],
                EndWidthMeters + (LaneWidthMeters - EndWidthMeters) * away));
        }
        PitLaneFile.Row[] rows = Respace(shape);

        int length = rows.Length - 1;
        float garagesFrom = RampMeters + BoxSpacingMeters * 0.5f;
        int boxes = (int)MathF.Floor((length - 2f * garagesFrom) / BoxSpacingMeters) + 1;
        var lane = new CircuitPitLane
        {
            LengthMeters = length,
            EntryMeters = entry,
            ExitMeters = exit,
            SpeedLimitKph = spec.SpeedLimitKph,
            SpeedLimitFromMeters = LimitLineMeters,
            SpeedLimitToMeters = length - LimitLineMeters,
            Boxes = new CircuitPitBoxes
            {
                Count = boxes,
                FirstMeters = garagesFrom,
                SpacingMeters = BoxSpacingMeters,
                // The working lane is the half away from the track.
                OffsetMeters = spec.Side * LaneWidthMeters * 0.25f
            }
        };
        return (lane, rows);
    }

    private static float Ease(float t)
    {
        t = Math.Clamp(t, 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    /// <summary>The same lane at equal steps along it, a whole number of them, each as near a metre as that allows.</summary>
    private static PitLaneFile.Row[] Respace(List<PitLaneFile.Row> shape)
    {
        var along = new double[shape.Count];
        for (int i = 1; i < shape.Count; i++)
            along[i] = along[i - 1] + Vector2.Distance(shape[i - 1].Center, shape[i].Center);
        int steps = (int)Math.Round(along[^1] / TrackData.StepLength);
        double step = along[^1] / steps;
        var rows = new PitLaneFile.Row[steps + 1];
        int k = 0;
        for (int j = 0; j <= steps; j++)
        {
            double s = Math.Min(j * step, along[^1]);
            while (k < shape.Count - 2 && along[k + 1] <= s)
                k++;
            float t = (float)((s - along[k]) / Math.Max(along[k + 1] - along[k], 1e-9));
            PitLaneFile.Row a = shape[k], b = shape[k + 1];
            rows[j] = new PitLaneFile.Row(
                Vector2.Lerp(a.Center, b.Center, t),
                a.Z + (b.Z - a.Z) * t,
                a.Width + (b.Width - a.Width) * t);
        }
        return rows;
    }
}
