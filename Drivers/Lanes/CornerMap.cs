namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// Pure geometric bend and pin extraction from a circuit's free racing line:
/// where each bend begins, peaks and ends, and where the free line turns in
/// and tracks out around it.
/// </summary>
public static class CornerMap
{
    public readonly record struct Bend(int First, int Apex, int Last);

    public readonly record struct Pins(int Entry, int Apex, int Exit, int Sign);

    /// <summary>The free line's signed curvature at every station, smoothed over ten metres either side.</summary>
    public static float[] FreeCurvature(LaneSet lanes)
    {
        int n = lanes.Count;
        var raw = new float[n];
        for (int i = 0; i < n; i++)
            raw[i] = LaneSet.Curvature(
                lanes.Position(i - 2, lanes.Offset(LaneId.Free, i - 2)),
                lanes.Position(i, lanes.Offset(LaneId.Free, i)),
                lanes.Position(i + 2, lanes.Offset(LaneId.Free, i + 2)));
        int half = Math.Max(1, (int)MathF.Round(10f / lanes.Spacing));
        var smooth = new float[n];
        for (int i = 0; i < n; i++)
        {
            float sum = 0f;
            for (int j = -half; j <= half; j++)
                sum += raw[lanes.Wrap(i + j)];
            smooth[i] = sum / (2 * half + 1);
        }
        return smooth;
    }

    /// <summary>
    /// Stretches turning tighter than <paramref name="radius"/> one way, joined
    /// across gaps shorter than <paramref name="joinMeters"/>.
    /// </summary>
    public static List<Bend> Bends(LaneSet lanes, float[] k, float radius = 600f, float joinMeters = 30f)
    {
        int n = lanes.Count;
        float limit = 1f / radius;
        // Start the walk on a straight so no bend is cut by the seam.
        int start = 0;
        while (start < n && MathF.Abs(k[start]) >= limit)
            start++;
        var bends = new List<Bend>();
        int join = (int)MathF.Round(joinMeters / lanes.Spacing);
        int? first = null;
        int last = 0, sign = 0;
        for (int step = 0; step <= n; step++)
        {
            int i = lanes.Wrap(start + step);
            bool turning = MathF.Abs(k[i]) >= limit;
            int s = MathF.Sign(k[i]);
            if (turning && first is null)
            {
                first = start + step;
                sign = s;
            }
            else if (turning && s != sign)
            {
                bends.Add(Close(first!.Value, last));
                first = start + step;
                sign = s;
            }
            if (turning)
                last = start + step;
            else if (first is not null && start + step - last > join)
            {
                bends.Add(Close(first.Value, last));
                first = null;
            }
        }
        if (first is not null)
            bends.Add(Close(first.Value, last));
        return bends;

        Bend Close(int a, int b)
        {
            int apex = a;
            for (int i = a; i <= b; i++)
                if (MathF.Abs(k[lanes.Wrap(i)]) > MathF.Abs(k[lanes.Wrap(apex)]))
                    apex = i;
            return new Bend(lanes.Wrap(a), lanes.Wrap(apex), lanes.Wrap(b));
        }
    }

    /// <summary>
    /// Where the free line turns in and tracks out round a bend: the station
    /// nearest the outside before the apex and after it, no further back
    /// than the last bend's apex nor on past the next one's, stopping once
    /// the line has come back in by a tenth of the road toward the next bend.
    /// </summary>
    public static Pins PinsOf(LaneSet lanes, float[] k, IReadOnlyList<Bend> bends, int index)
    {
        Bend bend = bends[index];
        int sign = MathF.Sign(k[bend.Apex]);
        int before = lanes.Wrap(bend.Apex - bends[(index - 1 + bends.Count) % bends.Count].Apex);
        int after = lanes.Wrap(bends[(index + 1) % bends.Count].Apex - bend.Apex);
        float Edge(int i) => lanes.HalfWidth(i) - LaneSet.EdgeMarginMeters;
        float Out(int i) => sign * lanes.Offset(LaneId.Free, i) / Edge(i);
        const float Back = 0.1f;
        int entry = bend.Apex - 5, exit = bend.Apex + 5;
        for (int j = 5; j < Math.Min(90, before); j++)
        {
            if (Out(bend.Apex - j) > Out(entry) + 1e-3f)
                entry = bend.Apex - j;
            else if (Out(bend.Apex - j) < Out(entry) - Back)
                break;
        }
        for (int j = 5; j < Math.Min(125, after); j++)
        {
            if (Out(bend.Apex + j) > Out(exit) + 1e-3f)
                exit = bend.Apex + j;
            else if (Out(bend.Apex + j) < Out(exit) - Back)
                break;
        }
        return new Pins(lanes.Wrap(entry), bend.Apex, lanes.Wrap(exit), sign);
    }
}
