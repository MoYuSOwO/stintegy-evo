using System.Globalization;
using System.Numerics;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>Which of the four baked lanes.</summary>
public enum LaneId
{
    Free = 0,
    Left = 1,
    Right = 2,
    /// <summary>The middle of the road, for the car between two others.</summary>
    Middle = 3
}

/// <summary>
/// Four closed lanes on one circuit, each an offset from the centreline at
/// the same stations: the free lane uses the whole road, the left and right
/// lanes leave a car's width on the other side, and the middle lane keeps to
/// the middle, for three abreast. Anywhere between them is a blend, because
/// at one station all four sit on the same normal.
///
/// A driver asset, baked offline and kept with the track's tuning file. Core
/// still stores no line.
/// </summary>
public sealed class LaneSet
{
    public const float DefaultStationSpacing = 2f;

    /// <summary>
    /// Half a two-metre FIA car plus a tenth of margin: the least a lane
    /// centre may sit from the white line before the body is over it.
    /// </summary>
    public const float EdgeMarginMeters = 1.1f;

    public const int LaneCount = 4;

    private readonly float[][] _offset;
    private readonly Vector2[] _centre;
    private readonly Vector2[] _normal;
    private readonly float[] _halfWidth;

    private LaneSet(
        float lengthMeters,
        float spacing,
        Vector2[] centre,
        Vector2[] normal,
        float[] halfWidth,
        float[][] offset
    )
    {
        LengthMeters = lengthMeters;
        Spacing = spacing;
        _centre = centre;
        _normal = normal;
        _halfWidth = halfWidth;
        _offset = offset;
    }

    public float LengthMeters { get; }
    public float Spacing { get; }
    public int Count => _centre.Length;

    public float Offset(LaneId lane, int station) => _offset[(int)lane][Wrap(station)];
    public float HalfWidth(int station) => _halfWidth[Wrap(station)];
    public Vector2 Centre(int station) => _centre[Wrap(station)];
    public Vector2 Normal(int station) => _normal[Wrap(station)];

    public int Wrap(int station) => (station % Count + Count) % Count;

    public float StationOf(float s)
    {
        float t = s / Spacing;
        t %= Count;
        return t < 0f ? t + Count : t;
    }

    /// <summary>
    /// The blended offset, Simplix's way round: <paramref name="range"/> is how
    /// far from the free lane (0..1), <paramref name="side"/> which way
    /// (+1 the left lane, -1 the right lane, 0 the middle lane).
    /// </summary>
    public float BlendedOffset(int station, float range, float side)
    {
        int i = Wrap(station);
        return Blend(_offset[0][i], _offset[1][i], _offset[2][i], _offset[3][i], range, side);
    }

    /// <summary>
    /// The one blend of four per-lane values, offsets or anything learned
    /// per lane: along the side from the middle lane out to the left or the
    /// right one, then from the free lane that far towards it.
    /// </summary>
    public static float Blend(float free, float left, float right, float middle, float range, float side)
    {
        if (range <= 0f)
            return free;
        side = Math.Clamp(side, -1f, 1f);
        float across = side >= 0f ? middle + (left - middle) * side : middle + (right - middle) * -side;
        return free + (across - free) * range;
    }

    /// <summary>Where a file has no middle lane: halfway between the side lanes.</summary>
    internal static float[] Between(float[] left, float[] right) =>
        left.Select((l, i) => 0.5f * (l + right[i])).ToArray();

    public Vector2 Position(int station, float offset) =>
        Centre(station) + Normal(station) * offset;

    /// <summary>Signed curvature through three points, positive turning left.</summary>
    public static float Curvature(Vector2 a, Vector2 b, Vector2 c)
    {
        Vector2 ab = b - a;
        Vector2 bc = c - b;
        float lab = ab.Length();
        float lbc = bc.Length();
        float lac = (c - a).Length();
        float denominator = lab * lbc * lac;
        if (denominator < 1e-6f)
            return 0f;
        return 2f * (ab.X * bc.Y - ab.Y * bc.X) / denominator;
    }

    /// <summary>
    /// Bakes the three lanes for <paramref name="track"/>. Each lane is a
    /// K1999-style relaxation: every point is moved along its normal until the
    /// curvature through it is the distance-weighted mean of its neighbours',
    /// coarse to fine, inside that lane's bounds.
    /// </summary>
    public static LaneSet Bake(
        Func<float, TrackSample> sample,
        float lengthMeters,
        LaneBakeOptions? options = null
    )
    {
        LaneBakeOptions o = options ?? new LaneBakeOptions();
        int count = Math.Max(64, (int)MathF.Round(lengthMeters / o.StationSpacing));
        float spacing = lengthMeters / count;
        var centre = new Vector2[count];
        var normal = new Vector2[count];
        var halfWidth = new float[count];
        for (int i = 0; i < count; i++)
        {
            TrackSample here = sample(i * spacing);
            centre[i] = here.Center;
            normal[i] = here.Normal;
            halfWidth[i] = here.HalfWidth;
        }

        var offsets = new float[LaneCount][];
        for (int lane = 0; lane < 3; lane++)
        {
            var low = new float[count];
            var high = new float[count];
            for (int i = 0; i < count; i++)
            {
                float edge = halfWidth[i] - o.EdgeMargin;
                low[i] = -edge;
                high[i] = edge;
                if (lane == (int)LaneId.Left)
                    low[i] = MathF.Min(edge, -edge + o.SideRoom);
                else if (lane == (int)LaneId.Right)
                    high[i] = MathF.Max(-edge, edge - o.SideRoom);
            }

            offsets[lane] = Relax(centre, normal, low, high, o);
        }
        offsets[(int)LaneId.Middle] = Between(offsets[1], offsets[2]);

        return new LaneSet(lengthMeters, spacing, centre, normal, halfWidth, offsets);
    }

    public static LaneSet Bake(TrackData track, LaneBakeOptions? options = null) =>
        Bake(track.Sample, track.LengthMeters, options);

    public static LaneSet Bake(DriverTrackView track, LaneBakeOptions? options = null) =>
        Bake(track.Sample, track.LengthMeters, options);

    private static float[] Relax(
        Vector2[] centre,
        Vector2[] normal,
        float[] low,
        float[] high,
        LaneBakeOptions o
    )
    {
        int count = centre.Length;
        var n = new float[count];
        for (int i = 0; i < count; i++)
            n[i] = Math.Clamp(0f, low[i], high[i]);

        Vector2 P(int i) => centre[(i % count + count) % count] +
                            normal[(i % count + count) % count] * n[(i % count + count) % count];

        int step = 1;
        while (step * 16 < count)
            step *= 2;

        while (step > 0)
        {
            for (int pass = 0; pass < o.PassesPerLevel; pass++)
            {
                for (int i = 0; i < count; i += step)
                {
                    Vector2 p0 = P(i - 2 * step);
                    Vector2 p1 = P(i - step);
                    Vector2 p3 = P(i + step);
                    Vector2 p4 = P(i + 2 * step);
                    Vector2 p2 = P(i);
                    float kPrev = Curvature(p0, p1, p2);
                    float kNext = Curvature(p2, p3, p4);
                    float lPrev = (p2 - p1).Length();
                    float lNext = (p3 - p2).Length();
                    float target = (lNext * kPrev + lPrev * kNext) /
                                   MathF.Max(lPrev + lNext, 1e-4f);

                    const float probe = 0.01f;
                    float k = Curvature(p1, p2, p3);
                    float kProbe = Curvature(p1, p2 + normal[i] * probe, p3);
                    float slope = (kProbe - k) / probe;
                    if (MathF.Abs(slope) < 1e-7f)
                        continue;

                    // The inner edge is given more room the tighter the bend,
                    // the outer edge none: a kerb is nearer the apex than the
                    // wall is to the exit.
                    float moved = n[i] + (target - k) / slope * o.Relaxation;
                    float inner = MathF.Min(o.MaxInnerMargin, o.InnerMarginPerCurvature * MathF.Abs(target));
                    // The track's normal is (Ty, -Tx): clockwise of the tangent.
                    // A counter-clockwise bend (positive curvature) has its
                    // inside at negative offsets, so that is the side given room.
                    float lo = low[i] + (target > 0f ? inner : 0f);
                    float hi = high[i] - (target < 0f ? inner : 0f);
                    if (lo > hi)
                        lo = hi = 0.5f * (lo + hi);
                    n[i] = Math.Clamp(moved, lo, hi);
                }

                if (step > 1)
                    FillBetween(n, step, low, high);
            }

            step /= 2;
        }

        return n;
    }

    private static void FillBetween(float[] n, int step, float[] low, float[] high)
    {
        int count = n.Length;
        for (int i = 0; i < count; i += step)
        {
            int j = i + step;
            float a = n[i];
            float b = n[j % count];
            int span = Math.Min(step, count - i);
            for (int k = 1; k < span; k++)
            {
                int m = (i + k) % count;
                float t = (float)k / step;
                n[m] = Math.Clamp(a + (b - a) * t, low[m], high[m]);
            }
        }
    }

    /// <summary>
    /// Lanes from the offline 3D bake (<c>Drivers/Bake/bake_lanes_3d.py</c>):
    /// x, y points per lane, projected onto this track and laid onto its
    /// stations with a periodic cubic through a lightly smoothed sequence. A
    /// lane the file does not carry is taken from <paramref name="fallback"/>.
    /// </summary>
    public static LaneSet ImportBaked(string path, TrackData track, LaneSet fallback)
    {
        var points = new Dictionary<string, List<(float S, float D)>>();
        string[] lines = File.ReadAllLines(path);
        string[] header = lines[0].Split(',');
        int laneCol = Array.IndexOf(header, "lane");
        int xCol = Array.IndexOf(header, "x_m");
        int yCol = Array.IndexOf(header, "y_m");
        for (int r = 1; r < lines.Length; r++)
        {
            string[] cols = lines[r].Split(',');
            var xy = new Vector2(
                float.Parse(cols[xCol], CultureInfo.InvariantCulture),
                float.Parse(cols[yCol], CultureInfo.InvariantCulture));
            TrackPose pose = track.Project(xy);
            if (!points.TryGetValue(cols[laneCol], out var list))
                points[cols[laneCol]] = list = new List<(float, float)>();
            list.Add((pose.S, pose.D));
        }

        float length = track.LengthMeters;
        int count = fallback.Count;
        float spacing = fallback.Spacing;
        var offsets = new float[LaneCount][];
        string[] names = ["free", "left", "right", "middle"];
        for (int lane = 0; lane < LaneCount; lane++)
        {
            if (!points.TryGetValue(names[lane], out var list) || list.Count < 16)
            {
                offsets[lane] = lane == (int)LaneId.Middle
                    ? Between(offsets[1], offsets[2])
                    : Enumerable.Range(0, count).Select(i => fallback.Offset((LaneId)lane, i)).ToArray();
                continue;
            }

            // Unwrap s along the bake's own order, then smooth d a little.
            int m = list.Count;
            if (list[0].S > length * 0.5f)
                list[0] = (list[0].S - length, list[0].D);
            var s = new float[m];
            var d = new float[m];
            s[0] = list[0].S;
            d[0] = list[0].D;
            for (int k = 1; k < m; k++)
            {
                float step = list[k].S - list[k - 1].S;
                if (step < -length * 0.5f) step += length;
                if (step > length * 0.5f) step -= length;
                s[k] = s[k - 1] + step;
                d[k] = list[k].D;
            }
            // A bake closes its lap by ending on its first point again. Kept,
            // the loop holds that point twice, a step of nothing that the
            // spline bends round: on Spa it put a 4 g kink in Blanchimont's
            // straight-lined middle, and the car lifted for it every lap.
            while (m > 16 && s[m - 1] > s[0] + length - 0.5f * spacing)
                m--;
            var smooth = new float[m];
            for (int k = 0; k < m; k++)
                smooth[k] = 0.25f * d[(k - 1 + m) % m] + 0.5f * d[k] + 0.25f * d[(k + 1) % m];

            var lane_ = new float[count];
            int j = 0;
            for (int i = 0; i < count; i++)
            {
                // Find the bake interval holding this station, round the loop.
                float target = i * spacing;
                while (target < s[0]) target += length;
                while (target >= s[0] + length) target -= length;
                while (j < m - 1 && s[j + 1] <= target) j++;
                while (j > 0 && s[j] > target) j--;
                int j0 = j, j1 = (j + 1) % m;
                float s0 = s[j0];
                float s1 = j1 == 0 ? s[0] + length : s[j1];
                float u = Math.Clamp((target - s0) / MathF.Max(s1 - s0, 1e-4f), 0f, 1f);
                float p0 = smooth[(j0 - 1 + m) % m], p1 = smooth[j0], p2 = smooth[j1], p3 = smooth[(j1 + 1) % m];
                float value = 0.5f * (2f * p1 + (-p0 + p2) * u + (2f * p0 - 5f * p1 + 4f * p2 - p3) * u * u +
                                      (-p0 + 3f * p1 - 3f * p2 + p3) * u * u * u);
                lane_[i] = value;
            }
            offsets[lane] = lane_;
        }

        return new LaneSet(length, spacing, fallback._centre, fallback._normal, fallback._halfWidth, offsets);
    }

    /// <summary>The same lanes with the free lane replaced, for comparing lines.</summary>
    internal LaneSet WithFreeOffsets(float[] free) =>
        new(LengthMeters, Spacing, _centre, _normal, _halfWidth, [free, _offset[1], _offset[2], _offset[3]]);

    public void Save(string path)
    {
        using var writer = new StreamWriter(path);
        writer.WriteLine($"# lanes length_m={LengthMeters.ToString(CultureInfo.InvariantCulture)} spacing_m={Spacing.ToString(CultureInfo.InvariantCulture)}");
        writer.WriteLine("station,s_m,free_m,left_m,right_m,half_width_m,middle_m");
        for (int i = 0; i < Count; i++)
        {
            writer.WriteLine(string.Join(',',
                i.ToString(CultureInfo.InvariantCulture),
                (i * Spacing).ToString("F2", CultureInfo.InvariantCulture),
                _offset[0][i].ToString("F4", CultureInfo.InvariantCulture),
                _offset[1][i].ToString("F4", CultureInfo.InvariantCulture),
                _offset[2][i].ToString("F4", CultureInfo.InvariantCulture),
                _halfWidth[i].ToString("F3", CultureInfo.InvariantCulture),
                _offset[3][i].ToString("F4", CultureInfo.InvariantCulture)));
        }
    }

    /// <summary>
    /// Loads lanes saved against this circuit. Geometry comes from the live
    /// track; only the offsets are read, so a file can never move the road.
    /// </summary>
    public static LaneSet Load(string path, Func<float, TrackSample> sample, float lengthMeters)
    {
        var free = new List<float>();
        var left = new List<float>();
        var right = new List<float>();
        var middle = new List<float>();
        int middleCol = -1;
        float spacing = DefaultStationSpacing;
        foreach (string line in File.ReadLines(path))
        {
            if (line.StartsWith('#'))
            {
                foreach (string part in line.Split(' '))
                {
                    if (part.StartsWith("spacing_m="))
                        spacing = float.Parse(part[10..], CultureInfo.InvariantCulture);
                }
                continue;
            }
            if (line.StartsWith("station"))
            {
                middleCol = Array.IndexOf(line.Split(','), "middle_m");
                continue;
            }
            string[] cols = line.Split(',');
            free.Add(float.Parse(cols[2], CultureInfo.InvariantCulture));
            left.Add(float.Parse(cols[3], CultureInfo.InvariantCulture));
            right.Add(float.Parse(cols[4], CultureInfo.InvariantCulture));
            if (middleCol >= 0)
                middle.Add(float.Parse(cols[middleCol], CultureInfo.InvariantCulture));
        }

        int count = free.Count;
        if (MathF.Abs(count * spacing - lengthMeters) > 2f * spacing)
            throw new InvalidOperationException($"Lane file {path} was baked for another circuit.");
        var centre = new Vector2[count];
        var normal = new Vector2[count];
        var halfWidth = new float[count];
        for (int i = 0; i < count; i++)
        {
            TrackSample here = sample(i * spacing);
            centre[i] = here.Center;
            normal[i] = here.Normal;
            halfWidth[i] = here.HalfWidth;
        }

        return new LaneSet(lengthMeters, spacing, centre, normal, halfWidth,
            [free.ToArray(), left.ToArray(), right.ToArray(),
             middleCol >= 0 ? middle.ToArray() : Between(left.ToArray(), right.ToArray())]);
    }
}

public sealed record LaneBakeOptions
{
    public float StationSpacing { get; init; } = LaneSet.DefaultStationSpacing;

    /// <summary>Half a two-metre car plus 0.1 m: from the centre of the car to the white line.</summary>
    public float EdgeMargin { get; init; } = LaneSet.EdgeMarginMeters;

    /// <summary>What a side lane leaves on its other side: a car and a gap.</summary>
    public float SideRoom { get; init; } = 3.0f;

    public float InnerMarginPerCurvature { get; init; } = 20f;
    public float MaxInnerMargin { get; init; } = 0.0f;
    public int PassesPerLevel { get; init; } = 1500;
    public float Relaxation { get; init; } = 0.8f;
}
