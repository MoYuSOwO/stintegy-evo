using System.Globalization;
using System.Numerics;
using System.Text;
using System.Text.Json;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Circuits;

/// <summary>
/// A circuit's pit lane as it really is: the OpenStreetMap ways that make
/// up the Grand Prix pit lane, from where it leaves the track to where it
/// comes back (where a circuit has more than one, the Formula One one),
/// and the limit there.
/// </summary>
internal sealed record PitSpec(long[] Ways, float SpeedLimitKph);

/// <summary>
/// Puts a real pit lane beside the road it belongs to. The lane is drawn
/// by OpenStreetMap; the road is ours, which is the same circuit smoothed
/// and scaled to its published length, so the two do not lie on each
/// other as they come. The lane is carried onto our road by the stretch
/// of the real circuit around it, its two ends are made two points of our
/// centreline, and the run-off on its side is cut back to the pit wall.
/// </summary>
internal static class Pits
{
    public static string Folder => Path.Combine(Sources.Folder, "pitlanes");

    /// <summary>The fast lane, and the working lane beside it where the cars stop: nine metres.</summary>
    private const float FastLaneMeters = 5f;
    private const float WorkingLaneMeters = 4f;
    private const float LaneWidthMeters = FastLaneMeters + WorkingLaneMeters;

    /// <summary>Where it leaves and rejoins the road, the lane is a single lane.</summary>
    private const float EndWidthMeters = FastLaneMeters;

    /// <summary>How far along the lane it widens from the one to the other.</summary>
    private const float WidenMeters = 60f;

    /// <summary>
    /// How far along the lane from each end the difference between the real
    /// lane's end and our centreline is taken up.
    /// </summary>
    private const float JoinMeters = 80f;

    /// <summary>How much of the real circuit round the lane its position on our road is taken from.</summary>
    private const float FitReachMeters = 600f;

    /// <summary>Rows either side over which how far the lane is moved out is eased.</summary>
    private const int PushEaseRows = 30;

    /// <summary>How many times the real stretch is paired afresh with our road and laid on it again.</summary>
    private const int FitPasses = 12;

    /// <summary>
    /// Between the road's edge and the lane's: the pit wall. There is no
    /// run-off in a pit lane; the track's barrier, where the run-off on that
    /// side is cut back to it, is the pit wall, and the lane runs along its
    /// foot: the kerb, which every edge of the track has, then the wall.
    /// </summary>
    private const float WallMeters = SurfaceGrip.KerbWidthMeters + 0.4f;

    /// <summary>
    /// A garage opens straight onto the lane's edge, and is at least
    /// this deep: Monaco's, between the harbour front and the pool, are
    /// shallow. The game draws each as deep as the ground behind it allows.
    /// </summary>
    private const float GarageSetbackMeters = 0f;
    private const float GarageMinDepthMeters = 5f;

    /// <summary>How far a garage must stand from any barrier.</summary>
    private const float GarageClearMeters = 1f;

    /// <summary>How far past the pit wall the lane's edge may be and still run along it.</summary>
    private const float PitWallReachMeters = 8f;

    /// <summary>How far in from each end the limit lines are painted.</summary>
    private const float LimitLineMeters = 45f;

    /// <summary>Box to box: one garage's frontage.</summary>
    private const float BoxSpacingMeters = 14f;

    /// <summary>How far the lane is smoothed along itself, so the corners of the map's polyline are not corners of the road.</summary>
    private const int SmoothHalfRows = 4;

    /// <summary>
    /// The lane and its rows for a road as it is written, the rows with the
    /// run-off on the lane's side cut back to the pit wall, and the most the
    /// lane was moved out from where the map has it.
    /// </summary>
    public static (CircuitPitLane Lane, PitLaneFile.Row[] Rows, CircuitTrackFile.Row[] Track, float Pushed) Build(
        string id, PitSpec spec, TrackData track, CircuitTrackFile.Row[] rows)
    {
        int n = rows.Length;
        Vector2[] centre = rows.Select(r => r.Center).ToArray();
        Elevation.SurveyLine line = Elevation.Line(id, centre)
            ?? throw new InvalidOperationException($"{id}: a pit lane needs the real circuit's line (Sources/elevation).");
        List<Vector2> real = ReadLane(id).Select(p => line.Project(p.Lat, p.Lon)).ToList();

        // Carry the lane onto our road by the real circuit round it: the
        // similarity that best lays the real line's stretch there on ours.
        // The two lines are paired first by how far round the lap they are,
        // then, again and again, each real point with the nearest point of
        // ours, so the stretch is laid on our road and not merely along it.
        var near = new List<Vector2>();
        var pairs = new List<(Vector2 Theirs, Vector2 Ours)>();
        for (int i = 0; i < n; i++)
            if (real.Min(p => Vector2.Distance(p, line.AtRow[i])) < FitReachMeters)
            {
                near.Add(line.AtRow[i]);
                pairs.Add((line.AtRow[i], centre[i]));
            }
        Func<Vector2, Vector2> carry = Similarity(pairs);
        for (int pass = 0; pass < FitPasses; pass++)
            carry = Similarity(near.Select(p => (p, Foot(centre, carry(p)))).ToList());
        List<Vector2> lane = real.Select(carry).ToList();

        // The way the lap goes, from the row it leaves at to the row it comes back at.
        int entry = NearestRow(centre, lane[0]);
        int exit = NearestRow(centre, lane[^1]);
        if ((exit - entry + n) % n > n / 2)
        {
            lane.Reverse();
            (entry, exit) = (exit, entry);
        }

        // Its ends are our centreline's points; what that moves is taken up
        // over the first and last stretch of the lane.
        lane = Respace(lane);
        Vector2 atEntry = centre[entry] - lane[0], atExit = centre[exit] - lane[^1];
        int last = lane.Count - 1;
        for (int k = 0; k <= last; k++)
            lane[k] += atEntry * (1f - Ease(k / JoinMeters)) + atExit * (1f - Ease((last - k) / JoinMeters));
        lane = Respace(Smooth(lane));
        lane[0] = centre[entry];
        lane[^1] = centre[exit];

        // Our road is a little wider than the real one in places, and the
        // map's line is not always the middle of the whole lane; where the
        // lane runs beside the road but, at its full width, would lie on
        // it, it is moved out just far enough to stand behind the pit wall.
        // Not where it is joining the road, from each end to where its
        // middle first leaves our road: that is as the map has it. How far
        // it is moved is eased along the lane, so it moves out smoothly.
        int points = lane.Count;
        last = points - 1;
        var move = new float[points];
        var outward = new Vector2[points];
        var beside = new bool[points];
        for (int k = 0; k < points; k++)
        {
            int row = NearestRow(centre, lane[k]);
            Vector2 tangent = Vector2.Normalize(centre[(row + 1) % n] - centre[(row - 1 + n) % n]);
            Vector2 normal = new(tangent.Y, -tangent.X);
            float d = Vector2.Dot(lane[k] - centre[row], normal);
            float half = rows[row].Width * 0.5f;
            beside[k] = MathF.Abs(d) > half;
            outward[k] = normal * (d >= 0f ? 1f : -1f);
            move[k] = MathF.Max(0f, half + WallMeters + Width(k, last) * 0.5f - MathF.Abs(d));
        }
        int from = Array.IndexOf(beside, true), to = Array.LastIndexOf(beside, true);
        float pushed = 0f;
        if (from >= 0)
        {
            for (int k = 0; k < points; k++)
                if (k < from || k > to)
                    move[k] = 0f;
            var eased = new float[points];
            for (int k = 1; k < last; k++)
            {
                int half = Math.Min(PushEaseRows, Math.Min(k, last - k));
                float sum = 0f;
                for (int j = -half; j <= half; j++)
                    sum += move[k + j];
                eased[k] = sum / (2 * half + 1);
            }
            for (int k = 1; k < last; k++)
                lane[k] += outward[k] * eased[k];
            for (int k = from; k <= to; k++)
                pushed = MathF.Max(pushed, eased[k]);
        }
        lane = Respace(Smooth(lane));
        lane[0] = centre[entry];
        lane[^1] = centre[exit];

        int length = lane.Count - 1;
        var laneRows = new PitLaneFile.Row[lane.Count];
        for (int k = 0; k <= length; k++)
        {
            int row = NearestRow(centre, lane[k]);
            laneRows[k] = new PitLaneFile.Row(lane[k], rows[row].Z, Width(k, length));
        }

        CircuitTrackFile.Row[] cut = CutRunoff(track, rows, laneRows, entry, exit);
        (float first, int count, float side) = Garages(track, laneRows);
        var pitLane = new CircuitPitLane
        {
            LengthMeters = length,
            EntryMeters = entry,
            ExitMeters = exit,
            SpeedLimitKph = spec.SpeedLimitKph,
            SpeedLimitFromMeters = LimitLineMeters,
            SpeedLimitToMeters = length - LimitLineMeters,
            Boxes = new CircuitPitBoxes
            {
                Count = count,
                FirstMeters = first,
                SpacingMeters = BoxSpacingMeters,
                // In the middle of the working lane, the side away from the track.
                OffsetMeters = side * (LaneWidthMeters - WorkingLaneMeters) * 0.5f
            }
        };
        return (pitLane, laneRows, cut, pushed);
    }

    /// <summary>
    /// The run-off on the lane's side, where the lane runs beside the road,
    /// cut back to what fits between the road and the lane behind its wall.
    /// Where the lane is on the road or the run-off, at its two ends, the
    /// run-off is left as it is: there the lane is coming through it.
    /// </summary>
    private static CircuitTrackFile.Row[] CutRunoff(TrackData track, CircuitTrackFile.Row[] rows, PitLaneFile.Row[] lane, int entry, int exit)
    {
        int n = rows.Length;
        var cut = (CircuitTrackFile.Row[])rows.Clone();
        int span = (exit - entry + n) % n;
        for (int j = 0; j <= span; j++)
        {
            int i = (entry + j) % n;
            TrackSample road = track.Sample(i * TrackData.StepLength);
            // The nearest point of the lane's edge, measured across the road.
            float best = float.PositiveInfinity;
            int side = 0;
            foreach (PitLaneFile.Row r in lane)
            {
                Vector2 offset = r.Center - road.Center;
                if (MathF.Abs(Vector2.Dot(offset, road.Tangent)) > r.Width)
                    continue;
                float across = Vector2.Dot(offset, road.Normal);
                float edge = MathF.Abs(across) - r.Width * 0.5f;
                if (edge < best)
                {
                    best = edge;
                    side = MathF.Sign(across);
                }
            }
            if (side == 0)
                continue;
            // On the road, the lane is joining it: the barrier opens there.
            if (best <= road.HalfWidth)
                continue;
            float room = MathF.Max(best - road.HalfWidth - WallMeters, SurfaceGrip.KerbWidthMeters);
            if (side > 0 && room < cut[i].RunoffLeft)
                cut[i] = cut[i] with { RunoffLeft = room };
            if (side < 0 && room < cut[i].RunoffRight)
                cut[i] = cut[i] with { RunoffRight = room };
        }
        return cut;
    }

    /// <summary>
    /// Where the garages go: the longest stretch of the lane that runs along
    /// the pit wall, which is the pit straight; a box for each team (two
    /// cars to a team) as many as the grid holds and the stretch has room
    /// for, in its middle, on the side away from the track.
    /// </summary>
    private static (float First, int Count, float Side) Garages(TrackData track, PitLaneFile.Row[] lane)
    {
        // Where a garage fits behind every box, if that stretch holds at
        // least half the boxes the wall does; otherwise the boxes go along
        // the wall, and the game builds no garage where there is no ground
        // for one (Monaco's lane is squeezed between two parts of the lap).
        (float First, int Count, float Side) withGarages = Garages(track, lane, needGarages: true);
        (float First, int Count, float Side) alongWall = Garages(track, lane, needGarages: false);
        return 2 * withGarages.Count >= alongWall.Count ? withGarages : alongWall;
    }

    private static (float First, int Count, float Side) Garages(TrackData track, PitLaneFile.Row[] lane, bool needGarages)
    {
        int bestFrom = 0, bestTo = -1, from = -1;
        var side = new float[lane.Length];
        for (int k = 0; k <= lane.Length; k++)
        {
            bool clear = false;
            if (k < lane.Length)
            {
                (float d, TrackSample road) = Across(track, lane[k].Center);
                side[k] = MathF.Sign(d);
                float gap = MathF.Abs(d) - lane[k].Width * 0.5f - road.HalfWidth;
                clear = gap > WallMeters - 0.25f && gap < WallMeters + PitWallReachMeters
                    && k >= LimitLineMeters && k <= lane.Length - 1 - LimitLineMeters
                    && (!needGarages || GarageFits(track, lane, k, side[k]));
            }
            if (clear && from < 0)
                from = k;
            if (!clear && from >= 0)
            {
                if (k - 1 - from > bestTo - bestFrom)
                    (bestFrom, bestTo) = (from, k - 1);
                from = -1;
            }
        }
        float room = bestTo - bestFrom - BoxSpacingMeters;
        if (room < 0f)
            return (0f, 0, 1f);
        int count = Math.Min((int)MathF.Floor(room / BoxSpacingMeters) + 1, (track.StartingGridCount + 1) / 2);
        float first = (bestFrom + bestTo - (count - 1) * BoxSpacingMeters) * 0.5f;
        return (first, count, side[(bestFrom + bestTo) / 2]);
    }

    /// <summary>
    /// Whether a garage behind the box at <paramref name="k"/> stands clear
    /// of every part of the circuit, road and run-off: its whole frontage,
    /// at least its least depth, on the side away from the track.
    /// </summary>
    private static bool GarageFits(TrackData track, PitLaneFile.Row[] lane, int k, float side)
    {
        float half = BoxSpacingMeters * 0.5f;
        if (k - half < 0 || k + half > lane.Length - 1)
            return false;
        foreach (float along in new[] { -half, 0f, half })
        {
            int at = (int)MathF.Round(k + along);
            Vector2 tangent = Vector2.Normalize(lane[Math.Min(at + 1, lane.Length - 1)].Center - lane[Math.Max(at - 1, 0)].Center);
            Vector2 normal = new(tangent.Y, -tangent.X);
            float front = lane[at].Width * 0.5f + GarageSetbackMeters;
            foreach (float depth in new[] { front, front + GarageMinDepthMeters })
            {
                (float d, TrackSample road) = Across(track, lane[at].Center + normal * side * depth);
                float barrier = road.HalfWidth + (d > 0f ? road.LeftBufferWidth : road.RightBufferWidth);
                if (MathF.Abs(d) < barrier + GarageClearMeters)
                    return false;
            }
        }
        return true;
    }

    /// <summary>How far to the side of the nearest point of the track a point is, found by brute force: the lane may be further from the road than a car can be.</summary>
    private static (float D, TrackSample Road) Across(TrackData track, Vector2 point)
    {
        float bestS = 0f, best = float.PositiveInfinity;
        for (float s = 0f; s < track.LengthMeters; s += 1f)
        {
            float d = Vector2.DistanceSquared(track.Sample(s).Center, point);
            if (d < best)
            {
                best = d;
                bestS = s;
            }
        }
        TrackSample sample = track.Sample(bestS);
        return (Vector2.Dot(point - sample.Center, sample.Normal), sample);
    }

    /// <summary>The nearest point of our centreline to a point: on the segment either side of the nearest row.</summary>
    private static Vector2 Foot(Vector2[] centre, Vector2 point)
    {
        int n = centre.Length, i = NearestRow(centre, point);
        Vector2 best = centre[i];
        foreach (int j in new[] { (i - 1 + n) % n, i })
        {
            Vector2 a = centre[j], ab = centre[(j + 1) % n] - a;
            Vector2 foot = a + ab * Math.Clamp(Vector2.Dot(point - a, ab) / ab.LengthSquared(), 0f, 1f);
            if (Vector2.DistanceSquared(foot, point) < Vector2.DistanceSquared(best, point))
                best = foot;
        }
        return best;
    }

    private static int NearestRow(Vector2[] centre, Vector2 point)
    {
        int best = 0;
        float distance = float.PositiveInfinity;
        for (int i = 0; i < centre.Length; i++)
        {
            float d = Vector2.DistanceSquared(centre[i], point);
            if (d < distance)
            {
                distance = d;
                best = i;
            }
        }
        return best;
    }

    /// <summary>The scale, turn and shift that best lay each first point on its second, least squares.</summary>
    private static Func<Vector2, Vector2> Similarity(List<(Vector2 Theirs, Vector2 Ours)> pairs)
    {
        if (pairs.Count < 3)
            throw new InvalidOperationException("Too little of the real circuit near the pit lane to place it.");
        Vector2 pc = Vector2.Zero, qc = Vector2.Zero;
        foreach ((Vector2 p, Vector2 q) in pairs)
        {
            pc += p;
            qc += q;
        }
        pc /= pairs.Count;
        qc /= pairs.Count;
        // As complex numbers: q - qc = z (p - pc).
        double re = 0, im = 0, norm = 0;
        foreach ((Vector2 p, Vector2 q) in pairs)
        {
            Vector2 a = p - pc, b = q - qc;
            re += a.X * b.X + a.Y * b.Y;
            im += a.X * b.Y - a.Y * b.X;
            norm += a.X * a.X + a.Y * a.Y;
        }
        float zr = (float)(re / norm), zi = (float)(im / norm);
        return p =>
        {
            Vector2 a = p - pc;
            return qc + new Vector2(zr * a.X - zi * a.Y, zi * a.X + zr * a.Y);
        };
    }

    /// <summary>The lane's width <paramref name="k"/> metres along a lane <paramref name="length"/> long: the fast lane at the ends, widening to both lanes.</summary>
    private static float Width(int k, int length) =>
        EndWidthMeters + (LaneWidthMeters - EndWidthMeters) * Ease(Math.Min(k, length - k) / WidenMeters);

    private static float Ease(float t)
    {
        t = Math.Clamp(t, 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    /// <summary>A moving average along the lane that leaves its two ends where they are.</summary>
    private static List<Vector2> Smooth(List<Vector2> points)
    {
        var result = new List<Vector2>(points);
        for (int k = 1; k < points.Count - 1; k++)
        {
            int half = Math.Min(SmoothHalfRows, Math.Min(k, points.Count - 1 - k));
            Vector2 sum = Vector2.Zero;
            for (int j = -half; j <= half; j++)
                sum += points[k + j];
            result[k] = sum / (2 * half + 1);
        }
        return result;
    }

    /// <summary>The same line at equal steps along it, a whole number of them, each as near a metre as that allows.</summary>
    private static List<Vector2> Respace(List<Vector2> shape)
    {
        var along = new double[shape.Count];
        for (int i = 1; i < shape.Count; i++)
            along[i] = along[i - 1] + Vector2.Distance(shape[i - 1], shape[i]);
        int steps = Math.Max(1, (int)Math.Round(along[^1] / TrackData.StepLength));
        double step = along[^1] / steps;
        var result = new List<Vector2>(steps + 1);
        int k = 0;
        for (int j = 0; j <= steps; j++)
        {
            double s = Math.Min(j * step, along[^1]);
            while (k < shape.Count - 2 && along[k + 1] <= s)
                k++;
            float t = (float)((s - along[k]) / Math.Max(along[k + 1] - along[k], 1e-9));
            result.Add(Vector2.Lerp(shape[k], shape[k + 1], t));
        }
        return result;
    }

    private static List<(double Lat, double Lon)> ReadLane(string id)
    {
        string path = Path.Combine(Folder, id + ".csv");
        return File.ReadAllLines(path).Where(l => l.Length > 0 && !l.StartsWith('#')).Skip(1)
            .Select(l => l.Split(','))
            .Select(f => (double.Parse(f[0], CultureInfo.InvariantCulture), double.Parse(f[1], CultureInfo.InvariantCulture)))
            .ToList();
    }

    /// <summary>
    /// Fetches each circuit's pit lane ways from OpenStreetMap through the
    /// Overpass API and writes them, joined end to end, to Sources/pitlanes.
    /// </summary>
    public static async Task Fetch(IEnumerable<(string Id, PitSpec Spec)> circuits)
    {
        Directory.CreateDirectory(Folder);
        using var http = new HttpClient { Timeout = TimeSpan.FromMinutes(3) };
        http.DefaultRequestHeaders.UserAgent.ParseAdd("StintegyEVO-circuits/1 (+https://github.com/MoYuSOwO/stintegy-evo)");
        foreach ((string id, PitSpec spec) in circuits)
        {
            string query = $"[out:json][timeout:60];way(id:{string.Join(',', spec.Ways)});out geom;";
            string reply = await Overpass(http, query);
            using JsonDocument doc = JsonDocument.Parse(reply);
            var ways = new Dictionary<long, List<(double Lat, double Lon)>>();
            foreach (JsonElement way in doc.RootElement.GetProperty("elements").EnumerateArray())
                ways[way.GetProperty("id").GetInt64()] = way.GetProperty("geometry").EnumerateArray()
                    .Select(g => (g.GetProperty("lat").GetDouble(), g.GetProperty("lon").GetDouble())).ToList();
            List<(double Lat, double Lon)> chain = Join(spec.Ways.Select(w => ways.TryGetValue(w, out var g) ? g
                : throw new InvalidDataException($"{id}: OpenStreetMap has no way {w}.")).ToList(), id);

            var text = new StringBuilder();
            text.Append($"# {id} pit lane: OpenStreetMap ways {string.Join(", ", spec.Ways)}, joined end to end; (c) OpenStreetMap contributors, ODbL\n");
            text.Append("lat,lon\n");
            foreach ((double lat, double lon) in chain)
                text.Append(string.Create(CultureInfo.InvariantCulture, $"{lat:F7},{lon:F7}\n"));
            File.WriteAllText(Path.Combine(Folder, id + ".csv"), text.ToString());
            Console.WriteLine($"{id,-12} {chain.Count} points from {spec.Ways.Length} ways");
            await Task.Delay(2000);
        }
    }

    /// <summary>The public Overpass server turns calls away when it is busy; it is asked again, a little later each time.</summary>
    private static async Task<string> Overpass(HttpClient http, string query)
    {
        for (int attempt = 1; ; attempt++)
        {
            try
            {
                return await http.GetStringAsync("https://overpass-api.de/api/interpreter?data=" + Uri.EscapeDataString(query));
            }
            catch (HttpRequestException) when (attempt < 6)
            {
                await Task.Delay(TimeSpan.FromSeconds(10 * attempt));
            }
        }
    }

    /// <summary>Ways joined into one line where they share an end, each turned whichever way joins it.</summary>
    private static List<(double Lat, double Lon)> Join(List<List<(double Lat, double Lon)>> ways, string id)
    {
        var chain = new List<(double Lat, double Lon)>(ways[0]);
        var left = ways.Skip(1).ToList();
        while (left.Count > 0)
        {
            int before = left.Count;
            for (int w = 0; w < left.Count; w++)
            {
                var way = left[w];
                if (way[0] == chain[^1]) chain.AddRange(way.Skip(1));
                else if (way[^1] == chain[^1]) chain.AddRange(Enumerable.Reverse(way).Skip(1));
                else if (way[^1] == chain[0]) chain.InsertRange(0, way.Take(way.Count - 1));
                else if (way[0] == chain[0]) chain.InsertRange(0, Enumerable.Reverse(way).Take(way.Count - 1));
                else continue;
                left.RemoveAt(w);
                break;
            }
            if (left.Count == before)
                throw new InvalidDataException($"{id}: the pit lane's ways do not join end to end.");
        }
        return chain;
    }
}
