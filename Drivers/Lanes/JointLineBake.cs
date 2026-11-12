using System.Globalization;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>Offline export for the five-place joint line and speed search.</summary>
public static class JointLineBake
{
    public static string TableKey(string circuitId) => $"joint-five-v1:{circuitId}";

    /// <summary>
    /// The lattice's fastest way through a stretch, held to given places at
    /// given stations: from <paramref name="from"/> on the free line at
    /// <paramref name="startSpeed"/>, over <paramref name="rows"/> rows, back
    /// on the free line at the end, through each pin's offset (metres from
    /// the centreline) within <paramref name="toleranceMeters"/>. Offsets
    /// come back from the centreline, one per station. Null where no way
    /// passes every pin.
    /// </summary>
    public static (float[] Offset, float[] Speed, float Seconds)? PinnedLine(TrackData track, LaneSet lanes, ICar car,
        int from, int rows, float startSpeed, IReadOnlyList<(int Station, float Offset)> pins, float toleranceMeters = 1f,
        float beyondMeters = 200f)
    {
        var state = new CarState();
        var tyres = (ICarTires)car;
        tyres.InstallFreshTires(state);
        tyres.WarmTires(state, 95f, 95f);
        (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
        var envelope = LaneEnvelope.From(state, (ICarEnvelope)car, (ICarGripFeel)car, CarStrategy.Default)
            .WithRoadGrip(track.AsphaltGrip);
        var search = new LineSearch(lanes, envelope, new(), _ => 1f, (station, offset) =>
        {
            TrackSample sample = track.Sample(lanes.Wrap(station) * lanes.Spacing);
            return new LaneRoad(sample.Grade, sample.BankSlopeAt(offset), sample.VerticalRate);
        });
        // Each pin on the row nearest it, as metres from the free line there.
        var held = new Dictionary<int, float>();
        foreach (var (station, offset) in pins)
        {
            int row = (int)MathF.Round(lanes.Wrap(station - from) / (float)search.RowStations);
            if (row <= 0 || row >= rows)
                throw new ArgumentOutOfRangeException(nameof(pins), $"Station {station} is not inside the stretch.");
            int at = from + row * search.RowStations;
            held[row] = offset - lanes.Offset(LaneId.Free, at);
        }
        LineSearch.Way? way = search.Search(from, rows, startSpeed, v => beyondMeters / MathF.Max(v, 1f),
            (row, y) => !held.TryGetValue(row, out float want) || MathF.Abs(y - want) <= toleranceMeters + 1e-3f);
        if (way is null)
            return null;
        var offsets = new float[way.Offset.Length];
        for (int k = 0; k < offsets.Length; k++)
            offsets[k] = lanes.Offset(LaneId.Free, from + k) + way.Offset[k];
        return (offsets, way.Speed, way.Seconds);
    }

    public static long ExportGeometry(string circuitId, LaneSet lanes, string tablePath, string path,
        Action<int, long>? progress = null)
    {
        LineSearch.Table table = LineSearch.Table.Load(tablePath, TableKey(circuitId), readSteps: false)
            ?? throw new InvalidDataException($"No matching five-place table in '{tablePath}'.");
        return LineSearch.FixedGeometry.Bake(path, lanes, table, progress);
    }

    /// <summary>
    /// Whether the online search agrees with the table it reads: from every
    /// node of the table's own lap, with nobody else on the road, the online
    /// decision should be the table's next node, since the table's value is
    /// what the online search adds to its one step. Where it is not, the two
    /// are pricing the same step differently, and the online plan leaves the
    /// line the table was baked to. Probes only.
    /// </summary>
    public static string Consistency(string circuitId, TrackData track, LaneSet lanes, ICar car, string tablePath,
        bool exactSpeeds, bool approximateBraking)
    {
        LineSearch.Table table = LineSearch.Table.Load(tablePath, TableKey(circuitId), readSteps: false)
            ?? throw new InvalidDataException($"No matching five-place table in '{tablePath}'.");
        table.IndexOnlineArrivals();
        var state = new CarState();
        var tyres = (ICarTires)car;
        tyres.InstallFreshTires(state);
        tyres.WarmTires(state, 95f, 95f);
        (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
        var envelope = LaneEnvelope.From(state, (ICarEnvelope)car, (ICarGripFeel)car, CarStrategy.Default)
            .WithRoadGrip(track.AsphaltGrip);
        var search = new LineSearch(lanes, envelope, new(), _ => 1f, (station, offset) =>
        {
            TrackSample sample = track.Sample(lanes.Wrap(station) * lanes.Spacing);
            return new LaneRoad(sample.Grade, sample.BankSlopeAt(offset), sample.VerticalRate);
        }) { ExactSpeeds = exactSpeeds, ApproximateOnlineBraking = approximateBraking };

        // The lap the table drives from the seam at the speed it closes at.
        var (lapY, lapSlope, lapSpeed) = search.ClosedLap(table, 71f) ?? throw new InvalidOperationException("No closed lap.");
        var seed = search.FollowPath(table, 0, lapY, lapSlope, lapSpeed) ?? throw new InvalidOperationException("No lap from the seam.");
        var path = new List<(int Row, int State)>();
        foreach (var node in seed)
            if (node.Row < table.LapEnd)
                path.Add(node);
        int same = 0, sameEnd = 0, tried = 0;
        var placeMiss = new List<float>();
        var valueGap = new List<float>();
        var report = new System.Text.StringBuilder();
        foreach (var (row, node) in path)
        {
            var (y, slope, speed) = search.StateOf(table, row, node);
            var decided = search.DecideOnline(table, row, y, slope, speed);
            if (decided.Row < 0)
                continue;
            tried++;
            int nextRow = table.NextRow[row][node], next = table.Next[row][node];
            if (decided.Row == nextRow && decided.State == next)
                same++;
            var (wantY, _, _) = search.StateOf(table, nextRow, next);
            var (gotY, _, _) = search.StateOf(table, decided.Row, decided.State);
            // Where the two lines are a fixed distance on, so a different span is compared fairly.
            if (decided.Row == nextRow)
                sameEnd++;
            placeMiss.Add(MathF.Abs(gotY - wantY));
            valueGap.Add(decided.Seconds - table.Value[row][node]);
        }
        placeMiss.Sort();
        valueGap.Sort();
        float P(List<float> v, double q) => v.Count == 0 ? float.NaN : v[Math.Min(v.Count - 1, (int)(v.Count * q))];

        // How much the table's line ahead depends on the speed it is entered
        // at: the same place and heading, a bin or two faster or slower, and
        // the line the table leads on over the next two hundred metres.
        var bySpeed = new Dictionary<int, List<float>> { [1] = [], [2] = [], [4] = [] };
        var sameValue = new Dictionary<int, List<float>> { [1] = [], [2] = [], [4] = [] };
        foreach (var (row, node) in path)
        {
            var (y, slope, speed) = search.StateOf(table, row, node);
            var baseWay = search.Follow(table, row, y, slope, speed, maxStations: 100);
            if (baseWay is null)
                continue;
            foreach (int db in bySpeed.Keys)
                foreach (int sign in new[] { -1, 1 })
                {
                    float v = speed + sign * db * search.SpeedStep;
                    var other = search.Follow(table, row, y, slope, v, maxStations: 100);
                    if (other is null)
                        continue;
                    float worst = 0f;
                    for (int k = 0; k < Math.Min(baseWay.Offset.Length, other.Offset.Length); k++)
                        worst = MathF.Max(worst, MathF.Abs(baseWay.Offset[k] - other.Offset[k]));
                    bySpeed[db].Add(worst);
                }
        }
        foreach (var list in bySpeed.Values)
            list.Sort();

        // Where the five places sit across the road: how far the outermost
        // one on each side stops short of the edge the car may use, and how
        // far apart neighbours are.
        {
            var shortInside = new List<float>();
            var shortLeft = new List<float>();
            var shortRight = new List<float>();
            var gaps = new List<float>();
            var widest = new List<float>();
            int uneven = 0, rowsSeen = 0;
            for (int r = 0; r < table.LapEnd; r++)
            {
                int st = table.Starts[r];
                List<float> places = table.Places[r];
                if (places.Count < 2)
                    continue;
                rowsSeen++;
                float free = lanes.Offset(LaneId.Free, st);
                float edge = lanes.HalfWidth(st) - LaneSet.EdgeMarginMeters;
                // Offsets are from the free line; the road's usable span from it is [-(edge + free), edge - free].
                float lo = -(edge + free), hi = edge - free;
                float left = places[0] - lo, right = hi - places[^1];
                shortLeft.Add(left);
                shortRight.Add(right);
                // The inside of the bend here, by the free line's own curvature.
                float k = LaneSet.Curvature(
                    lanes.Position(st - 3, lanes.Offset(LaneId.Free, st - 3)),
                    lanes.Position(st, free),
                    lanes.Position(st + 3, lanes.Offset(LaneId.Free, st + 3)));
                if (MathF.Abs(k) > 0.008f)
                    shortInside.Add(k > 0f ? left : right);
                float most = 0f, least = float.PositiveInfinity;
                for (int p = 1; p < places.Count; p++)
                {
                    float g = places[p] - places[p - 1];
                    gaps.Add(g);
                    most = MathF.Max(most, g);
                    least = MathF.Min(least, g);
                }
                widest.Add(hi - lo);
                if (most > 1.6f * least)
                    uneven++;
            }
            foreach (var v in new[] { shortInside, shortLeft, shortRight, gaps, widest })
                v.Sort();
            report.AppendLine($"    places: {rowsSeen} rows; usable width p50 {P(widest, .5):F1} m");
            report.AppendLine($"      outermost place short of the usable edge: left p50 {P(shortLeft, .5):F2} max {P(shortLeft, 1):F2} m, " +
                $"right p50 {P(shortRight, .5):F2} max {P(shortRight, 1):F2} m");
            report.AppendLine($"      at the inside of a bend (R < 125 m, {shortInside.Count} rows): p50 {P(shortInside, .5):F2}  p90 {P(shortInside, .9):F2}  max {P(shortInside, 1):F2} m");
            report.AppendLine($"      neighbour spacing p05 {P(gaps, .05):F1}  p50 {P(gaps, .5):F1}  p95 {P(gaps, .95):F1} m; " +
                $"{100.0 * uneven / Math.Max(1, rowsSeen):F0} % of rows have one gap over 1.6x another");
        }

        // The table's lap and the free line, timed by the speed plan the car
        // drives on (PlanAhead), corner by corner: if the free line is the
        // quicker anywhere, the table is not finding the quickest line there.
        if (search.Follow(table, 0, lapY, lapSlope, lapSpeed) is { } lapWay)
        {
            int count = lanes.Count;
            // Two laps, timed on the second: the speed plan reads the line
            // past either end as the free line, and a lap that starts off it
            // -- the seam is wherever the table's lap closes -- would be read
            // as a step there and charged for a corner that is not on it.
            float[] tableLine = new float[2 * count + 1];
            float[] freeLine = new float[2 * count + 1];
            for (int k = 0; k < tableLine.Length; k++)
                tableLine[k] = lapWay.Offset[k % count];
            var (_, _, tableSpeed) = search.Plan(0, tableLine, lapSpeed, lapSpeed);
            var (_, _, freeSpeed) = search.Plan(0, freeLine, lapSpeed, lapSpeed);
            float Seconds(float[] line, float[] speed, int a, int b)
            {
                float s = 0f;
                for (int k = a; k < b; k++)
                    s += (lanes.Position(k + 1, lanes.Offset(LaneId.Free, k + 1) + line[k + 1]) -
                          lanes.Position(k, lanes.Offset(LaneId.Free, k) + line[k])).Length() /
                         MathF.Max(1f, 0.5f * (speed[k] + speed[k + 1]));
                return s;
            }
            float tableSeconds = Seconds(tableLine, tableSpeed, count, 2 * count);
            float freeSeconds = Seconds(freeLine, freeSpeed, count, 2 * count);
            report.AppendLine($"    lap by the driver's speed plan (second of two): table's line {tableSeconds:F3} s, free line {freeSeconds:F3} s");
            const int stretch = 50;
            var worse = new List<(int Station, float Lost, float Most)>();
            for (int from = 0; from + stretch <= count; from += stretch)
            {
                int a = count + from, b = count + from + stretch;
                float lost = Seconds(tableLine, tableSpeed, a, b) - Seconds(freeLine, freeSpeed, a, b);
                float most = 0f;
                for (int k = a; k < b; k++)
                    most = MathF.Max(most, MathF.Abs(tableLine[k]));
                worse.Add((from, lost, most));
            }
            foreach (var (station, lost, most) in worse.OrderByDescending(w => w.Lost).Take(8))
                report.AppendLine($"      stations {station,4}-{station + stretch,4}: table's line {lost * 1000f,6:F0} ms slower than the free line, up to {most:F1} m off it");
        }
        report.AppendLine($"  exact={exactSpeeds} approximateBraking={approximateBraking}: {tried} nodes on the table's lap");
        report.AppendLine($"    online picks the table's next node   {100.0 * same / Math.Max(1, tried):F1} %   same row {100.0 * sameEnd / Math.Max(1, tried):F1} %");
        report.AppendLine($"    |online end place - table's|  p50 {P(placeMiss, .5):F2}  p95 {P(placeMiss, .95):F2}  max {P(placeMiss, 1):F2} m");
        report.AppendLine($"    online total - table value    p05 {P(valueGap, .05):F3}  p50 {P(valueGap, .5):F3}  p95 {P(valueGap, .95):F3} s");
        foreach (var (db, list) in bySpeed)
            report.AppendLine($"    table's line over 200 m, entered {db * search.SpeedStep:F1} m/s off: max |diff| " +
                $"p50 {P(list, .5):F2}  p95 {P(list, .95):F2}  max {P(list, 1):F2} m   ({100.0 * list.Count(x => x > 1f) / Math.Max(1, list.Count):F0} % over a metre)");
        return report.ToString();
    }

    public static float Export(string circuitId, TrackData track, LaneSet lanes, ICar car, string folder,
        bool exact, float seamSpeed, Action<int>? progress = null)
    {
        var state = new CarState();
        var tyres = (ICarTires)car;
        tyres.InstallFreshTires(state);
        tyres.WarmTires(state, 95f, 95f);
        (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
        var envelope = LaneEnvelope.From(state, (ICarEnvelope)car, (ICarGripFeel)car, CarStrategy.Default)
            .WithRoadGrip(track.AsphaltGrip);
        var search = new LineSearch(lanes, envelope, new(), _ => 1f, (station, offset) =>
        {
            TrackSample sample = track.Sample(lanes.Wrap(station) * lanes.Spacing);
            return new LaneRoad(sample.Grade, sample.BankSlopeAt(offset), sample.VerticalRate);
        }) { ExactSpeeds = exact, RetainBakedSteps = false, Baked = progress };

        // The lap's end is its start. The first pass pins the seam state so
        // there is something to start from; after that each pass charges the
        // lap's end with what the next lap from it costs, which is what makes
        // the value function periodic. Pinning the seam speed alone -- one
        // pass, no next -- leaves the whole table believing the car must pass
        // the start line at that speed, and it is a number chosen to close
        // the lap, not a speed the road asks for.
        LineSearch.Table table = search.BakeLap(0, null, seamSpeed: seamSpeed);
        for (int pass = 0; pass < 2; pass++)
            table = search.BakeLap(0, table);
        Directory.CreateDirectory(folder);
        string stem = Path.Combine(folder, "five");
        // Kept before the lap is drawn: a lap that will not close is a
        // question about the table, and the table is what answers it.
        table.Save(stem + ".table", TableKey(circuitId));
        var (startY, startSlope, startSpeed) = search.ClosedLap(table, seamSpeed)
            ?? throw new InvalidOperationException("The table's lap does not come back to where it starts.");
        LineSearch.Way way = search.Follow(table, 0, startY, startSlope, startSpeed, maxStations: lanes.Count)
            ?? throw new InvalidOperationException($"No closed lap at seam speed {seamSpeed} m/s.");
        if (way.Offset[0] != way.Offset[^1])
            throw new InvalidOperationException("The exported lap does not close at the seam.");
        Console.WriteLine($"five: seam {startY:F2} m, {way.Speed[0]:F2} -> {way.Speed[^1]:F2} m/s");

        float seconds = 0f;
        using (var writer = new StreamWriter(stem + ".csv"))
        {
            writer.WriteLine("station,s_m,x_m,y_m,offset_m,speed_mps,time_s");
            var previous = lanes.Position(0, lanes.Offset(LaneId.Free, 0) + way.Offset[0]);
            for (int k = 0; k < way.Offset.Length; k++)
            {
                float offset = lanes.Offset(LaneId.Free, k) + way.Offset[k];
                var point = lanes.Position(k, offset);
                if (k > 0)
                    seconds += (point - previous).Length() / MathF.Max(1f, 0.5f * (way.Speed[k - 1] + way.Speed[k]));
                writer.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"{k},{k * lanes.Spacing:R},{point.X:R},{point.Y:R},{offset:R},{way.Speed[k]:R},{seconds:R}"));
                previous = point;
            }
        }
        return seconds;
    }
}
