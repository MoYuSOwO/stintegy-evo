using System.Globalization;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>Offline export for the five-place joint line and speed search.</summary>
public static class JointLineBake
{
    public static string TableKey(string circuitId) => $"joint-five-v1:{circuitId}";

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
        var seed = search.FollowPath(table, 0, 0f, 0f, 71f) ?? throw new InvalidOperationException("No lap from the seam.");
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
        LineSearch.Way way = search.Follow(table, 0, 0f, 0f, seamSpeed, maxStations: lanes.Count)
            ?? throw new InvalidOperationException($"No closed lap at seam speed {seamSpeed} m/s.");
        // The line has to close; the speed it closes at is the value function's
        // to say once the bake is periodic, and it need not be the one the
        // first pass pinned.
        if (way.Offset[0] != way.Offset[^1])
            throw new InvalidOperationException("The exported lap does not close at the seam.");
        Console.WriteLine($"five: seam {way.Speed[0]:F2} -> {way.Speed[^1]:F2} m/s");

        Directory.CreateDirectory(folder);
        string stem = Path.Combine(folder, "five");
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
        table.Save(stem + ".table", TableKey(circuitId));
        return seconds;
    }
}
