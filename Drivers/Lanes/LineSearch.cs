using System.Numerics;
using System.Runtime.InteropServices;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// The fastest way along a stretch of road, line and speed found together:
/// a lattice of places across the road every few stations, each with the way
/// the car is heading there and the speed it is doing, searched from one
/// end to the other. A step from one place to the next is a smooth curve,
/// and what it costs is the time the speed plan's own steps say it takes:
/// the corner the tyres hold, the braking and the drive the car has. Where
/// the plan sweeps a line it was given, this finds the line.
/// </summary>
internal sealed partial class LineSearch
{
    /// <summary>Stations from one row of the lattice to the next.</summary>
    public int RowStations { get; init; } = 5;

    /// <summary>Metres between places across the road.</summary>
    public float LateralStep { get; init; } = 1f;

    /// <summary>Keep packed edges for Decide. Follow and DecideOnline only need the state table.</summary>
    public bool RetainBakedSteps { get; init; } = true;

    /// <summary>The furthest across the road one step may take the car, for each of <see cref="Spans"/>.</summary>
    public float[] MaxLateralJumps { get; init; } = [3f, 4f, 6f, 8f];

    /// <summary>The ways the line may be heading at a row, as metres aside per metre along.</summary>
    public float[] Slopes { get; init; } = [-0.2f, -0.1f, -0.05f, -0.025f, 0f, 0.025f, 0.05f, 0.1f, 0.2f];

    /// <summary>
    /// How many rows one smooth step may cross. A row apart is too short to
    /// move across the road at speed: at 300 km/h a few metres take a
    /// hundred, and a lattice of short steps could only brake to make them.
    /// </summary>
    public int[] Spans { get; init; } = [1, 2, 4, 8];

    /// <summary>Metres a second between the speeds a row is searched at.</summary>
    public float SpeedStep { get; init; } = 0.5f;

    public float MinSpeed { get; init; } = 10f;

    /// <summary>
    /// The driver's turn lag at a speed, seconds, and the speed it guesses
    /// at each station: with both, a step corners on the sharpest of the road
    /// as far ahead as the lag covers, as the driver's own plan does
    /// (<see cref="LaneSpeedPlanner.LeadCurvatures"/>); without, on the road
    /// under the car.
    /// </summary>
    public Func<float, float>? TurnLag { get; init; }
    public Func<int, float>? LeadSpeed { get; init; }

    // How many stations the driver's lead reads ahead from a station: none without one.
    private int LeadStations(int station)
    {
        if (TurnLag is null || LeadSpeed is null)
            return 0;
        float v = LeadSpeed(station);
        return (int)MathF.Ceiling(MathF.Max(0f, v) * TurnLag(v) / _lanes.Spacing);
    }

    // The furthest the lead reads ahead anywhere round the lap: how far past
    // its end a step is joined to the way on after it, with the chord the
    // curvature there is read on (Beyond).
    private int LeadMax
    {
        get
        {
            if (_leadMax < 0)
            {
                int most = 0;
                for (int st = 0; st < _lanes.Count; st++)
                    most = Math.Max(most, LeadStations(st));
                _leadMax = most;
            }
            return _leadMax;
        }
    }
    private int _leadMax = -1;

    private int Beyond => LeadMax + Chord;

    /// <summary>
    /// Time a step as the driver's own plan drives it (<see cref="LaneSpeedPlanner.PlanAhead"/>):
    /// braking back from the speed it ends at, full drive on from the one it
    /// starts at, station by station on the corner the lead plans each on,
    /// and the seconds summed station by station; not as a steady change of
    /// v虏. A table baked so is for drawing a way, not for deciding on.
    /// </summary>
    public bool ExactSpeeds { get; init; }

    /// <summary>Told each row's number as the bake finishes it, from the end back.</summary>
    public Action<int>? Baked { get; init; }


    /// <summary>Why the last <see cref="DecideOnline"/> turned its ways down, for probes.</summary>
    public struct Rejects
    {
        public int TooFastHere, CannotBrake, Blocked, Timed;
    }

    /// <summary>Why the last <see cref="DecideOnline"/> turned its ways down, for probes.</summary>
    public Rejects LastRejects;

    /// <summary>The tail of a state no way runs on from (<see cref="Table.Tail"/>).</summary>
    public const ushort NoTail = ushort.MaxValue;

    private readonly LaneSet _lanes;
    private LaneEnvelope _envelope;

    /// <summary>
    /// The motor the car has now, where it is not the one the table was
    /// baked on (a push held, say): <see cref="DecideOnline"/> drives each
    /// step on it. Null, the table's own.
    /// </summary>
    internal LaneEnvelope? Motor { get; set; }

    /// <summary>
    /// A charge on a step <see cref="DecideOnline"/> weighs, given the step,
    /// its end speed, and per station the speed full drive reaches and the
    /// speed braking to that end allows.
    /// </summary>
    public delegate float StepCost(Move move, float v1, float[] driven, float[] braked);
    public delegate float PaceLimit(Move move, int station, float time, float speed);
    private LaneShares _shares;
    private readonly Func<int, float> _factor;
    private readonly Func<int, float, LaneRoad> _road;

    // The curvature is read on the same chord the driver reads it on.
    private const int Chord = 3;

    public LineSearch(LaneSet lanes, LaneEnvelope envelope, LaneShares shares, Func<int, float> factor, Func<int, float, LaneRoad> road)
    {
        _lanes = lanes;
        _envelope = envelope;
        _shares = shares;
        _factor = factor;
        _road = road;
        _roads = new LaneRoad[lanes.Count][];
    }

    public void UpdateOnlineEnvelope(LaneEnvelope envelope, LaneShares shares)
    {
        _envelope = envelope;
        _shares = shares;
    }

    // The road across each station, read once on a fine grid and
    // interpolated: a step is read at every station it crosses, and the
    // same stations are crossed by every step that passes.
    private const float RoadGrid = 0.5f;
    private readonly LaneRoad[]?[] _roads;

    private LaneRoad RoadAt(int station, float y)
    {
        int st = _lanes.Wrap(station);
        (float lo, float hi) = Room(st);
        LaneRoad[]? across = _roads[st];
        if (across is null)
        {
            across = new LaneRoad[(int)MathF.Ceiling((hi - lo) / RoadGrid) + 1];
            float free = _lanes.Offset(LaneId.Free, st);
            for (int i = 0; i < across.Length; i++)
                across[i] = _road(st, free + lo + i * RoadGrid);
            _roads[st] = across;
        }
        float at = Math.Clamp((y - lo) / RoadGrid, 0f, across.Length - 1);
        int a = Math.Min((int)at, across.Length - 1), b = Math.Min(a + 1, across.Length - 1);
        float t = at - a;
        LaneRoad ra = across[a], rb = across[b];
        return new LaneRoad(ra.Grade + (rb.Grade - ra.Grade) * t, ra.Bank + (rb.Bank - ra.Bank) * t, ra.Vertical + (rb.Vertical - ra.Vertical) * t);
    }

    /// <summary>A way found: the offset from the racing line and the speed at each station, and the time it takes.</summary>
    public sealed record Way(int From, float[] Offset, float[] Speed, float Seconds, float EndSpeed, float TerminalSeconds);

    /// <summary>
    /// The fastest way over <paramref name="rows"/> rows from
    /// <paramref name="from"/>, starting on the racing line at
    /// <paramref name="startSpeed"/> and ending back on it, with
    /// <paramref name="terminal"/> the seconds the road beyond costs from a
    /// speed (infinite where that speed cannot be carried on). A row may
    /// hold only the places <paramref name="allowed"/> lets it.
    /// </summary>
    public Way? Search(int from, int rows, float startSpeed, Func<float, float> terminal, Func<int, float, bool>? allowed = null)
    {
        float topSpeed = LaneSpeedPlanner.TopSpeed;
        int bins = (int)MathF.Ceiling((topSpeed - MinSpeed) / SpeedStep) + 1;
        var places = new List<float>[rows + 1];
        for (int r = 0; r <= rows; r++)
        {
            int st = from + r * RowStations;
            (float lo, float hi) = Room(st);
            var ys = new List<float>();
            bool edge = r == 0 || r == rows;
            for (float y = MathF.Ceiling(lo / LateralStep) * LateralStep; y <= hi + 1e-3f; y += LateralStep)
                if ((!edge || MathF.Abs(y) < 1e-3f) && (allowed is null || allowed(r, y) || MathF.Abs(y) < 1e-3f && edge))
                    ys.Add(y);
            places[r] = LateralPlaces(ys);
        }
        int slopes = Slopes.Length;
        int zeroSlope = Array.IndexOf(Slopes, 0f);

        // Seconds to reach each row at each place, heading and speed; and
        // where it was reached from.
        var cost = new float[rows + 1][];
        var parent = new int[rows + 1][];
        var parentRow = new int[rows + 1][];
        for (int r = 0; r <= rows; r++)
        {
            cost[r] = new float[places[r].Count * slopes * bins];
            parent[r] = new int[cost[r].Length];
            parentRow[r] = new int[cost[r].Length];
            Array.Fill(cost[r], float.PositiveInfinity);
            Array.Fill(parent[r], -1);
        }
        int startBin = Math.Clamp((int)MathF.Round((startSpeed - MinSpeed) / SpeedStep), 0, bins - 1);
        int startPlace = places[0].FindIndex(y => MathF.Abs(y) < 1e-3f);
        if (startPlace < 0)
            return null;
        cost[0][(startPlace * slopes + zeroSlope) * bins + startBin] = 0f;

        for (int r1 = 1; r1 <= rows; r1++)
        {
            List<float> there = places[r1];
            bool last = r1 == rows;
            float[] costThere = cost[r1];
            int[] parentThere = parent[r1], parentRowThere = parentRow[r1];
            int row1 = r1;
            Parallel.For(0, there.Count * slopes, target =>
            {
                int j = target / slopes, hj = target % slopes;
                if (last && hj != zeroSlope)
                    return;
                for (int sp = 0; sp < Spans.Length; sp++)
                {
                    int span = Spans[sp];
                    int r0 = row1 - span;
                    if (r0 < 0)
                        continue;
                    List<float> here = places[r0];
                    float[] costHere = cost[r0];
                    var edge = new Edge(span * RowStations);
                    for (int i = 0; i < here.Count; i++)
                    {
                        if (MathF.Abs(there[j] - here[i]) > MaxLateralJumps[sp] + 1e-3f)
                            continue;
                        for (int hi = 0; hi < slopes; hi++)
                        {
                            int source = (i * slopes + hi) * bins;
                            bool any = false;
                            for (int b = 0; b < bins && !any; b++)
                                any = costHere[source + b] < float.PositiveInfinity;
                            if (!any || !Shape(edge, from + r0 * RowStations, r0, here[i], Slopes[hi], there[j], Slopes[hj], allowed))
                                continue;
                            for (int b0 = 0; b0 < bins; b0++)
                            {
                                float c0 = costHere[source + b0];
                                if (c0 == float.PositiveInfinity)
                                    continue;
                                float v0 = MinSpeed + b0 * SpeedStep;
                                if (v0 > edge.Limit[0] + 1e-3f)
                                    continue;
                                (float vLow, float vHigh) = Reach(edge, v0);
                                int bLow = Math.Max(0, (int)MathF.Ceiling((vLow - MinSpeed) / SpeedStep));
                                int bHigh = Math.Min(bins - 1, (int)MathF.Floor((vHigh - MinSpeed) / SpeedStep));
                                for (int b1 = bLow; b1 <= bHigh; b1++)
                                {
                                    float v1 = MinSpeed + b1 * SpeedStep;
                                    float seconds = Seconds(edge, v0, v1) + Bend(edge);
                                    if (seconds == float.PositiveInfinity)
                                        continue;
                                    int to = target * bins + b1;
                                    float c = c0 + seconds;
                                    if (c < costThere[to])
                                    {
                                        costThere[to] = c;
                                        parentThere[to] = source + b0;
                                        parentRowThere[to] = r0;
                                    }
                                }
                            }
                        }
                    }
                }
            });
            // The last row is left whole: the road beyond may not carry its fastest arrivals.
            if (!last)
                Prune(costThere, there.Count * slopes, bins);
        }

        // The end: back on the racing line, heading along it, the road
        // beyond charged at the speed it is reached at.
        int endPlace = places[rows].FindIndex(y => MathF.Abs(y) < 1e-3f);
        if (endPlace < 0)
            return null;
        int best = -1;
        float bestCost = float.PositiveInfinity, bestTerminal = 0f;
        for (int b = 0; b < bins; b++)
        {
            int at = (endPlace * slopes + zeroSlope) * bins + b;
            float beyond = terminal(MinSpeed + b * SpeedStep);
            if (cost[rows][at] + beyond < bestCost)
            {
                bestCost = cost[rows][at] + beyond;
                best = at;
                bestTerminal = beyond;
            }
        }
        if (best < 0)
            return null;

        // Back from the end, step by step, and the stations between drawn as
        // the search drew them.
        var path = new List<(int Row, int State)> { (rows, best) };
        while (path[^1].Row > 0)
        {
            (int r, int state) = path[^1];
            path.Add((parentRow[r][state], parent[r][state]));
        }
        path.Reverse();
        int n = rows * RowStations + 1;
        var offset = new float[n];
        var speed = new float[n];
        for (int p = 0; p + 1 < path.Count; p++)
        {
            (int r0, int s0) = path[p];
            (int r1, int s1) = path[p + 1];
            (int i, int hi, int b0) = Split(s0, slopes, bins);
            (int j, int hj, int b1) = Split(s1, slopes, bins);
            var shape = new Edge((r1 - r0) * RowStations);
            Shape(shape, from + r0 * RowStations, r0, places[r0][i], Slopes[hi], places[r1][j], Slopes[hj], null);
            float v0 = MinSpeed + b0 * SpeedStep, v1 = MinSpeed + b1 * SpeedStep;
            for (int k = 0; k <= shape.N; k++)
            {
                offset[r0 * RowStations + k] = shape.Offset[k];
                speed[r0 * RowStations + k] = Along(v0, v1, shape, k);
            }
        }
        return new Way(from, offset, speed, bestCost - bestTerminal, MinSpeed + Split(best, slopes, bins).Bin * SpeedStep, bestTerminal);
    }

    /// <summary>
    /// What is baked: for every place, heading and speed on every row, the
    /// seconds the fastest way from there to the end takes, and the step
    /// that way starts with.
    /// </summary>
    public sealed class Table
    {
        public required int From { get; init; }
        public required int Rows { get; init; }

        /// <summary>Where the lap ends: the row whose station is the start of the next lap. <see cref="Rows"/> carries on past it, repeating the first rows, so a search that runs off the end comes round to the start.</summary>
        public int LapRows { get; init; } = -1;

        /// <summary>The row the lap ends at: <see cref="LapRows"/> where the table carries on past it, else the last row.</summary>
        public int LapEnd => LapRows > 0 ? LapRows : Rows;
        /// <summary>The station each row is at, unwrapped; a lap's last row is its first.</summary>
        public required int[] Starts { get; init; }
        public required List<float>[] Places { get; init; }
        public required int Bins { get; init; }
        public required float[][] Value { get; init; }
        public required int[][] NextRow { get; init; }
        public required int[][] Next { get; init; }

        /// <summary>
        /// For every state, which of its row's <see cref="Tails"/> the chosen
        /// way runs on from there, <see cref="NoTail"/> where none does: the
        /// line the driver's lead reads past the end of a step into it.
        /// </summary>
        public required ushort[][] Tail { get; init; }

        /// <summary>The lines a row's ways run on, as metres from the racing line at each station from the row on.</summary>
        public required List<float[]>[] Tails { get; init; }

        /// <summary>Every step out of every place and heading on a row, as the bake charged it.</summary>
        public required Steps[] Steps { get; init; }

        /// <summary>
        /// The same table with its first rows repeated past the end, their
        /// stations carried on by a lap. A search that runs off the last row
        /// then comes round to the start instead of stopping there, which is
        /// what it did at the lap's seam: near the finish every step but the
        /// shortest was skipped and the plan covered six stations where it
        /// covers eighty anywhere else, so the speed wanted fell away and the
        /// car braked for a corner that was not there.
        /// </summary>
        public Table Wrapped(int extra, int lapStations)
        {
            if (extra <= 0 || Rows <= extra)
                return this;
            int rows = Rows, grown = rows + extra;
            var starts = new int[grown + 1];
            Array.Copy(Starts, starts, rows + 1);
            for (int k = 1; k <= extra; k++)
                starts[rows + k] = Starts[k] + lapStations;
            var places = new List<float>[grown + 1];
            var value = new float[grown + 1][];
            var nextRow = new int[grown + 1][];
            var next = new int[grown + 1][];
            var tail = new ushort[grown + 1][];
            var tails = new List<float[]>[grown + 1];
            var steps = new Steps[grown + 1];
            Array.Copy(Places, places, rows + 1);
            Array.Copy(Value, value, rows + 1);
            Array.Copy(NextRow, nextRow, rows + 1);
            Array.Copy(Next, next, rows + 1);
            Array.Copy(Tail, tail, rows + 1);
            Array.Copy(Tails, tails, rows + 1);
            Array.Copy(Steps, steps, Math.Min(rows, Steps.Length));
            // The lap's last row is its first, so the way on from it is the
            // way on from row one, carried round by a lap: the stations keep
            // rising and a walk down the table still ends.
            if (NextRow[0].Length > 0)
            {
                var onward = new int[NextRow[0].Length];
                for (int s = 0; s < onward.Length; s++)
                    onward[s] = Math.Min(NextRow[0][s] + rows, grown);
                nextRow[rows] = onward;
                next[rows] = Next[0];
            }
            for (int k = 1; k <= extra; k++)
            {
                places[rows + k] = Places[k];
                value[rows + k] = Value[k];
                nextRow[rows + k] = NextRow[k];
                next[rows + k] = Next[k];
                tail[rows + k] = Tail[k];
                tails[rows + k] = Tails[k];
                steps[rows + k] = Steps[k];
            }
            return new Table
            {
                From = From, Rows = grown, LapRows = rows, Starts = starts, Places = places, Bins = Bins,
                Value = value, NextRow = nextRow, Next = next, Tail = tail, Tails = tails, Steps = steps
            };
        }

        internal readonly record struct ArrivalGroup(ushort Tail, int[] Bins, float MinimumRest);
        internal ArrivalGroup[][][]? OnlineArrivals { get; private set; }
        internal float[]? MinimumRowValue { get; private set; }

        // Index the finite terminal speeds by continuation before the race.
        // Keep the old first-seen tail order and ascending speed order.
        internal void IndexOnlineArrivals()
        {
            var rows = new ArrivalGroup[Rows + 1][][];
            var minimum = new float[Rows + 1];
            for (int row = 0; row <= Rows; row++)
            {
                float[] values = Value[row];
                minimum[row] = values.Length == 0 ? float.PositiveInfinity : values.Min();
                ushort[] tails = Tail[row];
                var targets = rows[row] = new ArrivalGroup[values.Length / Bins][];
                for (int target = 0; target < targets.Length; target++)
                {
                    var groups = new Dictionary<ushort, List<int>>();
                    int start = target * Bins;
                    for (int b = 0; b < Bins; b++)
                    {
                        if (!float.IsFinite(values[start + b]))
                            continue;
                        ushort tail = tails[start + b];
                        if (!groups.TryGetValue(tail, out var speeds))
                            groups.Add(tail, speeds = []);
                        speeds.Add(b);
                    }
                    targets[target] = groups.Select(pair =>
                        new ArrivalGroup(pair.Key, pair.Value.ToArray(), pair.Value.Min(b => values[start + b]))).ToArray();
                }
            }
            OnlineArrivals = rows;
            MinimumRowValue = minimum;
        }

        private const int FileVersion = 4;

        /// <summary>
        /// Keep the table in a file, under <paramref name="key"/>: what it was
        /// baked for (the circuit, the car, the grip, the lattice), so a file
        /// baked for something else is never read back as this.
        /// </summary>
        public void Save(string path, string key)
        {
            string temp = path + ".part";
            using (var stream = new FileStream(temp, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 20))
            using (var w = new BinaryWriter(stream))
            {
                w.Write(FileVersion);
                w.Write(key);
                w.Write(From);
                w.Write(Rows);
                w.Write(LapRows);
                w.Write(Bins);
                Ints(w, Starts);
                foreach (List<float> places in Places)
                    Floats(w, places.ToArray());
                foreach (float[] value in Value)
                    Floats(w, value);
                foreach (int[] row in NextRow)
                    Ints(w, row);
                foreach (int[] next in Next)
                    Ints(w, next);
                foreach (ushort[] tail in Tail)
                    Bytes(w, MemoryMarshal.AsBytes<ushort>(tail));
                foreach (List<float[]> tails in Tails)
                {
                    w.Write(tails.Count);
                    foreach (float[] line in tails)
                        Floats(w, line);
                }
                foreach (Steps s in Steps)
                {
                    bool made = s is not null;
                    w.Write(made);
                    if (!made)
                        continue;
                    Ints(w, s!.First);
                    Ints(w, s.ChainFirst);
                    Bytes(w, s.Rows);
                    Bytes(w, MemoryMarshal.AsBytes<ushort>(s.Target));
                    Bytes(w, MemoryMarshal.AsBytes<ushort>(s.Length));
                    Bytes(w, MemoryMarshal.AsBytes<ushort>(s.Bend));
                    Ints(w, s.TailFirst);
                    Bytes(w, MemoryMarshal.AsBytes<ushort>(s.Tails));
                    Bytes(w, s.Bottom);
                    Bytes(w, s.Top);
                    Bytes(w, s.Chains);
                    w.Write(s.Escaped.Count);
                    foreach ((int at, float v) in s.Escaped)
                    {
                        w.Write(at);
                        w.Write(v);
                    }
                }
            }
            File.Move(temp, path, overwrite: true);
        }

        /// <summary>The table kept in a file under <paramref name="key"/>; null if there is none, or it was baked for something else.</summary>
        public static Table? Load(string path, string key, bool readSteps = true)
        {
            if (!File.Exists(path))
                return null;
            using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 20);
            using var r = new BinaryReader(stream);
            if (r.ReadInt32() != FileVersion || r.ReadString() != key)
                return null;
            int from = r.ReadInt32(), rows = r.ReadInt32(), lapRows = r.ReadInt32(), bins = r.ReadInt32();
            int[] starts = ReadInts(r);
            var places = new List<float>[rows + 1];
            for (int i = 0; i <= rows; i++)
                places[i] = [.. ReadFloats(r)];
            var value = new float[rows + 1][];
            for (int i = 0; i <= rows; i++)
                value[i] = ReadFloats(r);
            var nextRow = new int[rows + 1][];
            for (int i = 0; i < nextRow.Length; i++)
                nextRow[i] = ReadInts(r);
            var next = new int[rows + 1][];
            for (int i = 0; i < next.Length; i++)
                next[i] = ReadInts(r);
            var tail = new ushort[rows + 1][];
            for (int i = 0; i <= rows; i++)
                tail[i] = MemoryMarshal.Cast<byte, ushort>(ReadBytes(r)).ToArray();
            var tails = new List<float[]>[rows + 1];
            for (int i = 0; i <= rows; i++)
            {
                int count = r.ReadInt32();
                tails[i] = new List<float[]>(count);
                for (int t = 0; t < count; t++)
                    tails[i].Add(ReadFloats(r));
            }
            var steps = new Steps[rows];
            for (int i = 0; readSteps && i < steps.Length; i++)
            {
                if (!r.ReadBoolean())
                    continue;
                int[] first = ReadInts(r), chainFirst = ReadInts(r);
                byte[] rowsOf = ReadBytes(r);
                ushort[] target = MemoryMarshal.Cast<byte, ushort>(ReadBytes(r)).ToArray();
                ushort[] length = MemoryMarshal.Cast<byte, ushort>(ReadBytes(r)).ToArray();
                ushort[] bend = MemoryMarshal.Cast<byte, ushort>(ReadBytes(r)).ToArray();
                int[] tailFirst = ReadInts(r);
                ushort[] stepTails = MemoryMarshal.Cast<byte, ushort>(ReadBytes(r)).ToArray();
                byte[] bottom = ReadBytes(r), top = ReadBytes(r), chains = ReadBytes(r);
                int escapes = r.ReadInt32();
                var escaped = new Dictionary<int, float>(escapes);
                for (int e = 0; e < escapes; e++)
                    escaped[r.ReadInt32()] = r.ReadSingle();
                steps[i] = new Steps
                {
                    First = first, ChainFirst = chainFirst, Rows = rowsOf, Target = target, Length = length, Bend = bend,
                    TailFirst = tailFirst, Tails = stepTails, Bottom = bottom, Top = top, Chains = chains, Escaped = escaped
                };
            }
            return new Table
            {
                From = from, Rows = rows, LapRows = lapRows, Starts = starts, Places = places, Bins = bins,
                Value = value, NextRow = nextRow, Next = next, Tail = tail, Tails = tails, Steps = steps
            };
        }

        private static void Bytes(BinaryWriter w, ReadOnlySpan<byte> bytes)
        {
            w.Write(bytes.Length);
            w.Write(bytes);
        }

        private static void Ints(BinaryWriter w, int[] values) => Bytes(w, MemoryMarshal.AsBytes<int>(values));

        private static void Floats(BinaryWriter w, float[] values) => Bytes(w, MemoryMarshal.AsBytes<float>(values));

        private static byte[] ReadBytes(BinaryReader r)
        {
            int n = r.ReadInt32();
            byte[] bytes = r.ReadBytes(n);
            if (bytes.Length != n)
                throw new EndOfStreamException("A baked table's file ends early.");
            return bytes;
        }

        private static int[] ReadInts(BinaryReader r) => MemoryMarshal.Cast<byte, int>(ReadBytes(r)).ToArray();

        private static float[] ReadFloats(BinaryReader r) => MemoryMarshal.Cast<byte, float>(ReadBytes(r)).ToArray();
    }

    /// <summary>
    /// The steps out of one row, all of them: where each ends, how long it
    /// is, and how slow and how fast it can end from each speed it can start
    /// at. The bake times every step off these and nothing else, so a
    /// decision read off them is the bake's own; none is left out, since the
    /// one left out may be the way past a car that the others brake for.
    /// </summary>
    public sealed class Steps
    {
        /// <summary>Where the steps out of each place and heading start in the lists below, one more than there are.</summary>
        public required int[] First { get; init; }
        /// <summary>Where the first of those steps' chains starts in <see cref="Chains"/>; the rest follow in order.</summary>
        public required int[] ChainFirst { get; init; }
        public required byte[] Rows { get; init; }
        /// <summary>The place and heading a step ends on, on its own row.</summary>
        public required ushort[] Target { get; init; }
        /// <summary>How long a step is, in <see cref="LengthUnit"/>s.</summary>
        public required ushort[] Length { get; init; }
        /// <summary>What a step's bending is charged, in <see cref="BendUnit"/>s (<see cref="BendSeconds"/>).</summary>
        public required ushort[] Bend { get; init; }
        /// <summary>
        /// The tails of the target row a step's chains were made joined to,
        /// from <see cref="TailFirst"/> on in <see cref="Tails"/>: a step is
        /// driven on into the way on from where it ends, and that way may
        /// differ with the speed it ends at. A step is kept once for each set
        /// of ways on that gives it the same chains, and ends only on the
        /// speeds whose way on is one of its set.
        /// </summary>
        public required int[] TailFirst { get; init; }
        public required ushort[] Tails { get; init; }
        /// <summary>The slowest and fastest speed bins a step can start at.</summary>
        public required byte[] Bottom { get; init; }
        public required byte[] Top { get; init; }
        /// <summary>
        /// Each step's chains, slowest and fastest end in pairs, every chain
        /// stride from the bottom bin to the top one, in <see cref="ChainUnit"/>s:
        /// the slow end as how far under the speed it starts at, which it
        /// never ends over, the fast end as the change either way. A fast
        /// end that no speed is held to is <see cref="NoSpeed"/>; what does
        /// not fit is <see cref="Escape"/> or <see cref="SlowEscape"/> here
        /// and kept in <see cref="Escaped"/> as it is, by its place in this list.
        /// </summary>
        public required byte[] Chains { get; init; }
        public required Dictionary<int, float> Escaped { get; init; }
    }

    /// <summary>
    /// A chain's resolution: a quarter of a speed bin's width is as fine as a
    /// float's for every step's time the bake reaches (measured on
    /// Brooklands), and a byte of them spans thirty metres a second each way.
    /// </summary>
    public const float ChainUnit = 0.25f;

    /// <summary>
    /// What a line's bending is charged, seconds for each metre it runs at a
    /// curvature of one per metre, squared: as fast is as fast, but of two
    /// ways about as quick the one bent the least is taken, the wheel turned
    /// the least (the minimum-curvature line's choice). Small enough that a
    /// metre or four across a straight costs a fraction of a millisecond,
    /// and a way any quicker than that is still the one taken.
    /// </summary>
    public float BendSeconds { get; init; } = 0.1f;

    /// <summary>A step's bending charge's resolution: ten microseconds.</summary>
    public const float BendUnit = 1e-5f;

    /// <summary>A step's length's resolution: two millimetres.</summary>
    public const float LengthUnit = 1f / 512f;

    /// <summary>The stored chain number for a fast end that no speed is held to (<see cref="Held"/>).</summary>
    public const sbyte NoSpeed = sbyte.MinValue;

    /// <summary>The stored chain number for one kept apart in <see cref="Steps.Escaped"/>.</summary>
    public const sbyte Escape = sbyte.MinValue + 1;

    /// <summary>The same for a slow end.</summary>
    public const byte SlowEscape = byte.MaxValue;

    // One step's record, while the bake makes it.
    private readonly record struct Made(byte Rows, ushort Target, ushort Length, ushort Bend, ushort[] Tails, byte Bottom, byte Top, float[] Chains);

    /// <summary>
    /// Bake the stretch of <paramref name="rows"/> rows from
    /// <paramref name="from"/> from its end back: it ends on the racing line,
    /// the road beyond charged by <paramref name="terminal"/>. Speeds below
    /// <paramref name="slowest"/> at a station are not baked there: no car
    /// racing comes by that slow, and one that does is rejoining, not racing.
    /// </summary>
    public Table Bake(int from, int rows, Func<float, float> terminal, Func<int, float>? slowest = null)
    {
        int zero = Array.IndexOf(Slopes, 0f);
        var starts = new int[rows + 1];
        for (int r = 0; r <= rows; r++)
            starts[r] = from + r * RowStations;
        return Bake(from, starts, lap: false, (y, h, v) => MathF.Abs(y) < 1e-3f && h == zero ? terminal(v) : float.PositiveInfinity, slowest);
    }

    /// <summary>
    /// Bake a whole lap from <paramref name="from"/> round to itself, the
    /// last row one station longer or more where the lap's stations do not
    /// share out evenly. The lap's end is its start: <paramref name="next"/>
    /// charges each place, heading and speed there what the next lap from
    /// it costs, which is what the last bake gave its first row, or nothing
    /// on the first bake.
    /// </summary>
    public Table BakeLap(int from, Table? next, Func<int, float>? slowest = null, float? seamSpeed = null)
    {
        int rows = _lanes.Count / RowStations;
        var starts = new int[rows + 1];
        for (int r = 0; r < rows; r++)
            starts[r] = from + r * RowStations;
        starts[rows] = from + _lanes.Count;
        int slopes = Slopes.Length;
        Func<float, int, float, float> terminal = (y, h, v) => 0f;
        if (seamSpeed is float seam)
        {
            int seamBin = (int)MathF.Round((seam - MinSpeed) / SpeedStep);
            int straight = Array.IndexOf(Slopes, 0f);
            terminal = (y, h, v) => y == 0f && h == straight &&
                (int)MathF.Round((v - MinSpeed) / SpeedStep) == seamBin ? 0f : float.PositiveInfinity;
        }
        else if (next is not null)
        {
            terminal = (y, h, v) =>
            {
                int place = next.Places[0].FindIndex(p => MathF.Abs(p - y) < 1e-3f);
                int bin = (int)MathF.Round((v - MinSpeed) / SpeedStep);
                return place < 0 ? float.PositiveInfinity : next.Value[0][(place * slopes + h) * next.Bins + bin];
            };
        }
        return Bake(from, starts, lap: true, terminal, slowest, next).Wrapped(Spans[^1], _lanes.Count);
    }

    // The way on past the lap's end is the next lap's: the ways its first
    // row runs on, where it has been baked.
    private Table Bake(int from, int[] starts, bool lap, Func<float, int, float, float> terminal, Func<int, float>? slowest,
        Table? next = null)
    {
        int rows = starts.Length - 1;
        int widest = 0;
        for (int r = 0; r < rows; r++)
            widest = Math.Max(widest, starts[r + 1] - starts[r]);
        int bins = (int)MathF.Ceiling((LaneSpeedPlanner.TopSpeed - MinSpeed) / SpeedStep) + 1;
        if (bins > byte.MaxValue + 1)
            throw new InvalidOperationException($"{bins} speed bins do not fit a byte.");
        int slopes = Slopes.Length;
        var places = new List<float>[rows + 1];
        for (int r = 0; r <= rows; r++)
        {
            (float lo, float hi) = Room(starts[r]);
            var ys = new List<float>();
            for (float y = MathF.Ceiling(lo / LateralStep) * LateralStep; y <= hi + 1e-3f; y += LateralStep)
                if (r < rows || lap || MathF.Abs(y) < 1e-3f)
                    ys.Add(y);
            places[r] = LateralPlaces(ys);
        }
        var value = new float[rows + 1][];
        var nextRow = new int[rows + 1][];
        var nextState = new int[rows + 1][];
        var tail = new ushort[rows + 1][];
        var tails = new List<float[]>[rows + 1];
        for (int r = 0; r <= rows; r++)
        {
            value[r] = new float[places[r].Count * slopes * bins];
            nextRow[r] = new int[value[r].Length];
            nextState[r] = new int[value[r].Length];
            tail[r] = new ushort[value[r].Length];
            Array.Fill(value[r], float.PositiveInfinity);
            Array.Fill(nextState[r], -1);
            Array.Fill(tail[r], NoTail);
        }
        tails[rows] = [];
        if (next is not null)
        {
            tails[rows] = next.Tails[0];
            for (int p = 0; p < places[rows].Count; p++)
            {
                int q = next.Places[0].FindIndex(y => MathF.Abs(y - places[rows][p]) < 1e-3f);
                if (q < 0)
                    continue;
                for (int h = 0; h < slopes; h++)
                    for (int b = 0; b < bins; b++)
                        tail[rows][(p * slopes + h) * bins + b] = next.Tail[0][(q * slopes + h) * next.Bins + b];
            }
        }
        for (int p = 0; p < places[rows].Count; p++)
            for (int h = 0; h < slopes; h++)
                for (int b = 0; b < bins; b++)
                    value[rows][(p * slopes + h) * bins + b] = terminal(places[rows][p], h, MinSpeed + b * SpeedStep);

        var steps = new Steps[rows];
        for (int r0 = rows - 1; r0 >= 0; r0--)
        {
            int row0 = r0;
            List<float> here = places[r0];
            var made = new List<Made>[here.Count * slopes];
            Parallel.For(0, here.Count * slopes, source =>
            {
                int i = source / slopes, hi = source % slopes;
                float[] best = value[row0];
                var mine = made[source] = new List<Made>();
                var ways = new List<ushort>();
                var groups = new List<(int Bottom, int Top, float[] Chains, List<ushort> Tails)>();
                var wayGroup = new int[bins];
                var binGroup = new int[bins];
                int most = Spans[^1] * widest + 1;
                var driven = new float[most];
                var braked = new float[bins][];
                for (int b = 0; b < bins; b++)
                    braked[b] = new float[most];
                var brakedMade = new bool[bins];
                for (int sp = 0; sp < Spans.Length; sp++)
                {
                    int r1 = row0 + Spans[sp];
                    if (r1 > rows)
                        continue;
                    List<float> there = places[r1];
                    float[] later = value[r1];
                    ushort[] tailThere = tail[r1];
                    List<float[]> tailsThere = tails[r1];
                    var edge = new Edge(Spans[sp] * widest, LeadMax + 1) { N = starts[r1] - starts[row0] };
                    for (int j = 0; j < there.Count; j++)
                    {
                        if (MathF.Abs(there[j] - here[i]) > MaxLateralJumps[sp] + 1e-3f)
                            continue;
                        for (int hj = 0; hj < slopes; hj++)
                        {
                            int target = (j * slopes + hj) * bins;
                            ways.Clear();
                            for (int b = 0; b < bins; b++)
                                if (later[target + b] < float.PositiveInfinity && !ways.Contains(tailThere[target + b]))
                                    ways.Add(tailThere[target + b]);
                            if (ways.Count == 0 || !Shape(edge, starts[row0], row0, here[i], Slopes[hi], there[j], Slopes[hj], null))
                                continue;
                            // Timed on the length it is stored at.
                            ushort length = (ushort)MathF.Round(edge.Length / LengthUnit);
                            edge.Length = length * LengthUnit;
                            // Charged on the bend it is stored at, its own curve's.
                            ushort bend = (ushort)Math.Min(ushort.MaxValue, MathF.Round(Bend(edge) / BendUnit));
                            float bent = bend * BendUnit;
                            if (ExactSpeeds)
                            {
                                for (int w = 0; w < ways.Count; w++)
                                {
                                    ushort way = ways[w];
                                    Join(edge, starts[row0], way == NoTail ? null : tailsThere[way]);
                                    int n = edge.N;
                                    int top = Math.Min(bins - 1, (int)MathF.Floor((edge.Limit[0] + 1e-3f - MinSpeed) / SpeedStep));
                                    if (top < 0)
                                        continue;
                                    int bottom = slowest is null ? 0 : Math.Clamp((int)MathF.Ceiling((slowest(starts[row0]) - MinSpeed) / SpeedStep), 0, top);
                                    if (RetainBakedSteps)
                                        mine.Add(new Made((byte)Spans[sp], (ushort)(j * slopes + hj), length, bend, [way], (byte)bottom, (byte)top, []));
                                    Array.Clear(brakedMade);
                                    for (int b0 = bottom; b0 <= top; b0++)
                                    {
                                        float v0 = MinSpeed + b0 * SpeedStep;
                                        DriveOn(edge, v0, driven);
                                        int at = source * bins + b0;
                                        for (int b1 = 0; b1 < bins; b1++)
                                        {
                                            float v1 = MinSpeed + b1 * SpeedStep;
                                            if (v1 > driven[n] || v1 > edge.Limit[n])
                                                break;
                                            float rest = later[target + b1];
                                            if (rest == float.PositiveInfinity || tailThere[target + b1] != way)
                                                continue;
                                            if (!brakedMade[b1])
                                            {
                                                BrakeBack(edge, v1, braked[b1]);
                                                brakedMade[b1] = true;
                                            }
                                            float[] back = braked[b1];
                                            // Too fast to brake down to it in time.
                                            if (back[0] < v0)
                                                continue;
                                            float seconds = bent;
                                            for (int k = 0; k < n; k++)
                                                seconds += edge.Step[k] / MathF.Max(1f, 0.5f * (MathF.Min(driven[k], back[k]) + MathF.Min(driven[k + 1], back[k + 1])));
                                            if (seconds + rest < best[at])
                                            {
                                                best[at] = seconds + rest;
                                                nextRow[row0][at] = r1;
                                                nextState[row0][at] = target + b1;
                                            }
                                        }
                                    }
                                }
                                continue;
                            }
                            // Each way on, its chains from the station the lead first reads it on;
                            // the stations before are every way's, and driven once.
                            groups.Clear();
                            float[]? front = null;
                            int frontBottom = -1, frontTop = -1;
                            for (int w = 0; w < ways.Count; w++)
                            {
                                Join(edge, starts[row0], ways[w] == NoTail ? null : tailsThere[ways[w]]);
                                wayGroup[w] = -1;
                                int top = Math.Min(bins - 1, (int)MathF.Floor((edge.Limit[0] + 1e-3f - MinSpeed) / SpeedStep));
                                if (top < 0)
                                    continue;
                                int bottom = slowest is null ? 0 : Math.Clamp((int)MathF.Ceiling((slowest(starts[row0]) - MinSpeed) / SpeedStep), 0, top);
                                int split = ways[w] == NoTail ? 0 : Split(edge, starts[row0]);
                                if (split > 0 && (front is null || frontBottom != bottom || frontTop != top))
                                    (front, frontBottom, frontTop) = (Front(edge, bottom, top, split), bottom, top);
                                float[] chains = AllChains(edge, bottom, top, split > 0 ? split : 0, split > 0 ? front : null);
                                int g = groups.FindIndex(x => x.Bottom == bottom && x.Top == top && x.Chains.AsSpan().SequenceEqual(chains));
                                if (g < 0)
                                {
                                    g = groups.Count;
                                    groups.Add((bottom, top, chains, new List<ushort>()));
                                }
                                groups[g].Tails.Add(ways[w]);
                                wayGroup[w] = g;
                            }
                            for (int b = 0; b < bins; b++)
                                binGroup[b] = later[target + b] == float.PositiveInfinity ? -1 : wayGroup[ways.IndexOf(tailThere[target + b])];
                            for (int g = 0; g < groups.Count; g++)
                            {
                                (int bottom, int top, float[] chains, List<ushort> tailsOf) = groups[g];
                                if (RetainBakedSteps)
                                    mine.Add(new Made((byte)Spans[sp], (ushort)(j * slopes + hj), length, bend, [.. tailsOf], (byte)bottom, (byte)top, chains));
                                for (int b0 = bottom; b0 <= top; b0++)
                                {
                                    float v0 = MinSpeed + b0 * SpeedStep;
                                    (float vLow, float vHigh) = ChainAt(chains, bottom, top, b0);
                                    int bLow = Math.Max(0, (int)MathF.Ceiling((vLow - MinSpeed) / SpeedStep));
                                    int bHigh = Math.Min(bins - 1, (int)MathF.Floor((vHigh - MinSpeed) / SpeedStep));
                                    int at = source * bins + b0;
                                    for (int b1 = bLow; b1 <= bHigh; b1++)
                                    {
                                        if (binGroup[b1] != g)
                                            continue;
                                        float rest = later[target + b1];
                                        float v1 = MinSpeed + b1 * SpeedStep;
                                        float seconds = Seconds(edge, v0, v1) + bent;
                                        if (seconds + rest < best[at])
                                        {
                                            best[at] = seconds + rest;
                                            nextRow[row0][at] = r1;
                                            nextState[row0][at] = target + b1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            });
            if (RetainBakedSteps)
                steps[row0] = Pack(made);
            MakeTails(row0, starts, places, bins, nextRow, nextState, tail, tails);
            Baked?.Invoke(row0);
        }
        return new Table
        {
            From = from, Rows = rows, LapRows = -1, Starts = starts, Places = places, Bins = bins,
            Value = value, NextRow = nextRow, Next = nextState, Tail = tail, Tails = tails, Steps = steps
        };
    }

    // The lines a row's states' chosen ways run on, each as far as the lead
    // reads past the end of a step into the row; the same line kept once.
    private void MakeTails(int r, int[] starts, List<float>[] places, int bins, int[][] nextRow, int[][] next, ushort[][] tail, List<float[]>[] tails)
    {
        int slopes = Slopes.Length, beyond = Beyond;
        var ids = new Dictionary<long, ushort>();
        var mine = tails[r] = [];
        ushort[] of = tail[r];
        for (int s = 0; s < of.Length; s++)
        {
            int s2 = next[r][s];
            if (s2 < 0)
                continue;
            int r2 = nextRow[r][s];
            int n = starts[r2] - starts[r];
            ushort on = n < beyond ? tail[r2][s2] : NoTail;
            long key = s / bins | (long)(s2 / bins) << 16 | (long)(r2 - r) << 32 | (long)on << 40;
            if (!ids.TryGetValue(key, out ushort id))
            {
                if (mine.Count >= NoTail)
                    throw new InvalidOperationException($"Row {r} runs on more ways than a tail number holds.");
                (int i, int hi, _) = Split(s, slopes, bins);
                (int j, int hj, _) = Split(s2, slopes, bins);
                float[]? after = on == NoTail ? null : tails[r2][on];
                // Where the way on is not known, the line ends with the step.
                var line = new float[Math.Min(beyond, n + (after is null ? 0 : after.Length - 1)) + 1];
                for (int k = 0; k < line.Length; k++)
                    line[k] = k <= n ? Hermite(places[r][i], Slopes[hi], places[r2][j], Slopes[hj], n, k) : after![k - n];
                id = (ushort)mine.Count;
                mine.Add(line);
                ids[key] = id;
            }
            of[s] = id;
        }
    }

    /// <summary>The way the baked table leads from a place, heading and speed on a row to the end.</summary>
    public Way? Follow(Table table, int row, float y, float slope, float speed, int maxStations = int.MaxValue) =>
        FollowPath(table, row, y, slope, speed, maxStations) is { } path
            ? Draw(table, path, table.Value[path[0].Row][path[0].State])
            : null;

    /// <summary>The rows and states the baked table leads through from a place, heading and speed on a row.</summary>
    public List<(int Row, int State)>? FollowPath(Table table, int row, int state, int maxStations = int.MaxValue)
    {
        if (table.Value[row][state] == float.PositiveInfinity)
            return null;
        var path = new List<(int Row, int State)> { (row, state) };
        while (path[^1].Row < table.LapEnd && table.Starts[path[^1].Row] - table.Starts[row] < maxStations)
        {
            (int r, int s0) = path[^1];
            path.Add((table.NextRow[r][s0], table.Next[r][s0]));
        }
        return path;
    }

    /// <summary>The way along rows and states already chosen, drawn as <see cref="Follow"/> draws the table's own.</summary>
    public Way DrawPath(Table table, List<(int Row, int State)> path) =>
        Draw(table, path, table.Value[path[0].Row][path[0].State]);

    /// <summary>The rows and states the baked table leads through from a place, heading and speed on a row.</summary>
    public List<(int Row, int State)>? FollowPath(Table table, int row, float y, float slope, float speed, int maxStations = int.MaxValue)
    {
        int slopes = Slopes.Length;
        int place = table.Places[row].FindIndex(p => MathF.Abs(p - y) < 1e-3f);
        int heading = Array.IndexOf(Slopes, slope);
        if (place < 0 || heading < 0)
            return null;
        int state = (place * slopes + heading) * table.Bins + Math.Clamp((int)MathF.Round((speed - MinSpeed) / SpeedStep), 0, table.Bins - 1);
        float total = table.Value[row][state];
        if (total == float.PositiveInfinity)
            return null;
        var path = new List<(int Row, int State)> { (row, state) };
        while (path[^1].Row < table.LapEnd && table.Starts[path[^1].Row] - table.Starts[row] < maxStations)
        {
            (int r, int s0) = path[^1];
            path.Add((table.NextRow[r][s0], table.Next[r][s0]));
        }
        return path;
    }

    private Way Draw(Table table, List<(int Row, int State)> path, float total)
    {
        int slopes = Slopes.Length;
        int first = path[0].Row;
        int n = table.Starts[path[^1].Row] - table.Starts[first] + 1;
        var offset = new float[n];
        var speed = new float[n];
        for (int p = 0; p + 1 < path.Count; p++)
        {
            (int r0, int s0) = path[p];
            (int r1, int s1) = path[p + 1];
            (int i, int hi, int b0) = Split(s0, slopes, table.Bins);
            (int j, int hj, int b1) = Split(s1, slopes, table.Bins);
            var shape = new Edge(table.Starts[r1] - table.Starts[r0], LeadMax + 1);
            Shape(shape, table.Starts[r0], r0, table.Places[r0][i], Slopes[hi], table.Places[r1][j], Slopes[hj], null);
            float v0 = MinSpeed + b0 * SpeedStep, v1 = MinSpeed + b1 * SpeedStep;
            int at = table.Starts[r0] - table.Starts[first];
            float[]? planned = null;
            if (ExactSpeeds)
            {
                ushort way = table.Tail[r1][s1];
                Join(shape, table.Starts[r0], way == NoTail ? null : table.Tails[r1][way]);
                int m = shape.N + 1;
                planned = new float[m];
                var ones = new float[m];
                Array.Fill(ones, 1f);
                LaneSpeedPlanner.PlanAhead(_envelope, shape.Corner.AsSpan(0, m), shape.Step.AsSpan(0, m), shape.Factor.AsSpan(0, m), ones, shape.Road.AsSpan(0, m), _shares, v0, v1, planned);
            }
            for (int k = 0; k <= shape.N; k++)
            {
                offset[at + k] = shape.Offset[k];
                speed[at + k] = planned is null ? Along(v0, v1, shape, k) : planned[k];
            }
        }
        float endSpeed = MinSpeed + Split(path[^1].State, slopes, table.Bins).Bin * SpeedStep;
        float terminal = table.Value[path[^1].Row][path[^1].State];
        return new Way(table.Starts[first], offset, speed, total - terminal, endSpeed, terminal);
    }

    /// <summary>
    /// A step of the lattice: from a place and heading on one row to a place
    /// and heading on a later one.
    /// </summary>
    public readonly record struct Move(int Row, int ToRow, int Stations, float Y, float Slope, float ToY, float ToSlope);

    public (float[] Offset, float[] Speed)? MaterializeMove(Table table, int row, float y, float slope,
        float speed, int endRow, int endState, StepCost? cost = null, PaceLimit? pace = null, int? startStation = null,
        float[]? nextOffsets = null)
    {
        if (endRow <= row || endState < 0)
            return null;
        var (endY, endSlope, endSpeed) = StateOf(table, endRow, endState);
        int start = startStation ?? table.Starts[row];
        var edge = new Edge(table.Starts[endRow] - start, LeadMax + 1);
        if (!Shape(edge, start, row, y, slope, endY, endSlope, null))
            return null;
        ushort tail = table.Tail[endRow][endState];
        Join(edge, start, nextOffsets ?? (tail == NoTail ? null : table.Tails[endRow][tail]));
        var driven = new float[edge.N + 1];
        var braked = new float[edge.N + 1];
        var move = new Move(row, endRow, edge.N, y, slope, endY, endSlope);
        DriveOn(edge, speed, driven, move: move, pace: pace);
        BrakeBack(edge, endSpeed, braked);
        if (speed > braked[0] + 1e-3f || endSpeed > driven[^1] + 1e-3f ||
            (cost is not null && !float.IsFinite(cost(
                move, endSpeed, driven, braked))))
            return null;
        for (int k = 0; k < driven.Length; k++)
            driven[k] = MathF.Min(driven[k], braked[k]);
        return (edge.Offset, driven);
    }

    /// <summary>Where a move has the car across the road, as metres from the racing line, a station into it.</summary>
    public float OffsetOn(Move move, int station) =>
        Hermite(move.Y, move.Slope, move.ToY, move.ToSlope, move.Stations, station);

    private float Hermite(float y0, float h0, float y1, float h1, int n, int k)
    {
        float length = n * _lanes.Spacing;
        float t = k / (float)n;
        float t2 = t * t, t3 = t2 * t;
        return (2f * t3 - 3f * t2 + 1f) * y0 + (t3 - 2f * t2 + t) * length * h0 +
               (-2f * t3 + 3f * t2) * y1 + (t3 - t2) * length * h1;
    }

    /// <summary>
    /// The step to take now, from a car at a place and heading on a row and
    /// at any speed: every step out of there shaped and driven here from the
    /// car's own speed, each charged its own seconds and the baked rest of
    /// the way; and <paramref name="cost"/>, what the cars around add to a
    /// step (infinite where it runs into one). Timed as the table was baked,
    /// the driver's way or not. The seconds, the row it ends on, and the
    /// state there.
    /// </summary>
    /// <summary>Ticks spent in each part of <see cref="DecideOnline"/>, and how often: shape, join, drive, brake, time.</summary>
    public readonly long[] OnlineTicks = new long[6];
    /// <summary>Join each way on only from the station it first changes (<see cref="DecideOnline"/>); off, every station again.</summary>
    public bool ShareJoin { get; set; } = true;
    public bool ApproximateOnlineBraking { get; init; }
    internal FixedGeometry? Geometry { get; init; }
    public readonly long[] OnlineCalls = new long[6];
    /// <summary>Ticks spent laying the geometry and fitting it to the car: the two halves of a shape.</summary>
    public readonly long[] ShapeTicks = new long[2];
    /// <summary>Shapes and joins that came out of the precomputed geometry file.</summary>
    public long GeometryShapeHits;
    public long GeometryJoinHits;
    /// <summary>Decisions taken, successors walked, and nodes that had to rebuild their arrival groups.</summary>
    public long DecideCalls;
    public long LoopIterations;
    public long SlowWayNodes;

    public (float Seconds, int Row, int State) DecideOnline(Table table, int row, float y, float slope, float speed,
        Func<Move, float, float, float>? extra = null, StepCost? cost = null,
        int minimumEndStation = 0, bool nonnegativeCost = false, Func<Move, bool>? allowedMove = null,
        PaceLimit? pace = null, int? startStation = null, (float Seconds, int Row, int State)[]? beam = null,
        Func<int, int, float, float>? terminal = null,
        float maximumTotal = float.PositiveInfinity, float prefixTimeLower = 0f,
        Func<int, int, float>? preference = null, IReadOnlyList<(int Row, int Node)>? alsoEnd = null)
    {
        long mark = System.Diagnostics.Stopwatch.GetTimestamp();
        DecideCalls++;
        LastRejects = default;
        void Lap(int part)
        {
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            OnlineTicks[part] += now - mark;
            OnlineCalls[part]++;
            mark = now;
        }
        int slopes = Slopes.Length, bins = table.Bins;
        float bestSeconds = float.PositiveInfinity;
        int bestRow = -1, bestState = -1;
        if (beam is not null)
            Array.Fill(beam, (float.PositiveInfinity, -1, -1));
        float Bound() => beam is { Length: > 0 } ? beam[^1].Seconds : bestSeconds;
        int place = table.Places[row].FindIndex(p => MathF.Abs(p - y) < 1e-3f);
        int heading = Array.IndexOf(Slopes, slope);
        if (row >= table.LapEnd || (startStation is null && (place < 0 || heading < 0)))
            return (bestSeconds, bestRow, bestState);
        int st0 = startStation ?? table.Starts[row];
        bool fixedStart = st0 == table.Starts[row] && place >= 0 && heading >= 0 && y == table.Places[row][place];
        var ways = new List<ushort>();
        int most = table.Starts[Math.Min(table.Rows, row + Spans[^1])] - st0 + 1;
        var driven = new float[most];
        var driveTimes = pace is null ? null : new float[most];
        var back = new float[most];
        var brakeBudget = new float[most];
        var liveLimits = new Dictionary<(int Station, float Corner), float>();
        bool bounded = float.IsFinite(maximumTotal);
        Vector2 startPosition = bounded ? _lanes.Position(st0, _lanes.Offset(LaneId.Free, st0) + y) : default;
        // The spans, and then any place and heading asked for by row whatever
        // its distance: the way a plan already chose runs through rows a span
        // from here rarely lands on, so without them the search could not end
        // on it and had to choose afresh every time.
        int extraEnds = alsoEnd?.Count ?? 0;
        for (int e = 0; e < Spans.Length + extraEnds; e++)
        {
            int sp, r1, only = -1;
            if (e < Spans.Length)
                (sp, r1) = (e, row + Spans[e]);
            else
            {
                (r1, only) = alsoEnd![e - Spans.Length];
                int reach = r1 - row;
                if (reach <= 0 || reach > Spans[^1] || Array.IndexOf(Spans, reach) >= 0)
                    continue;
                sp = 0;
                while (Spans[sp] < reach)
                    sp++;
            }
            if (r1 > table.Rows || table.Starts[r1] < minimumEndStation)
                continue;
            int stations = table.Starts[r1] - st0;
            var edge = new Edge(stations, LeadMax + 1)
            {
                ApproximateLimits = ExactSpeeds && ApproximateOnlineBraking
            };
            List<float> there = table.Places[r1];
            float[] later = table.Value[r1];
            ushort[] tailThere = table.Tail[r1];
            for (int j = 0; j < there.Count; j++)
            {
                if (MathF.Abs(there[j] - y) > MaxLateralJumps[sp] + 1e-3f || only >= 0 && j != only / slopes)
                    continue;
                float chordTime = 0f;
                if (bounded)
                {
                    int endStation = table.Starts[r1];
                    Vector2 endPosition = _lanes.Position(endStation, _lanes.Offset(LaneId.Free, endStation) + there[j]);
                    chordTime = TimeLowerBound(Vector2.Distance(startPosition, endPosition));
                }
                for (int hj = 0; hj < slopes; hj++)
                {
                    if (only >= 0 && hj != only % slopes)
                        continue;
                    LoopIterations++;
                    var move = new Move(row, r1, stations, y, slope, there[j], Slopes[hj]);
                    if (allowedMove is not null && !allowedMove(move))
                        continue;
                    int node = j * slopes + hj;
                    int target = node * bins;
                    // What ending on this place and heading is favoured by,
                    // seconds: taken off the rest of the way, so every bound
                    // below that reads the rest reads it favoured too.
                    float favour = preference?.Invoke(r1, node) ?? 0f;
                    Table.ArrivalGroup[]? groups = table.OnlineArrivals?[r1][node];
                    if (groups is null)
                    {
                        SlowWayNodes++;
                        ways.Clear();
                        for (int b = 0; b < bins; b++)
                            if (later[target + b] < float.PositiveInfinity && !ways.Contains(tailThere[target + b]))
                                ways.Add(tailThere[target + b]);
                    }
                    int groupCount = groups?.Length ?? ways.Count;
                    if (groupCount == 0)
                        continue;
                    if (bounded && groups is not null &&
                        AllAboveBound(groups, chordTime, prefixTimeLower, maximumTotal + favour))
                        continue;
                    mark = System.Diagnostics.Stopwatch.GetTimestamp();
                    bool shaped = Geometry is not null && fixedStart && only < 0
                        ? ReadFixedShape(edge, st0, row, place, heading, sp, j, hj, y, slope, there[j], Slopes[hj])
                        : Shape(edge, st0, row, y, slope, there[j], Slopes[hj], null);
                    Lap(0);
                    if (!shaped)
                        continue;
                    float bent = MathF.Round(Bend(edge) / BendUnit) * BendUnit;
                    float length = MathF.Round(edge.Length / LengthUnit) * LengthUnit;
                    int split = ShareJoin ? Split(edge, st0) : 0;
                    int drivePrefix = Math.Max(0, split - 1);
                    bool prefixReady = false;
                    bool drivePrefixReady = false;
                    liveLimits.Clear();
                    // Every sample is capped by TopSpeed, even before the joined
                    // geometry and live envelope give us a tighter bound.
                    bool canPruneJoins = ExactSpeeds && nonnegativeCost && extra is null && terminal is null &&
                        groups is not null;
                    float quickest = bent;
                    if (canPruneJoins)
                        for (int k = 0; k < edge.N; k++)
                            quickest += edge.Step[k] / LaneSpeedPlanner.TopSpeed;
                    float prefixBound = quickest;
                    float lengthTime = bounded ? TimeLowerBound(edge.Length) : 0f;
                    for (int group = 0; group < groupCount; group++)
                    {
                        if (bounded && groups is not null && CannotImprove(prefixTimeLower, lengthTime,
                            groups[group].MinimumRest - favour, maximumTotal))
                            continue;
                        if (canPruneJoins && prefixBound + groups![group].MinimumRest - favour >= Bound())
                            continue;
                        ushort way = groups is null ? ways[group] : groups[group].Tail;
                        int[]? arrivals = groups?[group].Bins;
                        mark = System.Diagnostics.Stopwatch.GetTimestamp();
                        if (groups is not null && Geometry is not null && ShareJoin && ExactSpeeds &&
                            TurnLag is null && LeadSpeed is null && Geometry.ReadJoin(edge, group, split))
                        {
                            GeometryJoinHits++;
                            SetCornerLimits(edge, split, edge.N + 1, liveLimits);
                        }
                        else
                            Join(edge, st0, way == NoTail ? null : table.Tails[r1][way], split, liveLimits);
                        Lap(1);
                        if (speed > edge.Limit[0] + 1e-3f)
                        {
                            LastRejects.TooFastHere++;
                            continue;
                        }
                        int n = edge.N;
                        if (ExactSpeeds)
                        {
                            // A different tail changes neither the earlier
                            // speeds nor their elapsed time for traffic.
                            DriveOn(edge, speed, driven, drivePrefixReady ? drivePrefix : 0,
                                move, pace, driveTimes);
                            if (!drivePrefixReady && canPruneJoins && drivePrefix > 0)
                            {
                                // Every later join shares this driven prefix;
                                // assume top speed over the unknown suffix.
                                float lower = bent;
                                for (int k = 0; k < n; k++)
                                {
                                    float upper = k < drivePrefix
                                        ? MathF.Max(1f, 0.5f * (driven[k] + driven[k + 1]))
                                        : LaneSpeedPlanner.TopSpeed;
                                    lower += edge.Step[k] / upper;
                                }
                                prefixBound = MathF.Max(quickest, lower - 1e-4f);
                            }
                            drivePrefixReady = true;
                            Lap(2);
                            float fastest = bent;
                            if (nonnegativeCost && extra is null)
                                for (int k = 0; k < n; k++)
                                    fastest += edge.Step[k] / MathF.Max(1f, 0.5f * (driven[k] + driven[k + 1]));
                            bool brakeBudgetReady = false;
                            int arrivalCount = arrivals?.Length ?? bins;
                            for (int i = 0; i < arrivalCount; i++)
                            {
                                int b1 = arrivals is null ? i : arrivals[i];
                                float v1 = MinSpeed + b1 * SpeedStep;
                                if (v1 > driven[n] || v1 > edge.Limit[n])
                                    break;
                                float rest = later[target + b1] - favour;
                                if (rest == float.PositiveInfinity || tailThere[target + b1] != way)
                                    continue;
                                if (bounded && CannotImprove(prefixTimeLower, lengthTime, rest, maximumTotal))
                                    continue;
                                if (nonnegativeCost && extra is null && terminal is null && fastest + rest >= Bound())
                                    continue;
                                mark = System.Diagnostics.Stopwatch.GetTimestamp();
                                if (ApproximateOnlineBraking)
                                {
                                    if (!brakeBudgetReady)
                                    {
                                        if (!prefixReady)
                                        {
                                            PrepareBrakeBudget(edge, driven, brakeBudget, 0, split);
                                            prefixReady = true;
                                        }
                                        PrepareBrakeBudget(edge, driven, brakeBudget, split, n);
                                        brakeBudgetReady = true;
                                    }
                                    ReadBrakeBudget(edge, v1, brakeBudget, back);
                                }
                                else
                                    BrakeBack(edge, v1, back);
                                Lap(3);
                                if (back[0] < speed)
                                {
                                    LastRejects.CannotBrake++;
                                    continue;
                                }
                                LastRejects.Timed++;
                                float seconds = bent;
                                for (int k = 0; k < n; k++)
                                    seconds += edge.Step[k] / MathF.Max(1f, 0.5f * (MathF.Min(driven[k], back[k]) + MathF.Min(driven[k + 1], back[k + 1])));
                                Lap(4);
                                if (extra is not null)
                                    seconds += extra(move, speed, v1);
                                if (terminal is not null)
                                    rest += terminal(r1, target + b1, seconds);
                                if (nonnegativeCost && seconds + rest >= Bound())
                                    continue;
                                if (cost is not null)
                                {
                                    seconds += cost(move, v1, driven, back);
                                    if (float.IsPositiveInfinity(seconds))
                                        LastRejects.Blocked++;
                                    Lap(5);
                                }
                                if (seconds + rest < bestSeconds)
                                    (bestSeconds, bestRow, bestState) = (seconds + rest, r1, target + b1);
                                if (beam is { Length: > 0 } && seconds + rest < Bound())
                                    Keep(seconds + rest, r1, target + b1);
                            }
                            continue;
                        }
                        (float vLow, float vHigh) = Reach(edge, speed);
                        int bLow = Math.Max(0, (int)MathF.Ceiling((vLow - MinSpeed) / SpeedStep));
                        int bHigh = Math.Min(bins - 1, (int)MathF.Floor((vHigh - MinSpeed) / SpeedStep));
                        for (int b1 = bLow; b1 <= bHigh; b1++)
                        {
                            float rest = later[target + b1] - favour;
                            if (rest == float.PositiveInfinity || tailThere[target + b1] != way)
                                continue;
                            float v1 = MinSpeed + b1 * SpeedStep;
                            float seconds = length / MathF.Max(0.5f * (speed + v1), 1f) + bent;
                            if (extra is not null)
                                seconds += extra(new Move(row, r1, stations, y, slope, there[j], Slopes[hj]), speed, v1);
                            if (seconds + rest < bestSeconds)
                                (bestSeconds, bestRow, bestState) = (seconds + rest, r1, target + b1);
                        }
                    }
                }
            }
        }
        return (bestSeconds, bestRow, bestState);

        void Keep(float seconds, int endRow, int state)
        {
            // Keep alternatives with different lateral destinations or speeds.
            int place = state / (bins * slopes);
            int speedBand = (int)((state % bins) * SpeedStep / 4f);
            int at = beam!.Length - 1;
            for (int i = 0; i < beam.Length && beam[i].Row >= 0; i++)
            {
                int previous = beam[i].State;
                if (previous / (bins * slopes) != place || (int)((previous % bins) * SpeedStep / 4f) != speedBand)
                    continue;
                if (beam[i].Seconds <= seconds)
                    return;
                at = i;
                break;
            }
            while (at > 0 && seconds < beam[at - 1].Seconds)
            {
                beam[at] = beam[at - 1];
                at--;
            }
            beam[at] = (seconds, endRow, state);
        }
    }

    private static bool AllAboveBound(Table.ArrivalGroup[] groups, float edgeTime, float prefixTime, float maximum)
    {
        foreach (var group in groups)
            if (!CannotImprove(prefixTime, edgeTime, group.MinimumRest, maximum))
                return false;
        return true;
    }

    internal static bool CannotImprove(float prefixTime, float edgeTime, float terminalScore, float maximum) =>
        prefixTime + edgeTime + terminalScore > maximum + 1e-3f;

    internal static float TimeLowerBound(float distance) => distance / (LaneSpeedPlanner.TopSpeed + 0.01f);

    /// <summary>
    /// Each step out of every <paramref name="every"/>th place and heading on
    /// a row of a table timed the driver's way, joined to each way on its
    /// target runs on, as the driver plans it: for every start bin it can
    /// take, the fastest end full drive reaches; for every end bin, the
    /// fastest start braking down to it allows; and the seconds of every pair
    /// of the two the plan drives (NaN where it cannot). For fitting.
    /// </summary>
    public void Curves(Table table, int row, int every, Func<int, float>? slowest,
        Action<float[], float[], float[], float[], float[,]> into)
    {
        int slopes = Slopes.Length, bins = table.Bins;
        int st0 = table.Starts[row];
        var ways = new List<ushort>();
        List<float> here = table.Places[row];
        for (int node = 0; node < here.Count * slopes; node += every)
        {
            float y = here[node / slopes], slope = Slopes[node % slopes];
            for (int sp = 0; sp < Spans.Length; sp++)
            {
                int r1 = row + Spans[sp];
                if (r1 > table.Rows)
                    continue;
                var edge = new Edge(table.Starts[r1] - st0, LeadMax + 1);
                List<float> there = table.Places[r1];
                float[] later = table.Value[r1];
                ushort[] tailThere = table.Tail[r1];
                for (int j = 0; j < there.Count; j++)
                {
                    if (MathF.Abs(there[j] - y) > MaxLateralJumps[sp] + 1e-3f)
                        continue;
                    for (int hj = 0; hj < slopes; hj++)
                    {
                        int target = (j * slopes + hj) * bins;
                        ways.Clear();
                        for (int b = 0; b < bins; b++)
                            if (later[target + b] < float.PositiveInfinity && !ways.Contains(tailThere[target + b]))
                                ways.Add(tailThere[target + b]);
                        if (ways.Count == 0 || !Shape(edge, st0, row, y, slope, there[j], Slopes[hj], null))
                            continue;
                        int split = Split(edge, st0);
                        foreach (ushort way in ways)
                        {
                            Join(edge, st0, way == NoTail ? null : table.Tails[r1][way], split);
                            int n = edge.N;
                            int top = Math.Min(bins - 1, (int)MathF.Floor((edge.Limit[0] + 1e-3f - MinSpeed) / SpeedStep));
                            if (top < 0)
                                continue;
                            int bottom = slowest is null ? 0 : Math.Clamp((int)MathF.Ceiling((slowest(st0) - MinSpeed) / SpeedStep), 0, top);
                            int starts = top - bottom + 1;
                            var v0s = new float[starts];
                            var fastestEnd = new float[starts];
                            var drives = new float[starts][];
                            float most = 0f;
                            for (int q = 0; q < starts; q++)
                            {
                                v0s[q] = MinSpeed + (bottom + q) * SpeedStep;
                                drives[q] = new float[n + 1];
                                DriveOn(edge, v0s[q], drives[q]);
                                fastestEnd[q] = drives[q][n];
                                most = MathF.Max(most, fastestEnd[q]);
                            }
                            int last = Math.Min(bins - 1, (int)MathF.Floor((MathF.Min(most, edge.Limit[n]) - MinSpeed) / SpeedStep));
                            if (last < 0)
                                continue;
                            var v1s = new float[last + 1];
                            var fastestStart = new float[last + 1];
                            var seconds = new float[starts, last + 1];
                            var back = new float[n + 1];
                            for (int b1 = 0; b1 <= last; b1++)
                            {
                                v1s[b1] = MinSpeed + b1 * SpeedStep;
                                BrakeBack(edge, v1s[b1], back);
                                fastestStart[b1] = back[0];
                                for (int q = 0; q < starts; q++)
                                {
                                    float[] d = drives[q];
                                    if (v1s[b1] > d[n] || back[0] < v0s[q])
                                    {
                                        seconds[q, b1] = float.NaN;
                                        continue;
                                    }
                                    float t = 0f;
                                    for (int k = 0; k < n; k++)
                                        t += edge.Step[k] / MathF.Max(1f, 0.5f * (MathF.Min(d[k], back[k]) + MathF.Min(d[k + 1], back[k + 1])));
                                    seconds[q, b1] = t;
                                }
                            }
                            into(v0s, fastestEnd, v1s, fastestStart, seconds);
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Every step out of every place on the table's rows shaped once, as a
    /// table that keeps none of its steps' geometry would at load: the
    /// seconds it takes.
    /// </summary>
    public double ShapeAll(Table table)
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();
        int slopes = Slopes.Length;
        Parallel.For(0, table.Rows, row =>
        {
            int st0 = table.Starts[row];
            List<float> here = table.Places[row];
            for (int sp = 0; sp < Spans.Length; sp++)
            {
                int r1 = row + Spans[sp];
                if (r1 > table.Rows)
                    continue;
                var edge = new Edge(table.Starts[r1] - st0, LeadMax + 1);
                List<float> there = table.Places[r1];
                for (int i = 0; i < here.Count; i++)
                    for (int hi = 0; hi < slopes; hi++)
                        for (int j = 0; j < there.Count; j++)
                        {
                            if (MathF.Abs(there[j] - here[i]) > MaxLateralJumps[sp] + 1e-3f)
                                continue;
                            for (int hj = 0; hj < slopes; hj++)
                                Shape(edge, st0, row, here[i], Slopes[hi], there[j], Slopes[hj], null);
                        }
            }
        });
        return clock.Elapsed.TotalSeconds;
    }

    /// <summary>The place, heading and speed a baked state stands for.</summary>
    public (float Y, float Slope, float Speed) StateOf(Table table, int row, int state)
    {
        (int place, int heading, int bin) = Split(state, Slopes.Length, table.Bins);
        return (table.Places[row][place], Slopes[heading], MinSpeed + bin * SpeedStep);
    }

    /// <summary>
    /// The way's line with the joins between its steps eased: each step is
    /// smooth and meets the next heading the same way, but the corner can
    /// change at a join all at once, a wheel turned in no time. Averaged
    /// over <paramref name="halfWindow"/> stations either side, twice, and
    /// kept on the road.
    /// </summary>
    public float[] Smoothed(int from, float[] offset, int halfWindow = 3)
    {
        int n = offset.Length;
        float[] a = (float[])offset.Clone(), b = new float[n];
        for (int pass = 0; pass < 2; pass++)
        {
            for (int k = 0; k < n; k++)
            {
                float sum = 0f;
                for (int j = -halfWindow; j <= halfWindow; j++)
                    sum += a[Math.Clamp(k + j, 0, n - 1)];
                b[k] = sum / (2 * halfWindow + 1);
            }
            (a, b) = (b, a);
        }
        a[0] = offset[0];
        a[n - 1] = offset[n - 1];
        for (int k = 0; k < n; k++)
        {
            (float lo, float hi) = Room(from + k);
            a[k] = Math.Clamp(a[k], lo, hi);
        }
        return a;
    }

    /// <summary>
    /// What the speed plan makes of a line over the stretch: the seconds it
    /// takes from <paramref name="startSpeed"/>, ending no faster than
    /// <paramref name="endSpeed"/>, and the speed it ends at.
    /// </summary>
    public (float Seconds, float EndSpeed, float[] Speed) Plan(int from, float[] offset, float startSpeed, float endSpeed)
    {
        int n = offset.Length;
        float Y(int k) => k < 0 || k >= n ? 0f : offset[k];
        System.Numerics.Vector2 P(int k) => _lanes.Position(from + k, _lanes.Offset(LaneId.Free, from + k) + Y(k));
        var curvature = new float[n];
        var step = new float[n];
        var factor = new float[n];
        var brake = new float[n];
        var road = new LaneRoad[n];
        for (int k = 0; k < n; k++)
        {
            curvature[k] = LaneSet.Curvature(P(k - Chord), P(k), P(k + Chord));
            step[k] = (P(k + 1) - P(k)).Length();
            factor[k] = _factor(from + k);
            brake[k] = 1f;
            road[k] = _road(from + k, _lanes.Offset(LaneId.Free, from + k) + Y(k));
        }
        if (TurnLag is not null && LeadSpeed is not null)
        {
            var guess = new float[n];
            for (int k = 0; k < n; k++)
                guess[k] = LeadSpeed(from + k);
            LaneSpeedPlanner.LeadCurvatures((float[])curvature.Clone(), guess, TurnLag, _lanes.Spacing, curvature);
        }
        var speed = new float[n];
        LaneSpeedPlanner.PlanAhead(_envelope, curvature, step, factor, brake, road, _shares, startSpeed, endSpeed, speed);
        float seconds = 0f;
        for (int k = 0; k + 1 < n; k++)
            seconds += step[k] / MathF.Max(1f, 0.5f * (speed[k] + speed[k + 1]));
        return (seconds, speed[n - 1], speed);
    }

    /// <summary>
    /// How much faster an arrival may be and still stand in for a slower one
    /// that came no sooner: near enough that braking the difference away
    /// costs nothing worth keeping. Any further apart and the faster one may
    /// not be able to slow in time for what comes next.
    /// </summary>
    public float StandInSpeed { get; init; } = 2f;

    // At one place and heading, an arrival no sooner than a slightly faster
    // one is never searched on.
    private void Prune(float[] cost, int nodes, int bins)
    {
        int window = Math.Max(1, (int)MathF.Round(StandInSpeed / SpeedStep));
        for (int node = 0; node < nodes; node++)
        {
            int at0 = node * bins;
            for (int b = bins - 2; b >= 0; b--)
            {
                if (cost[at0 + b] == float.PositiveInfinity)
                    continue;
                for (int f = b + 1; f <= Math.Min(bins - 1, b + window); f++)
                    if (cost[at0 + f] <= cost[at0 + b])
                    {
                        cost[at0 + b] = float.PositiveInfinity;
                        break;
                    }
            }
        }
    }

    private static (int Place, int Slope, int Bin) Split(int state, int slopes, int bins) =>
        (state / bins / slopes, state / bins % slopes, state % bins);

    // Sample the existing grid across the road, keeping both extremes and
    // the racing line whenever it was an allowed place.
    private List<float> LateralPlaces(List<float> places)
    {
        const int count = 5;
        if (places.Count <= count)
            return places;

        var selected = new List<float>(count);
        for (int i = 0; i < count; i++)
            selected.Add(places[(int)MathF.Round(i * (places.Count - 1f) / (count - 1))]);

        if (places.Contains(0f) && !selected.Contains(0f))
        {
            int nearest = 1;
            for (int i = 2; i < count - 1; i++)
                if (MathF.Abs(selected[i]) < MathF.Abs(selected[nearest]))
                    nearest = i;
            selected[nearest] = 0f;
        }
        return selected;
    }

    // Where across the road a car may be, as metres from the racing line.
    private (float Lo, float Hi) Room(int station)
    {
        float edge = _lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters;
        float free = _lanes.Offset(LaneId.Free, station);
        return (MathF.Min(0f, -(edge + free)), MathF.Max(0f, edge - free));
    }

    // One step of the lattice as the car drives it.
    internal sealed class Edge(int stations, int ahead = 0)
    {
        /// <summary>How many stations the step crosses: up to as many as it was made for.</summary>
        public int N = stations;
        /// <summary>Its curve's ends: place and heading.</summary>
        public float Y0, H0, Y1, H1;
        public readonly float[] Offset = new float[stations + 1];
        public readonly Vector2[] Points = new Vector2[stations + 2 * Chord + 1];
        /// <summary>The curvature at each station, and on as far past the end as the lead reads.</summary>
        public readonly float[] Curvature = new float[stations + ahead + 1];
        /// <summary>The speed the driver guesses at each station, which sets how far its lead reads.</summary>
        public readonly float[] Guess = new float[stations + 1];
        /// <summary>The corner each station is planned on: its own, or the sharpest the driver's lead reads ahead; the far end's too.</summary>
        public readonly float[] Corner = new float[stations + 1];
        public readonly float[] Step = new float[stations + 1];
        public readonly float[] Limit = new float[stations + 1];
        public readonly float[] Factor = new float[stations + 1];
        public readonly LaneRoad[] Road = new LaneRoad[stations + 1];
        public float Length;
        public long GeometryJoins;
        public bool ApproximateLimits;
    }

    // The smooth curve from one place and heading to the next, kept on the
    // road; its corner, step lengths and the speed its corner allows at
    // each station. False if it leaves the road.
    private bool Shape(Edge edge, int st0, int row, float y0, float h0, float y1, float h1, Func<int, float, bool>? allowed)
    {
        edge.GeometryJoins = 0;
        long mark = System.Diagnostics.Stopwatch.GetTimestamp();
        if (!ShapeGeometry(edge, st0, row, y0, h0, y1, h1, allowed))
            return false;
        ShapeTicks[0] += System.Diagnostics.Stopwatch.GetTimestamp() - mark;
        mark = System.Diagnostics.Stopwatch.GetTimestamp();
        FitShapeToCar(edge, st0, y0, h0, y1, h1);
        ShapeTicks[1] += System.Diagnostics.Stopwatch.GetTimestamp() - mark;
        return true;
    }

    private bool ShapeGeometry(Edge edge, int st0, int row, float y0, float h0, float y1, float h1, Func<int, float, bool>? allowed)
    {
        int n = edge.N;
        // A step may start nearer the edge than the margin -- the car is
        // where it is -- but may not go any nearer than it starts. Held to
        // the margin from its first station, a car a metre from the kerb had
        // no step at all, and no plan, every time it ran out to an exit.
        (float lo0, float hi0) = Room(st0);
        float leeway = MathF.Max(0f, MathF.Max(lo0 - y0, y0 - hi0));
        for (int k = 0; k <= n; k++)
        {
            float y = Hermite(y0, h0, y1, h1, n, k);
            (float lo, float hi) = Room(st0 + k);
            if (y < lo - leeway - 1e-3f || y > hi + leeway + 1e-3f)
                return false;
            // A row crossed between the ends still holds the line to what it allows.
            if (allowed is not null && k > 0 && k < n && k % RowStations == 0 && !allowed(row + k / RowStations, y))
                return false;
            edge.Offset[k] = y;
        }
        for (int k = -Chord; k <= n + Chord; k++)
        {
            float y = k >= 0 && k <= n ? edge.Offset[k] : Hermite(y0, h0, y1, h1, n, k);
            edge.Points[k + Chord] = _lanes.Position(st0 + k, _lanes.Offset(LaneId.Free, st0 + k) + y);
        }
        edge.Length = 0f;
        for (int k = 0; k < n; k++)
        {
            edge.Curvature[k] = LaneSet.Curvature(edge.Points[k], edge.Points[k + Chord], edge.Points[k + 2 * Chord]);
            edge.Step[k] = (edge.Points[k + Chord + 1] - edge.Points[k + Chord]).Length();
            edge.Length += edge.Step[k];
        }
        edge.Step[n] = 0f;
        return true;
    }

    private void FitShapeToCar(Edge edge, int st0, float y0, float h0, float y1, float h1)
    {
        int n = edge.N;
        for (int k = 0; k <= n; k++)
        {
            edge.Factor[k] = _factor(st0 + k);
            edge.Road[k] = RoadAt(st0 + k, edge.Offset[k]);
        }
        (edge.Y0, edge.H0, edge.Y1, edge.H1) = (y0, h0, y1, h1);
        if (LeadSpeed is not null)
            for (int k = 0; k <= n; k++)
                edge.Guess[k] = LeadSpeed(st0 + k);
        Join(edge, st0, null);
    }

    // The step joined to the line the car runs on after it, tail (from the
    // step's end; null where it is not known, and the step is read on its
    // own curve): the curvature at its last stations and as far past its end
    // as the lead reads, all on the one line, the corner the lead plans each
    // station on (LaneSpeedPlanner.LeadCurvatures), and the speed that
    // corner allows: the driver's own plan of the same line, read the same.
    // Joined again to another tail, a step's stations before from are as the
    // last join left them: their corners read none of the road the tail
    // changes (Split), so only the stations from there on are read again.
    private void Join(Edge edge, int st0, float[]? tail, int from = 0,
        Dictionary<(int Station, float Corner), float>? limits = null)
    {
        int end = JoinGeometry(edge, st0, tail, from);
        SetCornerLimits(edge, from, end, limits);
    }

    private int JoinGeometry(Edge edge, int st0, float[]? tail, int from)
    {
        int n = edge.N;
        Vector2 P(int k)
        {
            float y = k >= 0 && k <= n ? edge.Offset[k] :
                tail is not null && k > n && k - n < tail.Length ? tail[k - n] :
                Hermite(edge.Y0, edge.H0, edge.Y1, edge.H1, n, k);
            return _lanes.Position(st0 + k, _lanes.Offset(LaneId.Free, st0 + k) + y);
        }
        // Timed as the driver plans, the step's far end is a station of it too.
        int m = ExactSpeeds ? n + 1 : n;
        int extent = m + (tail is null ? 0 : Math.Clamp(tail.Length - 1 - Chord - (m - n), 0, LeadMax));
        for (int k = Math.Max(0, n - Chord); k < extent; k++)
            edge.Curvature[k] = LaneSet.Curvature(P(k - Chord), P(k), P(k + Chord));
        if (TurnLag is null || LeadSpeed is null)
            Array.Copy(edge.Curvature, from, edge.Corner, from, m - from);
        else
            LaneSpeedPlanner.LeadCurvatures(edge.Curvature.AsSpan(from, extent - from), edge.Guess.AsSpan(from, m - from), TurnLag, _lanes.Spacing, edge.Corner.AsSpan(from, m - from));
        return m;
    }

    private void SetCornerLimits(Edge edge, int from, int end, Dictionary<(int Station, float Corner), float>? limits)
    {
        for (int k = from; k < end; k++)
        {
            if (limits is null)
                edge.Limit[k] = CornerLimit(edge, k);
            else
            {
                ref float limit = ref CollectionsMarshal.GetValueRefOrAddDefault(limits, (k, edge.Corner[k]), out bool found);
                if (!found)
                    limit = CornerLimit(edge, k);
                edge.Limit[k] = limit;
            }
        }
    }

    private float CornerLimit(Edge edge, int station)
    {
        float corner = edge.Corner[station];
        LaneEnvelope envelope = _envelope.ForTurn(corner);
        float factor = edge.Factor[station] * _shares.Lateral;
        LaneRoad road = edge.Road[station];
        return edge.ApproximateLimits
            ? LaneSpeedPlanner.OnlineCornerLimit(envelope, corner, factor, road)
            : LaneSpeedPlanner.CornerLimit(envelope, corner, factor, road);
    }

    // The slowest and the fastest the car can be at the far end of a step
    // from v0: braking all the way, or full drive held under each corner;
    // and no faster than a steady change of v虏 from v0 keeps every station
    // on the way inside the corner its tyres hold.
    private (float Low, float High) Reach(Edge edge, float v0)
    {
        (float low, float high) = Chains(edge, v0);
        return (low, Held(edge, v0, high));
    }

    // Braking all the way and full drive held under each corner, station by
    // station: what a step costs to bake.
    private (float Low, float High) Chains(Edge edge, float v0)
    {
        float low = v0, high = v0, rise = float.PositiveInfinity;
        Drive(edge, v0, 0, edge.N, ref low, ref high, ref rise);
        return (low, high);
    }

    // No faster than a steady change of v虏 from v0 keeps every station on
    // the way inside the corner its tyres hold; negative if none does.
    private static float Held(Edge edge, float v0, float high)
    {
        float rise = float.PositiveInfinity;
        Rise(edge, v0, 1, edge.N, ref rise);
        return Held(v0, high, rise);
    }

    private static float Held(float v0, float high, float rise)
    {
        if (rise == float.PositiveInfinity)
            return high;
        return v0 * v0 + rise < 0f ? -1f : MathF.Min(high, MathF.Sqrt(v0 * v0 + rise));
    }

    // The chains and the rise held from station k0 to k1, on from where they stand.
    private void Drive(Edge edge, float v0, int k0, int k1, ref float low, ref float high, ref float rise)
    {
        for (int k = k0; k < k1; k++)
        {
            float decel = LaneSpeedPlanner.Deceleration(_envelope, low, edge.Corner[k], edge.Factor[k], 1f, edge.Road[k], _shares);
            low = MathF.Sqrt(MathF.Max(0f, low * low - 2f * decel * edge.Step[k]));
            high = LaneSpeedPlanner.Driven(_envelope, MathF.Min(high, edge.Limit[k]), edge.Corner[k], edge.Step[k], edge.Factor[k], edge.Road[k], _shares);
        }
        Rise(edge, v0, Math.Max(1, k0), k1, ref rise);
    }

    private static void Rise(Edge edge, float v0, int k0, int k1, ref float rise)
    {
        int n = edge.N;
        for (int k = k0; k < k1; k++)
        {
            float limit = edge.Limit[k] + 1e-3f;
            rise = MathF.Min(rise, (limit * limit - v0 * v0) * n / k);
        }
    }

    // Full drive on from v0 under each station's corner, the forward sweep of
    // the driver's plan (LaneSpeedPlanner.PlanAhead) before the braking back
    // holds it: where that braking is lower the plan takes it, and full drive
    // on from a lower speed never passes it, so the plan is the lower of the two.
    private void DriveOn(Edge edge, float v0, float[] into, int from = 0, Move move = default, PaceLimit? pace = null,
        float[]? times = null)
    {
        if (from == 0)
            into[0] = MathF.Min(edge.Limit[0], v0);
        float time = times is null ? 0f : times[from];
        for (int k = from; k < edge.N; k++)
        {
            float speed = into[k], ds = edge.Step[k];
            float next = MathF.Min(edge.Limit[k + 1], LaneSpeedPlanner.Driven(Motor ?? _envelope, speed,
                edge.Corner[k], ds, edge.Factor[k], edge.Road[k], _shares));
            if (pace is not null)
            {
                float wanted = pace(move, k + 1, time + ds / MathF.Max(1f, speed), speed);
                if (wanted < next)
                {
                    float decel = LaneSpeedPlanner.Deceleration(_envelope, speed, edge.Corner[k],
                        edge.Factor[k], 1f, edge.Road[k], _shares);
                    float reachable = MathF.Sqrt(MathF.Max(0f, speed * speed - 2f * decel * ds));
                    next = MathF.Min(next, MathF.Max(wanted, reachable));
                }
                time += ds / MathF.Max(1f, 0.5f * (speed + next));
                if (times is not null)
                    times[k + 1] = time;
            }
            into[k + 1] = next;
        }
    }

    // Braking back from v1 at the far end under each station's corner, the
    // backward sweep of the driver's plan.
    private void BrakeBack(Edge edge, float v1, float[] into)
    {
        int n = edge.N;
        into[n] = MathF.Min(edge.Limit[n], v1);
        for (int k = n - 1; k >= 0; k--)
            into[k] = MathF.Min(edge.Limit[k], LaneSpeedPlanner.BrakeFrom(_envelope, into[k + 1],
                edge.Corner[k], edge.Step[k], edge.Factor[k], 1f, edge.Road[k], _shares));
    }

    // The forward sweep supplies a reference speed for the live tyre and road
    // model. The resulting changes in v虏 are shared across all end speeds.
    private void PrepareBrakeBudget(Edge edge, float[] driven, float[] budget, int from, int to)
    {
        for (int k = from; k < to; k++)
        {
            float speed = 0.5f * (driven[k] + driven[k + 1]);
            float decel = LaneSpeedPlanner.Deceleration(_envelope, speed, edge.Corner[k],
                edge.Factor[k], 1f, edge.Road[k], _shares);
            budget[k] = 2f * decel * edge.Step[k];
        }
    }

    private static void ReadBrakeBudget(Edge edge, float endSpeed, float[] budget, float[] into)
    {
        int n = edge.N;
        into[n] = MathF.Min(edge.Limit[n], endSpeed);
        float squared = into[n] * into[n];
        for (int k = n - 1; k >= 0; k--)
        {
            float limit = edge.Limit[k];
            squared = MathF.Min(limit * limit, squared + budget[k]);
            into[k] = MathF.Sqrt(squared);
        }
    }

    // The first station of a step whose corner the lead reads past the
    // stations its own curve alone settles: from there on the way on counts.
    internal int UnchangedDriveStations(int stations, int start) => Math.Max(0, Split(stations, start) - 1);

    private int Split(Edge edge, int st0) => Split(edge.N, st0);

    private int Split(int stations, int st0)
    {
        int settled = stations - Chord;
        for (int k = 0; k < stations; k++)
            if (k + LeadStations(st0 + k) >= settled)
                return k;
        return stations;
    }

    // Every chain sample's slow and fast end and rise held at a station, from
    // the start: what every way on after the step shares.
    private float[] Front(Edge edge, int bottom, int top, int split)
    {
        int count = ChainCount(bottom, top);
        var front = new float[3 * count];
        for (int q = 0; q < count; q++)
        {
            float v0 = SampleSpeed(bottom, top, q);
            float low = v0, high = v0, rise = float.PositiveInfinity;
            Drive(edge, v0, 0, split, ref low, ref high, ref rise);
            (front[3 * q], front[3 * q + 1], front[3 * q + 2]) = (low, high, rise);
        }
        return front;
    }

    /// <summary>
    /// How many speed bins apart the bake rides a step's chains; the bins
    /// between take them in a straight line. They bend gently with the
    /// speed they start from, and a step's whole cost is in them.
    /// </summary>
    public int ChainStride { get; init; } = 8;

    // A step's chains every stride from the bottom bin to the top one, the
    // fastest end held under the corners on the way.
    private float[] AllChains(Edge edge, int bottom, int top, int split = 0, float[]? front = null)
    {
        int count = ChainCount(bottom, top);
        var chains = new float[2 * count];
        for (int q = 0; q < count; q++)
        {
            float v0 = SampleSpeed(bottom, top, q);
            float low = v0, high = v0, rise = float.PositiveInfinity;
            int k0 = 0;
            if (front is not null)
                (low, high, rise, k0) = (front[3 * q], front[3 * q + 1], front[3 * q + 2], split);
            Drive(edge, v0, k0, edge.N, ref low, ref high, ref rise);
            chains[2 * q] = StoredSlow(v0, low);
            chains[2 * q + 1] = StoredFast(v0, Held(v0, high, rise));
        }
        return chains;
    }

    // A chain's number as it is stored, so the bake charges what a decision
    // will read: on the quarter metre a second from the start speed where a
    // byte holds it, as it is where not.
    private static float StoredFast(float v0, float x)
    {
        if (x < 0f)
            return x;
        float q = MathF.Round((x - v0) / ChainUnit);
        return q > Escape && q <= sbyte.MaxValue ? v0 + q * ChainUnit : x;
    }

    private static float StoredSlow(float v0, float x)
    {
        float q = MathF.Round((v0 - x) / ChainUnit);
        return q >= 0f && q < SlowEscape ? v0 - q * ChainUnit : x;
    }

    private int ChainCount(int bottom, int top) => (top - bottom + ChainStride - 1) / ChainStride + 1;

    // The chains at a speed bin, fractional, between the strides either side.
    private (float Low, float High) ChainAt(ReadOnlySpan<float> chains, int bottom, int top, float bin)
    {
        (int q, int pairs, float t) = Locate(chains.Length / 2, bottom, top, bin);
        if (pairs == 1)
            return (chains[2 * q], chains[2 * q + 1]);
        return (chains[2 * q] + (chains[2 * q + 2] - chains[2 * q]) * t,
                chains[2 * q + 1] + (chains[2 * q + 3] - chains[2 * q + 1]) * t);
    }

    // Which sample a speed bin falls after, whether there is one after it
    // to go towards, and how far.
    private (int Sample, int Pairs, float T) Locate(int samples, int bottom, int top, float bin)
    {
        float x = Math.Clamp(bin, bottom, top) - bottom;
        int q = Math.Min((int)(x / ChainStride), samples - 1);
        int b0 = q * ChainStride, b1 = Math.Min(b0 + ChainStride, top - bottom);
        if (b1 <= b0 || q + 1 >= samples)
            return (q, 1, 0f);
        return (q, 2, (x - b0) / (b1 - b0));
    }

    private Steps Pack(List<Made>[] made)
    {
        int total = 0, numbers = 0;
        foreach (List<Made> list in made)
            foreach (Made m in list)
            {
                total++;
                numbers += m.Chains.Length;
            }
        var first = new int[made.Length + 1];
        var chainFirst = new int[made.Length + 1];
        var rowsOf = new byte[total];
        var target = new ushort[total];
        var length = new ushort[total];
        var bend = new ushort[total];
        var tailFirst = new int[total + 1];
        var tailList = new List<ushort>();
        var bottom = new byte[total];
        var top = new byte[total];
        var chains = new byte[numbers];
        var escaped = new Dictionary<int, float>();
        int e = 0, f = 0;
        for (int node = 0; node < made.Length; node++)
        {
            first[node] = e;
            chainFirst[node] = f;
            foreach (Made m in made[node])
            {
                (rowsOf[e], target[e], length[e], bend[e], bottom[e], top[e]) = (m.Rows, m.Target, m.Length, m.Bend, m.Bottom, m.Top);
                tailFirst[e] = tailList.Count;
                tailList.AddRange(m.Tails);
                for (int k = 0; k < m.Chains.Length; k++, f++)
                {
                    float v0 = SampleSpeed(m.Bottom, m.Top, k / 2);
                    float x = m.Chains[k];
                    if (k % 2 == 0)
                    {
                        float q = MathF.Round((v0 - x) / ChainUnit);
                        if (q >= 0f && q < SlowEscape)
                            chains[f] = (byte)q;
                        else
                        {
                            chains[f] = SlowEscape;
                            escaped[f] = x;
                        }
                    }
                    else
                    {
                        float q = MathF.Round((x - v0) / ChainUnit);
                        if (x < 0f)
                            chains[f] = unchecked((byte)NoSpeed);
                        else if (q <= Escape || q > sbyte.MaxValue)
                        {
                            chains[f] = unchecked((byte)Escape);
                            escaped[f] = x;
                        }
                        else
                            chains[f] = unchecked((byte)(sbyte)q);
                    }
                }
                e++;
            }
        }
        first[made.Length] = e;
        tailFirst[total] = tailList.Count;
        chainFirst[made.Length] = f;
        return new Steps
        {
            First = first, ChainFirst = chainFirst, Rows = rowsOf, Target = target, Length = length, Bend = bend,
            TailFirst = tailFirst, Tails = [.. tailList], Bottom = bottom, Top = top, Chains = chains, Escaped = escaped,
        };
    }

    // The speed a chain's sample starts at.
    private float SampleSpeed(int bottom, int top, int sample) =>
        MinSpeed + Math.Min(bottom + sample * ChainStride, top) * SpeedStep;

    // The step taken at a steady change of v虏 from v0 to v1, which is a
    // steady acceleration: its length over the mean of its ends.
    // What a step's bending is charged: its curvature squared over its length.
    private float Bend(Edge edge)
    {
        float sum = 0f;
        for (int k = 0; k < edge.N; k++)
            sum += edge.Curvature[k] * edge.Curvature[k] * edge.Step[k];
        return BendSeconds * sum;
    }

    private static float Seconds(Edge edge, float v0, float v1) =>
        edge.Length / MathF.Max(0.5f * (v0 + v1), 1f);

    private static float Along(float v0, float v1, Edge edge, int k) => SpeedOn(v0, v1, edge.N, k);

    /// <summary>The speed a step from <paramref name="v0"/> to <paramref name="v1"/> over so many stations is timed at, a station into it: v虏 changing steadily.</summary>
    public static float SpeedOn(float v0, float v1, int stations, int station)
    {
        float t = station / (float)stations;
        return MathF.Sqrt(v0 * v0 + (v1 * v1 - v0 * v0) * t);
    }
}
