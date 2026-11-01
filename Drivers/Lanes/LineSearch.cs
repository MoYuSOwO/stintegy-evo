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
internal sealed class LineSearch
{
    /// <summary>Stations from one row of the lattice to the next.</summary>
    public int RowStations { get; init; } = 5;

    /// <summary>Metres between places across the road.</summary>
    public float LateralStep { get; init; } = 1f;

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
    /// v². A table baked so is for drawing a way, not for deciding on.
    /// </summary>
    public bool ExactSpeeds { get; init; }

    /// <summary>Told each row's number as the bake finishes it, from the end back.</summary>
    public Action<int>? Baked { get; init; }

    /// <summary>The tail of a state no way runs on from (<see cref="Table.Tail"/>).</summary>
    public const ushort NoTail = ushort.MaxValue;

    private readonly LaneSet _lanes;
    private readonly LaneEnvelope _envelope;
    private readonly LaneShares _shares;
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
            places[r] = ys;
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
        public static Table? Load(string path, string key)
        {
            if (!File.Exists(path))
                return null;
            using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 20);
            using var r = new BinaryReader(stream);
            if (r.ReadInt32() != FileVersion || r.ReadString() != key)
                return null;
            int from = r.ReadInt32(), rows = r.ReadInt32(), bins = r.ReadInt32();
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
            for (int i = 0; i < steps.Length; i++)
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
                From = from, Rows = rows, Starts = starts, Places = places, Bins = bins,
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
    public Table BakeLap(int from, Table? next, Func<int, float>? slowest = null)
    {
        int rows = _lanes.Count / RowStations;
        var starts = new int[rows + 1];
        for (int r = 0; r < rows; r++)
            starts[r] = from + r * RowStations;
        starts[rows] = from + _lanes.Count;
        int slopes = Slopes.Length;
        Func<float, int, float, float> terminal = (y, h, v) => 0f;
        if (next is not null)
        {
            terminal = (y, h, v) =>
            {
                int place = next.Places[0].FindIndex(p => MathF.Abs(p - y) < 1e-3f);
                int bin = (int)MathF.Round((v - MinSpeed) / SpeedStep);
                return place < 0 ? float.PositiveInfinity : next.Value[0][(place * slopes + h) * next.Bins + bin];
            };
        }
        return Bake(from, starts, lap: true, terminal, slowest, next);
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
            places[r] = ys;
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
            steps[row0] = Pack(made);
            MakeTails(row0, starts, places, bins, nextRow, nextState, tail, tails);
            Baked?.Invoke(row0);
        }
        return new Table
        {
            From = from, Rows = rows, Starts = starts, Places = places, Bins = bins,
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
    public Way? Follow(Table table, int row, float y, float slope, float speed)
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
        while (path[^1].Row < table.Rows)
        {
            (int r, int s0) = path[^1];
            path.Add((table.NextRow[r][s0], table.Next[r][s0]));
        }
        return Draw(table, path, total);
    }

    private Way Draw(Table table, List<(int Row, int State)> path, float total)
    {
        int slopes = Slopes.Length;
        int first = path[0].Row;
        int n = table.Starts[table.Rows] - table.Starts[first] + 1;
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
        float terminal = table.Value[table.Rows][path[^1].State];
        return new Way(table.Starts[first], offset, speed, total - terminal, endSpeed, terminal);
    }

    /// <summary>
    /// A step of the lattice: from a place and heading on one row to a place
    /// and heading on a later one.
    /// </summary>
    public readonly record struct Move(int Row, int ToRow, int Stations, float Y, float Slope, float ToY, float ToSlope);

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
    /// at any speed: every step out of there and every speed it could end
    /// at, each charged its own seconds and the baked rest of the way; and
    /// <paramref name="extra"/>, what the cars around add to a step (infinite
    /// where it runs into one). All of it read off the table: nothing is
    /// shaped or driven here. The seconds, the row it ends on, and the state
    /// there.
    /// </summary>
    public (float Seconds, int Row, int State) Decide(Table table, int row, float y, float slope, float speed,
        Func<Move, float, float, float>? extra = null)
    {
        if (ExactSpeeds)
            throw new NotSupportedException("A table timed as the driver plans is for drawing a way, not for deciding on.");
        int slopes = Slopes.Length;
        int bins = table.Bins;
        float bestSeconds = float.PositiveInfinity;
        int bestRow = -1, bestState = -1;
        int place = table.Places[row].FindIndex(p => MathF.Abs(p - y) < 1e-3f);
        int heading = Array.IndexOf(Slopes, slope);
        if (place < 0 || heading < 0 || row >= table.Rows)
            return (bestSeconds, bestRow, bestState);
        Steps steps = table.Steps[row];
        int node = place * slopes + heading;
        float bin = (speed - MinSpeed) / SpeedStep;
        int at = steps.ChainFirst[node];
        for (int e = steps.First[node]; e < steps.First[node + 1]; e++)
        {
            int count = 2 * ChainCount(steps.Bottom[e], steps.Top[e]);
            int here = at;
            at += count;
            if (bin > steps.Top[e] + 1e-3f)
                continue;
            int r1 = row + steps.Rows[e];
            float[] later = table.Value[r1];
            ushort[] tailThere = table.Tail[r1];
            ReadOnlySpan<ushort> ways = steps.Tails.AsSpan(steps.TailFirst[e], steps.TailFirst[e + 1] - steps.TailFirst[e]);
            int target = steps.Target[e] * bins;
            (float vLow, float vHigh) = ChainAt(steps, e, here, count / 2, bin);
            int bLow = Math.Max(0, (int)MathF.Ceiling((vLow - MinSpeed) / SpeedStep));
            int bHigh = Math.Min(bins - 1, (int)MathF.Floor((vHigh - MinSpeed) / SpeedStep));
            Move move = default;
            if (extra is not null)
            {
                (int j, int hj) = (steps.Target[e] / slopes, steps.Target[e] % slopes);
                move = new Move(row, r1, table.Starts[r1] - table.Starts[row], y, slope, table.Places[r1][j], Slopes[hj]);
            }
            for (int b1 = bLow; b1 <= bHigh; b1++)
            {
                float rest = later[target + b1];
                if (rest == float.PositiveInfinity || !ways.Contains(tailThere[target + b1]))
                    continue;
                float v1 = MinSpeed + b1 * SpeedStep;
                float seconds = steps.Length[e] * LengthUnit / MathF.Max(0.5f * (speed + v1), 1f) + steps.Bend[e] * BendUnit;
                if (extra is not null)
                    seconds += extra(move, speed, v1);
                if (seconds + rest < bestSeconds)
                    (bestSeconds, bestRow, bestState) = (seconds + rest, r1, target + b1);
            }
        }
        return (bestSeconds, bestRow, bestState);
    }

    /// <summary>
    /// The same decision as <see cref="Decide"/>, every step out of the place
    /// shaped and driven here from the car's own speed instead of read off
    /// stored chains: what a table that keeps only its values asks of a
    /// decision. Timed as the table was baked, the driver's way or not.
    /// </summary>
    /// <summary>Ticks spent in each part of <see cref="DecideOnline"/>, and how often: shape, join, drive, brake, time.</summary>
    public readonly long[] OnlineTicks = new long[5];

    /// <summary>Join each way on only from the station it first changes (<see cref="DecideOnline"/>); off, every station again.</summary>
    public bool ShareJoin { get; set; } = true;
    public readonly long[] OnlineCalls = new long[5];

    public (float Seconds, int Row, int State) DecideOnline(Table table, int row, float y, float slope, float speed,
        Func<Move, float, float, float>? extra = null)
    {
        long mark = System.Diagnostics.Stopwatch.GetTimestamp();
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
        int place = table.Places[row].FindIndex(p => MathF.Abs(p - y) < 1e-3f);
        int heading = Array.IndexOf(Slopes, slope);
        if (place < 0 || heading < 0 || row >= table.Rows)
            return (bestSeconds, bestRow, bestState);
        int st0 = table.Starts[row];
        var ways = new List<ushort>();
        int most = table.Starts[Math.Min(table.Rows, row + Spans[^1])] - st0 + 1;
        var driven = new float[most];
        float[]?[] braked = new float[bins][];
        for (int sp = 0; sp < Spans.Length; sp++)
        {
            int r1 = row + Spans[sp];
            if (r1 > table.Rows)
                continue;
            int stations = table.Starts[r1] - st0;
            var edge = new Edge(stations, LeadMax + 1);
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
                    if (ways.Count == 0)
                        continue;
                    mark = System.Diagnostics.Stopwatch.GetTimestamp();
                    bool shaped = Shape(edge, st0, row, y, slope, there[j], Slopes[hj], null);
                    Lap(0);
                    if (!shaped)
                        continue;
                    float bent = MathF.Round(Bend(edge) / BendUnit) * BendUnit;
                    float length = MathF.Round(edge.Length / LengthUnit) * LengthUnit;
                    int split = ShareJoin ? Split(edge, st0) : 0;
                    foreach (ushort way in ways)
                    {
                        mark = System.Diagnostics.Stopwatch.GetTimestamp();
                        Join(edge, st0, way == NoTail ? null : table.Tails[r1][way], split);
                        Lap(1);
                        if (speed > edge.Limit[0] + 1e-3f)
                            continue;
                        int n = edge.N;
                        if (ExactSpeeds)
                        {
                            Array.Clear(braked);
                            DriveOn(edge, speed, driven);
                            Lap(2);
                            for (int b1 = 0; b1 < bins; b1++)
                            {
                                float v1 = MinSpeed + b1 * SpeedStep;
                                if (v1 > driven[n] || v1 > edge.Limit[n])
                                    break;
                                float rest = later[target + b1];
                                if (rest == float.PositiveInfinity || tailThere[target + b1] != way)
                                    continue;
                                float[] back = braked[b1] ??= new float[most];
                                mark = System.Diagnostics.Stopwatch.GetTimestamp();
                                BrakeBack(edge, v1, back);
                                Lap(3);
                                if (back[0] < speed)
                                    continue;
                                float seconds = bent;
                                for (int k = 0; k < n; k++)
                                    seconds += edge.Step[k] / MathF.Max(1f, 0.5f * (MathF.Min(driven[k], back[k]) + MathF.Min(driven[k + 1], back[k + 1])));
                                Lap(4);
                                if (extra is not null)
                                    seconds += extra(new Move(row, r1, stations, y, slope, there[j], Slopes[hj]), speed, v1);
                                if (seconds + rest < bestSeconds)
                                    (bestSeconds, bestRow, bestState) = (seconds + rest, r1, target + b1);
                            }
                            continue;
                        }
                        (float vLow, float vHigh) = Reach(edge, speed);
                        int bLow = Math.Max(0, (int)MathF.Ceiling((vLow - MinSpeed) / SpeedStep));
                        int bHigh = Math.Min(bins - 1, (int)MathF.Floor((vHigh - MinSpeed) / SpeedStep));
                        for (int b1 = bLow; b1 <= bHigh; b1++)
                        {
                            float rest = later[target + b1];
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
    }

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

    // Where across the road a car may be, as metres from the racing line.
    private (float Lo, float Hi) Room(int station)
    {
        float edge = _lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters;
        float free = _lanes.Offset(LaneId.Free, station);
        return (MathF.Min(0f, -(edge + free)), MathF.Max(0f, edge - free));
    }

    // One step of the lattice as the car drives it.
    private sealed class Edge(int stations, int ahead = 0)
    {
        /// <summary>How many stations the step crosses: up to as many as it was made for.</summary>
        public int N = stations;
        /// <summary>Its curve's ends: place and heading.</summary>
        public float Y0, H0, Y1, H1;
        public readonly float[] Offset = new float[stations + 1];
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
    }

    // The smooth curve from one place and heading to the next, kept on the
    // road; its corner, step lengths and the speed its corner allows at
    // each station. False if it leaves the road.
    private bool Shape(Edge edge, int st0, int row, float y0, float h0, float y1, float h1, Func<int, float, bool>? allowed)
    {
        int n = edge.N;
        float Y(int k) => Hermite(y0, h0, y1, h1, n, k);
        Vector2 P(int k) => _lanes.Position(st0 + k, _lanes.Offset(LaneId.Free, st0 + k) + Y(k));
        for (int k = 0; k <= n; k++)
        {
            float y = Y(k);
            (float lo, float hi) = Room(st0 + k);
            if (y < lo - 1e-3f || y > hi + 1e-3f)
                return false;
            // A row crossed between the ends still holds the line to what it allows.
            if (allowed is not null && k > 0 && k < n && k % RowStations == 0 && !allowed(row + k / RowStations, y))
                return false;
            edge.Offset[k] = y;
        }
        edge.Length = 0f;
        for (int k = 0; k < n; k++)
        {
            int st = st0 + k;
            edge.Curvature[k] = LaneSet.Curvature(P(k - Chord), P(k), P(k + Chord));
            edge.Step[k] = (P(k + 1) - P(k)).Length();
            edge.Length += edge.Step[k];
            edge.Factor[k] = _factor(st);
            edge.Road[k] = RoadAt(st, edge.Offset[k]);
        }
        edge.Factor[n] = _factor(st0 + n);
        edge.Road[n] = RoadAt(st0 + n, edge.Offset[n]);
        edge.Step[n] = 0f;
        (edge.Y0, edge.H0, edge.Y1, edge.H1) = (y0, h0, y1, h1);
        if (LeadSpeed is not null)
            for (int k = 0; k <= n; k++)
                edge.Guess[k] = LeadSpeed(st0 + k);
        Join(edge, st0, null);
        return true;
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
    private void Join(Edge edge, int st0, float[]? tail, int from = 0)
    {
        int n = edge.N;
        Vector2 P(int k)
        {
            float y = tail is not null && k > n && k - n < tail.Length ? tail[k - n] : Hermite(edge.Y0, edge.H0, edge.Y1, edge.H1, n, k);
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
        for (int k = from; k < m; k++)
        {
            float kappa = edge.Corner[k];
            edge.Limit[k] = LaneSpeedPlanner.CornerLimit(_envelope.ForTurn(kappa), kappa, edge.Factor[k] * _shares.Lateral, edge.Road[k]);
        }
    }

    // The slowest and the fastest the car can be at the far end of a step
    // from v0: braking all the way, or full drive held under each corner;
    // and no faster than a steady change of v² from v0 keeps every station
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

    // No faster than a steady change of v² from v0 keeps every station on
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
    private void DriveOn(Edge edge, float v0, float[] into)
    {
        into[0] = MathF.Min(edge.Limit[0], v0);
        for (int k = 0; k < edge.N; k++)
            into[k + 1] = MathF.Min(edge.Limit[k + 1], LaneSpeedPlanner.Driven(_envelope, into[k], edge.Corner[k], edge.Step[k], edge.Factor[k], edge.Road[k], _shares));
    }

    // Braking back from v1 at the far end under each station's corner, the
    // backward sweep of the driver's plan.
    private void BrakeBack(Edge edge, float v1, float[] into)
    {
        int n = edge.N;
        into[n] = MathF.Min(edge.Limit[n], v1);
        for (int k = n - 1; k >= 0; k--)
            into[k] = MathF.Min(edge.Limit[k], LaneSpeedPlanner.BrakeFrom(_envelope, into[k + 1], edge.Corner[k], edge.Step[k], edge.Factor[k], 1f, edge.Road[k], _shares));
    }

    // The first station of a step whose corner the lead reads past the
    // stations its own curve alone settles: from there on the way on counts.
    private int Split(Edge edge, int st0)
    {
        int n = edge.N, settled = n - Chord;
        for (int k = 0; k < n; k++)
            if (k + LeadStations(st0 + k) >= settled)
                return k;
        return n;
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

    // The same off the stored step: only the samples either side are read.
    private (float Low, float High) ChainAt(Steps steps, int e, int at, int count, float bin)
    {
        (int q, int pairs, float t) = Locate(count, steps.Bottom[e], steps.Top[e], bin);
        Span<float> near = stackalloc float[4];
        Unpack(steps, e, at, 2 * q, near[..(2 * pairs)]);
        if (pairs == 1)
            return (near[0], near[1]);
        return (near[0] + (near[2] - near[0]) * t, near[1] + (near[3] - near[1]) * t);
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

    // A step's chains read back as the bake stored them.
    private void Unpack(Steps steps, int e, int at, int from, Span<float> into)
    {
        for (int i = 0; i < into.Length; i++)
        {
            int k = from + i;
            float v0 = SampleSpeed(steps.Bottom[e], steps.Top[e], k / 2);
            byte stored = steps.Chains[at + k];
            if (k % 2 == 0)
                into[i] = stored == SlowEscape ? steps.Escaped[at + k] : v0 - stored * ChainUnit;
            else
                into[i] = unchecked((sbyte)stored) switch
                {
                    NoSpeed => -1f,
                    Escape => steps.Escaped[at + k],
                    var q => v0 + q * ChainUnit,
                };
        }
    }

    // The step taken at a steady change of v² from v0 to v1, which is a
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

    /// <summary>The speed a step from <paramref name="v0"/> to <paramref name="v1"/> over so many stations is timed at, a station into it: v² changing steadily.</summary>
    public static float SpeedOn(float v0, float v1, int stations, int station)
    {
        float t = station / (float)stations;
        return MathF.Sqrt(v0 * v0 + (v1 * v1 - v0 * v0) * t);
    }
}
