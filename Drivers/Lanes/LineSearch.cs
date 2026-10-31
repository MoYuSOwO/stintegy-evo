using System.Numerics;

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
                            if (!any || !Shape(edge, from, r0, here[i], Slopes[hi], there[j], Slopes[hj], allowed))
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
                                    float seconds = Seconds(edge, v0, v1);
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
            Shape(shape, from, r0, places[r0][i], Slopes[hi], places[r1][j], Slopes[hj], null);
            float v0 = MinSpeed + b0 * SpeedStep, v1 = MinSpeed + b1 * SpeedStep;
            for (int k = 0; k <= shape.Step.Length; k++)
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
        public required List<float>[] Places { get; init; }
        public required int Bins { get; init; }
        public required float[][] Value { get; init; }
        public required int[][] NextRow { get; init; }
        public required int[][] Next { get; init; }
    }

    /// <summary>
    /// Bake the stretch of <paramref name="rows"/> rows from
    /// <paramref name="from"/> from its end back: it ends on the racing line,
    /// the road beyond charged by <paramref name="terminal"/>. Speeds below
    /// <paramref name="slowest"/> at a station are not baked there: no car
    /// racing comes by that slow, and one that does is rejoining, not racing.
    /// </summary>
    public Table Bake(int from, int rows, Func<float, float> terminal, Func<int, float>? slowest = null)
    {
        int bins = (int)MathF.Ceiling((LaneSpeedPlanner.TopSpeed - MinSpeed) / SpeedStep) + 1;
        int slopes = Slopes.Length;
        int zeroSlope = Array.IndexOf(Slopes, 0f);
        var places = new List<float>[rows + 1];
        for (int r = 0; r <= rows; r++)
        {
            (float lo, float hi) = Room(from + r * RowStations);
            var ys = new List<float>();
            for (float y = MathF.Ceiling(lo / LateralStep) * LateralStep; y <= hi + 1e-3f; y += LateralStep)
                if (r < rows || MathF.Abs(y) < 1e-3f)
                    ys.Add(y);
            places[r] = ys;
        }
        var value = new float[rows + 1][];
        var nextRow = new int[rows + 1][];
        var next = new int[rows + 1][];
        for (int r = 0; r <= rows; r++)
        {
            value[r] = new float[places[r].Count * slopes * bins];
            nextRow[r] = new int[value[r].Length];
            next[r] = new int[value[r].Length];
            Array.Fill(value[r], float.PositiveInfinity);
            Array.Fill(next[r], -1);
        }
        int end = places[rows].FindIndex(y => MathF.Abs(y) < 1e-3f);
        for (int b = 0; b < bins; b++)
            value[rows][(end * slopes + zeroSlope) * bins + b] = terminal(MinSpeed + b * SpeedStep);

        for (int r0 = rows - 1; r0 >= 0; r0--)
        {
            int row0 = r0;
            List<float> here = places[r0];
            Parallel.For(0, here.Count * slopes, source =>
            {
                int i = source / slopes, hi = source % slopes;
                float[] best = value[row0];
                for (int sp = 0; sp < Spans.Length; sp++)
                {
                    int r1 = row0 + Spans[sp];
                    if (r1 > rows)
                        continue;
                    List<float> there = places[r1];
                    float[] later = value[r1];
                    var edge = new Edge(Spans[sp] * RowStations);
                    var lows = new float[bins];
                    var highs = new float[bins];
                    for (int j = 0; j < there.Count; j++)
                    {
                        if (MathF.Abs(there[j] - here[i]) > MaxLateralJumps[sp] + 1e-3f)
                            continue;
                        for (int hj = 0; hj < slopes; hj++)
                        {
                            int target = (j * slopes + hj) * bins;
                            bool any = false;
                            for (int b = 0; b < bins && !any; b++)
                                any = later[target + b] < float.PositiveInfinity;
                            if (!any || !Shape(edge, from, row0, here[i], Slopes[hi], there[j], Slopes[hj], null))
                                continue;
                            int top = Math.Min(bins - 1, (int)MathF.Floor((edge.Limit[0] + 1e-3f - MinSpeed) / SpeedStep));
                            if (top < 0)
                                continue;
                            int bottom = slowest is null ? 0 : Math.Clamp((int)MathF.Ceiling((slowest(from + row0 * RowStations) - MinSpeed) / SpeedStep), 0, top);
                            AllChains(edge, bottom, top, lows, highs);
                            for (int b0 = bottom; b0 <= top; b0++)
                            {
                                float v0 = MinSpeed + b0 * SpeedStep;
                                float vLow = lows[b0], vHigh = Held(edge, v0, highs[b0]);
                                int bLow = Math.Max(0, (int)MathF.Ceiling((vLow - MinSpeed) / SpeedStep));
                                int bHigh = Math.Min(bins - 1, (int)MathF.Floor((vHigh - MinSpeed) / SpeedStep));
                                int at = source * bins + b0;
                                for (int b1 = bLow; b1 <= bHigh; b1++)
                                {
                                    float rest = later[target + b1];
                                    if (rest == float.PositiveInfinity)
                                        continue;
                                    float seconds = Seconds(edge, v0, MinSpeed + b1 * SpeedStep);
                                    if (seconds + rest < best[at])
                                    {
                                        best[at] = seconds + rest;
                                        nextRow[row0][at] = r1;
                                        next[row0][at] = target + b1;
                                    }
                                }
                            }
                        }
                    }
                }
            });
        }
        return new Table { From = from, Rows = rows, Places = places, Bins = bins, Value = value, NextRow = nextRow, Next = next };
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
        int n = (table.Rows - first) * RowStations + 1;
        var offset = new float[n];
        var speed = new float[n];
        for (int p = 0; p + 1 < path.Count; p++)
        {
            (int r0, int s0) = path[p];
            (int r1, int s1) = path[p + 1];
            (int i, int hi, int b0) = Split(s0, slopes, table.Bins);
            (int j, int hj, int b1) = Split(s1, slopes, table.Bins);
            var shape = new Edge((r1 - r0) * RowStations);
            Shape(shape, table.From, r0, table.Places[r0][i], Slopes[hi], table.Places[r1][j], Slopes[hj], null);
            float v0 = MinSpeed + b0 * SpeedStep, v1 = MinSpeed + b1 * SpeedStep;
            for (int k = 0; k <= shape.Step.Length; k++)
            {
                offset[(r0 - first) * RowStations + k] = shape.Offset[k];
                speed[(r0 - first) * RowStations + k] = Along(v0, v1, shape, k);
            }
        }
        float endSpeed = MinSpeed + Split(path[^1].State, slopes, table.Bins).Bin * SpeedStep;
        float terminal = table.Value[table.Rows][path[^1].State];
        return new Way(table.From + first * RowStations, offset, speed, total - terminal, endSpeed, terminal);
    }

    /// <summary>
    /// The step to take now, from a car at a place and heading on a row and
    /// at any speed: every step out of there and every speed it could end
    /// at, each charged its own seconds and the baked rest of the way; and
    /// <paramref name="extra"/>, what the cars around add to a step (infinite
    /// where it runs into one). The seconds, the row it ends on, and the
    /// state there.
    /// </summary>
    public (float Seconds, int Row, int State) Decide(Table table, int row, float y, float slope, float speed,
        Func<int, int, float[], float, float, float>? extra = null)
    {
        int slopes = Slopes.Length;
        int bins = table.Bins;
        float bestSeconds = float.PositiveInfinity;
        int bestRow = -1, bestState = -1;
        int sourceAt = table.Places[row].FindIndex(p => MathF.Abs(p - y) < 1e-3f);
        if (sourceAt < 0)
            return (bestSeconds, bestRow, bestState);
        for (int sp = 0; sp < Spans.Length; sp++)
        {
            int r1 = row + Spans[sp];
            if (r1 > table.Rows)
                continue;
            List<float> there = table.Places[r1];
            float[] later = table.Value[r1];
            var edge = new Edge(Spans[sp] * RowStations);
            for (int j = 0; j < there.Count; j++)
            {
                if (MathF.Abs(there[j] - y) > MaxLateralJumps[sp] + 1e-3f)
                    continue;
                for (int hj = 0; hj < slopes; hj++)
                {
                    int target = (j * slopes + hj) * bins;
                    if (!Shape(edge, table.From, row, y, slope, there[j], Slopes[hj], null) || speed > edge.Limit[0] + 1e-3f)
                        continue;
                    (float vLow, float vHigh) = Reach(edge, speed);
                    int bLow = Math.Max(0, (int)MathF.Ceiling((vLow - MinSpeed) / SpeedStep));
                    int bHigh = Math.Min(bins - 1, (int)MathF.Floor((vHigh - MinSpeed) / SpeedStep));
                    for (int b1 = bLow; b1 <= bHigh; b1++)
                    {
                        float rest = later[target + b1];
                        if (rest == float.PositiveInfinity)
                            continue;
                        float v1 = MinSpeed + b1 * SpeedStep;
                        float seconds = Seconds(edge, speed, v1);
                        if (extra is not null)
                            seconds += extra(row, r1, edge.Offset, speed, v1);
                        if (seconds + rest < bestSeconds)
                            (bestSeconds, bestRow, bestState) = (seconds + rest, r1, target + b1);
                    }
                }
            }
        }
        return (bestSeconds, bestRow, bestState);
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
    private sealed class Edge(int stations)
    {
        public readonly float[] Offset = new float[stations + 1];
        public readonly float[] Curvature = new float[stations];
        public readonly float[] Step = new float[stations];
        public readonly float[] Limit = new float[stations];
        public readonly float[] Factor = new float[stations];
        public readonly LaneRoad[] Road = new LaneRoad[stations];
        public float Length;
    }

    // The smooth curve from one place and heading to the next, kept on the
    // road; its corner, step lengths and the speed its corner allows at
    // each station. False if it leaves the road.
    private bool Shape(Edge edge, int from, int row, float y0, float h0, float y1, float h1, Func<int, float, bool>? allowed)
    {
        int n = edge.Step.Length;
        int st0 = from + row * RowStations;
        float length = n * _lanes.Spacing;
        float Y(int k)
        {
            float t = k / (float)n;
            float t2 = t * t, t3 = t2 * t;
            return (2f * t3 - 3f * t2 + 1f) * y0 + (t3 - 2f * t2 + t) * length * h0 +
                   (-2f * t3 + 3f * t2) * y1 + (t3 - t2) * length * h1;
        }
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
            float kappa = LaneSet.Curvature(P(k - Chord), P(k), P(k + Chord));
            edge.Curvature[k] = kappa;
            edge.Step[k] = (P(k + 1) - P(k)).Length();
            edge.Length += edge.Step[k];
            edge.Factor[k] = _factor(st);
            edge.Road[k] = RoadAt(st, edge.Offset[k]);
            edge.Limit[k] = LaneSpeedPlanner.CornerLimit(_envelope.ForTurn(kappa), kappa, edge.Factor[k] * _shares.Lateral, edge.Road[k]);
        }
        return true;
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
        float low = v0, high = v0;
        for (int k = 0; k < edge.Step.Length; k++)
        {
            float decel = LaneSpeedPlanner.Deceleration(_envelope, low, edge.Curvature[k], edge.Factor[k], 1f, edge.Road[k], _shares);
            low = MathF.Sqrt(MathF.Max(0f, low * low - 2f * decel * edge.Step[k]));
            high = LaneSpeedPlanner.Driven(_envelope, MathF.Min(high, edge.Limit[k]), edge.Curvature[k], edge.Step[k], edge.Factor[k], edge.Road[k], _shares);
        }
        return (low, high);
    }

    // No faster than a steady change of v² from v0 keeps every station on
    // the way inside the corner its tyres hold; negative if none does.
    private static float Held(Edge edge, float v0, float high)
    {
        int n = edge.Step.Length;
        float rise = float.PositiveInfinity;
        for (int k = 1; k < n; k++)
        {
            float limit = edge.Limit[k] + 1e-3f;
            rise = MathF.Min(rise, (limit * limit - v0 * v0) * n / k);
        }
        if (rise == float.PositiveInfinity)
            return high;
        return v0 * v0 + rise < 0f ? -1f : MathF.Min(high, MathF.Sqrt(v0 * v0 + rise));
    }

    /// <summary>
    /// How many speed bins apart the bake rides a step's chains; the bins
    /// between take them in a straight line. They bend gently with the
    /// speed they start from, and a step's whole cost is in them.
    /// </summary>
    public int ChainStride { get; init; } = 8;

    // The chains for every speed bin from bottom to top.
    private void AllChains(Edge edge, int bottom, int top, float[] low, float[] high)
    {
        int last = -1;
        for (int b = bottom; ; b = Math.Min(b + ChainStride, top))
        {
            (low[b], high[b]) = Chains(edge, MinSpeed + b * SpeedStep);
            if (last >= 0)
                for (int m = last + 1; m < b; m++)
                {
                    float t = (m - last) / (float)(b - last);
                    low[m] = low[last] + (low[b] - low[last]) * t;
                    high[m] = high[last] + (high[b] - high[last]) * t;
                }
            last = b;
            if (b == top)
                break;
        }
    }

    // The step taken at a steady change of v² from v0 to v1, which is a
    // steady acceleration: its length over the mean of its ends.
    private static float Seconds(Edge edge, float v0, float v1) =>
        edge.Length / MathF.Max(0.5f * (v0 + v1), 1f);

    private static float Along(float v0, float v1, Edge edge, int k)
    {
        float t = k / (float)edge.Step.Length;
        return MathF.Sqrt(v0 * v0 + (v1 * v1 - v0 * v0) * t);
    }
}
