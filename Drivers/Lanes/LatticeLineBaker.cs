using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// How a lattice lap is searched. Places sit on the usable road, edge to
/// edge, so an apex can lie on the kerb rather than a metre short of it.
/// <see cref="EvenPlaces"/> of 2 or more spaces that many places evenly
/// between the edges; 0 uses a <see cref="LateralStep"/> grid that still
/// includes both edges.
/// </summary>
public sealed record LatticeBakeOptions
{
    /// <summary>Stations from one row of the lattice to the next. Five is ten metres at the usual spacing.</summary>
    public int RowStations { get; init; } = 5;

    /// <summary>Metres between places when <see cref="EvenPlaces"/> is 0.</summary>
    public float LateralStep { get; init; } = 1f;

    /// <summary>Places across a row, edge to edge. Zero selects the lateral-step grid. One follows the geometric line.</summary>
    public int EvenPlaces { get; init; } = 5;

    /// <summary>
    /// How far from the geometric line a place may sit, metres. The road edges
    /// stay reachable wherever they are closer than this. Zero searches the
    /// whole road; a few metres is the band a racing line actually uses.
    /// </summary>
    public float SearchBandMeters { get; init; } = 4f;

    /// <summary>How many rows one smooth step may cross.</summary>
    public int[] Spans { get; init; } = [1, 2, 4, 8];

    /// <summary>The furthest a step of each span may move the car across the road, metres.</summary>
    public float[] MaxLateralJumps { get; init; } = [3f, 4f, 6f, 8f];

    /// <summary>Headings at a row, metres aside per metre along. Must contain 0.</summary>
    public float[] Slopes { get; init; } = [-0.2f, -0.1f, -0.05f, -0.025f, 0f, 0.025f, 0.05f, 0.1f, 0.2f];

    /// <summary>Metres a second between the speeds a row is searched at.</summary>
    public float SpeedStep { get; init; } = 1f;

    /// <summary>Slowest speed a row keeps, metres a second.</summary>
    public float MinSpeed { get; init; } = 10f;

    /// <summary>
    /// Time each step the way the driver plans it: drive forward from the
    /// speed it starts at, brake back from the speed it ends at, and take
    /// the lower one. Off, a step is a steady change of v² held under the
    /// corner limits.
    /// </summary>
    public bool ExactSpeeds { get; init; } = true;

    /// <summary>
    /// Read the corner a step is planned on as the sharpest the driver's
    /// turn lag reaches ahead, the way <see cref="LaneSpeedPlanner.LeadCurvatures"/>
    /// does. Off, a station is planned on the road under it.
    /// </summary>
    public bool UseTurnLag { get; init; } = true;

    /// <summary>How many times the lap's end is recharged with the value of starting the next lap from it.</summary>
    public int ClosurePasses { get; init; } = 3;

    /// <summary>Metres of road a side lane leaves on its other side.</summary>
    public float SideRoomMeters { get; init; } = 3f;

    /// <summary>
    /// Half-width of the band the curvature polish may move a lattice line
    /// inside, metres. Zero leaves the lattice knots as they were.
    /// </summary>
    public float PolishBandMeters { get; init; } = 0.25f;

    /// <summary>Seconds charged per m⁻¹ of curvature squared along a step. Zero is pure time.</summary>
    public float BendSeconds { get; init; } = 0f;
}

/// <summary>
/// The minimum-time line over a lattice of places, headings and speeds,
/// baked offline. A step is a cubic between two places; what it costs is
/// the time the driver's own speed plan spends on it. Nothing here is
/// driven online, and nothing is written to a table file.
/// </summary>
public static class LatticeLineBaker
{
    /// <summary>
    /// The fastest way over <paramref name="rows"/> rows from
    /// <paramref name="fromStation"/>, starting at <paramref name="startSpeed"/>.
    /// Offsets are metres from the centreline. <paramref name="terminalCost"/>
    /// is the seconds the road beyond costs from the speed the stretch ends
    /// at. <paramref name="allowedOffset"/> may refuse a station: a corridor
    /// or a pin. Null when no way stays on the road.
    /// </summary>
    public static (float[] OffsetsFromCentreline, float[] Speeds, float Seconds)? BakeStretch(
        TrackData track, LaneSet lanes, ICar car, int fromStation, int rows, float startSpeed,
        Func<float, float> terminalCost, float? startOffset = null, float? endOffset = null,
        Func<int, float, bool>? allowedOffset = null, LatticeBakeOptions? options = null)
    {
        if (rows < 1)
            throw new ArgumentOutOfRangeException(nameof(rows));
        var search = new Search(track, lanes, car, options ?? new LatticeBakeOptions(), allowedOffset);
        return search.Stretch(fromStation, rows, startSpeed, terminalCost, startOffset, endOffset);
    }

    /// <summary>The minimum-time line once round <paramref name="lanes"/>, offsets from the centreline.</summary>
    public static (float[] OffsetsFromCentreline, float[] Speeds, float Seconds)? BakeFreeLine(
        TrackData track, LaneSet lanes, ICar car, LatticeBakeOptions? options = null,
        Func<int, float, bool>? allowedOffset = null)
    {
        var search = new Search(track, lanes, car, options ?? new LatticeBakeOptions(), allowedOffset);
        float[]? line = search.ClosedLap(search.RoadBounds);
        if (line is null)
            return null;
        line = search.Polish(line, search.RoadBounds, periodic: true, origin: 0);
        (float[] speeds, float seconds) = search.TimeLine(line, periodic: true, 0f, 0f);
        return (line, speeds, seconds);
    }

    /// <summary>
    /// A lap the driver can race: the geometric lanes as the road, then the
    /// minimum-time free line, the left and right corridors (each leaving
    /// <see cref="LatticeBakeOptions.SideRoomMeters"/> on the far side), the
    /// middle halfway between those, and a narrow curvature polish so the
    /// knots meet smoothly without walking off the line the lattice chose.
    /// </summary>
    public static LaneSet BakeLaneSet(TrackData track, ICar car, LatticeBakeOptions? options = null)
    {
        LatticeBakeOptions o = options ?? new LatticeBakeOptions();
        LaneSet lanes = LaneSet.Bake(track);
        var search = new Search(track, lanes, car, o, null);
        float[] free = search.ClosedLap(search.RoadBounds) ?? throw new InvalidOperationException("The free line did not close.");
        float[] left = search.ClosedLap(search.LeftBounds) ?? throw new InvalidOperationException("The left line did not close.");
        float[] right = search.ClosedLap(search.RightBounds) ?? throw new InvalidOperationException("The right line did not close.");
        free = search.Polish(free, search.RoadBounds, true, 0);
        left = search.Polish(left, search.LeftBounds, true, 0);
        right = search.Polish(right, search.RightBounds, true, 0);
        var geometric = new float[lanes.Count];
        for (int i = 0; i < geometric.Length; i++)
            geometric[i] = lanes.Offset(LaneId.Free, i);
        free = search.KeepFaster(geometric, free);
        var middle = new float[lanes.Count];
        for (int i = 0; i < middle.Length; i++)
            middle[i] = 0.5f * (left[i] + right[i]);
        return lanes
            .WithOffsets(LaneId.Free, free)
            .WithOffsets(LaneId.Left, left)
            .WithOffsets(LaneId.Right, right)
            .WithOffsets(LaneId.Middle, middle);
    }

    private sealed class Search
    {
        private const int Chord = 3;
        private const int GripSamples = 6;
        private const float RoadGrid = 0.5f;

        private readonly LaneSet _lanes;
        private readonly LatticeBakeOptions _o;
        private readonly LaneEnvelope _envelope;
        private readonly LaneShares _shares = new();
        private readonly Func<int, float, bool>? _allowed;
        private readonly Func<float, float>? _lag;
        private readonly float[] _leadSpeed;
        private readonly LaneRoad[][] _roads;
        private readonly float[] _roadLo;
        private readonly int _zeroSlope;
        private readonly int _bins;
        private readonly int _leadMax;
        private readonly float _sample0;
        private readonly float _sampleDv;

        private Func<int, (float Lo, float Hi)> _bounds = null!;
        private int[] _starts = null!;
        private List<float>[] _places = null!;
        private float[][] _value = null!;
        private int[][] _nextRow = null!;
        private int[][] _next = null!;
        private float[]?[][] _tails = null!;
        private int _rows;

        public Search(TrackData track, LaneSet lanes, ICar car, LatticeBakeOptions options, Func<int, float, bool>? allowed)
        {
            if (options.SpeedStep <= 0f || options.RowStations < 1 || options.Spans.Length == 0 || options.Slopes.Length == 0)
                throw new ArgumentException("Lattice options need a positive speed step, row spacing, span and slope.");
            _zeroSlope = Array.IndexOf(options.Slopes, 0f);
            if (_zeroSlope < 0)
                throw new ArgumentException("Slopes must include 0.", nameof(options));
            _lanes = lanes;
            _o = options;
            _allowed = allowed;
            _bins = (int)MathF.Ceiling((LaneSpeedPlanner.TopSpeed - options.MinSpeed) / options.SpeedStep) + 1;
            if (_bins < 2 || _bins > 401)
                throw new ArgumentException($"Speed step {options.SpeedStep} gives {_bins} bins.", nameof(options));

            var state = new CarState();
            var tyres = (ICarTires)car;
            tyres.InstallFreshTires(state);
            tyres.WarmTires(state, 95f, 95f);
            (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
            _envelope = LaneEnvelope.From(state, (ICarEnvelope)car, (ICarGripFeel)car, CarStrategy.Default, turnAware: true)
                .WithRoadGrip(track.AsphaltGrip);
            if (options.UseTurnLag)
                _lag = TurnLag.Measure(car).Seconds;

            int n = lanes.Count;
            _roads = new LaneRoad[n][];
            _roadLo = new float[n];
            for (int i = 0; i < n; i++)
            {
                (float lo, float hi) = RoadBounds(i);
                _roadLo[i] = lo;
                TrackSample sample = track.Sample(i * lanes.Spacing);
                var across = new LaneRoad[(int)MathF.Ceiling((hi - lo) / RoadGrid) + 1];
                for (int g = 0; g < across.Length; g++)
                {
                    float y = lo + g * RoadGrid;
                    across[g] = new LaneRoad(sample.Grade, sample.BankSlopeAt(y), sample.VerticalRate);
                }
                _roads[i] = across;
            }

            _leadSpeed = new float[n];
            Array.Fill(_leadSpeed, 40f);
            if (_lag is not null)
                PlanLeadGuess();
            _leadMax = 0;
            if (_lag is not null)
            {
                for (int i = 0; i < n; i++)
                    _leadMax = Math.Max(_leadMax, (int)MathF.Ceiling(MathF.Max(0f, _leadSpeed[i]) * _lag(_leadSpeed[i]) / lanes.Spacing));
                _leadMax = Math.Min(_leadMax, 48);
            }
            _sample0 = options.MinSpeed;
            _sampleDv = (LaneSpeedPlanner.TopSpeed - options.MinSpeed) / (GripSamples - 1);
        }

        public (float Lo, float Hi) RoadBounds(int station)
        {
            float edge = MathF.Max(0f, _lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters);
            return (-edge, edge);
        }

        public (float Lo, float Hi) LeftBounds(int station)
        {
            (float lo, float hi) = RoadBounds(station);
            float edge = hi;
            return (MathF.Min(edge, -edge + _o.SideRoomMeters), hi);
        }

        public (float Lo, float Hi) RightBounds(int station)
        {
            (float lo, float hi) = RoadBounds(station);
            float edge = hi;
            return (lo, MathF.Max(-edge, edge - _o.SideRoomMeters));
        }

        public (float[] OffsetsFromCentreline, float[] Speeds, float Seconds)? Stretch(
            int fromStation, int rows, float startSpeed, Func<float, float> terminalCost, float? startOffset, float? endOffset)
        {
            _bounds = RoadBounds;
            int from = _lanes.Wrap(fromStation);
            _stretchFrom = from;
            _rows = rows;
            _starts = new int[rows + 1];
            for (int r = 0; r <= rows; r++)
                _starts[r] = from + r * _o.RowStations;
            float y0 = startOffset ?? _lanes.Offset(LaneId.Free, _starts[0]);
            float y1 = endOffset ?? _lanes.Offset(LaneId.Free, _starts[rows]);
            _places = new List<float>[rows + 1];
            for (int r = 0; r <= rows; r++)
            {
                bool pin = r == 0 || r == rows;
                _places[r] = pin
                    ? [r == 0 ? y0 : y1]
                    : Places(_starts[r]);
                if (_places[r].Count == 0)
                    return null;
            }
            int zero = _zeroSlope;
            if (!Bake(state =>
                {
                    int slope = state / _bins % _o.Slopes.Length;
                    int bin = state % _bins;
                    return slope == zero ? terminalCost(_o.MinSpeed + bin * _o.SpeedStep) : float.PositiveInfinity;
                }))
                return null;

            int startBin = Math.Clamp((int)MathF.Round((startSpeed - _o.MinSpeed) / _o.SpeedStep), 0, _bins - 1);
            int start = zero * _bins + startBin;
            if (float.IsPositiveInfinity(_value[0][start]))
                return null;
            List<(int Row, int State)>? path = Walk(0, start);
            if (path is null)
                return null;
            int count = rows * _o.RowStations + 1;
            var offset = new float[count];
            Draw(path, offset, lap: false);
            float endSpeed = _o.MinSpeed + (path[^1].State % _bins) * _o.SpeedStep;
            offset = Polish(offset, RoadBounds, periodic: false, origin: from);
            (float[] speeds, float seconds) = TimeLine(offset, periodic: false, startSpeed, endSpeed);
            return (offset, speeds, seconds);
        }

        public float[]? ClosedLap(Func<int, (float Lo, float Hi)> bounds)
        {
            _bounds = bounds;
            int rows = _lanes.Count / _o.RowStations;
            if (rows < 2)
                return null;
            _rows = rows;
            _starts = new int[rows + 1];
            for (int r = 0; r < rows; r++)
                _starts[r] = r * _o.RowStations;
            _starts[rows] = _lanes.Count;
            _places = new List<float>[rows + 1];
            for (int r = 0; r < rows; r++)
            {
                _places[r] = Places(_starts[r]);
                if (_places[r].Count == 0)
                    return null;
            }
            _places[rows] = _places[0];

            float[]? terminal = null;
            int passes = Math.Max(1, _o.ClosurePasses);
            for (int pass = 0; pass < passes; pass++)
            {
                float[]? carried = terminal;
                if (!Bake(state => carried is null ? 0f : carried[state]))
                    return null;
                terminal = (float[])_value[0].Clone();
            }

            List<(int Row, int State)>? cycle = Cycle();
            if (cycle is null)
                return null;
            var offset = new float[_lanes.Count];
            Draw(cycle, offset, lap: true);
            return offset;
        }

        private bool Bake(Func<int, float> terminal)
        {
            int slopes = _o.Slopes.Length;
            _value = new float[_rows + 1][];
            _nextRow = new int[_rows + 1][];
            _next = new int[_rows + 1][];
            _tails = new float[]?[_rows + 1][];
            for (int r = 0; r <= _rows; r++)
            {
                int m = _places[r].Count * slopes * _bins;
                _value[r] = new float[m];
                _nextRow[r] = new int[m];
                _next[r] = new int[m];
                Array.Fill(_value[r], float.PositiveInfinity);
                Array.Fill(_next[r], -1);
                _tails[r] = new float[]?[_places[r].Count * slopes];
            }
            for (int s = 0; s < _value[_rows].Length; s++)
                _value[_rows][s] = terminal(s);

            int maxN = 0;
            for (int r = 0; r < _rows; r++)
                for (int sp = 0; sp < _o.Spans.Length; sp++)
                {
                    int r1 = r + _o.Spans[sp];
                    if (r1 <= _rows)
                        maxN = Math.Max(maxN, _starts[r1] - _starts[r]);
                }
            int tail = _leadMax + Chord + 1;

            for (int r0 = _rows - 1; r0 >= 0; r0--)
            {
                int row = r0;
                List<float> here = _places[row];
                Parallel.For(0, here.Count * slopes, source =>
                {
                    int place = source / slopes, slope = source % slopes;
                    Edge edge = EdgeFor(maxN, tail);
                    float[] best = _value[row];
                    int[] nextRow = _nextRow[row], next = _next[row];
                    int sourceBase = source * _bins;
                    for (int sp = 0; sp < _o.Spans.Length; sp++)
                    {
                        int r1 = row + _o.Spans[sp];
                        if (r1 > _rows)
                            continue;
                        float jump = sp < _o.MaxLateralJumps.Length ? _o.MaxLateralJumps[sp] : _o.MaxLateralJumps[^1];
                        List<float> there = _places[r1];
                        float[] later = _value[r1];
                        int n = _starts[r1] - _starts[row];
                        for (int j = 0; j < there.Count; j++)
                        {
                            if (MathF.Abs(there[j] - here[place]) > jump + 1e-3f)
                                continue;
                            for (int hj = 0; hj < slopes; hj++)
                            {
                                int targetBase = (j * slopes + hj) * _bins;
                                bool any = false;
                                for (int b = 0; b < _bins; b++)
                                    if (later[targetBase + b] < float.PositiveInfinity)
                                    {
                                        any = true;
                                        break;
                                    }
                                if (!any)
                                    continue;
                                float[]? on = _tails[r1][j * slopes + hj];
                                if (!edge.Fit(this, _starts[row], n, here[place], _o.Slopes[slope], there[j], _o.Slopes[hj], on))
                                    continue;
                                edge.Price(this, later, targetBase, best, sourceBase, nextRow, next, r1);
                            }
                        }
                    }
                });
                RememberTails(row);
            }
            return true;
        }

        private void RememberTails(int row)
        {
            int slopes = _o.Slopes.Length;
            int length = _leadMax + Chord + 1;
            for (int node = 0; node < _places[row].Count * slopes; node++)
            {
                int bestBin = -1;
                float best = float.PositiveInfinity;
                int at = node * _bins;
                for (int b = 0; b < _bins; b++)
                    if (_value[row][at + b] < best)
                        (best, bestBin) = (_value[row][at + b], b);
                if (bestBin < 0)
                    continue;
                int state = at + bestBin;
                int r1 = _nextRow[row][state];
                int s1 = _next[row][state];
                if (s1 < 0)
                    continue;
                int j = s1 / _bins / slopes, hj = s1 / _bins % slopes;
                int n = _starts[r1] - _starts[row];
                float y0 = _places[row][node / slopes], h0 = _o.Slopes[node % slopes];
                float y1 = _places[r1][j], h1 = _o.Slopes[hj];
                float[]? after = _tails[r1][j * slopes + hj];
                var line = new float[length];
                float len = n * _lanes.Spacing;
                for (int k = 0; k < length; k++)
                    line[k] = k <= n ? Hermite(y0, h0, y1, h1, len, n, k)
                        : after is not null && k - n < after.Length ? after[k - n]
                        : Hermite(y0, h0, y1, h1, len, n, k);
                _tails[row][node] = line;
            }
        }

        private List<(int Row, int State)>? Cycle()
        {
            int state = -1;
            float best = float.PositiveInfinity;
            for (int s = 0; s < _value[0].Length; s++)
                if (_value[0][s] < best)
                    (best, state) = (_value[0][s], s);
            if (state < 0)
                return null;
            var seen = new HashSet<int>();
            for (int lap = 0; lap < 16 && seen.Add(state); lap++)
            {
                List<(int Row, int State)>? once = Walk(0, state);
                if (once is null)
                    return null;
                state = once[^1].State;
            }
            return Walk(0, state);
        }

        private List<(int Row, int State)>? Walk(int row, int state)
        {
            var path = new List<(int Row, int State)>();
            for (int guard = 0; guard <= _rows + 1; guard++)
            {
                path.Add((row, state));
                if (row == _rows)
                    return path;
                int s1 = _next[row][state];
                int r1 = _nextRow[row][state];
                if (s1 < 0)
                    return null;
                (row, state) = (r1, s1);
            }
            return null;
        }

        private void Draw(List<(int Row, int State)> path, float[] offset, bool lap)
        {
            int slopes = _o.Slopes.Length;
            for (int p = 0; p + 1 < path.Count; p++)
            {
                (int r0, int s0) = path[p];
                (int r1, int s1) = path[p + 1];
                int i = s0 / _bins / slopes, hi = s0 / _bins % slopes;
                int j = s1 / _bins / slopes, hj = s1 / _bins % slopes;
                int n = _starts[r1] - _starts[r0];
                float y0 = _places[r0][i], h0 = _o.Slopes[hi];
                float y1 = _places[r1][j], h1 = _o.Slopes[hj];
                float len = n * _lanes.Spacing;
                for (int k = 0; k <= n; k++)
                {
                    float y = Hermite(y0, h0, y1, h1, len, n, k);
                    int at = _starts[r0] + k;
                    if (lap)
                        offset[_lanes.Wrap(at)] = y;
                    else
                        offset[at - _starts[0]] = y;
                }
            }
        }

        private List<float> Places(int station)
        {
            (float lo, float hi) = _bounds(station);
            if (hi < lo)
                (lo, hi) = (hi, lo);
            float anchor = Math.Clamp(_lanes.Offset(LaneId.Free, _lanes.Wrap(station)), lo, hi);
            if (_o.SearchBandMeters > 0f)
            {
                lo = MathF.Max(lo, anchor - _o.SearchBandMeters);
                hi = MathF.Min(hi, anchor + _o.SearchBandMeters);
                if (hi < lo)
                    lo = hi = anchor;
            }
            var ys = new List<float>();
            if (hi - lo < 1e-3f)
                ys.Add(0.5f * (lo + hi));
            else if (_o.EvenPlaces == 1)
                ys.Add(anchor);
            else if (_o.EvenPlaces >= 2)
            {
                int n = _o.EvenPlaces;
                for (int i = 0; i < n; i++)
                    ys.Add(lo + (hi - lo) * i / (n - 1));
            }
            else
            {
                float step = MathF.Max(0.05f, _o.LateralStep);
                ys.Add(lo);
                for (float y = MathF.Ceiling((lo + 1e-4f) / step) * step; y < hi - 1e-3f; y += step)
                    ys.Add(y);
                if (ys[^1] < hi - 1e-3f)
                    ys.Add(hi);
            }
            if (_allowed is not null)
            {
                int st = _lanes.Wrap(station);
                ys.RemoveAll(y => !_allowed(st, y));
            }
            // The geometric line is a knot too, unless the corridor already has a place on it.
            // Even samples otherwise step over it, and every apex sits a metre off the line the road actually rewards.
            if (ys.Count > 0 && _o.EvenPlaces != 1)
            {
                bool have = false;
                foreach (float y in ys)
                    if (MathF.Abs(y - anchor) < 0.2f)
                        have = true;
                if (!have && (_allowed is null || _allowed(_lanes.Wrap(station), anchor)))
                    ys.Add(anchor);
            }
            return ys;
        }

        public float[] Polish(float[] line, Func<int, (float Lo, float Hi)> bounds, bool periodic, int origin)
        {
            float band = _o.PolishBandMeters;
            if (band <= 0f)
                return (float[])line.Clone();
            int count = line.Length;
            var n = (float[])line.Clone();
            var low = new float[count];
            var high = new float[count];
            for (int i = 0; i < count; i++)
            {
                (float lo, float hi) = bounds(periodic ? i : origin + i);
                float a = MathF.Max(lo, line[i] - band);
                float b = MathF.Min(hi, line[i] + band);
                if (a > b)
                    a = b = 0.5f * (a + b);
                low[i] = a;
                high[i] = b;
                n[i] = Math.Clamp(line[i], a, b);
            }

            Vector2 P(int i)
            {
                int j = periodic ? _lanes.Wrap(i) : Math.Clamp(i, 0, count - 1);
                return _lanes.Position(periodic ? j : origin + j, n[j]);
            }

            int step = 1;
            while (step * 8 < count)
                step *= 2;
            while (step > 0)
            {
                for (int pass = 0; pass < 48; pass++)
                {
                    for (int i = 0; i < count; i += step)
                    {
                        if (!periodic && (i == 0 || i == count - 1))
                            continue;
                        Vector2 p0 = P(i - 2 * step), p1 = P(i - step), p2 = P(i), p3 = P(i + step), p4 = P(i + 2 * step);
                        float kPrev = LaneSet.Curvature(p0, p1, p2);
                        float kNext = LaneSet.Curvature(p2, p3, p4);
                        float lPrev = (p2 - p1).Length(), lNext = (p3 - p2).Length();
                        float target = (lNext * kPrev + lPrev * kNext) / MathF.Max(lPrev + lNext, 1e-4f);
                        const float probe = 0.01f;
                        float slope = (LaneSet.Curvature(p1, p2 + _lanes.Normal(periodic ? i : origin + i) * probe, p3)
                            - LaneSet.Curvature(p1, p2, p3)) / probe;
                        if (MathF.Abs(slope) < 1e-7f)
                            continue;
                        n[i] = Math.Clamp(n[i] + (target - LaneSet.Curvature(p1, p2, p3)) / slope * 0.8f, low[i], high[i]);
                    }
                }
                step /= 2;
            }
            if (!periodic)
            {
                n[0] = line[0];
                n[^1] = line[^1];
            }
            return n;
        }

        /// <summary>
        /// The lattice line is a proposal. A bump is kept only when the driver's
        /// own closed-lap plan is quicker, so a step the approximate timing liked
        /// and the car would not is put back.
        /// </summary>
        public float[] KeepFaster(float[] geometric, float[] proposal)
        {
            var line = (float[])geometric.Clone();
            float best = TimeLine(line, true, 0f, 0f).Seconds;
            int count = line.Length;
            int width = 16;
            for (int i = 0; i < count; i += 32)
            {
                float toward = proposal[_lanes.Wrap(i)] - line[_lanes.Wrap(i)];
                float k = LaneSet.Curvature(
                    _lanes.Position(i - Chord, line[_lanes.Wrap(i - Chord)]),
                    _lanes.Position(i, line[i]),
                    _lanes.Position(i + Chord, line[_lanes.Wrap(i + Chord)]));
                float inside = MathF.Abs(k) > 0.008f ? -MathF.Sign(k) * 0.7f : 0f;
                foreach (float delta in new[] { toward * 0.5f, toward, inside, inside * 1.8f })
                {
                    if (MathF.Abs(delta) < 0.04f)
                        continue;
                    var trial = (float[])line.Clone();
                    for (int j = -width; j <= width; j++)
                    {
                        int at = _lanes.Wrap(i + j);
                        float w = 1f - MathF.Abs(j) / (width + 1f);
                        (float lo, float hi) = RoadBounds(at);
                        trial[at] = Math.Clamp(line[at] + delta * w, lo, hi);
                    }
                    float seconds = TimeLine(trial, true, 0f, 0f).Seconds;
                    if (seconds < best - 0.003f)
                    {
                        line = trial;
                        best = seconds;
                    }
                }
            }
            return line;
        }

        public (float[] Speeds, float Seconds) TimeLine(float[] offset, bool periodic, float startSpeed, float endSpeed)
        {
            int n = offset.Length;
            var curvature = new float[n];
            var step = new float[n];
            var factor = new float[n];
            var brake = new float[n];
            var road = new LaneRoad[n];
            Array.Fill(factor, 1f);
            Array.Fill(brake, 1f);
            float Y(int k) => periodic ? offset[_lanes.Wrap(k)] : offset[Math.Clamp(k, 0, n - 1)];
            for (int k = 0; k < n; k++)
            {
                int s = StationOf(k, periodic);
                curvature[k] = LaneSet.Curvature(
                    _lanes.Position(s - Chord, Y(k - Chord)),
                    _lanes.Position(s, Y(k)),
                    _lanes.Position(s + Chord, Y(k + Chord)));
                Vector2 a = _lanes.Position(s, Y(k));
                Vector2 b = _lanes.Position(s + (periodic || k + 1 < n ? 1 : 0), periodic || k + 1 < n ? Y(k + 1) : Y(k));
                step[k] = periodic || k + 1 < n ? (b - a).Length() : 0f;
                road[k] = RoadAt(s, Y(k));
            }
            if (_lag is not null)
            {
                var guess = new float[n];
                for (int k = 0; k < n; k++)
                    guess[k] = _leadSpeed[_lanes.Wrap(StationOf(k, periodic))];
                var led = new float[n + _leadMax + 1];
                for (int k = 0; k < led.Length; k++)
                {
                    int s = periodic ? k : _stretchFrom + k;
                    led[k] = LaneSet.Curvature(
                        _lanes.Position(s - Chord, Y(k - Chord)),
                        _lanes.Position(s, Y(k)),
                        _lanes.Position(s + Chord, Y(k + Chord)));
                }
                LaneSpeedPlanner.LeadCurvatures(led, guess, _lag, _lanes.Spacing, curvature);
            }
            var speed = new float[n];
            if (periodic)
                LaneSpeedPlanner.PlanClosed(_envelope, curvature, step, factor, brake, road, _shares, speed);
            else
                LaneSpeedPlanner.PlanAhead(_envelope, curvature, step, factor, brake, road, _shares, startSpeed, endSpeed, speed);
            float seconds = 0f;
            int last = periodic ? n : n - 1;
            for (int k = 0; k < last; k++)
            {
                int k1 = periodic ? (k + 1) % n : k + 1;
                seconds += step[k] / MathF.Max(1f, 0.5f * (speed[k] + speed[k1]));
            }
            return (speed, seconds);
        }

        private int _stretchFrom;

        private int StationOf(int index, bool periodic) => periodic ? index : _stretchFrom + index;

        private void PlanLeadGuess()
        {
            int n = _lanes.Count;
            var curvature = new float[n];
            var step = new float[n];
            var factor = new float[n];
            var brake = new float[n];
            var road = new LaneRoad[n];
            Array.Fill(factor, 1f);
            Array.Fill(brake, 1f);
            for (int i = 0; i < n; i++)
            {
                float y = _lanes.Offset(LaneId.Free, i);
                curvature[i] = LaneSet.Curvature(
                    _lanes.Position(i - Chord, _lanes.Offset(LaneId.Free, i - Chord)),
                    _lanes.Position(i, y),
                    _lanes.Position(i + Chord, _lanes.Offset(LaneId.Free, i + Chord)));
                step[i] = (_lanes.Position(i + 1, _lanes.Offset(LaneId.Free, i + 1)) - _lanes.Position(i, y)).Length();
                road[i] = RoadAt(i, y);
            }
            LaneSpeedPlanner.PlanClosed(_envelope, curvature, step, factor, brake, road, _shares, _leadSpeed);
        }

        private LaneRoad RoadAt(int station, float y)
        {
            int st = _lanes.Wrap(station);
            LaneRoad[] across = _roads[st];
            float at = Math.Clamp((y - _roadLo[st]) / RoadGrid, 0f, across.Length - 1);
            int a = Math.Min((int)at, across.Length - 1);
            int b = Math.Min(a + 1, across.Length - 1);
            float t = at - a;
            LaneRoad ra = across[a], rb = across[b];
            return new LaneRoad(ra.Grade + (rb.Grade - ra.Grade) * t, ra.Bank + (rb.Bank - ra.Bank) * t, ra.Vertical + (rb.Vertical - ra.Vertical) * t);
        }

        private static float Hermite(float y0, float h0, float y1, float h1, float length, int n, int k)
        {
            float t = k / (float)n;
            float t2 = t * t, t3 = t2 * t;
            return (2f * t3 - 3f * t2 + 1f) * y0 + (t3 - 2f * t2 + t) * length * h0
                + (-2f * t3 + 3f * t2) * y1 + (t3 - t2) * length * h1;
        }

        [ThreadStatic] private static Edge? _threadEdge;

        private Edge EdgeFor(int maxN, int tail)
        {
            Edge? edge = _threadEdge;
            if (edge is null || edge.Cap < maxN || edge.TailCap < tail || edge.BinCap < _bins)
                _threadEdge = edge = new Edge(maxN, tail, GripSamples, _bins);
            return edge;
        }

        private sealed class Edge
        {
            public readonly int Cap;
            public readonly int TailCap;
            public readonly int BinCap;
            private readonly int _samples;
            private readonly int _bins;
            private readonly float[] _offset;
            private readonly float[] _curvature;
            private readonly float[] _corner;
            private readonly float[] _step;
            private readonly float[] _limit;
            private readonly float[] _drivenAt;
            private readonly float[] _decelAt;
            private readonly float[] _drive;
            private readonly float[] _brake;
            private readonly float[] _guess;
            private readonly bool[] _live;
            private int _n;
            private float _bend;
            private float _sampleBase;
            private float _sampleDv;

            public Edge(int maxN, int tail, int samples, int bins)
            {
                Cap = maxN;
                TailCap = tail;
                BinCap = bins;
                _samples = samples;
                _bins = bins;
                int curv = maxN + tail + Chord + 2;
                _offset = new float[maxN + 1];
                _curvature = new float[curv];
                _corner = new float[maxN + 1];
                _step = new float[maxN + 1];
                _limit = new float[maxN + 1];
                _drivenAt = new float[maxN * samples];
                _decelAt = new float[maxN * samples];
                _drive = new float[maxN + 1];
                _brake = new float[bins * (maxN + 1)];
                _guess = new float[maxN + 1];
                _live = new bool[bins];
            }

            public bool Fit(Search s, int st0, int n, float y0, float h0, float y1, float h1, float[]? tail)
            {
                _sampleBase = s._sample0;
                _sampleDv = s._sampleDv;
                _n = n;
                float length = n * s._lanes.Spacing;
                (float lo0, float hi0) = s._bounds(st0);
                float leeway = MathF.Max(0f, MathF.Max(lo0 - y0, y0 - hi0));
                for (int k = 0; k <= n; k++)
                {
                    float y = Hermite(y0, h0, y1, h1, length, n, k);
                    (float lo, float hi) = s._bounds(st0 + k);
                    if (y < lo - leeway - 1e-3f || y > hi + leeway + 1e-3f)
                        return false;
                    if (s._allowed is not null && k > 0 && k < n && !s._allowed(s._lanes.Wrap(st0 + k), y))
                        return false;
                    _offset[k] = y;
                }
                float bulge = 0f;
                for (int k = 1; k < n; k++)
                {
                    float chord = y0 + (y1 - y0) * (k / (float)n);
                    bulge = MathF.Max(bulge, MathF.Abs(_offset[k] - chord));
                }
                // A cubic whose end slopes fight the chord leaves the road's line by
                // several metres between knots. A real corner's sagitta over this
                // step is much smaller; anything past it is a loop, not an apex.
                float sag = length * length / (8f * 35f);
                if (bulge > MathF.Min(3f, 0.8f + sag))
                    return false;

                float At(int k)
                {
                    if ((uint)k <= (uint)n)
                        return _offset[k];
                    if (tail is not null && k > n && k - n < tail.Length)
                        return tail[k - n];
                    return Hermite(y0, h0, y1, h1, length, n, k);
                }

                int ahead = s._leadMax;
                int curvCount = n + 1 + ahead;
                for (int k = 0; k < curvCount; k++)
                    _curvature[k] = LaneSet.Curvature(
                        s._lanes.Position(st0 + k - Chord, At(k - Chord)),
                        s._lanes.Position(st0 + k, At(k)),
                        s._lanes.Position(st0 + k + Chord, At(k + Chord)));
                for (int k = 0; k < n; k++)
                    _step[k] = (s._lanes.Position(st0 + k + 1, _offset[k + 1]) - s._lanes.Position(st0 + k, _offset[k])).Length();
                _step[n] = 0f;

                if (s._lag is null)
                    Array.Copy(_curvature, _corner, n + 1);
                else
                {
                    for (int k = 0; k <= n; k++)
                        _guess[k] = s._leadSpeed[s._lanes.Wrap(st0 + k)];
                    LaneSpeedPlanner.LeadCurvatures(_curvature.AsSpan(0, curvCount), _guess.AsSpan(0, n + 1), s._lag, s._lanes.Spacing, _corner.AsSpan(0, n + 1));
                }

                _bend = 0f;
                for (int k = 0; k <= n; k++)
                {
                    float corner = _corner[k];
                    LaneRoad road = s.RoadAt(st0 + k, k <= n ? At(k) : At(k));
                    _limit[k] = LaneSpeedPlanner.CornerLimit(s._envelope.ForTurn(corner), corner, s._shares.Lateral, road, passes: 12);
                    if (k < n)
                    {
                        _bend += s._o.BendSeconds * _curvature[k] * _curvature[k] * _step[k];
                        int sampleAt = k * _samples;
                        for (int g = 0; g < _samples; g++)
                        {
                            float v = s._sample0 + g * s._sampleDv;
                            _drivenAt[sampleAt + g] = LaneSpeedPlanner.Driven(s._envelope, v, corner, _step[k], 1f, road, s._shares);
                            _decelAt[sampleAt + g] = LaneSpeedPlanner.Deceleration(s._envelope, v, corner, 1f, 1f, road, s._shares);
                        }
                    }
                }
                return true;
            }

            public void Price(Search s, float[] later, int targetBase, float[] best, int sourceBase, int[] nextRow, int[] next, int r1)
            {
                int n = _n;
                int bins = _bins;
                float min = s._o.MinSpeed, step = s._o.SpeedStep;
                if (!s._o.ExactSpeeds)
                {
                    for (int b0 = 0; b0 < bins; b0++)
                    {
                        float v0 = min + b0 * step;
                        if (v0 > _limit[0] + 1e-3f)
                            break;
                        (float low, float high) = Reach(v0);
                        int bLow = Math.Max(0, (int)MathF.Ceiling((low - min) / step));
                        int bHigh = Math.Min(bins - 1, (int)MathF.Floor((high - min) / step));
                        for (int b1 = bLow; b1 <= bHigh; b1++)
                        {
                            float rest = later[targetBase + b1];
                            if (rest == float.PositiveInfinity)
                                continue;
                            float v1 = min + b1 * step;
                            float seconds = _bend;
                            float v2 = v0 * v0, dv2 = n == 0 ? 0f : (v1 * v1 - v0 * v0) / n;
                            bool held = true;
                            for (int k = 0; k < n; k++)
                            {
                                float a = MathF.Sqrt(MathF.Max(0f, v2 + dv2 * k));
                                if (a > _limit[k] + 1e-3f)
                                {
                                    held = false;
                                    break;
                                }
                                float b = MathF.Sqrt(MathF.Max(0f, v2 + dv2 * (k + 1)));
                                seconds += _step[k] / MathF.Max(0.5f * (a + b), 1f);
                            }
                            if (held && seconds + rest < best[sourceBase + b0])
                            {
                                best[sourceBase + b0] = seconds + rest;
                                nextRow[sourceBase + b0] = r1;
                                next[sourceBase + b0] = targetBase + b1;
                            }
                        }
                    }
                    return;
                }

                Array.Clear(_live);
                for (int b1 = 0; b1 < bins; b1++)
                {
                    float v1 = min + b1 * step;
                    if (later[targetBase + b1] == float.PositiveInfinity || v1 > _limit[n] + 1e-3f)
                        continue;
                    BrakeBack(b1, v1);
                    _live[b1] = true;
                }
                for (int b0 = 0; b0 < bins; b0++)
                {
                    float v0 = min + b0 * step;
                    if (v0 > _limit[0] + 1e-3f)
                        break;
                    DriveOn(v0);
                    int at = sourceBase + b0;
                    for (int b1 = 0; b1 < bins; b1++)
                    {
                        if (!_live[b1])
                            continue;
                        float v1 = min + b1 * step;
                        if (v1 > _drive[n] + 1e-3f)
                            break;
                        int brakeAt = b1 * (Cap + 1);
                        if (_brake[brakeAt] < v0 - 1e-3f)
                            continue;
                        float rest = later[targetBase + b1];
                        float seconds = _bend;
                        for (int k = 0; k < n; k++)
                        {
                            float a = MathF.Min(_drive[k], _brake[brakeAt + k]);
                            float b = MathF.Min(_drive[k + 1], _brake[brakeAt + k + 1]);
                            seconds += _step[k] / MathF.Max(0.5f * (a + b), 1f);
                        }
                        if (seconds + rest < best[at])
                        {
                            best[at] = seconds + rest;
                            nextRow[at] = r1;
                            next[at] = targetBase + b1;
                        }
                    }
                }
            }

            private void DriveOn(float v0)
            {
                _drive[0] = MathF.Min(_limit[0], v0);
                for (int k = 0; k < _n; k++)
                    _drive[k + 1] = MathF.Min(_limit[k + 1], Lerp(_drivenAt, k, _drive[k]));
            }

            private void BrakeBack(int bin, float v1)
            {
                int at = bin * (Cap + 1);
                _brake[at + _n] = MathF.Min(_limit[_n], v1);
                for (int k = _n - 1; k >= 0; k--)
                {
                    float next = _brake[at + k + 1];
                    float v0 = next;
                    float ds = _step[k];
                    for (int pass = 0; pass < 3; pass++)
                    {
                        float mid = 0.5f * (v0 + next);
                        v0 = MathF.Sqrt(next * next + 2f * LerpDecel(k, mid) * ds);
                    }
                    _brake[at + k] = MathF.Min(_limit[k], v0);
                }
            }

            private (float Low, float High) Reach(float v0)
            {
                float low = v0, high = v0;
                for (int k = 0; k < _n; k++)
                {
                    low = MathF.Sqrt(MathF.Max(0f, low * low - 2f * LerpDecel(k, low) * _step[k]));
                    high = MathF.Min(_limit[k + 1], Lerp(_drivenAt, k, MathF.Min(high, _limit[k])));
                }
                return (low, high);
            }

            private float Lerp(float[] table, int station, float v)
            {
                float x = (v - _sampleBase) / _sampleDv;
                int i = Math.Clamp((int)MathF.Floor(x), 0, _samples - 2);
                float t = Math.Clamp(x - i, 0f, 1f);
                int at = station * _samples + i;
                return table[at] + (table[at + 1] - table[at]) * t;
            }

            private float LerpDecel(int station, float v) => Lerp(_decelAt, station, v);
        }
    }
}
