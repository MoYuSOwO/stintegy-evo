namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// A line drawn over the road ahead by someone other than the driver: an
/// offset from the racing line at each of a fixed set of control points ahead
/// of the car, joined into one smooth curve and ending back on the racing line.
/// The part the car reaches within about a second is held as it was last
/// drawn, so a new drawing changes only the road the car has time to take
/// it on, and the hands are never handed a line that jumps under them.
/// </summary>
public sealed class DrawnLine
{
    /// <summary>How far ahead the drawn part reaches.</summary>
    public const float LengthMeters = 400f;

    /// <summary>Beyond the last point the line eases back onto the racing line over this much road.</summary>
    public const float TaperMeters = 50f;

    /// <summary>The near part held as drawn is this long in time at the car's speed, and never shorter than <see cref="MinFreezeMeters"/>.</summary>
    public const float FreezeSeconds = 1f;

    public const float MinFreezeMeters = 25f;

    /// <summary>
    /// The most sideways acceleration the line's own bending away from the
    /// racing line may ask of the car, at the speed the car will be doing
    /// there: points that zigzag faster than this are ridden through as a
    /// curve that leans towards them, never as a weave the car slows for.
    /// </summary>
    public const float BendAccelMetersPerSecond2 = 5f;

    // How far along the drawing the bent line looks to see where it heads.
    private const float PreviewSeconds = 0.3f;
    private const float MinPreviewMeters = 10f;

    /// <summary>
    /// Where the control points lie ahead of the car: every 6 m to 120 m,
    /// where a slow corner's shape is decided, then every 20 m to 400 m.
    /// </summary>
    public static readonly float[] PointMeters = Points();

    private static float[] Points()
    {
        var points = new List<float>();
        for (float d = 6f; d <= 120f + 1e-3f; d += 6f)
            points.Add(d);
        for (float d = 140f; d <= LengthMeters + 1e-3f; d += 20f)
            points.Add(d);
        return points.ToArray();
    }

    // Stations behind the car the line is kept for, so a curvature read on a
    // chord through the car still sees the line the car is on.
    private const int BehindStations = 30;

    private readonly LaneSet _lanes;
    private readonly float[] _delta;
    // The span the drawing covers, as stations from where it starts; outside
    // it the line is the racing line.
    private int _from;
    private int _span;
    private readonly List<float> _knotX = new();
    private readonly List<float> _knotY = new();

    public DrawnLine(LaneSet lanes)
    {
        _lanes = lanes;
        _delta = new float[lanes.Count];
    }

    /// <summary>Whether a line is drawn at all.</summary>
    public bool Active => _span > 0;

    /// <summary>Bumped every time the line changes, so a plan made on an older drawing is known to be stale.</summary>
    public int Version { get; private set; }

    /// <summary>The share of the planned braking the next braking zone is taken at.</summary>
    public float BrakeFactor { get; private set; } = 1f;

    /// <summary>The drawn line's offset from the racing line at a station; nothing outside the drawing.</summary>
    public float At(int station)
    {
        if (!Active)
            return 0f;
        int rel = _lanes.Wrap(station - _from);
        return rel <= _span ? _delta[_lanes.Wrap(station)] : 0f;
    }

    /// <summary>
    /// A new drawing, from the car at <paramref name="station"/> going at
    /// <paramref name="speed"/>, <paramref name="carOffset"/> off the racing
    /// line: one offset from the racing line per <see cref="PointMeters"/>,
    /// positive to the left. The near part keeps the last drawing (the first
    /// drawing starts where the car is); the rest is joined to it smoothly,
    /// bent no harder than <see cref="BendAccelMetersPerSecond2"/> at the
    /// <paramref name="plannedSpeed"/> of each station, and kept on the road.
    /// </summary>
    public void Draw(float station, float speed, float carOffset, ReadOnlySpan<float> plannedSpeed,
        ReadOnlySpan<float> offsets, float brakeFactor)
    {
        if (offsets.Length != PointMeters.Length)
            throw new ArgumentException($"A drawn line has {PointMeters.Length} points, not {offsets.Length}.", nameof(offsets));
        float spacing = _lanes.Spacing;
        int car = (int)MathF.Floor(station);
        bool held = Active;
        if (!held)
            Array.Clear(_delta);

        // Where the new drawing takes over: past the part the car reaches in
        // about a second, never beyond the end of the last drawing.
        float freeze = held ? MathF.Max(MinFreezeMeters, MathF.Max(0f, speed) * FreezeSeconds) : 0f;
        int join = car + (int)MathF.Ceiling(freeze / spacing);
        if (held)
            join = Math.Min(join, _from + _span);
        float joinX = (join - car) * spacing;
        float joinY = held ? At(join) : carOffset;
        float joinSlope = held ? (At(join + 1) - At(join - 1)) / (2f * spacing) : 0f;

        _knotX.Clear();
        _knotY.Clear();
        _knotX.Add(joinX);
        _knotY.Add(joinY);
        for (int k = 0; k < offsets.Length; k++)
        {
            if (PointMeters[k] <= joinX + 1e-3f)
                continue;
            _knotX.Add(PointMeters[k]);
            _knotY.Add(offsets[k]);
        }
        _knotX.Add(LengthMeters + TaperMeters);
        _knotY.Add(0f);

        int drawnEnd = car + (int)MathF.Ceiling((LengthMeters + TaperMeters) / spacing);
        int count = drawnEnd - join + 1;
        if (_raw.Length < count)
            _raw = new float[count];
        for (int k = 0; k < count; k++)
            _raw[k] = Hermite((join + k - car) * spacing, joinSlope);

        // The bent line: from the join, heading where the drawing will be a
        // short way on, turning across no faster than the bend the speed
        // there allows, and levelling off in time not to swing past it.
        // Past the drawing it settles back onto the racing line.
        int settle = (int)MathF.Ceiling(LengthMeters / spacing);
        float y = joinY, slope = joinSlope;
        int end = join;
        for (int k = 0; k < count + settle; k++)
        {
            int i = join + k;
            if (k > 0)
            {
                float v = MathF.Max(10f, plannedSpeed[_lanes.Wrap(i)]);
                float bend = BendAccelMetersPerSecond2 / (v * v);
                float preview = MathF.Max(MinPreviewMeters, v * PreviewSeconds);
                int ahead = Math.Min(count - 1, k + (int)MathF.Ceiling(preview / spacing));
                float target = ahead < count && k < count ? _raw[ahead] : 0f;
                float error = target - y;
                float wanted = Math.Clamp(error / preview, -MathF.Sqrt(2f * bend * MathF.Abs(error)), MathF.Sqrt(2f * bend * MathF.Abs(error)));
                slope += Math.Clamp(wanted - slope, -bend * spacing, bend * spacing);
                y += slope * spacing;
            }
            _delta[_lanes.Wrap(i)] = OnRoad(i, y);
            end = i;
            if (k >= count && MathF.Abs(y) < 0.01f && MathF.Abs(slope) < 1e-3f)
                break;
        }
        for (int i = end + 1; i <= drawnEnd; i++)
            _delta[_lanes.Wrap(i)] = 0f;
        end = Math.Max(end, drawnEnd);

        _from = car - BehindStations;
        _span = end - _from;
        BrakeFactor = brakeFactor;
        Version++;
    }

    /// <summary>Back to the racing line everywhere.</summary>
    public void Clear()
    {
        Array.Clear(_delta);
        _span = 0;
        BrakeFactor = 1f;
        Version++;
    }

    private float[] _raw = [];

    // An offset from the racing line, kept on the road the lanes may use.
    // Not cut off at the edge: the racing line runs along the edge for whole
    // entries and exits, and a line cut off there turned a corner wherever
    // it met it, and the plan slowed for every one, nine seconds a lap on a
    // line a metre off the racing one. Instead the offset gives way smoothly
    // as the room towards that side runs out, and never reaches the edge.
    private float OnRoad(int station, float delta)
    {
        float free = _lanes.Offset(LaneId.Free, station);
        float edge = MathF.Max(0f, _lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters);
        float room = delta >= 0f ? edge - free : edge + free;
        if (room <= 1e-3f)
            return 0f;
        return room * MathF.Tanh(delta / room);
    }

    // The knots joined by a shape-preserving cubic (Fritsch-Carlson): it never
    // swings past two neighbouring points, so a drawing cannot carry the line
    // further aside than any of its points asks.
    private float Hermite(float x, float startSlope)
    {
        int n = _knotX.Count;
        if (x <= _knotX[0])
            return _knotY[0];
        if (x >= _knotX[n - 1])
            return _knotY[n - 1];
        int k = 0;
        while (x > _knotX[k + 1])
            k++;
        float h = _knotX[k + 1] - _knotX[k];
        float t = (x - _knotX[k]) / h;
        float m0 = k == 0 ? startSlope : Slope(k);
        float m1 = k + 1 == n - 1 ? 0f : Slope(k + 1);
        float t2 = t * t, t3 = t2 * t;
        return (2f * t3 - 3f * t2 + 1f) * _knotY[k] + (t3 - 2f * t2 + t) * h * m0 +
               (-2f * t3 + 3f * t2) * _knotY[k + 1] + (t3 - t2) * h * m1;
    }

    private float Slope(int k)
    {
        float before = (_knotY[k] - _knotY[k - 1]) / (_knotX[k] - _knotX[k - 1]);
        float after = (_knotY[k + 1] - _knotY[k]) / (_knotX[k + 1] - _knotX[k]);
        if (before * after <= 0f)
            return 0f;
        float h0 = _knotX[k] - _knotX[k - 1], h1 = _knotX[k + 1] - _knotX[k];
        float w0 = 2f * h1 + h0, w1 = h1 + 2f * h0;
        return (w0 + w1) / (w0 / before + w1 / after);
    }
}
