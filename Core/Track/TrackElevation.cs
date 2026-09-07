using System;

namespace StintegyEVO.Core.Track;

/// <summary>
/// Elevation profiles for closed circuits.
///
/// A lap must come back to the height it left, or a car would win or lose
/// energy every time round. Heights are therefore what is specified and the
/// gradient is read off them, rather than the other way about: a profile
/// given as grades would have to be made to sum to zero by hand and would
/// drift the moment anyone edited it.
///
/// The profiles below are approximations of each circuit's documented
/// character — where it climbs and where it falls, and by roughly how much
/// in total — not survey data. The centrelines they sit on are real and
/// start at the real start/finish line, so a fraction of a lap does
/// correspond to a real place on the circuit; the heights at those places
/// are the part that is modelled.
/// </summary>
public static class TrackElevation
{
    /// <summary>
    /// Combines a height profile with a cross-section model into one
    /// surface function. Heights are given as (fraction of a lap, metres),
    /// wrapping round, and are read with a periodic Catmull-Rom so the
    /// gradient is continuous across the start/finish line.
    /// </summary>
    public static Func<TrackSurfaceContext, TrackSurface> Profile(
        (float Fraction, float Height)[] controlPoints,
        Func<TrackSurfaceContext, TrackSurface>? crossSection = null
    )
    {
        ArgumentNullException.ThrowIfNull(controlPoints);
        if (controlPoints.Length < 2)
            throw new ArgumentException("A profile needs at least two heights.");

        return context =>
        {
            const float probe = 1f;
            float lap = MathF.Max(context.LapLengthMeters, 1f);
            float ahead = HeightAt(
                controlPoints,
                (context.DistanceMeters + probe) / lap
            );
            float behind = HeightAt(
                controlPoints,
                (context.DistanceMeters - probe) / lap
            );
            TrackSurface section = crossSection is null
                ? TrackSurface.Flat
                : crossSection(context);
            return section with { Grade = (ahead - behind) / (2f * probe) };
        };
    }

    /// <summary>
    /// The same profile written in metres from the start/finish line rather
    /// than in fractions, which is what a hand-authored layout wants: a
    /// straight's length is known when the layout is written, its share of
    /// a lap is not.
    /// </summary>
    public static Func<TrackSurfaceContext, TrackSurface> ProfileByDistance(
        (float Metres, float Height)[] controlPoints,
        Func<TrackSurfaceContext, TrackSurface>? crossSection = null
    )
    {
        ArgumentNullException.ThrowIfNull(controlPoints);
        if (controlPoints.Length < 2)
            throw new ArgumentException("A profile needs at least two heights.");

        return context =>
        {
            float lap = MathF.Max(context.LapLengthMeters, 1f);
            (float, float)[] fractions = new (float, float)[controlPoints.Length];
            for (int i = 0; i < controlPoints.Length; i++)
                fractions[i] = (controlPoints[i].Metres / lap, controlPoints[i].Height);
            return Profile(fractions, crossSection)(context);
        };
    }

    /// <summary>
    /// Height at a fraction of the lap, interpolated periodically.
    ///
    /// The tangents are taken over the real spacing of the control points
    /// rather than over their index. That distinction is the whole of it:
    /// the textbook Catmull-Rom is smooth in its own parameter, and if one
    /// segment covers five hundred metres of road and the next covers two
    /// hundred and eighty, the same smooth parameter runs at two different
    /// speeds either side of the point where they meet. The gradient, which
    /// is height per metre and not height per parameter, then steps by the
    /// ratio of the two — and once the load knows about the road's vertical
    /// bend, that step is not a cosmetic kink but a car pressed into the
    /// tarmac at five times its weight for one metre.
    /// </summary>
    public static float HeightAt(
        (float Fraction, float Height)[] points,
        float fraction
    )
    {
        int count = points.Length;
        float wrapped = fraction - MathF.Floor(fraction);

        int index = 0;
        while (index + 1 < count && points[index + 1].Fraction <= wrapped)
            index++;

        (float x0, float p0) = ControlPoint(points, index - 1);
        (float x1, float p1) = ControlPoint(points, index);
        (float x2, float p2) = ControlPoint(points, index + 1);
        (float x3, float p3) = ControlPoint(points, index + 2);

        float span = MathF.Max(x2 - x1, 1e-6f);
        float t = Math.Clamp((wrapped - x1) / span, 0f, 1f);

        // Slope at each end, in height per unit of lap. Shared with the
        // neighbouring segment by construction, which is what makes the
        // gradient continuous where two segments of different length meet.
        float slopeAtStart = (p2 - p0) / MathF.Max(x2 - x0, 1e-6f);
        float slopeAtEnd = (p3 - p1) / MathF.Max(x3 - x1, 1e-6f);

        float t2 = t * t;
        float t3 = t2 * t;
        return (2f * t3 - 3f * t2 + 1f) * p1 +
               (t3 - 2f * t2 + t) * span * slopeAtStart +
               (-2f * t3 + 3f * t2) * p2 +
               (t3 - t2) * span * slopeAtEnd;
    }

    /// <summary>
    /// One control point by index, wrapping round the lap and carrying the
    /// wrap into its position so the four points of a segment always read
    /// as increasing even where the run passes the start line.
    /// </summary>
    private static (float Fraction, float Height) ControlPoint(
        (float Fraction, float Height)[] points,
        int index
    )
    {
        int count = points.Length;
        int laps = (int)MathF.Floor(index / (float)count);
        (float fraction, float height) = points[index - laps * count];
        return (fraction + laps, height);
    }

    /// <summary>
    /// Zandvoort is built through coastal dunes and rides over them. The
    /// lap climbs away from Tarzan into the Hunserug, crests before
    /// Scheivlak and drops through it — the corner is famous for arriving
    /// blind over the top — runs low along the back of the circuit, and
    /// climbs again through the last sequence to the banked final corner.
    /// Something over fifteen metres between the highest and lowest points,
    /// which is a great deal for a circuit this short.
    /// </summary>
    /// <summary>
    /// Baku is a seafront street circuit and mostly flat; the exception is
    /// the climb along the old city wall into the castle section, which is
    /// also where the road narrows to the tightest squeeze in the sport.
    /// The grade tops out under seven percent on purpose: this circuit is
    /// bound for the training set, and the training set's steepest climb
    /// must stay below Monaco's 8.6 so that Monaco remains a gradient the
    /// policy has never been asked for.
    /// </summary>
    public static readonly (float, float)[] BakuHeights =
    [
        (0.00f, 0f), (0.20f, 0f), (0.30f, 2f), (0.34f, 6f),
        (0.39f, 24f), (0.44f, 26f), (0.52f, 14f), (0.62f, 4f),
        (0.75f, 1f), (0.90f, 0f)
    ];

    /// <summary>
    /// Authored in the style of the Ardennes original: a long descent to
    /// the valley floor, the climb back along the top, and then the
    /// signature — a fast drop followed by a compression and a climb far
    /// steeper than anything in the training set. Held out precisely for
    /// that climb: eleven-plus percent against a training maximum around
    /// seven.
    /// </summary>
    public static readonly (float, float)[] SpaHeights =
    [
        (0.00f, 55f), (0.08f, 47f), (0.16f, 29f), (0.26f, 12f),
        (0.36f, 4f), (0.44f, 0f), (0.52f, 7f), (0.62f, 20f),
        (0.72f, 34f), (0.82f, 50f), (0.87f, 40f), (0.90f, 27f),
        (0.92f, 25f), (0.955f, 48f)
    ];

    /// <summary>Monza is a park and the park is flat.</summary>
    public static readonly (float, float)[] MonzaHeights =
    [
        (0.00f, 0f), (0.25f, 2f), (0.55f, -1f), (0.80f, 1f)
    ];

    /// <summary>
    /// Rolling and anticlockwise, cut into a hillside: down from the
    /// start, a low back stretch, and the long climb home.
    /// </summary>
    public static readonly (float, float)[] InterlagosHeights =
    [
        (0.00f, 12f), (0.10f, 4f), (0.22f, 0f), (0.36f, 6f),
        (0.50f, 15f), (0.64f, 23f), (0.78f, 30f), (0.90f, 24f)
    ];

    /// <summary>
    /// Marina Bay is famously flat — the only relief is the pair of bridge
    /// crossings, a few metres each. Authored honestly: this circuit is in
    /// the set for its walls and its width, and pretending it climbs would
    /// cover a hole with fiction.
    /// </summary>
    public static readonly (float, float)[] SingaporeHeights =
    [
        (0.00f, 0f), (0.18f, 1f), (0.30f, 4f), (0.36f, 1f),
        (0.55f, 0f), (0.72f, 3f), (0.80f, 0f)
    ];

    /// <summary>
    /// The Algarve rollercoaster. Authored to what the venue is famous
    /// for: blind crests and plunges, with the drop into turn one and the
    /// climb to the final corner both past ten percent. This is the
    /// gradient trainer — the one dimension of Monaco that Baku's capped
    /// climb deliberately leaves out — carried on a road wide enough that
    /// nothing else is being tested at the same time.
    /// </summary>
    public static readonly (float, float)[] PortimaoHeights =
    [
        (0.000f, 32f), (0.035f, 13f), (0.090f, 6f), (0.140f, 14f),
        (0.190f, 3f), (0.270f, 11f), (0.330f, 2f), (0.420f, 9f),
        (0.500f, 0f), (0.600f, 8f), (0.660f, 2f), (0.760f, 13f),
        (0.820f, 6f), (0.880f, 9f), (0.930f, 26f)
    ];

    public static readonly (float, float)[] ZandvoortHeights =
    [
        (0.00f, 4f), (0.10f, 2f), (0.22f, 11f), (0.32f, 16f),
        (0.40f, 6f), (0.52f, 1f), (0.64f, 0f), (0.76f, 5f),
        (0.86f, 12f), (0.94f, 9f)
    ];

    /// <summary>
    /// Monaco climbs harder than anywhere else on the calendar: out of
    /// Sainte Dévote up Beau Rivage to Casino, then down through Mirabeau
    /// and the hairpin to Portier and the tunnel at sea level, flat along
    /// the harbour, and back up through Anthony Noghes to the line. About
    /// forty metres between the highest and lowest points.
    /// </summary>
    public static readonly (float, float)[] MonacoHeights =
    [
        (0.00f, 8f), (0.05f, 4f), (0.10f, 14f), (0.17f, 32f),
        (0.24f, 38f), (0.31f, 30f), (0.37f, 16f), (0.43f, 4f),
        (0.50f, 0f), (0.62f, 0f), (0.72f, 1f), (0.82f, 2f),
        (0.90f, 4f), (0.96f, 7f)
    ];

    /// <summary>
    /// Shanghai's opening complex climbs into the first corner and unwinds
    /// downhill through the snail; the rest of the lap, including the long
    /// back straight, is close to level. Around ten metres in all.
    /// </summary>
    public static readonly (float, float)[] ShanghaiHeights =
    [
        (0.00f, 0f), (0.04f, 6f), (0.08f, 7f), (0.13f, 2f),
        (0.22f, 0f), (0.38f, -1f), (0.52f, -2f), (0.66f, -1f),
        (0.80f, 0f), (0.92f, 0f)
    ];

    /// <summary>
    /// Silverstone is a converted airfield and the flattest circuit here.
    /// What little there is falls away through the Village loop and rises
    /// again toward Stowe, inside about eight metres.
    /// </summary>
    public static readonly (float, float)[] SilverstoneHeights =
    [
        (0.00f, 3f), (0.12f, 1f), (0.24f, -2f), (0.38f, -3f),
        (0.52f, 0f), (0.66f, 2f), (0.78f, 4f), (0.90f, 4f)
    ];

    /// <summary>
    /// Sepang rises through the first sector and falls away again to the
    /// second long straight, with something under twenty metres between the
    /// extremes.
    /// </summary>
    public static readonly (float, float)[] SepangHeights =
    [
        (0.00f, 0f), (0.08f, 5f), (0.18f, 12f), (0.28f, 15f),
        (0.40f, 9f), (0.52f, 2f), (0.64f, -2f), (0.76f, -1f),
        (0.88f, 0f)
    ];
}
