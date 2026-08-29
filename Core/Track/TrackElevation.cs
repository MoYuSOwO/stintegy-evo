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
    /// Height at a fraction of the lap, interpolated periodically. Catmull-
    /// Rom gives a smooth curve through every control point, which matters
    /// because the gradient is its derivative and a kink in the height
    /// would read as a step in the road.
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

        float startFraction = points[index].Fraction;
        float endFraction = index + 1 < count
            ? points[index + 1].Fraction
            : points[0].Fraction + 1f;
        float span = MathF.Max(endFraction - startFraction, 1e-6f);
        float t = Math.Clamp((wrapped - startFraction) / span, 0f, 1f);

        float p0 = points[(index - 1 + count) % count].Height;
        float p1 = points[index].Height;
        float p2 = points[(index + 1) % count].Height;
        float p3 = points[(index + 2) % count].Height;
        return CatmullRom(p0, p1, p2, p3, t);
    }

    private static float CatmullRom(
        float p0, float p1, float p2, float p3, float t
    )
    {
        float t2 = t * t;
        float t3 = t2 * t;
        return 0.5f * (
            2f * p1 +
            (p2 - p0) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t3
        );
    }

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
