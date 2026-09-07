using System;

namespace StintegyEVO.Core.Track;

/// <summary>
/// How much grip the road offers at a point, as a multiplier on what the
/// tyre would give on clean racing surface.
///
/// Two layers, multiplied. The static layer is the road itself - tarmac,
/// kerb, grass, gravel - and it is defined everywhere, because the thing it
/// mainly has to say is whether a wheel is on the circuit at all. The
/// dynamic layer is what gets laid down and washed off it - rubber, water -
/// and it will only ever cover the racing surface, because nobody lays a
/// racing line across a gravel trap. It is not built here; the slot is,
/// and it defaults to one so that everything written against this signature
/// keeps working when the grid arrives behind it.
///
/// This replaces a bookkeeping trick with a physical fact. Leaving the road
/// used to be a penalty term with a rate in it: the environment noticed a
/// region flag and subtracted reward. That is a rule, and rules about lines
/// invite arguments about lines. Grass is slippery, and a car with two
/// wheels on it is a car with less grip - priced by the physics, in the
/// currency of lap time, with no flag to read and nothing to litigate.
/// </summary>
public static class SurfaceGrip
{
    /// <summary>
    /// Clean racing surface. Everything else is quoted against this, so it
    /// is one by definition rather than by measurement.
    /// </summary>
    public const float RacingSurface = 1f;

    /// <summary>
    /// The painted-and-ribbed strip at the edge of the road. Real kerbs
    /// cost a little grip and a lot of composure; only the grip is modelled
    /// here, and the strip is narrow enough that a car riding it has most
    /// of itself still on the road.
    /// </summary>
    public const float KerbWidthMeters = 0.6f;
    public const float Kerb = 0.9f;

    /// <summary>
    /// Beyond the white line. A mixture of the things run-off is actually
    /// made of - grass at about four tenths, gravel at about a third,
    /// asphalt run-off at nearly full - taken at the pessimistic end,
    /// because the run-off a driver reaches by mistake is rarely the paved
    /// kind.
    /// </summary>
    public const float Buffer = 0.45f;

    /// <summary>
    /// How far a boundary is smeared over. A step change in grip is a step
    /// change in the force on one corner of the car, which is a cliff for
    /// anything integrating it and a cliff for anything learning against
    /// it. Short enough that the edge is still an edge.
    /// </summary>
    public const float TransitionMeters = 0.25f;

    /// <summary>
    /// The grip multiplier at a lateral offset from the centre line.
    ///
    /// <paramref name="dynamic"/> is the second layer, reserved: pass what
    /// the rubber and water grid says when there is one, and leave it at
    /// one until then.
    /// </summary>
    public static float At(in TrackSample sample, float offset, float dynamic = 1f)
    {
        return StaticAt(sample, offset) * MathF.Max(0f, dynamic);
    }

    /// <summary>
    /// The road's own contribution, everywhere, with both boundaries
    /// smoothed.
    /// </summary>
    public static float StaticAt(in TrackSample sample, float offset)
    {
        float half = MathF.Max(sample.HalfWidth, 0f);
        float distance = MathF.Abs(offset);
        float kerbInner = MathF.Max(0f, half - KerbWidthMeters);

        // Tarmac, then the kerb, then whatever is past the line. Written as
        // two blends rather than three cases so that a circuit whose road
        // is narrower than a kerb still gets a continuous answer.
        float onKerb = SmoothStep(kerbInner, half, distance);
        float pastLine = SmoothStep(
            half,
            half + TransitionMeters,
            distance
        );
        float surface = RacingSurface + (Kerb - RacingSurface) * onKerb;
        return surface + (Buffer - surface) * pastLine;
    }

    private static float SmoothStep(float from, float to, float value)
    {
        if (to - from <= 1e-5f)
            return value >= to ? 1f : 0f;

        float t = Math.Clamp((value - from) / (to - from), 0f, 1f);
        return t * t * (3f - 2f * t);
    }
}
