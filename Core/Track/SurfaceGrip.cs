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
    /// The painted-and-ribbed strip at the edge of the road, and where it
    /// lives: <b>beyond</b> the white line, at the head of the run-off,
    /// never inside the racing surface.
    ///
    /// This is the one grammar every edge on every circuit is built from.
    /// The racing surface is racing surface right up to its own line; the
    /// first 0.6 m past that line is kerb; everything past the kerb is
    /// grass. A car has the whole road, then a strip that costs it a
    /// little, then a lot.
    ///
    /// It was the other way round once — the last 0.6 m of the road was
    /// kerb and the run-off was uniformly slippery from the line outwards.
    /// That put the price of a mistake entirely inside one step: the wheel
    /// crossed the line and lost more than half its grip with nothing in
    /// between. A real circuit does not do that, and neither does a real
    /// driver's decision: riding a kerb is a choice with a small price,
    /// and it is the price that makes it a choice.
    /// </summary>
    public const float KerbWidthMeters = 0.6f;
    public const float Kerb = 0.9f;

    /// <summary>
    /// The narrowest run-off there is: kerb, then wall, and nothing else.
    /// A street circuit. Every buffer is at least this wide, so that the
    /// kerb always exists — the grammar has no case for a white line with
    /// a wall immediately behind it, because on a real circuit there is
    /// always something painted before the barrier.
    /// </summary>
    public const float MinimumBufferMeters = KerbWidthMeters;

    /// <summary>
    /// Beyond the kerb. A mixture of the things run-off is actually made
    /// of — grass at about four tenths, gravel at about a third, asphalt
    /// run-off at nearly full — taken at the pessimistic end, because the
    /// run-off a driver reaches by mistake is rarely the paved kind.
    /// </summary>
    public const float Buffer = 0.45f;

    /// <summary>
    /// How far a boundary is smeared over. A step change in grip is a step
    /// change in the force on one corner of the car, which is a cliff for
    /// anything integrating it and a cliff for anything learning against
    /// it. Short enough that the edge is still an edge.
    ///
    /// There are two boundaries now — tarmac to kerb at the white line,
    /// and kerb to grass 0.6 m past it — and both use this same distance.
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

        // Two blends rather than three cases, so that every offset gets a
        // continuous answer and no circuit's geometry can fall between the
        // branches.
        float ontoKerb = SmoothStep(half, half + TransitionMeters, distance);
        float ontoGrass = SmoothStep(
            half + KerbWidthMeters,
            half + KerbWidthMeters + TransitionMeters,
            distance
        );
        float surface = RacingSurface + (Kerb - RacingSurface) * ontoKerb;
        return surface + (Buffer - surface) * ontoGrass;
    }

    private static float SmoothStep(float from, float to, float value)
    {
        if (to - from <= 1e-5f)
            return value >= to ? 1f : 0f;

        float t = Math.Clamp((value - from) / (to - from), 0f, 1f);
        return t * t * (3f - 2f * t);
    }
}
