using System;

namespace StintegyEVO.Core.Track;

/// <summary>
/// Surface profiles for circuits, derived from how tracks are actually
/// built rather than from any particular survey.
///
/// A word on why the Grand Prix circuits here get so little bank: modern
/// Formula One venues have essentially none. Silverstone is a converted
/// airfield and famously flat; Shanghai, Sepang and Monaco carry only the
/// crossfall a road needs to shed water, on the order of a couple of
/// degrees. Real banking of the kind that changes how a corner is driven
/// belongs to speedways and to a handful of purpose-built corners
/// elsewhere. Inventing a banked Silverstone would make the physics look
/// exercised while teaching a car something untrue about the place, so the
/// road circuits get the construction model and the speedway gets the real
/// article.
/// </summary>
public static class TrackSurfaces
{
    /// <summary>Crossfall a straight is built with to drain, as a fraction.</summary>
    private const float CrownFraction = 0.02f;

    /// <summary>Tangent of the most superelevation a road circuit's corner gets.</summary>
    private const float MaximumCornerBankTangent = 0.0437f;

    /// <summary>
    /// Curvature at which a corner has most of its superelevation, about a
    /// hundred-metre radius.
    /// </summary>
    private const float CornerBankReferenceCurvature = 0.01f;

    /// <summary>
    /// Which way a corner leans and how committed it is, as one signed
    /// number that eases through zero rather than switching sign at it.
    /// Anything deciding a bank's direction has to use something like this:
    /// the curvature a track is built from carries noise around zero on the
    /// straights, and a hard sign turns that noise into a full-magnitude
    /// bank pointing whichever way the noise happened to fall.
    /// </summary>
    public static float CornerLean(float curvature, float referenceCurvature)
    {
        return MathF.Tanh(curvature / MathF.Max(referenceCurvature, 1e-6f));
    }

    /// <summary>
    /// How much of a stretch's treatment applies at one place on the lap:
    /// one in the middle of it, nothing outside, easing between over the
    /// blend length at either end.
    ///
    /// Measured round the lap rather than along a number line, so a corner
    /// that straddles the start/finish line is treated like any other. A
    /// straight comparison would clip such a corner's ramp at the seam and
    /// leave the bank stepping there -- and a layout is free to put a corner
    /// wherever the corner is.
    /// </summary>
    public static float SectionWeight(
        float distanceMeters,
        float startMeters,
        float endMeters,
        float blendMeters,
        float lapLengthMeters
    )
    {
        float blend = MathF.Max(blendMeters, 1e-3f);
        float from = startMeters - blend;
        float span = (endMeters + blend) - from;
        if (span <= 0f)
            return 0f;

        float offset = distanceMeters - from;
        if (lapLengthMeters > 0f)
        {
            offset -= lapLengthMeters *
                      MathF.Floor(offset / lapLengthMeters);
        }
        if (offset >= span)
            return 0f;

        float t = MathF.Min(
            Math.Clamp(offset / blend, 0f, 1f),
            Math.Clamp((span - offset) / blend, 0f, 1f)
        );
        return t * t * (3f - 2f * t);
    }

    /// <summary>
    /// How a road circuit is built: a crown across the straights so water
    /// runs off both edges, easing into a shallow superelevation through
    /// the corners as the section tilts into the turn. Both are drainage
    /// features first; the grip they add is real but small, which is the
    /// honest description of a modern Grand Prix surface.
    /// </summary>
    public static TrackSurface RoadCircuit(TrackSurfaceContext context)
    {
        float curvature = context.CentrelineCurvature;
        float tightness = MathF.Tanh(
            MathF.Abs(curvature) / CornerBankReferenceCurvature
        );
        float bankSlope = MathF.CopySign(
            MaximumCornerBankTangent * tightness,
            curvature
        );

        // A crowned section and a banked one are alternatives: where the
        // road tilts into a corner it no longer sheds water off both sides,
        // so the crown fades out as the bank comes in.
        float halfWidth = MathF.Max(context.HalfWidthMeters, 1f);
        float crown = -CrownFraction / halfWidth * (1f - tightness);
        return new TrackSurface(
            Grade: 0f,
            BankSlope: bankSlope,
            BankCurvature: crown
        );
    }

    /// <summary>
    /// A speedway's turns, in the shape Daytona made familiar: steep at the
    /// wall, shallow at the apron, so the high line carries more bank and
    /// more grip in exchange for a longer way round. The banking eases in
    /// and out with the curvature rather than switching on at the corner,
    /// which is how the transitions are actually built.
    /// </summary>
    public static TrackSurface Speedway(TrackSurfaceContext context)
    {
        const float innerTangent = 0.325f;   // about eighteen degrees
        const float outerTangent = 0.601f;   // about thirty-one degrees

        float tightness = MathF.Tanh(
            MathF.Abs(context.CentrelineCurvature) / 0.004f
        );
        if (tightness <= 1e-3f)
            return new TrackSurface(BankCurvature: -CrownFraction /
                                                   MathF.Max(context.HalfWidthMeters, 1f));

        float halfWidth = MathF.Max(context.HalfWidthMeters, 1f);
        float mean = 0.5f * (outerTangent + innerTangent) * tightness;
        float spread = 0.5f * (outerTangent - innerTangent) * tightness;
        return new TrackSurface(
            Grade: 0f,
            BankSlope: MathF.CopySign(mean, context.CentrelineCurvature),
            BankCurvature: spread / (2f * halfWidth)
        );
    }
}
