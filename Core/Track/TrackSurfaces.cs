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
