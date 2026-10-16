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
}
