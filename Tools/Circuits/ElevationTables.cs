namespace StintegyEVO.Circuits;

/// <summary>
/// Height profiles, as (fraction of a lap, metres), for the circuits whose
/// survey is not usable (see Elevation.Unusable). Authored in the style of
/// the venue, not surveyed.
/// </summary>
internal static class ElevationTables
{
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
}
