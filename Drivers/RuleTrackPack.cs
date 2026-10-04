using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers;

/// <summary>
/// Per-circuit TUM racing line (path only) and follow gains. Speed is planned
/// on that line from the live car. Core still has no racing line.
/// </summary>
public sealed record RuleTrackPack
{
    public required string Id { get; init; }
    public required float LengthMeters { get; init; }
    public float LengthToleranceMeters { get; init; } = 25f;
    public float GgUsage { get; init; } = 0.97f;
    public float CatchKappa { get; init; } = 0.03f;
    public float SpeedGain { get; init; } = 2.5f;
    public float SpeedBoost { get; init; } = 1.06f;
    public float SpeedLookaheadMeters { get; init; } = 24f;
    public float KappaWindowMeters { get; init; } = 2f;
    public float TopSpeedMetersPerSecond { get; init; } = 80f;
    public float BrakeUsage { get; init; } = 1f;
    /// <summary>
    /// Share of lateral grip the speed plan may spend. The rest is there so
    /// the car can still turn back onto the line.
    /// </summary>
    public float LateralPlanShare { get; init; } = 0.90f;
    /// <summary>
    /// Multiplier on the path curvature used for the speed table. One uses the
    /// line as measured. Below one plans a faster corner, which is only safe
    /// when the follower cuts a wider arc than the chord curvature.
    /// </summary>
    public float CurvatureScale { get; init; } = 1f;
    public (float FromS, float ToS, float Gg)[] TightSectors { get; init; } = [];

    public float GgAt(float s)
    {
        foreach ((float from, float to, float gg) in TightSectors)
        {
            if (s >= from && s <= to)
                return gg;
        }

        return GgUsage;
    }

    public static readonly RuleTrackPack Silverstone = new()
    {
        Id = "silverstone",
        LengthMeters = 5_891f,
        GgUsage = 1.00f,
        CatchKappa = 0.018f,
        SpeedGain = 4f,
        SpeedBoost = 1.00f,
        SpeedLookaheadMeters = 0f,
        KappaWindowMeters = 4f,
        TopSpeedMetersPerSecond = 88f,
        BrakeUsage = 0.55f,
        LateralPlanShare = 0.90f,
        CurvatureScale = 1f
    };

    public static readonly RuleTrackPack Simple = new()
    {
        Id = "simple",
        LengthMeters = 1_804f,
        LengthToleranceMeters = 80f,
        GgUsage = 0.88f,
        CatchKappa = 0.028f,
        SpeedGain = 2.8f,
        SpeedBoost = 1.00f,
        SpeedLookaheadMeters = 0f,
        KappaWindowMeters = 4f,
        TopSpeedMetersPerSecond = 80f
    };

    public static readonly RuleTrackPack[] All = [Silverstone, Simple];

    public static RuleTrackPack For(float lengthMeters)
    {
        RuleTrackPack? best = null;
        float bestError = float.PositiveInfinity;
        foreach (RuleTrackPack pack in All)
        {
            float error = MathF.Abs(pack.LengthMeters - lengthMeters);
            if (error < bestError)
            {
                best = pack;
                bestError = error;
            }
        }

        if (best is null || bestError > best.LengthToleranceMeters)
            return Simple with { Id = "geometric", LengthMeters = lengthMeters };
        return best;
    }

    public bool Matches(float lengthMeters) =>
        MathF.Abs(LengthMeters - lengthMeters) <= LengthToleranceMeters;
}
