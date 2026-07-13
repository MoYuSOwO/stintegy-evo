using System;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Stable, manager-facing driver ratings. Ratings describe the driver's mean
/// ability; consistency controls how widely individual sessions, laps, and
/// track segments vary around that mean.
/// </summary>
public sealed record DriverAbilities
{
    public float Pace { get; init; } = 100f;
    public float Consistency { get; init; } = 100f;
    public float CarControl { get; init; } = 100f;
    public float TireManagement { get; init; } = 80f;
    public float Adaptability { get; init; } = 100f;
    public float Reactions { get; init; } = 100f;
    public float Awareness { get; init; } = 100f;
    public float Overtaking { get; init; } = 100f;
    public float Defending { get; init; } = 100f;

    internal void Validate()
    {
        ValidateRating(Pace, nameof(Pace));
        ValidateRating(Consistency, nameof(Consistency));
        ValidateRating(CarControl, nameof(CarControl));
        ValidateRating(TireManagement, nameof(TireManagement));
        ValidateRating(Adaptability, nameof(Adaptability));
        ValidateRating(Reactions, nameof(Reactions));
        ValidateRating(Awareness, nameof(Awareness));
        ValidateRating(Overtaking, nameof(Overtaking));
        ValidateRating(Defending, nameof(Defending));
    }

    private static void ValidateRating(float rating, string name)
    {
        if (!float.IsFinite(rating) || rating < 0f || rating > 100f)
            throw new ArgumentOutOfRangeException(name, "Driver ratings must be finite and in [0, 100].");
    }
}

public sealed record DriverProfile
{
    public static readonly DriverProfile LegacyBaseline = new(
        "legacy-baseline",
        new DriverAbilities(),
        randomSeed: 0x5354494E54454759UL
    );

    public DriverProfile(string id, DriverAbilities abilities, ulong randomSeed = 1UL)
    {
        if (string.IsNullOrWhiteSpace(id))
            throw new ArgumentException("Driver id is required.", nameof(id));
        ArgumentNullException.ThrowIfNull(abilities);
        abilities.Validate();

        Id = id;
        Abilities = abilities;
        RandomSeed = randomSeed == 0UL ? 1UL : randomSeed;
    }

    public string Id { get; }
    public DriverAbilities Abilities { get; }
    public ulong RandomSeed { get; }
}
