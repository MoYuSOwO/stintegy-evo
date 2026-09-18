using System;

namespace StintegyEVO.Core.Drivers;

public sealed record DriverProfile
{
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
