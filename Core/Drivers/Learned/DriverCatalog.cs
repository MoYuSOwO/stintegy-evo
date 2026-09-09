using System;
using System.Collections.Generic;
using System.Linq;

namespace StintegyEVO.Core.Drivers.Learned;

/// <summary>
/// Where a pack came from, which is how conflicts between them are
/// settled. Ordered so that a larger value wins.
/// </summary>
public enum ContentPackTier
{
    /// <summary>Ships with the game. Pack zero is one of these.</summary>
    Builtin = 0,

    /// <summary>Installed by the player.</summary>
    User = 1,
}

/// <summary>
/// One driver, as declared by whichever pack supplies it.
///
/// A driver can come from a car pack, a circuit pack, or a pack that is
/// nothing but drivers — a circuit mod is allowed to ship a driver for the
/// official car, and a car mod is allowed to ship drivers for the official
/// circuits. The key is the pair, and who supplies it is not part of the
/// key.
///
/// <see cref="ReadWeights"/> is a thunk rather than the bytes, because
/// scanning a hundred packs at startup should not read a hundred networks
/// into memory to find the one that gets driven.
/// </summary>
public sealed record DriverPackEntry(
    string PackId,
    ContentPackTier Tier,
    string Car,
    string Track,
    int FormatVersion,
    string? CertifiedOn,
    float CleanLapSeconds,
    Func<byte[]> ReadWeights
);

/// <summary>
/// Which driver drives which circuit, aggregated from whatever is
/// installed.
///
/// This is a scan result, not a file. There is no central list anybody
/// edits: a pack declares the drivers it carries by putting them in its
/// own <c>drivers/</c> directory beside their metadata, the game walks the
/// installed packs at startup, and what comes out is this. Shipping a new
/// driver is dropping a file into a pack, and shipping a circuit with its
/// driver is one package rather than a package plus an edit to a global
/// file that a mod has no business editing.
///
/// The built-in car is pack zero and declares itself exactly the way a mod
/// would, through the same scan. Official content goes through the mod
/// door so that the door stays in repair: a format only the modders use is
/// a format that breaks and nobody notices until a modder complains.
///
/// There is deliberately no fallback for a circuit nobody supplies. The
/// thing a fallback would have substituted no longer exists — the analytic
/// driver was retired as a baseline — and a silent substitution would put a
/// player against something nobody certified while the game said nothing.
/// So the key set does double duty: the circuits here are exactly the
/// circuits that can be played.
/// </summary>
public sealed class DriverCatalog
{
    /// <summary>
    /// The car every entry belongs to while there is one. Named rather
    /// than omitted so the key is a pair from the start — a second car
    /// should add rows, not a schema.
    /// </summary>
    public const string DefaultCar = "default";

    private readonly Dictionary<(string Car, string Track), DriverPackEntry>
        _entries;

    private DriverCatalog(
        Dictionary<(string, string), DriverPackEntry> entries
    )
    {
        _entries = entries;
    }

    /// <summary>
    /// Fold everything the scan found into one catalogue.
    ///
    /// Two packs may claim the same car and circuit, and the order they
    /// were walked in must not decide it. A player's pack beats a built-in
    /// one, because installing something is asking for it. Within a tier
    /// the newer certificate wins, because that is the one measured
    /// against the more recent world.
    ///
    /// A tie the rules cannot break is raised rather than guessed. Two
    /// uncertified drivers claiming the same slot is a packaging mistake,
    /// and quietly picking one is exactly the silent substitution the
    /// missing-driver rule exists to prevent.
    /// </summary>
    public static DriverCatalog Aggregate(IEnumerable<DriverPackEntry> found)
    {
        ArgumentNullException.ThrowIfNull(found);
        Dictionary<(string, string), DriverPackEntry> entries = new();

        foreach (DriverPackEntry entry in found)
        {
            (string, string) key = (entry.Car, entry.Track);
            if (!entries.TryGetValue(key, out DriverPackEntry? held))
            {
                entries[key] = entry;
                continue;
            }

            int verdict = Compare(entry, held);
            if (verdict > 0)
                entries[key] = entry;
            else if (verdict == 0)
            {
                throw new InvalidOperationException(
                    $"Packs '{held.PackId}' and '{entry.PackId}' both supply a " +
                    $"driver for '{entry.Car}' on '{entry.Track}', and neither " +
                    "is installed over the other or certified more recently. " +
                    "One of them has to go."
                );
            }
        }

        return new DriverCatalog(entries);
    }

    private static int Compare(DriverPackEntry a, DriverPackEntry b)
    {
        if (a.Tier != b.Tier)
            return a.Tier > b.Tier ? 1 : -1;

        bool aCertified = !string.IsNullOrWhiteSpace(a.CertifiedOn);
        bool bCertified = !string.IsNullOrWhiteSpace(b.CertifiedOn);
        if (aCertified != bCertified)
            return aCertified ? 1 : -1;
        if (!aCertified)
            return 0;

        return string.CompareOrdinal(a.CertifiedOn, b.CertifiedOn) switch
        {
            > 0 => 1,
            < 0 => -1,
            _ => 0
        };
    }

    /// <summary>Every circuit this car has a driver for — which is to say,
    /// every circuit that can be played in it.</summary>
    public IReadOnlyList<string> PlayableTracks(string car) =>
        _entries.Keys
            .Where(key => key.Car == car)
            .Select(key => key.Track)
            .OrderBy(track => track, StringComparer.Ordinal)
            .ToList();

    public bool TryFind(string car, string track, out DriverPackEntry entry) =>
        _entries.TryGetValue((car, track), out entry!);

    public IRaceDriver Load(string car, string track)
    {
        if (!TryFind(car, track, out DriverPackEntry entry))
        {
            IReadOnlyList<string> playable = PlayableTracks(car);
            throw new InvalidOperationException(
                $"No driver for '{track}' yet. Installed packs supply " +
                (playable.Count == 0
                    ? "none at all"
                    : string.Join(", ", playable)) +
                ". A circuit becomes playable when a policy is baked for it " +
                "and passes certification; there is nothing to fall back to."
            );
        }

        byte[] weights = entry.ReadWeights();
        if (weights is null || weights.Length == 0)
        {
            throw new InvalidOperationException(
                $"Pack '{entry.PackId}' declares a driver for '{track}' and " +
                "its weights are missing or empty."
            );
        }

        return new DirectDriveRaceDriver(MlpDrivingPolicy.FromBytes(weights));
    }
}
