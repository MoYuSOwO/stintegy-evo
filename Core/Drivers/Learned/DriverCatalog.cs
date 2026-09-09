using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;

namespace StintegyEVO.Core.Drivers.Learned;

/// <summary>
/// One driver in the catalogue: which car and circuit it was baked for,
/// where its weights are, which observation contract it speaks, and what
/// it did to earn its place.
///
/// The graduation figures are not bookkeeping. A policy gets into this
/// file by passing certification, and the number it passed with is the
/// thing a driver profile screen will eventually show — so it is recorded
/// when it is known, at the moment of certification, rather than
/// reconstructed later from a log nobody kept.
/// </summary>
public sealed record DriverCatalogEntry(
    string Car,
    string Track,
    string Weights,
    int FormatVersion,
    string? CertifiedOn = null,
    float CleanLapSeconds = 0f
);

/// <summary>
/// Which driver drives which circuit, and nothing else knows.
///
/// The enumerated-circuit policy means there is no general driver: there
/// is one baked policy per car and circuit, and the game has to be able to
/// ask for the right one by name. Every caller used to hold its own
/// hard-coded path to a single file, which worked for exactly as long as
/// there was one circuit.
///
/// There is deliberately no fallback. A circuit with no entry raises,
/// saying so in those words, because the thing a fallback would have
/// quietly substituted no longer exists: the analytic driver was retired
/// as a baseline, and a silent substitution would mean a player driving
/// against something nobody certified while the game says nothing.
///
/// Which makes the key set do double duty: the circuits in this file are
/// exactly the circuits that can be played, so the product's content
/// list is supplied by the catalogue instead of being maintained
/// alongside it and drifting from it.
/// </summary>
public sealed class DriverCatalog
{
    /// <summary>
    /// The car every v1 entry belongs to. There is one car, and naming it
    /// rather than leaving the field out keeps the key a pair from the
    /// start — a second car should add rows, not a schema.
    /// </summary>
    public const string DefaultCar = "default";

    private readonly Dictionary<(string Car, string Track), DriverCatalogEntry>
        _entries;

    private DriverCatalog(
        Dictionary<(string, string), DriverCatalogEntry> entries
    )
    {
        _entries = entries;
    }

    public static DriverCatalog Parse(string json)
    {
        if (string.IsNullOrWhiteSpace(json))
            throw new ArgumentException("The catalogue is empty.", nameof(json));

        DriverCatalogEntry[]? rows;
        try
        {
            rows = JsonSerializer.Deserialize<DriverCatalogEntry[]>(
                json,
                new JsonSerializerOptions
                {
                    PropertyNameCaseInsensitive = true,
                    ReadCommentHandling = JsonCommentHandling.Skip,
                    AllowTrailingCommas = true
                }
            );
        }
        catch (JsonException error)
        {
            throw new InvalidOperationException(
                $"The driver catalogue is not valid JSON: {error.Message}",
                error
            );
        }

        if (rows is null)
            throw new InvalidOperationException("The driver catalogue is null.");

        Dictionary<(string, string), DriverCatalogEntry> entries = new();
        foreach (DriverCatalogEntry row in rows)
        {
            (string, string) key = (row.Car, row.Track);
            if (!entries.TryAdd(key, row))
            {
                throw new InvalidOperationException(
                    $"The driver catalogue lists '{row.Car}' on '{row.Track}' " +
                    "twice; a car and circuit name one driver."
                );
            }
        }
        return new DriverCatalog(entries);
    }

    /// <summary>Every circuit this car has a certified driver for — which
    /// is to say, every circuit that can be played in it.</summary>
    public IReadOnlyList<string> PlayableTracks(string car) =>
        _entries.Keys
            .Where(key => key.Car == car)
            .Select(key => key.Track)
            .OrderBy(track => track, StringComparer.Ordinal)
            .ToList();

    public bool TryFind(
        string car,
        string track,
        out DriverCatalogEntry entry
    ) => _entries.TryGetValue((car, track), out entry!);

    /// <summary>
    /// The driver for this car and circuit, built from its weights.
    ///
    /// <paramref name="readWeights"/> is passed in rather than the file
    /// being opened here, because the engine reads through Godot's own
    /// virtual file system and the core has no business knowing that.
    /// </summary>
    public IRaceDriver Load(
        string car,
        string track,
        Func<string, byte[]> readWeights
    )
    {
        ArgumentNullException.ThrowIfNull(readWeights);
        if (!TryFind(car, track, out DriverCatalogEntry entry))
        {
            throw new InvalidOperationException(
                $"No driver for '{track}' yet. The catalogue has " +
                $"{string.Join(", ", PlayableTracks(car))}. A circuit becomes " +
                "playable when a policy is baked for it and passes " +
                "certification; there is nothing to fall back to."
            );
        }

        byte[] weights = readWeights(entry.Weights);
        if (weights is null || weights.Length == 0)
        {
            throw new InvalidOperationException(
                $"The driver for '{track}' is listed at '{entry.Weights}' and " +
                "that file is missing or empty."
            );
        }

        return new DirectDriveRaceDriver(MlpDrivingPolicy.FromBytes(weights));
    }
}
