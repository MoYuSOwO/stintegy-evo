using System;
using System.Collections.Generic;
using System.Text.Json;

namespace StintegyEVO.Core.Drivers.Learned;

/// <summary>
/// What a pack says about itself.
/// </summary>
public sealed record ContentPackManifest(
    string Id,
    string Car,
    string? Label = null
);

/// <summary>
/// What a driver's sidecar says about it.
///
/// It sits beside the weights rather than in a list somewhere because the
/// two travel together: a driver that is copied, moved or deleted takes
/// what is known about it along, and there is no second place to forget to
/// update.
/// </summary>
public sealed record DriverSidecar(
    int FormatVersion,
    string? CertifiedOn = null,
    float CleanLapSeconds = 0f
);

/// <summary>
/// Turns one pack's declared contents into catalogue entries.
///
/// The walking itself is not here. Which directories exist and how bytes
/// are read belong to whatever is hosting the game — Godot has its own
/// virtual file system and the core has no business knowing about it — so
/// the host enumerates and this parses.
/// </summary>
public static class DriverPackScanner
{
    private static readonly JsonSerializerOptions Json = new()
    {
        PropertyNameCaseInsensitive = true,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true
    };

    public static ContentPackManifest ReadManifest(string json, string where)
    {
        ContentPackManifest? manifest;
        try
        {
            manifest = JsonSerializer.Deserialize<ContentPackManifest>(json, Json);
        }
        catch (JsonException error)
        {
            throw new InvalidOperationException(
                $"The pack manifest at {where} is not valid JSON: {error.Message}",
                error
            );
        }

        if (manifest is null || string.IsNullOrWhiteSpace(manifest.Id))
        {
            throw new InvalidOperationException(
                $"The pack at {where} does not name itself."
            );
        }
        if (string.IsNullOrWhiteSpace(manifest.Car))
        {
            throw new InvalidOperationException(
                $"The pack '{manifest.Id}' does not say which car its drivers " +
                "are for."
            );
        }
        return manifest;
    }

    /// <summary>
    /// One pack's drivers, as catalogue entries.
    ///
    /// <paramref name="tracks"/> is the circuit ids the host found in the
    /// pack's <c>drivers/</c> directory — the file stem, so
    /// <c>drivers/silverstone.nn</c> is the driver for <c>silverstone</c>.
    /// Naming the circuit with the file is what makes a pack declarative:
    /// there is nothing to write down twice and therefore nothing that can
    /// disagree with itself.
    /// </summary>
    public static IEnumerable<DriverPackEntry> Entries(
        ContentPackManifest manifest,
        ContentPackTier tier,
        IEnumerable<string> tracks,
        Func<string, string?> readSidecar,
        Func<string, byte[]> readWeights
    )
    {
        ArgumentNullException.ThrowIfNull(manifest);
        ArgumentNullException.ThrowIfNull(tracks);
        ArgumentNullException.ThrowIfNull(readSidecar);
        ArgumentNullException.ThrowIfNull(readWeights);

        foreach (string track in tracks)
        {
            string? sidecarJson = readSidecar(track);
            DriverSidecar sidecar;
            if (string.IsNullOrWhiteSpace(sidecarJson))
            {
                // A driver with no sidecar is readable but unvouched: it
                // claims no contract version and no certificate, so it
                // loses every conflict it takes part in and shows nothing
                // on a profile screen. Better than refusing to load a mod
                // over a missing file, and honest about what is unknown.
                sidecar = new DriverSidecar(FormatVersion: 0);
            }
            else
            {
                try
                {
                    sidecar =
                        JsonSerializer.Deserialize<DriverSidecar>(
                            sidecarJson!, Json
                        ) ?? new DriverSidecar(0);
                }
                catch (JsonException error)
                {
                    throw new InvalidOperationException(
                        $"The sidecar for '{track}' in pack '{manifest.Id}' is " +
                        $"not valid JSON: {error.Message}",
                        error
                    );
                }
            }

            string captured = track;
            yield return new DriverPackEntry(
                manifest.Id,
                tier,
                manifest.Car,
                captured,
                sidecar.FormatVersion,
                sidecar.CertifiedOn,
                sidecar.CleanLapSeconds,
                () => readWeights(captured)
            );
        }
    }
}
