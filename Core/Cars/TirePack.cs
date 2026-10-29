using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using StintegyEVO.Core.Content;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// A tyre pack's <c>tyres.json</c>: the compounds it offers, each the ten
/// numbers a compound sets (<see cref="TireConfig"/>), every one written
/// out. No code: the tyre is the game's (<see cref="Tire"/>).
/// </summary>
public sealed record TirePackManifest
{
    public const string FileName = "tyres.json";
    public const int CurrentFormat = 1;

    public int Format { get; init; } = CurrentFormat;

    /// <summary>The pack's name within its mod's namespace.</summary>
    public string Id { get; init; } = "";

    public string Name { get; init; } = "";

    public IReadOnlyList<CompoundEntry> Compounds { get; init; } = [];

    /// <summary>
    /// One compound: the same numbers on both axles (<see cref="Tyre"/>), or
    /// its own on each (<see cref="Front"/> and <see cref="Rear"/>).
    /// </summary>
    public sealed record CompoundEntry
    {
        public string Id { get; init; } = "";
        public string Name { get; init; } = "";
        public JsonElement? Tyre { get; init; }
        public JsonElement? Front { get; init; }
        public JsonElement? Rear { get; init; }
    }

    internal static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true
    };

    public static TirePackManifest Parse(string json) =>
        JsonSerializer.Deserialize<TirePackManifest>(json, Json)
        ?? throw new InvalidDataException("tyres.json is empty.");
}

/// <summary>A tyre pack, read: what it is, where in its mod it came from, and its compounds.</summary>
/// <param name="Fingerprint">A short hash of its tyres.json: what anything learned on these tyres was learned on.</param>
public sealed record LoadedTires(ResourceId Id, TirePackManifest Manifest, string Folder,
    IReadOnlyList<TireCompound> Compounds, string Fingerprint);

/// <summary>
/// Reads a tyre pack's <c>tyres.json</c>, with every check a load makes. A
/// mod lists its packs in its <c>mod.json</c>; a car says which it runs
/// (<see cref="CarManifest.Tyres"/>).
/// </summary>
public static class TirePack
{
    /// <summary>Between a pack's id and a compound's in a compound's full id: <c>official:formula_a/medium</c>.</summary>
    public const char CompoundSeparator = '/';

    // The numbers a compound must set: every one TireConfig lets it.
    private static readonly string[] Numbers = typeof(TireConfig)
        .GetProperties(BindingFlags.Public | BindingFlags.Instance)
        .Where(p => p.SetMethod is not null)
        .Select(p => JsonNamingPolicy.CamelCase.ConvertName(p.Name))
        .ToArray();

    /// <summary>
    /// A tyre pack read from its <c>tyres.json</c> as it would be loaded,
    /// with every check a load makes: for a tool or a test to try one.
    /// </summary>
    public static LoadedTires Read(ResourceId id, string text, string folder = "")
    {
        TirePackManifest manifest;
        try
        {
            manifest = TirePackManifest.Parse(text);
        }
        catch (JsonException error)
        {
            throw new InvalidDataException($"Tyre pack '{id}': {error.Message}", error);
        }
        if (manifest.Format > TirePackManifest.CurrentFormat)
            throw new InvalidDataException($"Tyre pack '{id}' is format {manifest.Format}; this game reads up to {TirePackManifest.CurrentFormat}.");
        if (manifest.Id != id.Name)
            throw new InvalidDataException($"Tyre pack '{id}' calls itself '{manifest.Id}'.");
        if (manifest.Compounds.Count == 0)
            throw new InvalidDataException($"Tyre pack '{id}' offers no compounds.");

        var compounds = new List<TireCompound>();
        foreach (TirePackManifest.CompoundEntry entry in manifest.Compounds)
        {
            // A compound's own name follows the same rule as any id's.
            if (!ResourceId.TryParse($"{id.Namespace}:{entry.Id}", out _))
                throw new InvalidDataException($"Tyre pack '{id}': '{entry.Id}' is not a compound id (lower case letters, digits and underscores).");
            string full = $"{id}{CompoundSeparator}{entry.Id}";
            if (compounds.Any(c => c.Id == full))
                throw new InvalidDataException($"Tyre pack '{id}' offers '{entry.Id}' twice.");
            bool both = entry.Tyre is not null;
            bool split = entry.Front is not null || entry.Rear is not null;
            if (both == split || (split && (entry.Front is null || entry.Rear is null)))
                throw new InvalidDataException($"{full}: give either \"tyre\", the same on both axles, or both \"front\" and \"rear\".");
            TireConfig front = NumbersOf(full, both ? entry.Tyre!.Value : entry.Front!.Value);
            TireConfig rear = both ? front : NumbersOf(full, entry.Rear!.Value);
            compounds.Add(new TireCompound(full, string.IsNullOrWhiteSpace(entry.Name) ? entry.Id : entry.Name, front, rear));
        }

        string fingerprint = Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(text)), 0, 6).ToLowerInvariant();
        return new LoadedTires(id, manifest, folder, compounds, fingerprint);
    }

    // Every number written out: one left to a default would be the game's
    // number standing in a pack that never chose it.
    private static TireConfig NumbersOf(string compound, JsonElement numbers)
    {
        if (numbers.ValueKind != JsonValueKind.Object)
            throw new InvalidDataException($"{compound}: the tyre's numbers are an object.");
        string[] missing = Numbers
            .Where(name => !numbers.EnumerateObject().Any(p => string.Equals(p.Name, name, StringComparison.OrdinalIgnoreCase)))
            .ToArray();
        if (missing.Length > 0)
            throw new InvalidDataException($"{compound} leaves out {string.Join(", ", missing)}.");
        TireConfig tire;
        try
        {
            tire = numbers.Deserialize<TireConfig>(TirePackManifest.Json)
                   ?? throw new InvalidDataException($"{compound}: no numbers.");
        }
        catch (JsonException error)
        {
            throw new InvalidDataException($"{compound}: {error.Message}", error);
        }
        if (!(tire.BaseMu > 0f) || !(tire.LoadSensitivity >= 0f) || !(tire.LoadSensitivity < 1f))
            throw new InvalidDataException($"{compound}: baseMu must be above zero and loadSensitivity from zero up to one.");
        if (!(tire.IdealSurfaceTempLowC < tire.IdealSurfaceTempHighC))
            throw new InvalidDataException($"{compound}: the window's low edge must be below its high one.");
        if (!(tire.HeatScale > 0f) || !(tire.WearScale > 0f) || !(tire.ColdGripLossPerCSquared >= 0f) ||
            !(tire.HotGripLossPerCSquared >= 0f) || !(tire.WearCliffGripLoss >= 0f))
            throw new InvalidDataException($"{compound}: the scales must be above zero and the losses not below it.");
        return tire;
    }
}
