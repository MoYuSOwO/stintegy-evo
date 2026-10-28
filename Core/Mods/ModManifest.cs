using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using StintegyEVO.Core.Content;

namespace StintegyEVO.Core.Mods;

/// <summary>What a mod is on disk.</summary>
public static class ModFormat
{
    /// <summary>
    /// A packed mod's file extension. A folder of the same name, unpacked,
    /// is the same mod while it is being made.
    /// </summary>
    public const string Extension = ".stmod";

    /// <summary>The entry of every mod, at its top.</summary>
    public const string ManifestFile = "mod.json";

    public const int CurrentFormat = 1;
}

/// <summary>
/// A mod's <c>mod.json</c>: who it is, what it needs, where its code is,
/// and every piece of data it carries, listed by path. Nothing in a mod is
/// found by looking through its folders: what is not listed here is not
/// there.
/// </summary>
public sealed record ModManifest
{
    public int Format { get; init; } = ModFormat.CurrentFormat;

    /// <summary>The mod's id, and the namespace of everything it registers.</summary>
    public string Id { get; init; } = "";

    public ModText Name { get; init; } = new("");

    /// <summary>Its version, <c>major.minor.patch</c> (<see cref="ModVersion"/>).</summary>
    public string Version { get; init; } = "";

    /// <summary>The mods it needs, by id, each with the versions it takes (<see cref="ModVersionRequirement"/>).</summary>
    public IReadOnlyDictionary<string, string> Depends { get; init; } = new Dictionary<string, string>();

    /// <summary>Its code, by path in the mod; none for a mod of data alone.</summary>
    public string? Assembly { get; init; }

    /// <summary>The <see cref="IMod"/> in <see cref="Assembly"/> to build and register, by full type name.</summary>
    public string? Entrypoint { get; init; }

    /// <summary>Its circuits: each a folder holding a <c>circuit.json</c>.</summary>
    public IReadOnlyList<string> Circuits { get; init; } = [];

    /// <summary>Its tyre packs: each a file in the form of a <c>tyres.json</c>.</summary>
    public IReadOnlyList<string> Tyres { get; init; } = [];

    /// <summary>Its props: files mapping a prop's id to its model.</summary>
    public IReadOnlyList<string> Props { get; init; } = [];

    /// <summary>
    /// Scenery it adds to circuits, its own or another mod's: the one thing
    /// a mod may add to what is not its own, and only add, never take away.
    /// </summary>
    public IReadOnlyList<ModSceneryEntry> Scenery { get; init; } = [];

    internal static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true
    };

    public static ModManifest Parse(string json) =>
        JsonSerializer.Deserialize<ModManifest>(json, Json)
        ?? throw new InvalidDataException($"{ModFormat.ManifestFile} is empty.");

    /// <summary>What is wrong with it, read on its own; nothing for a manifest the game can load.</summary>
    public IEnumerable<string> Problems()
    {
        if (Format > ModFormat.CurrentFormat)
            yield return $"is format {Format}; this game reads up to {ModFormat.CurrentFormat}";
        if (!ResourceId.IsPart(Id))
            yield return $"'{Id}' is not a mod id (lower case letters, digits and underscores)";
        if (!ModVersion.TryParse(Version, out _))
            yield return $"version '{Version}' is not major.minor.patch";
        foreach ((string needed, string versions) in Depends)
        {
            if (!ResourceId.IsPart(needed))
                yield return $"depends on '{needed}', which is not a mod id";
            if (!ModVersionRequirement.TryParse(versions, out _))
                yield return $"needs '{needed}' at '{versions}'; only \">=major.minor.patch\" is understood, within the same major version";
        }
        if (string.IsNullOrWhiteSpace(Assembly) != string.IsNullOrWhiteSpace(Entrypoint))
            yield return "names an assembly without an entrypoint, or an entrypoint without an assembly; a mod with code names both";
    }
}

/// <summary>Scenery a mod adds to a circuit: which circuit, by its full id, and the file in the mod.</summary>
public sealed record ModSceneryEntry
{
    public string Circuit { get; init; } = "";
    public string File { get; init; } = "";
}

/// <summary>
/// Words shown to a player: one text, or one per language
/// (<c>{ "zh": "…", "en": "…" }</c>).
/// </summary>
[JsonConverter(typeof(ModTextConverter))]
public sealed class ModText
{
    public ModText(string text) => Languages = new Dictionary<string, string> { [""] = text };

    public ModText(IReadOnlyDictionary<string, string> languages) => Languages = languages;

    /// <summary>The text by language; the empty key is a text given in no language.</summary>
    public IReadOnlyDictionary<string, string> Languages { get; }

    /// <summary>The text in <paramref name="language"/>, or else in English, or else the first there is.</summary>
    public string In(string? language = null) =>
        (language is not null && Languages.TryGetValue(language, out string? text)) ? text
        : Languages.TryGetValue("", out text) ? text
        : Languages.TryGetValue("en", out text) ? text
        : Languages.Values.FirstOrDefault() ?? "";

    public override string ToString() => In();
}

internal sealed class ModTextConverter : JsonConverter<ModText>
{
    public override ModText Read(ref Utf8JsonReader reader, Type type, JsonSerializerOptions options) =>
        reader.TokenType == JsonTokenType.String
            ? new ModText(reader.GetString() ?? "")
            : new ModText(JsonSerializer.Deserialize<Dictionary<string, string>>(ref reader)
                          ?? throw new JsonException("A name is a text, or a text per language."));

    public override void Write(Utf8JsonWriter writer, ModText value, JsonSerializerOptions options)
    {
        if (value.Languages.Count == 1 && value.Languages.ContainsKey(""))
            writer.WriteStringValue(value.Languages[""]);
        else
            JsonSerializer.Serialize(writer, value.Languages);
    }
}

/// <summary>A mod's version: <c>major.minor.patch</c>, compared part by part.</summary>
public readonly record struct ModVersion(int Major, int Minor, int Patch) : IComparable<ModVersion>
{
    public static bool TryParse(string? text, out ModVersion version)
    {
        version = default;
        string[] parts = (text ?? "").Split('.');
        if (parts.Length != 3)
            return false;
        var numbers = new int[3];
        for (int i = 0; i < 3; i++)
            if (!int.TryParse(parts[i], NumberStyles.None, CultureInfo.InvariantCulture, out numbers[i]))
                return false;
        version = new ModVersion(numbers[0], numbers[1], numbers[2]);
        return true;
    }

    public static ModVersion Parse(string text) =>
        TryParse(text, out ModVersion version) ? version
        : throw new FormatException($"'{text}' is not a version (major.minor.patch).");

    public int CompareTo(ModVersion other) =>
        Major != other.Major ? Major.CompareTo(other.Major)
        : Minor != other.Minor ? Minor.CompareTo(other.Minor)
        : Patch.CompareTo(other.Patch);

    public static bool operator >=(ModVersion a, ModVersion b) => a.CompareTo(b) >= 0;
    public static bool operator <=(ModVersion a, ModVersion b) => a.CompareTo(b) <= 0;
    public static bool operator >(ModVersion a, ModVersion b) => a.CompareTo(b) > 0;
    public static bool operator <(ModVersion a, ModVersion b) => a.CompareTo(b) < 0;

    public override string ToString() => $"{Major}.{Minor}.{Patch}";
}

/// <summary>
/// The versions of a mod another takes: this one or later
/// (<c>&gt;=1.2.0</c>), with the same major version. A new major version is
/// its author saying it changed in a way that breaks what was built on it,
/// so <c>&gt;=1.2.0</c> takes 1.5.0 and not 2.0.0.
/// </summary>
public readonly record struct ModVersionRequirement(ModVersion AtLeast)
{
    private const string AtLeastPrefix = ">=";

    public bool IsMetBy(ModVersion version) => version.Major == AtLeast.Major && version >= AtLeast;

    public static bool TryParse(string? text, out ModVersionRequirement requirement)
    {
        requirement = default;
        string trimmed = (text ?? "").Trim();
        if (!trimmed.StartsWith(AtLeastPrefix, StringComparison.Ordinal) ||
            !ModVersion.TryParse(trimmed[AtLeastPrefix.Length..].Trim(), out ModVersion least))
            return false;
        requirement = new ModVersionRequirement(least);
        return true;
    }

    public override string ToString() => $"{AtLeastPrefix}{AtLeast}";
}
