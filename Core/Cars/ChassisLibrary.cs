using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Loader;
using System.Text.Json;
using StintegyEVO.Core.Content;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// A chassis pack's <c>chassis.json</c>: what it is called and where its code
/// is. A chassis is physics a car is built on -- the game's own single-track
/// chassis is one -- and is code a car pack builds against, never a car
/// itself: what it offers is whatever its assembly makes public.
/// </summary>
public sealed record ChassisManifest
{
    public const string FileName = "chassis.json";
    public const int CurrentFormat = 1;

    public int Format { get; init; } = CurrentFormat;

    /// <summary>The chassis's name within its namespace; the folder is named the same.</summary>
    public string Id { get; init; } = "";

    public string Name { get; init; } = "";

    /// <summary>
    /// The chassis's own version: its author raises it whenever a change to
    /// it changes how a car on it drives. Everything learned on any car on
    /// it was learned on a version, and is stale on another
    /// (<see cref="LoadedCar.Fingerprint"/>).
    /// </summary>
    public int Version { get; init; } = 1;

    /// <summary>The pack's assembly, relative to its folder.</summary>
    public string Assembly { get; init; } = "";

    private static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        WriteIndented = true
    };

    public static ChassisManifest Parse(string json) =>
        JsonSerializer.Deserialize<ChassisManifest>(json, Json)
        ?? throw new InvalidDataException("chassis.json is empty.");
}

/// <summary>A chassis pack, loaded: what it is, where it came from, and its code.</summary>
public sealed record LoadedChassis(ResourceId Id, ChassisManifest Manifest, string Folder, Assembly Assembly);

/// <summary>
/// Finds chassis packs under <c>data/&lt;namespace&gt;/chassis/&lt;name&gt;/</c>
/// and loads them, before any car that names one (<see cref="CarManifest.Chassis"/>),
/// so the car's assembly finds the chassis it was built against. A car with
/// physics of its own names none.
///
/// A pack is code, and loading it runs it: install packs from people you
/// trust, as with any game's code mods.
/// </summary>
public static class ChassisLibrary
{
    public const string ChassisFolder = "chassis";

    private static readonly Dictionary<ResourceId, LoadedChassis> Loaded = new();
    private static readonly object Gate = new();

    /// <summary>The chassis named by <paramref name="id"/>; a bare name is the game's own.</summary>
    public static LoadedChassis Load(string id) => Load(ResourceId.Parse(id));

    public static LoadedChassis Load(ResourceId id)
    {
        lock (Gate)
        {
            if (Loaded.TryGetValue(id, out LoadedChassis? loaded))
                return loaded;
            string folder = DataFolders.Find(id, ChassisFolder, ChassisManifest.FileName)
                            ?? throw new FileNotFoundException($"No chassis '{id}' under {string.Join(", ", DataFolders.Roots)}.");
            loaded = LoadFrom(folder, id);
            Loaded[id] = loaded;
            return loaded;
        }
    }

    /// <summary>Every chassis pack the roots hold, by id, unloaded.</summary>
    public static IEnumerable<ResourceId> Available() => DataFolders.Available(ChassisFolder, ChassisManifest.FileName);

    private static LoadedChassis LoadFrom(string folder, ResourceId id)
    {
        ChassisManifest manifest = ChassisManifest.Parse(File.ReadAllText(Path.Combine(folder, ChassisManifest.FileName)));
        if (manifest.Format > ChassisManifest.CurrentFormat)
            throw new InvalidDataException($"Chassis '{id}' is format {manifest.Format}; this game reads up to {ChassisManifest.CurrentFormat}.");
        if (manifest.Id != id.Name)
            throw new InvalidDataException($"Chassis '{id}' calls itself '{manifest.Id}'; a pack's id is its folder's name.");
        if (string.IsNullOrWhiteSpace(manifest.Assembly))
            throw new InvalidDataException($"Chassis '{id}' names no assembly.");

        string path = Path.GetFullPath(Path.Combine(folder, manifest.Assembly));
        if (!File.Exists(path))
            throw new FileNotFoundException($"Chassis '{id}' has no {manifest.Assembly} in {folder}; has the pack been built?", path);

        // Into the context Core itself lives in, beside the cars built on it,
        // so a car's reference to the chassis is to this one. One already
        // there -- a test built against it -- is the same chassis.
        AssemblyLoadContext context = AssemblyLoadContext.GetLoadContext(typeof(ICar).Assembly) ?? AssemblyLoadContext.Default;
        string name = AssemblyName.GetAssemblyName(path).Name ?? "";
        Assembly assembly = context.Assemblies.FirstOrDefault(a => a.GetName().Name == name)
                            ?? context.LoadFromAssemblyPath(path);
        return new LoadedChassis(id, manifest, folder, assembly);
    }
}
