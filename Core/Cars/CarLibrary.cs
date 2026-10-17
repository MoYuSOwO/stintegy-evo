using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Loader;
using System.Security.Cryptography;
using System.Text.Json;
using StintegyEVO.Core.Content;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// What a car pack's assembly offers: a way to build the car. One per pack.
/// The game's own car is a pack like any other and is found the same way.
/// </summary>
public interface ICarDefinition
{
    ICar Build();
}

/// <summary>
/// A car pack's <c>car.json</c>: what it is called and where its code is.
/// </summary>
public sealed record CarManifest
{
    public const string FileName = "car.json";
    public const int CurrentFormat = 1;

    public int Format { get; init; } = CurrentFormat;

    /// <summary>The car's name within its namespace; the folder is named the same.</summary>
    public string Id { get; init; } = "";

    public string Name { get; init; } = "";

    /// <summary>The pack's assembly, relative to its folder.</summary>
    public string Assembly { get; init; } = "";

    /// <summary>The <see cref="ICarDefinition"/> to build, when the assembly holds more than one.</summary>
    public string? Definition { get; init; }

    /// <summary>The car's model, relative to its folder; none to be drawn the game's way.</summary>
    public string? Model { get; init; }

    private static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        WriteIndented = true
    };

    public static CarManifest Parse(string json) =>
        JsonSerializer.Deserialize<CarManifest>(json, Json)
        ?? throw new InvalidDataException("car.json is empty.");
}

/// <summary>A car pack, loaded: what it is, where it came from, and the car.</summary>
/// <param name="Fingerprint">
/// A short hash of the pack's code. Anything learned on this car -- lanes, a
/// driver's notes -- is learned on this code, and is stale when it changes.
/// </param>
public sealed record LoadedCar(ResourceId Id, CarManifest Manifest, string Folder, ICar Car, string Fingerprint);

/// <summary>
/// Finds car packs under <c>data/&lt;namespace&gt;/cars/&lt;name&gt;/</c> and
/// loads them. Every car, the game's own included, comes from here.
///
/// A pack is code, and loading it runs it: install packs from people you
/// trust, as with any game's code mods.
/// </summary>
public static class CarLibrary
{
    public const string DataFolder = "data";
    public const string CarsFolder = "cars";

    /// <summary>Data folders to look in, in order. The first holding the car wins.</summary>
    public static List<string> Roots { get; } = DefaultRoots();

    private static readonly Dictionary<ResourceId, LoadedCar> Loaded = new();
    private static readonly object Gate = new();

    /// <summary>The car named by <paramref name="id"/>; a bare name is the game's own.</summary>
    public static LoadedCar Load(string id) => Load(ResourceId.Parse(id));

    public static LoadedCar Load(ResourceId id)
    {
        lock (Gate)
        {
            if (Loaded.TryGetValue(id, out LoadedCar? loaded))
                return loaded;
            loaded = LoadFrom(Find(id), id);
            Loaded[id] = loaded;
            return loaded;
        }
    }

    /// <summary>Every car pack the roots hold, by id, unloaded.</summary>
    public static IEnumerable<ResourceId> Available()
    {
        var seen = new HashSet<ResourceId>();
        foreach (string root in Roots)
        {
            if (!Directory.Exists(root))
                continue;
            foreach (string space in Directory.EnumerateDirectories(root))
            {
                string cars = Path.Combine(space, CarsFolder);
                if (!Directory.Exists(cars))
                    continue;
                foreach (string folder in Directory.EnumerateDirectories(cars))
                {
                    if (!File.Exists(Path.Combine(folder, CarManifest.FileName)))
                        continue;
                    if (ResourceId.TryParse($"{Path.GetFileName(space)}:{Path.GetFileName(folder)}", out ResourceId id) &&
                        seen.Add(id))
                        yield return id;
                }
            }
        }
    }

    private static string Find(ResourceId id)
    {
        foreach (string root in Roots)
        {
            string folder = Path.Combine(root, id.Namespace, CarsFolder, id.Name);
            if (File.Exists(Path.Combine(folder, CarManifest.FileName)))
                return folder;
        }
        throw new FileNotFoundException($"No car '{id}' under {string.Join(", ", Roots)}.");
    }

    private static LoadedCar LoadFrom(string folder, ResourceId id)
    {
        CarManifest manifest = CarManifest.Parse(File.ReadAllText(Path.Combine(folder, CarManifest.FileName)));
        if (manifest.Format > CarManifest.CurrentFormat)
            throw new InvalidDataException($"Car '{id}' is format {manifest.Format}; this game reads up to {CarManifest.CurrentFormat}.");
        if (manifest.Id != id.Name)
            throw new InvalidDataException($"Car '{id}' calls itself '{manifest.Id}'; a pack's id is its folder's name.");
        if (string.IsNullOrWhiteSpace(manifest.Assembly))
            throw new InvalidDataException($"Car '{id}' names no assembly.");

        string path = Path.GetFullPath(Path.Combine(folder, manifest.Assembly));
        if (!File.Exists(path))
            throw new FileNotFoundException($"Car '{id}' has no {manifest.Assembly} in {folder}; has the pack been built?", path);

        // Into the context Core itself lives in, so the pack and the game
        // share one Core: an ICar from the pack is the game's ICar.
        AssemblyLoadContext context = AssemblyLoadContext.GetLoadContext(typeof(ICar).Assembly) ?? AssemblyLoadContext.Default;
        Assembly assembly = context.LoadFromAssemblyPath(path);

        Type[] definitions = assembly.GetTypes()
            .Where(t => typeof(ICarDefinition).IsAssignableFrom(t) && t is { IsAbstract: false, IsInterface: false })
            .Where(t => manifest.Definition is null || t.FullName == manifest.Definition || t.Name == manifest.Definition)
            .ToArray();
        if (definitions.Length != 1)
            throw new InvalidDataException(
                $"Car '{id}' needs exactly one car definition in {manifest.Assembly}" +
                (manifest.Definition is null ? "" : $" named '{manifest.Definition}'") + $"; found {definitions.Length}.");

        var definition = (ICarDefinition)(Activator.CreateInstance(definitions[0])
            ?? throw new InvalidDataException($"Car '{id}': {definitions[0].Name} cannot be made."));
        ICar car = definition.Build() ?? throw new InvalidDataException($"Car '{id}' built nothing.");
        if (car.Ladders.Count > CarStrategy.MaximumLadders)
            throw new InvalidDataException($"Car '{id}' declares {car.Ladders.Count} ladders; at most {CarStrategy.MaximumLadders}.");

        string fingerprint = Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(path)), 0, 6).ToLowerInvariant();
        return new LoadedCar(id, manifest, folder, car, fingerprint);
    }

    private static List<string> DefaultRoots()
    {
        var roots = new List<string>();
        if (Environment.GetEnvironmentVariable("STINTEGY_DATA") is { Length: > 0 } configured)
            roots.AddRange(configured.Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries));
        foreach (string start in new[] { AppContext.BaseDirectory, Environment.CurrentDirectory })
        {
            for (DirectoryInfo? dir = new(start); dir is not null; dir = dir.Parent)
            {
                string data = Path.Combine(dir.FullName, DataFolder);
                if (!Directory.Exists(data))
                    continue;
                if (!roots.Contains(data))
                    roots.Add(data);
                break;
            }
        }
        return roots;
    }
}
