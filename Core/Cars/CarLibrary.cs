using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Loader;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using StintegyEVO.Core.Content;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// What a car pack's assembly offers: a way to build the car. One per pack.
/// The game's own car is a pack like any other and is found the same way.
/// </summary>
public interface ICarDefinition
{
    /// <param name="compounds">
    /// The compounds the pack's <c>car.json</c> declares, loaded from their
    /// tyre packs, in the order declared. The car runs these and no others;
    /// its code says nothing about tyres' numbers.
    /// </param>
    ICar Build(IReadOnlyList<TireCompound> compounds);
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

    /// <summary>
    /// The tyres the car runs: whole tyre packs (<c>official:formula_a</c>)
    /// or single compounds of them (<c>official:formula_a/medium</c>). At
    /// least one; a car that does not say what it runs does not run.
    /// </summary>
    public IReadOnlyList<string> Tyres { get; init; } = [];

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
/// A short hash of the pack's code and the tyres it runs. Anything learned
/// on this car -- lanes, a driver's notes -- is learned on these, and is
/// stale when either changes.
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
    public const string CarsFolder = "cars";

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

    /// <summary>
    /// The pack <paramref name="car"/> was built by, for whoever draws it or
    /// files what was learned on it; none for a car made some other way, as
    /// a test makes one.
    /// </summary>
    public static LoadedCar? Origin(ICar car)
    {
        lock (Gate)
            return Loaded.Values.FirstOrDefault(loaded => ReferenceEquals(loaded.Car, car));
    }

    /// <summary>Every car pack the roots hold, by id, unloaded.</summary>
    public static IEnumerable<ResourceId> Available() => DataFolders.Available(CarsFolder, CarManifest.FileName);

    private static string Find(ResourceId id) =>
        DataFolders.Find(id, CarsFolder, CarManifest.FileName)
        ?? throw new FileNotFoundException($"No car '{id}' under {string.Join(", ", DataFolders.Roots)}.");

    /// <summary>
    /// The compounds a car's manifest declares, loaded, in the order declared
    /// and each once. A car that declares none is refused: it does not run.
    /// </summary>
    public static IReadOnlyList<TireCompound> TiresFor(CarManifest manifest, ResourceId id)
    {
        if (manifest.Tyres.Count == 0)
            throw new InvalidDataException($"Car '{id}' names no tyres; car.json's \"tyres\" lists the packs or compounds it runs.");
        var compounds = new List<TireCompound>();
        foreach (string declared in manifest.Tyres)
            foreach (TireCompound compound in TireLibrary.Resolve(declared))
                if (!compounds.Contains(compound))
                    compounds.Add(compound);
        return compounds;
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
        IReadOnlyList<TireCompound> compounds = TiresFor(manifest, id);

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
        ICar car = definition.Build(compounds) ?? throw new InvalidDataException($"Car '{id}' built nothing.");
        if (car is ICarTires tires
                ? tires.Compounds.Count == 0 || tires.Compounds.Any(c => !compounds.Contains(c))
                : compounds.Count > 0)
            throw new InvalidDataException($"Car '{id}' must run the compounds it declares and only those.");
        if (car.Ladders.Count > CarStrategy.MaximumLadders)
            throw new InvalidDataException($"Car '{id}' declares {car.Ladders.Count} ladders; at most {CarStrategy.MaximumLadders}.");

        // The code and every tyre pack it runs, in the order declared.
        var hashed = new StringBuilder(Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(path))));
        foreach (string pack in manifest.Tyres.Select(t => t.Split(TireLibrary.CompoundSeparator)[0]).Distinct())
            hashed.Append(':').Append(TireLibrary.Load(pack).Fingerprint);
        string fingerprint = Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(hashed.ToString())), 0, 6).ToLowerInvariant();
        return new LoadedCar(id, manifest, folder, car, fingerprint);
    }
}
