using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;

namespace StintegyEVO.Core.Mods;

/// <summary>
/// A chassis a mod's code registers: physics cars are built on, shared by
/// every car that names it. It is code a car builds against, never a car
/// itself; what it offers is whatever the mod's assembly makes public, and
/// what the registry keeps of it is what it is called and its version.
/// </summary>
/// <param name="Name">What it is called.</param>
/// <param name="Version">
/// Its author raises it whenever a change to the chassis changes how a car on
/// it drives: everything learned on any car on it was learned on a version,
/// and is stale on another (<see cref="ModCar.Fingerprint"/>).
/// </param>
public sealed record ChassisDeclaration(string Name, int Version);

/// <summary>A car's <c>car.json</c> as a mod lists it, waiting for the code that builds it.</summary>
internal sealed record CarDeclaration(ResourceId Id, CarManifest Manifest, string Folder, IModResources Resources);

/// <summary>
/// A car in the registry: what its <c>car.json</c> declares, joined to the
/// code its mod registered under the same id, built once and checked when
/// the mods load. Every race that runs this car runs this one
/// <see cref="Car"/>: it keeps nothing of a race in itself, only in each
/// car's <see cref="CarState"/>.
/// </summary>
public sealed class ModCar
{
    internal ModCar(CarDeclaration declaration, string mod, ICarDefinition definition,
        IReadOnlyList<TireCompound> compounds, ICar car, string fingerprint)
    {
        Id = declaration.Id;
        Manifest = declaration.Manifest;
        Folder = declaration.Folder;
        Resources = declaration.Resources;
        Mod = mod;
        Definition = definition;
        Compounds = compounds;
        Car = car;
        Fingerprint = fingerprint;
    }

    public ResourceId Id { get; }
    public CarManifest Manifest { get; }

    /// <summary>Its folder in the mod, for what it carries beside its declaration: its model.</summary>
    public string Folder { get; }

    public IModResources Resources { get; }

    /// <summary>The mod that registered it.</summary>
    public string Mod { get; }

    public ICarDefinition Definition { get; }

    /// <summary>The compounds it runs: those its declaration names, in that order.</summary>
    public IReadOnlyList<TireCompound> Compounds { get; }

    public ICar Car { get; }

    /// <summary>
    /// A short hash of what the car is: its version, the version of the
    /// chassis it is built on, what it says of itself and the tyres it runs.
    /// Anything learned on this car -- a driver's notes -- is learned on these,
    /// and is stale when any of them changes; building the same mod again
    /// does not change it.
    /// </summary>
    public string Fingerprint { get; }

    /// <summary>Its model as <c>&lt;mod&gt;:&lt;path&gt;</c>, for <see cref="ModRegistry.Open"/>; none to be drawn the game's way.</summary>
    public string? Model => Manifest.Model is { Length: > 0 } model ? $"{Mod}:{ModResources.Join(Folder, model)}" : null;

    /// <summary>
    /// Where its folder is on disk, when its mod is a folder and not packed:
    /// where what drivers learned on it is kept, for now (<c>DriverNotes</c>).
    /// </summary>
    public string? DiskFolder => Directory.Exists(Resources.Location)
        ? Path.Combine(Resources.Location, Folder.Replace('/', Path.DirectorySeparatorChar))
        : null;
}

/// <summary>Reading and building cars for the loader.</summary>
internal static class ModCarBuilder
{
    // A folder with a car.json: the declaration, read with the mod's data.
    public static void Read(ModRegistry registry, LoadedMod mod, string folder, Func<string, ResourceId> own)
    {
        string file = ModResources.Join(folder, CarManifest.FileName);
        CarManifest manifest = CarManifest.Parse(mod.Resources.ReadAllText(file));
        if (manifest.Format > CarManifest.CurrentFormat)
            throw new InvalidDataException($"is format {manifest.Format}; this game reads up to {CarManifest.CurrentFormat}.");
        ResourceId id = own(manifest.Id);
        if (manifest.Model is { Length: > 0 } model && !mod.Resources.Exists(ModResources.Join(folder, model)))
            throw new InvalidDataException($"car '{id}' is drawn with {model}, which the mod does not have.");
        registry.CarDeclarations.Add(mod.Id, id, new CarDeclaration(id, manifest, folder, mod.Resources));
    }

    /// <summary>
    /// Every declared car joined to its code and built, each checked the way
    /// a race would find it out: a declaration with no code, code with no
    /// declaration, a chassis or tyres not there or not depended on, a car
    /// that runs other tyres than it declares or declares itself wrongly.
    /// </summary>
    public static IEnumerable<string> Build(ModRegistry registry, Func<LoadedMod, HashSet<string>> dependsOn)
    {
        var problems = new List<string>();
        foreach (ResourceId id in registry.CarDefinitions.Ids.Where(id => !registry.CarDeclarations.Contains(id)).ToList())
            problems.Add($"{id.Namespace}: registers code for car '{id}', which its mod.json lists no car.json for.");
        foreach (ResourceId id in registry.CarDeclarations.Ids.ToList())
        {
            CarDeclaration declaration = registry.CarDeclarations.Get(id);
            string mod = registry.CarDeclarations.Owner(id);
            try
            {
                if (!registry.CarDefinitions.TryGet(id, out ICarDefinition definition))
                    throw new InvalidDataException("its mod registers no code to build it (Registry.Cars).");
                HashSet<string> allowed = dependsOn(registry.Mod(mod));
                allowed.Add(mod);
                int chassisVersion = Chassis(registry, declaration, allowed);
                IReadOnlyList<TireCompound> compounds = Tyres(registry, declaration, allowed);
                ICar car = definition.Build(compounds) ?? throw new InvalidDataException("its code built nothing.");
                if (car is ICarTires tires
                        ? tires.Compounds.Count == 0 || tires.Compounds.Any(c => !compounds.Contains(c))
                        : compounds.Count > 0)
                    throw new InvalidDataException("it must run the compounds it declares and only those.");
                if (car.Spec.Problems().ToArray() is { Length: > 0 } undeclared)
                    throw new InvalidDataException($"it declares itself wrongly: {string.Join("; ", undeclared)}.");
                if (car.Ladders.Count > CarStrategy.MaximumLadders)
                    throw new InvalidDataException($"it declares {car.Ladders.Count} ladders; at most {CarStrategy.MaximumLadders}.");
                string fingerprint = Fingerprint(registry, declaration, chassisVersion, car);
                registry.Cars.Add(mod, id, new ModCar(declaration, mod, definition, compounds, car, fingerprint));
            }
            catch (Exception error) when (error is not OutOfMemoryException)
            {
                problems.Add($"{mod}: car '{id}': {error.Message}");
            }
        }
        return problems;
    }

    private static int Chassis(ModRegistry registry, CarDeclaration car, HashSet<string> allowed)
    {
        if (car.Manifest.Chassis is not { Length: > 0 } named)
            return 0;
        ResourceId chassis = ResourceId.Parse(named);
        if (!registry.Chassis.TryGet(chassis, out ChassisDeclaration declaration))
            throw new InvalidDataException($"its chassis '{chassis}' is not registered.");
        if (!allowed.Contains(chassis.Namespace))
            throw new InvalidDataException($"it is built on {chassis.Namespace}'s chassis without depending on {chassis.Namespace}.");
        return declaration.Version;
    }

    /// <summary>
    /// What a declaration's tyres name: whole packs (<c>official:formula_a</c>)
    /// or single compounds of them (<c>official:formula_a/medium</c>), in
    /// order and each once. A car that names none does not run.
    /// </summary>
    private static IReadOnlyList<TireCompound> Tyres(ModRegistry registry, CarDeclaration car, HashSet<string> allowed)
    {
        if (car.Manifest.Tyres.Count == 0)
            throw new InvalidDataException("it names no tyres; car.json's \"tyres\" lists the packs or compounds it runs.");
        var compounds = new List<TireCompound>();
        foreach (string declared in car.Manifest.Tyres)
        {
            int at = declared.IndexOf(TirePack.CompoundSeparator);
            ResourceId pack = ResourceId.Parse(at < 0 ? declared : declared[..at]);
            if (!registry.Tyres.TryGet(pack, out LoadedTires tyres))
                throw new InvalidDataException($"its tyre pack '{pack}' is not registered.");
            if (!allowed.Contains(pack.Namespace))
                throw new InvalidDataException($"it runs {pack.Namespace}'s tyres without depending on {pack.Namespace}.");
            IEnumerable<TireCompound> named = at < 0 ? tyres.Compounds
                : tyres.Compounds.Where(c => c.Id == declared) is var one && one.Any() ? one
                : throw new InvalidDataException(
                    $"tyre pack '{pack}' has no compound '{declared[(at + 1)..]}'; it has {string.Join(", ", tyres.Compounds.Select(c => c.Id))}.");
            foreach (TireCompound compound in named)
                if (!compounds.Contains(compound))
                    compounds.Add(compound);
        }
        return compounds;
    }

    internal static string Fingerprint(ModRegistry registry, CarDeclaration car, int chassisVersion, ICar built)
    {
        // What the car is, not the bytes it was built into: the same mod
        // built again against a new game is the same car.
        var hashed = new StringBuilder()
            .Append(car.Id).Append(':').Append(car.Manifest.Version)
            .Append(':').Append(chassisVersion)
            .Append(':').Append(JsonSerializer.Serialize(built.Spec));
        foreach (ModeLadder ladder in built.Ladders)
            hashed.Append(':').Append(ladder.Id).Append('=').Append(string.Join(',', ladder.Rungs));
        foreach (string pack in car.Manifest.Tyres.Select(t => t.Split(TirePack.CompoundSeparator)[0]).Distinct())
            hashed.Append(':').Append(registry.Tyres.Get(pack).Fingerprint);
        return Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(hashed.ToString())), 0, 6).ToLowerInvariant();
    }
}
