using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.Json;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Mods;

/// <summary>
/// Loads every mod into one registry, in this order: find them, order
/// them by what they depend on, load their code, register the data their
/// <c>mod.json</c> lists, run each entrypoint's <see cref="IMod.Register"/>,
/// check what refers to what, freeze. Anything wrong stops the load with
/// every problem of that step named (<see cref="ModLoadException"/>), so it
/// is found when the game starts and never in a race.
/// </summary>
public static class ModLoader
{
    public static ModRegistry Load(IEnumerable<ModRoot> roots)
    {
        var registry = new ModRegistry();
        IReadOnlyList<FoundMod> found = ModDiscovery.Find(roots, registry.NoticeLog);
        IReadOnlyList<FoundMod> ordered = ModDiscovery.Order(found);
        var problems = new List<string>();

        foreach (FoundMod each in ordered)
        {
            var mod = new LoadedMod(each);
            registry.Add(mod);
            if (each.Manifest.Assembly is not { Length: > 0 } assembly)
                continue;
            try
            {
                mod.Assembly = ModAssemblies.Load(each, assembly);
            }
            catch (Exception error) when (error is IOException or InvalidDataException or BadImageFormatException or ArgumentException)
            {
                problems.Add($"{mod.Id}: {error.Message}");
            }
        }
        Stop(problems);

        foreach (LoadedMod mod in registry.Mods)
        {
            foreach (ModSceneryEntry scenery in mod.Manifest.Scenery)
            {
                try
                {
                    registry.Scenery.Add(ModDataKind.Overlay(mod, scenery));
                }
                catch (Exception error) when (error is InvalidDataException or ArgumentException)
                {
                    problems.Add($"{mod.Id}: scenery '{scenery.File}' for '{scenery.Circuit}': {error.Message}");
                }
            }
            foreach (ModDataKind kind in ModDataKind.All)
                foreach (string path in kind.Listed(mod.Manifest))
                {
                    try
                    {
                        kind.Read(registry, mod, ModResources.Normalize(path));
                    }
                    catch (ModRegistrationException error)
                    {
                        problems.Add(error.Message);
                    }
                    catch (Exception error) when (error is IOException or InvalidDataException or JsonException or ArgumentException)
                    {
                        problems.Add($"{mod.Id}: {kind.Field} '{path}': {error.Message}");
                    }
                }
        }
        Stop(problems);

        foreach (LoadedMod mod in registry.Mods)
            if (mod.Assembly is { } assembly && Entrypoint(mod, assembly, problems) is { } entrypoint)
            {
                var registrar = new ModRegistrar(registry, mod.Id);
                try
                {
                    entrypoint.Register(new ModContext(mod, registrar, new ModLog(mod.Id, registry.NoticeLog)));
                    mod.Entrypoint = entrypoint;
                }
                catch (ModRegistrationException error)
                {
                    problems.Add(error.Message);
                }
                catch (Exception error)
                {
                    problems.Add($"{mod.Id}: its Register failed: {error.GetType().Name}: {error.Message}");
                }
                finally
                {
                    registrar.Closed = true;
                }
            }
        Stop(problems);

        foreach (Func<ModRegistry, IEnumerable<string>> check in Checks)
            problems.AddRange(check(registry));
        Stop(problems);

        // Scenery for a circuit nobody installed is left out, and said so:
        // decoration missing its circuit stops nothing.
        foreach (ModSceneryOverlay lost in registry.Scenery.Where(s => !registry.Circuits.Contains(s.Circuit)).ToList())
        {
            registry.Scenery.Remove(lost);
            registry.NoticeLog.Add($"{lost.Mod}: scenery {lost.File} is for {lost.Circuit}, which is not installed; it is left out.");
        }

        registry.Freeze();
        return registry;
    }

    private static void Stop(List<string> problems)
    {
        if (problems.Count > 0)
            throw new ModLoadException(problems.ToArray());
    }

    private static IMod? Entrypoint(LoadedMod mod, Assembly assembly, List<string> problems)
    {
        string name = mod.Manifest.Entrypoint!;
        Type? type = assembly.GetType(name, throwOnError: false);
        if (type is null)
            problems.Add($"{mod.Id}: its entrypoint {name} is not in {mod.Manifest.Assembly}.");
        else if (!typeof(IMod).IsAssignableFrom(type) || type.IsAbstract || type.IsInterface)
            problems.Add($"{mod.Id}: its entrypoint {name} is not an {nameof(IMod)} the game can make.");
        else if (type.GetConstructor(Type.EmptyTypes) is null)
            problems.Add($"{mod.Id}: its entrypoint {name} has no public constructor that takes nothing.");
        else
        {
            try
            {
                return (IMod)Activator.CreateInstance(type)!;
            }
            catch (TargetInvocationException error)
            {
                problems.Add($"{mod.Id}: making its entrypoint {name} failed: {error.InnerException?.Message ?? error.Message}");
            }
        }
        return null;
    }

    /// <summary>What refers across mods, checked once everything is registered.</summary>
    private static readonly Func<ModRegistry, IEnumerable<string>>[] Checks =
    [
        CodeUsesOnlyWhatItDependsOn,
    ];

    // A mod's code may use another mod's only if it says it depends on that
    // mod: otherwise it works or not by which mods the player happens to have.
    private static IEnumerable<string> CodeUsesOnlyWhatItDependsOn(ModRegistry registry)
    {
        var owners = registry.Mods.Where(m => m.Assembly is not null)
            .ToDictionary(m => m.Assembly!.GetName().Name!, m => m.Id, StringComparer.Ordinal);
        foreach (LoadedMod mod in registry.Mods.Where(m => m.Assembly is not null))
        {
            HashSet<string> allowed = DependsOn(registry, mod);
            foreach (AssemblyName used in mod.Assembly!.GetReferencedAssemblies())
                if (used.Name is { } name && owners.TryGetValue(name, out string? owner) && owner != mod.Id && !allowed.Contains(owner))
                    yield return $"{mod.Id}: its code uses {owner}'s ({name}) without depending on {owner}.";
        }
    }

    private static HashSet<string> DependsOn(ModRegistry registry, LoadedMod mod)
    {
        var all = new HashSet<string>(StringComparer.Ordinal);
        var next = new Stack<string>(mod.Manifest.Depends.Keys);
        while (next.Count > 0)
            if (all.Add(next.Peek()))
                foreach (string deeper in registry.Mod(next.Pop()).Manifest.Depends.Keys)
                    next.Push(deeper);
            else
                next.Pop();
        return all;
    }
}

/// <summary>
/// A kind of data a mod lists in its <c>mod.json</c>, and the game's own
/// reader for it. Data is only ever read by these: a mod writes files, not
/// code, for it. A new kind is a new entry here; the loader does not change.
/// </summary>
internal sealed record ModDataKind(string Field, Func<ModManifest, IReadOnlyList<string>> Listed, Action<ModRegistry, LoadedMod, string> Read)
{
    public static readonly ModDataKind[] All =
    [
        new("circuits", m => m.Circuits, ReadCircuit),
        new("tyres", m => m.Tyres, ReadTyres),
        new("props", m => m.Props, ReadProps),
    ];

    // A folder with a circuit.json. The road itself is read when the
    // circuit is first used: a dozen circuits do not slow the start.
    private static void ReadCircuit(ModRegistry registry, LoadedMod mod, string folder)
    {
        IModResources files = mod.Resources;
        CircuitManifest manifest = CircuitManifest.Parse(files.ReadAllText(ModResources.Join(folder, CircuitManifest.FileName)));
        if (manifest.Format > CircuitManifest.CurrentFormat)
            throw new InvalidDataException($"is format {manifest.Format}; this game reads up to {CircuitManifest.CurrentFormat}.");
        ResourceId id = Own(mod, manifest.Id);
        if (manifest.LengthMeters <= 0)
            throw new InvalidDataException($"circuit '{id}' is {manifest.LengthMeters} m long.");
        if (manifest.StartLineMeters < 0 || manifest.StartLineMeters >= manifest.LengthMeters)
            throw new InvalidDataException($"circuit '{id}' starts at {manifest.StartLineMeters} m, outside its {manifest.LengthMeters} m lap.");
        if (!files.Exists(ModResources.Join(folder, manifest.Track)))
            throw new InvalidDataException($"circuit '{id}' has no track file {manifest.Track}.");
        if (manifest.Source.LicenseFile is { Length: > 0 } license && !files.Exists(ModResources.Join(folder, license)))
            throw new InvalidDataException($"circuit '{id}' names a licence file {license} it does not have.");
        registry.Circuits.Add(mod.Id, id, new ModCircuit(id, manifest, folder, files));
    }

    // A file in the form of a tyre pack's tyres.json, checked as one is.
    private static void ReadTyres(ModRegistry registry, LoadedMod mod, string file)
    {
        string text = mod.Resources.ReadAllText(file);
        ResourceId id = Own(mod, TirePackManifest.Parse(text).Id);
        registry.Tyres.Add(mod.Id, id, TireLibrary.Read(id, text, file));
    }

    private static void ReadProps(ModRegistry registry, LoadedMod mod, string file)
    {
        PropsFile props = JsonSerializer.Deserialize<PropsFile>(mod.Resources.ReadAllText(file), ModManifest.Json)
                          ?? throw new InvalidDataException("is empty.");
        foreach ((string name, PropEntry prop) in props.Props)
        {
            ResourceId id = Own(mod, name);
            if (string.IsNullOrWhiteSpace(prop.Model))
                throw new InvalidDataException($"prop '{id}' names no model.");
            string model = ModResources.Normalize(prop.Model);
            if (!mod.Resources.Exists(model))
                throw new InvalidDataException($"prop '{id}' is drawn with {model}, which the mod does not have.");
            if (!(prop.Scale > 0f) || !float.IsFinite(prop.Scale))
                throw new InvalidDataException($"prop '{id}' is scaled by {prop.Scale}; a scale is above zero.");
            registry.Props.Add(mod.Id, id, new ModProp(id, model, prop.Scale, mod.Resources));
        }
    }

    // Scenery is the one thing a mod may lay on another mod's circuit, so
    // its circuit is a full id in any namespace. What it holds is read when
    // the circuit is drawn, not here.
    public static ModSceneryOverlay Overlay(LoadedMod mod, ModSceneryEntry entry)
    {
        if (!entry.Circuit.Contains(':'))
            throw new InvalidDataException("names its circuit without its mod: <mod>:<circuit>.");
        ResourceId circuit = ResourceId.Parse(entry.Circuit);
        string file = ModResources.Normalize(entry.File);
        if (!mod.Resources.Exists(file))
            throw new InvalidDataException($"the mod has no {file}.");
        return new ModSceneryOverlay(circuit, mod.Id, file, mod.Resources);
    }

    // A data item's id is the mod's namespace and the name it gives itself.
    private static ResourceId Own(LoadedMod mod, string name) =>
        ResourceId.IsPart(name) ? new ResourceId(mod.Id, name)
        : throw new InvalidDataException($"'{name}' is not a name for it (lower case letters, digits and underscores).");

    private sealed record PropsFile
    {
        public IReadOnlyDictionary<string, PropEntry> Props { get; init; } = new Dictionary<string, PropEntry>();
    }

    private sealed record PropEntry
    {
        public string Model { get; init; } = "";
        public float Scale { get; init; } = 1f;
    }
}
