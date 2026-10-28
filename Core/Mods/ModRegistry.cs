using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Mods;

/// <summary>
/// What a mod's code offers: its entrypoint, named in its <c>mod.json</c>.
/// The game makes one with its constructor that takes nothing, calls
/// <see cref="Register"/> once, and keeps it for the game's life. A mod
/// registers standard parts and is done: it listens to nothing and the
/// game asks for what it registered, by id, when it needs it.
/// </summary>
public interface IMod
{
    void Register(ModContext context);
}

/// <summary>
/// What a mod is handed to register with: the registry, as far as this mod
/// may write to it, its own files, its <c>mod.json</c> and a log. One
/// object, so that what it carries can grow without breaking any mod.
/// </summary>
public sealed class ModContext
{
    internal ModContext(LoadedMod mod, ModRegistrar registry, ModLog log)
    {
        Manifest = mod.Manifest;
        Resources = mod.Resources;
        Registry = registry;
        Log = log;
    }

    public ModManifest Manifest { get; }
    public IModResources Resources { get; }
    public ModRegistrar Registry { get; }
    public ModLog Log { get; }
}

/// <summary>What a mod wants the player told while it loads.</summary>
public sealed class ModLog
{
    private readonly string _mod;
    private readonly ICollection<string> _notices;

    internal ModLog(string mod, ICollection<string> notices)
    {
        _mod = mod;
        _notices = notices;
    }

    public void Info(string message) => _notices.Add($"{_mod}: {message}");
}

/// <summary>Why a registration was refused: the id is not the mod's to use, or is taken, or the registry is closed.</summary>
public sealed class ModRegistrationException : InvalidOperationException
{
    public ModRegistrationException(string message) : base(message)
    {
    }
}

/// <summary>A mod, loaded: what it is, its files, its code and the entrypoint the game keeps.</summary>
public sealed class LoadedMod
{
    internal LoadedMod(FoundMod found) => Found = found;

    internal FoundMod Found { get; }
    public string Id => Found.Id;
    public ModManifest Manifest => Found.Manifest;
    public ModVersion Version => Found.Version;
    public ModOrigin Origin => Found.Origin;
    public IModResources Resources => Found.Resources;
    public Assembly? Assembly { get; internal set; }
    public IMod? Entrypoint { get; internal set; }
}

/// <summary>
/// One kind of thing in the registry, by id. Anyone reads it; only the
/// mod whose namespace an id is in puts it there, once, before the
/// registry is frozen.
/// </summary>
public sealed class RegistrySlot<T> where T : class
{
    private readonly Dictionary<ResourceId, (T Value, string Mod)> _entries = new();
    private readonly ModRegistry _registry;

    internal RegistrySlot(ModRegistry registry, string kind)
    {
        _registry = registry;
        Kind = kind;
    }

    /// <summary>What the slot holds, in words: "circuit", "car".</summary>
    public string Kind { get; }

    public int Count => _entries.Count;
    public IEnumerable<ResourceId> Ids => _entries.Keys;

    public bool Contains(ResourceId id) => _entries.ContainsKey(id);

    public bool TryGet(ResourceId id, out T value)
    {
        bool found = _entries.TryGetValue(id, out var entry);
        value = entry.Value;
        return found;
    }

    /// <summary>The <see cref="Kind"/> named <paramref name="id"/>; a bare name is the game's own.</summary>
    public T Get(string id) => Get(ResourceId.Parse(id));

    public T Get(ResourceId id) =>
        _entries.TryGetValue(id, out var entry) ? entry.Value
        : throw new KeyNotFoundException($"No {Kind} '{id}' is registered.");

    /// <summary>The mod that registered <paramref name="id"/>.</summary>
    public string Owner(ResourceId id) => _entries.TryGetValue(id, out var entry) ? entry.Mod
        : throw new KeyNotFoundException($"No {Kind} '{id}' is registered.");

    internal void Add(string mod, ResourceId id, T value)
    {
        ArgumentNullException.ThrowIfNull(value);
        if (_registry.IsFrozen)
            throw new ModRegistrationException($"{mod}: the registry is frozen; {Kind} '{id}' came too late.");
        if (id.Namespace != mod)
            throw new ModRegistrationException($"{mod}: {Kind} '{id}' is in {id.Namespace}'s namespace; a mod registers only in its own.");
        if (_entries.TryGetValue(id, out var taken))
            throw new ModRegistrationException($"{mod}: {Kind} '{id}' is already registered by {taken.Mod}; an id is registered once.");
        _entries[id] = (value, mod);
    }
}

/// <summary>Where one mod puts one kind of thing: in its own namespace and nowhere else.</summary>
public sealed class ModSlot<T> where T : class
{
    private readonly RegistrySlot<T> _slot;
    private readonly ModRegistrar _owner;

    internal ModSlot(RegistrySlot<T> slot, ModRegistrar owner)
    {
        _slot = slot;
        _owner = owner;
    }

    /// <summary>
    /// Registers <paramref name="value"/> as <c>&lt;this mod&gt;:<paramref name="name"/></c>.
    /// A full id is taken too, if it is in this mod's namespace.
    /// </summary>
    public ResourceId Register(string name, T value)
    {
        if (_owner.Closed)
            throw new ModRegistrationException($"{_owner.Mod}: registers only while its Register runs; {_slot.Kind} '{name}' came after.");
        ResourceId id;
        try
        {
            id = name.Contains(':') ? ResourceId.Parse(name) : new ResourceId(_owner.Mod, name);
        }
        catch (ArgumentException error)
        {
            throw new ModRegistrationException($"{_owner.Mod}: {_slot.Kind} '{name}' is not an id: {error.Message}");
        }
        _slot.Add(_owner.Mod, id, value);
        return id;
    }
}

/// <summary>The registry as one mod's code may write to it: the kinds that need code.</summary>
public sealed class ModRegistrar
{
    internal ModRegistrar(ModRegistry registry, string mod)
    {
        Mod = mod;
        Chassis = new ModSlot<ChassisDeclaration>(registry.Chassis, this);
        Cars = new ModSlot<ICarDefinition>(registry.CarDefinitions, this);
        Drivers = new ModSlot<IDriverDefinition>(registry.Drivers, this);
    }

    /// <summary>The mod writing, and the namespace it writes in.</summary>
    public string Mod { get; }

    public ModSlot<ChassisDeclaration> Chassis { get; }

    /// <summary>
    /// The code that builds each car the mod's <c>mod.json</c> lists, under
    /// the name its <c>car.json</c> gives: the two are one car.
    /// </summary>
    public ModSlot<ICarDefinition> Cars { get; }

    public ModSlot<IDriverDefinition> Drivers { get; }

    internal bool Closed { get; set; }
}

/// <summary>A circuit a mod lists: its <c>circuit.json</c> read, its road read when first asked for.</summary>
public sealed class ModCircuit
{
    private readonly Lazy<TrackData> _track;

    internal ModCircuit(ResourceId id, CircuitManifest manifest, string folder, IModResources resources)
    {
        Id = id;
        Manifest = manifest;
        Folder = folder;
        Resources = resources;
        _track = new Lazy<TrackData>(() =>
            CircuitTrackFile.Parse(manifest, resources.ReadAllText(ModResources.Join(folder, manifest.Track))));
    }

    public ResourceId Id { get; }
    public CircuitManifest Manifest { get; }

    /// <summary>Its folder in the mod, for whatever else it carries beside the road.</summary>
    public string Folder { get; }

    public IModResources Resources { get; }

    /// <summary>The road, read from its track file the first time it is asked for.</summary>
    public TrackData Track => _track.Value;

    /// <summary>Where its folder is on disk, when its mod is a folder and not packed.</summary>
    public string? DiskFolder => Directory.Exists(Resources.Location)
        ? Path.Combine(Resources.Location, Folder.Replace('/', Path.DirectorySeparatorChar))
        : null;
}

/// <summary>
/// Scenery a mod adds to a circuit, which may be another mod's: laid on top
/// of what the circuit has, never in place of it.
/// </summary>
public sealed record ModSceneryOverlay(ResourceId Circuit, string Mod, string File, IModResources Resources);

/// <summary>A prop scenery places, by id: the model it is drawn with, in the mod that registered it.</summary>
public sealed record ModProp(ResourceId Id, string Model, float Scale, IModResources Resources);

/// <summary>
/// Everything every mod registered, by kind and id, and the mods
/// themselves. Built by <see cref="ModLoader"/>, then frozen: read only
/// while the game runs.
/// </summary>
public sealed class ModRegistry
{
    private readonly Dictionary<string, LoadedMod> _mods = new(StringComparer.Ordinal);
    private readonly List<LoadedMod> _order = [];
    private readonly List<string> _notices = [];
    private readonly List<ModSceneryOverlay> _scenery = [];

    internal ModRegistry()
    {
        Circuits = new RegistrySlot<ModCircuit>(this, "circuit");
        Tyres = new RegistrySlot<LoadedTires>(this, "tyre pack");
        Props = new RegistrySlot<ModProp>(this, "prop");
        Chassis = new RegistrySlot<ChassisDeclaration>(this, "chassis");
        CarDeclarations = new RegistrySlot<CarDeclaration>(this, "car");
        CarDefinitions = new RegistrySlot<ICarDefinition>(this, "car's code");
        Cars = new RegistrySlot<ModCar>(this, "car");
        Drivers = new RegistrySlot<IDriverDefinition>(this, "driver");
    }

    /// <summary>The mods loaded, each after those it depends on.</summary>
    public IReadOnlyList<LoadedMod> Mods => _order;

    public RegistrySlot<ModCircuit> Circuits { get; }
    public RegistrySlot<LoadedTires> Tyres { get; }
    public RegistrySlot<ModProp> Props { get; }
    public RegistrySlot<ChassisDeclaration> Chassis { get; }

    /// <summary>Every car, declared, built and checked.</summary>
    public RegistrySlot<ModCar> Cars { get; }

    public RegistrySlot<IDriverDefinition> Drivers { get; }

    internal RegistrySlot<CarDeclaration> CarDeclarations { get; }
    internal RegistrySlot<ICarDefinition> CarDefinitions { get; }

    /// <summary>
    /// The car <paramref name="car"/> was built as, for whoever draws it or
    /// files what was learned on it; none for a car made some other way, as
    /// a test makes one.
    /// </summary>
    public ModCar? Origin(ICar car) => Cars.Ids.Select(Cars.Get).FirstOrDefault(c => ReferenceEquals(c.Car, car));

    /// <summary>What the player should be told about loading: nothing that stopped it.</summary>
    public IReadOnlyList<string> Notices => _notices;

    public bool IsFrozen { get; private set; }

    /// <summary>The scenery mods add to <paramref name="circuit"/>, in the order the mods load.</summary>
    public IReadOnlyList<ModSceneryOverlay> SceneryFor(ResourceId circuit) =>
        _scenery.Where(s => s.Circuit == circuit).ToList();

    internal List<ModSceneryOverlay> Scenery => _scenery;

    public LoadedMod Mod(string id) =>
        _mods.TryGetValue(id, out LoadedMod? mod) ? mod : throw new KeyNotFoundException($"No mod '{id}' is loaded.");

    /// <summary>The file a mod carries, as <c>&lt;mod&gt;:&lt;path&gt;</c> (<c>official:models/formula_a.glb</c>).</summary>
    public Stream Open(string resource)
    {
        int at = resource.IndexOf(':');
        if (at <= 0)
            throw new ArgumentException($"'{resource}' does not say whose file it is: <mod>:<path>.", nameof(resource));
        return Mod(resource[..at]).Resources.Open(resource[(at + 1)..]);
    }

    internal List<string> NoticeLog => _notices;

    internal void Add(LoadedMod mod)
    {
        _mods[mod.Id] = mod;
        _order.Add(mod);
    }

    internal void Freeze() => IsFrozen = true;
}
