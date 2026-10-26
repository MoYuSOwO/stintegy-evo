using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Loader;
using System.Text.Json;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// What a driver pack's assembly offers: a way to seat its driver in a car.
/// One per pack. The driver is a pack like a car is, found the same way and
/// loaded the same way, and the game knows it only through this.
/// </summary>
public interface IDriverDefinition
{
    /// <summary>
    /// What the driver needs of a car, known before it is put in one: a car
    /// without a required feature is refused before the race, not found out
    /// during it.
    /// </summary>
    CarNeeds Needs { get; }

    /// <summary>The driver's controller for one race, seated as <paramref name="seat"/> says.</summary>
    IDriverController Seat(DriverSeat seat);
}

/// <summary>
/// Everything a driver is seated with: who is driving, in what, where, what
/// has been learned about that car there, and where this person's own
/// experience of it is kept.
/// </summary>
/// <param name="Person">The person driving: their name and ratings.</param>
/// <param name="Driver">The driver pack doing the driving.</param>
/// <param name="Car">The car, as its pack loaded it.</param>
/// <param name="Circuit">The circuit's id.</param>
/// <param name="Track">The circuit's road.</param>
/// <param name="Notes">What has been learned about this car at this circuit, by this driver pack.</param>
/// <param name="Progress">This person's own experience of the car there, if the game keeps it.</param>
public sealed record DriverSeat(
    DriverProfile Person,
    ResourceId Driver,
    LoadedCar Car,
    ResourceId Circuit,
    TrackData Track,
    DriverNotes Notes,
    DriverProgress? Progress);

/// <summary>A driver pack's <c>driver.json</c>: what it is called and where its code is.</summary>
public sealed record DriverManifest
{
    public const string FileName = "driver.json";
    public const int CurrentFormat = 1;

    public int Format { get; init; } = CurrentFormat;

    /// <summary>The driver's name within its namespace; the folder is named the same.</summary>
    public string Id { get; init; } = "";

    public string Name { get; init; } = "";

    /// <summary>The pack's assembly, relative to its folder.</summary>
    public string Assembly { get; init; } = "";

    /// <summary>The <see cref="IDriverDefinition"/> to use, when the assembly holds more than one.</summary>
    public string? Definition { get; init; }

    private static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        WriteIndented = true
    };

    public static DriverManifest Parse(string json) =>
        JsonSerializer.Deserialize<DriverManifest>(json, Json)
        ?? throw new InvalidDataException("driver.json is empty.");

    public string ToJson() => JsonSerializer.Serialize(this, Json).Replace("\r\n", "\n");
}

/// <summary>A driver pack, loaded: what it is, where it came from, and its definition.</summary>
public sealed record LoadedDriver(ResourceId Id, DriverManifest Manifest, string Folder, IDriverDefinition Definition);

/// <summary>
/// What stands between a driver and a car at a circuit, asked before the
/// race: what refuses the seat, and what only the pit wall should be told.
/// </summary>
/// <param name="Refusals">Why the driver cannot drive the car at all; none when it can.</param>
/// <param name="Notices">What the driver can drive through, but worse: no notes for this car there, or notes from another car.</param>
public sealed record DriverEntryCheck(IReadOnlyList<string> Refusals, IReadOnlyList<string> Notices)
{
    public bool CanRace => Refusals.Count == 0;
}

/// <summary>
/// Finds driver packs under <c>data/&lt;namespace&gt;/drivers/&lt;name&gt;/</c>
/// and loads them, the way <see cref="CarLibrary"/> loads cars, and seats a
/// person in a car with one.
/// </summary>
public static class DriverLibrary
{
    public const string DriversFolder = "drivers";

    private static readonly Dictionary<ResourceId, LoadedDriver> Loaded = new();
    private static readonly object Gate = new();

    /// <summary>The driver pack named by <paramref name="id"/>; a bare name is the game's own.</summary>
    public static LoadedDriver Load(string id) => Load(ResourceId.Parse(id));

    public static LoadedDriver Load(ResourceId id)
    {
        lock (Gate)
        {
            if (Loaded.TryGetValue(id, out LoadedDriver? loaded))
                return loaded;
            string folder = DataFolders.Find(id, DriversFolder, DriverManifest.FileName)
                ?? throw new FileNotFoundException($"No driver '{id}' under {string.Join(", ", DataFolders.Roots)}.");
            loaded = LoadFrom(folder, id);
            Loaded[id] = loaded;
            return loaded;
        }
    }

    /// <summary>Every driver pack the roots hold, by id, unloaded.</summary>
    public static IEnumerable<ResourceId> Available() => DataFolders.Available(DriversFolder, DriverManifest.FileName);

    /// <summary>
    /// Whether <paramref name="driver"/> can drive <paramref name="car"/> at
    /// <paramref name="circuit"/>, and what the pit wall should know if it
    /// can: refused when the car lacks what the driver needs; told when
    /// nothing has been learned about this car there, or what has was
    /// learned on another car, since the driver still drives, but worse.
    /// </summary>
    public static DriverEntryCheck Check(LoadedDriver driver, LoadedCar car, ResourceId circuit)
    {
        var refusals = new List<string>();
        var notices = new List<string>();
        IReadOnlyList<Type> missing = driver.Definition.Needs.MissingFrom(car.Car);
        if (missing.Count > 0)
            refusals.Add($"{driver.Id} cannot drive {car.Id}: it has no {string.Join(", ", missing.Select(t => t.Name))}.");
        DriverNotes notes = DriverNotes.For(driver.Id, car, circuit);
        if (notes.Stamp is not { } stamp)
            notices.Add($"{driver.Id} has no notes for {car.Id} at {circuit}: it will drive without, and slower.");
        else if (!notes.IsCurrent)
            notices.Add($"{driver.Id}'s notes for {car.Id} at {circuit} were learned on another car ({stamp.CarFingerprint}, now {car.Fingerprint}): it will drive on them, and slower.");
        return new DriverEntryCheck(refusals, notices);
    }

    /// <summary>
    /// <paramref name="person"/> in <paramref name="car"/> at
    /// <paramref name="circuit"/>, driven by <paramref name="driver"/>, with
    /// what has been learned there and, when the game keeps it, this
    /// person's own experience. Refused, as <see cref="CarUnfitForDriverException"/>,
    /// when the car lacks what the driver needs.
    /// </summary>
    public static Driver Seat(
        LoadedDriver driver,
        DriverProfile person,
        LoadedCar car,
        string circuit,
        IDriverProgressStore? progress = null)
    {
        ResourceId circuitId = ResourceId.Parse(circuit);
        IReadOnlyList<Type> missing = driver.Definition.Needs.MissingFrom(car.Car);
        if (missing.Count > 0)
            throw new CarUnfitForDriverException(person.Id, car.Car, missing);
        var seat = new DriverSeat(
            person,
            driver.Id,
            car,
            circuitId,
            CircuitLibrary.Load(circuitId.ToString()),
            DriverNotes.For(driver.Id, car, circuitId),
            progress is null ? null : new DriverProgress(progress, new DriverProgressKey(person.Id, driver.Id, car.Id, circuitId)));
        IDriverController controller = driver.Definition.Seat(seat)
            ?? throw new InvalidDataException($"Driver '{driver.Id}' seated nobody.");
        return new Driver(person, controller);
    }

    internal static LoadedDriver LoadFrom(string folder, ResourceId id)
    {
        DriverManifest manifest = DriverManifest.Parse(File.ReadAllText(Path.Combine(folder, DriverManifest.FileName)));
        if (manifest.Format > DriverManifest.CurrentFormat)
            throw new InvalidDataException($"Driver '{id}' is format {manifest.Format}; this game reads up to {DriverManifest.CurrentFormat}.");
        if (manifest.Id != id.Name)
            throw new InvalidDataException($"Driver '{id}' calls itself '{manifest.Id}'; a pack's id is its folder's name.");
        if (string.IsNullOrWhiteSpace(manifest.Assembly))
            throw new InvalidDataException($"Driver '{id}' names no assembly.");

        string path = Path.GetFullPath(Path.Combine(folder, manifest.Assembly));
        if (!File.Exists(path))
            throw new FileNotFoundException($"Driver '{id}' has no {manifest.Assembly} in {folder}; has the pack been built?", path);

        // Into the context Core itself lives in, as a car pack is, so the
        // pack and the game share one Core.
        AssemblyLoadContext context = AssemblyLoadContext.GetLoadContext(typeof(ICar).Assembly) ?? AssemblyLoadContext.Default;
        Assembly assembly = context.LoadFromAssemblyPath(path);

        Type[] definitions = assembly.GetTypes()
            .Where(t => typeof(IDriverDefinition).IsAssignableFrom(t) && t is { IsAbstract: false, IsInterface: false })
            .Where(t => manifest.Definition is null || t.FullName == manifest.Definition || t.Name == manifest.Definition)
            .ToArray();
        if (definitions.Length != 1)
            throw new InvalidDataException(
                $"Driver '{id}' needs exactly one driver definition in {manifest.Assembly}" +
                (manifest.Definition is null ? "" : $" named '{manifest.Definition}'") + $"; found {definitions.Length}.");

        var definition = (IDriverDefinition)(Activator.CreateInstance(definitions[0])
            ?? throw new InvalidDataException($"Driver '{id}': {definitions[0].Name} cannot be made."));
        return new LoadedDriver(id, manifest, folder, definition);
    }
}
