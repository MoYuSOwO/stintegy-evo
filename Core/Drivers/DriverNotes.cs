using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.Json;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Mods;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// What a <c>notes.json</c> says: which driver pack learned the notes beside
/// it, on which car, and when. The notes themselves are the driver's own,
/// in whatever form it keeps them; this is all the game reads of them.
/// </summary>
public sealed record NotesStamp
{
    public const int CurrentFormat = 1;

    public int Format { get; init; } = CurrentFormat;
    public string Driver { get; init; } = "";
    public string Car { get; init; } = "";

    /// <summary>The car's fingerprint when the notes were learned (<see cref="ModCar.Fingerprint"/>).</summary>
    public string CarFingerprint { get; init; } = "";

    /// <summary>The day they were learned, <c>yyyy-MM-dd</c>.</summary>
    public string Learned { get; init; } = "";

    private static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        WriteIndented = true
    };

    public static NotesStamp Parse(string json) =>
        JsonSerializer.Deserialize<NotesStamp>(json, Json)
        ?? throw new InvalidDataException($"{DriverNotes.StampFile} is empty.");

    public string ToJson() => JsonSerializer.Serialize(this, Json).Replace("\r\n", "\n");
}

/// <summary>
/// What a driver pack has learned about one car at one circuit. Notes are
/// about a car, so they are kept in the car's pack, under the driver and
/// the circuit by their ids:
/// <c>&lt;car&gt;/drivers/&lt;driver namespace&gt;/&lt;driver name&gt;/&lt;circuit namespace&gt;/&lt;circuit name&gt;/</c>.
/// Whoever makes a car can ship what a driver learned on it; whoever makes
/// a driver learns in the car's pack.
/// </summary>
public sealed class DriverNotes
{
    public const string StampFile = "notes.json";
    public const string NotesFolder = "drivers";

    private DriverNotes(string folder, string carFingerprint, NotesStamp? stamp)
    {
        Folder = folder;
        CarFingerprint = carFingerprint;
        Stamp = stamp;
    }

    /// <summary>Where the notes are, or would be.</summary>
    public string Folder { get; }

    /// <summary>The car as it is now (<see cref="ModCar.Fingerprint"/>).</summary>
    public string CarFingerprint { get; }

    /// <summary>What the notes say of themselves; none when there are none.</summary>
    public NotesStamp? Stamp { get; }

    /// <summary>Whether there are notes, and they were learned on the car as it is now.</summary>
    public bool IsCurrent => Stamp is { } stamp && stamp.CarFingerprint == CarFingerprint;

    /// <summary>The notes <paramref name="driver"/> keeps for <paramref name="car"/> at <paramref name="circuit"/>.</summary>
    public static DriverNotes For(ResourceId driver, ModCar car, ResourceId circuit)
    {
        // Kept in the car's folder while its mod is a folder: a packed mod
        // is read only, and its notes are none until the save keeps them.
        if (car.DiskFolder is not { } carFolder)
            return new DriverNotes("", car.Fingerprint, null);
        string folder = Path.Combine(carFolder, NotesFolder, driver.Namespace, driver.Name, circuit.Namespace, circuit.Name);
        string stampPath = Path.Combine(folder, StampFile);
        NotesStamp? stamp = File.Exists(stampPath) ? NotesStamp.Parse(File.ReadAllText(stampPath)) : null;
        if (stamp is not null && stamp.Format > NotesStamp.CurrentFormat)
            throw new InvalidDataException($"{stampPath} is format {stamp.Format}; this game reads up to {NotesStamp.CurrentFormat}.");
        return new DriverNotes(folder, car.Fingerprint, stamp);
    }

    /// <summary>One of the driver's own files in the notes, or none when it is not there.</summary>
    public string? ReadText(string file)
    {
        if (Folder.Length == 0)
            return null;
        string path = PathOf(file);
        return File.Exists(path) ? File.ReadAllText(path) : null;
    }

    /// <summary>Writes one of the driver's own files into the notes.</summary>
    public void WriteText(string file, string text)
    {
        string path = PathOf(file);
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path, text);
    }

    /// <summary>
    /// Stamps the notes as learned today by <paramref name="driver"/> on
    /// <paramref name="car"/> as it is now.
    /// </summary>
    public void StampLearned(ResourceId driver, ModCar car, DateTime today) =>
        WriteText(StampFile, new NotesStamp
        {
            Driver = driver.ToString(),
            Car = car.Id.ToString(),
            CarFingerprint = car.Fingerprint,
            Learned = today.ToString("yyyy-MM-dd", CultureInfo.InvariantCulture)
        }.ToJson());

    // A file the driver names is inside its notes and nowhere else.
    private string PathOf(string file)
    {
        if (Folder.Length == 0)
            throw new InvalidOperationException("The car's mod is packed: notes on it cannot be kept in it.");
        if (string.IsNullOrWhiteSpace(file) || Path.IsPathRooted(file) ||
            file.Replace('\\', '/').Split('/').Any(part => part is "" or "." or ".."))
            throw new ArgumentException($"'{file}' is not a file in the notes.", nameof(file));
        return Path.Combine(Folder, file);
    }
}
