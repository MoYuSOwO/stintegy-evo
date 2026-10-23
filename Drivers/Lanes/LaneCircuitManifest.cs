using System.Collections;
using System.Globalization;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using StintegyEVO.Core.Cars;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// <c>driver.json</c>, the front page of what the driver knows about one car
/// on one circuit: which files hold the learned lanes, what the car was when
/// they were learned, and the numbers that are one per circuit rather than
/// one per station.
///
/// It lives under the car, <c>Cars/{car}/{circuit}/</c>, not with the
/// circuit: every factor in it is what this car's physics got wrong about a
/// corner, and another car on the same road needs notes of its own. A new
/// car adds a folder of circuits; a new circuit adds a folder under each car
/// that has driven it.
/// </summary>
public sealed record LaneCircuitManifest
{
    public const int CurrentFormat = 1;
    public const string FileName = "driver.json";

    public int Format { get; init; } = CurrentFormat;
    public string Circuit { get; init; } = "";

    /// <summary>The per-station table, relative to the folder.</summary>
    public string Lanes { get; init; } = "lanes.csv";

    public LaneLearnedFor? LearnedFor { get; init; }

    /// <summary>Absent: the driver's general table, which is not set for this circuit.</summary>
    public LaneEnergyRungs? Energy { get; init; }

    /// <summary>Absent: the driver's general tyre styles, which are not set for this circuit.</summary>
    public LaneTyreRungs? Tyres { get; init; }

    /// <summary>When and on what car the rungs were last calibrated, and how close they came.</summary>
    public LaneCalibration? Calibration { get; init; }

    private static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        WriteIndented = true
    };

    public static LaneCircuitManifest Read(string path) =>
        JsonSerializer.Deserialize<LaneCircuitManifest>(File.ReadAllText(path), Json)
        ?? throw new InvalidDataException($"{path} is empty.");

    public void Write(string path) => File.WriteAllText(path, JsonSerializer.Serialize(this, Json) + "\n");
}

/// <summary>
/// What the car was when the lanes were learned. The factors are what the
/// physics got wrong about each corner, so they belong to that physics: a
/// change to the car makes them a different car's notes, and the file says
/// so instead of being quietly driven on.
/// </summary>
public sealed record LaneLearnedFor
{
    /// <summary><see cref="PhysicsFingerprint.Stock"/> on the day.</summary>
    public string Physics { get; init; } = "";
    public string Date { get; init; } = "";
}

/// <summary>
/// The energy rungs on this circuit, Save to Attack: seconds of lift before
/// each braking zone. How much a second of lift saves depends on how many
/// braking zones a lap has and how fast the car arrives at them, so a table
/// set on one circuit spends the pack at the wrong rate on another.
/// </summary>
public sealed record LaneEnergyRungs
{
    public float Save { get; init; }
    public float Eco { get; init; }
    public float Normal { get; init; }
    public float Push { get; init; }
    public float Attack { get; init; }

    /// <summary>The race length the table was set for: Normal spends the pack over this many laps.</summary>
    public int RaceLaps { get; init; }

    public float[] LiftSeconds() => [Save, Eco, Normal, Push, Attack];

    public static LaneEnergyRungs From(float[] lift, int raceLaps) => new()
    {
        Save = lift[0], Eco = lift[1], Normal = lift[2], Push = lift[3], Attack = lift[4], RaceLaps = raceLaps
    };
}

/// <summary>
/// The tyre rungs on this circuit, Protect to Attack: how much of the
/// corner, the brakes, the traction and the grip cap each one spends. The
/// same share of a corner costs different rubber on different circuits, so
/// the gaps between the rungs are set per circuit like the energy ones.
/// </summary>
public sealed record LaneTyreRungs
{
    public TyreStyle Protect { get; init; }
    public TyreStyle Light { get; init; }
    public TyreStyle Normal { get; init; }
    public TyreStyle Push { get; init; }
    public TyreStyle Attack { get; init; }

    public TyreStyle[] Table() => [Protect, Light, Normal, Push, Attack];

    public static LaneTyreRungs From(TyreStyle[] table) => new()
    {
        Protect = table[0], Light = table[1], Normal = table[2], Push = table[3], Attack = table[4]
    };
}

/// <summary>What the last calibration measured, in words a person reads.</summary>
public sealed record LaneCalibration
{
    public string Date { get; init; } = "";
    public string Physics { get; init; } = "";
    public string Energy { get; init; } = "";
    public string Tyres { get; init; } = "";
}

/// <summary>
/// A short hash of every number the car and its tyres are made of: the
/// configs' properties, followed into what they hold, and the constants the
/// physics declares. It changes when the car does. It cannot see a change to
/// the physics code that leaves every number alone, so a fix of that kind
/// still means relearning by hand.
/// </summary>
public static class PhysicsFingerprint
{
    /// <summary>
    /// The stock car on the stock tyre: what the lanes are learned on. A
    /// car started with warm tyres or a part-filled pack is the same car.
    /// </summary>
    public static string Stock { get; } = Of(new CarConfig(), new TireConfig());

    public static string Of(CarConfig car, TireConfig tires)
    {
        var text = new StringBuilder();
        Write(text, "car", car, 0);
        Write(text, "tires", tires, 0);
        WriteConstants(text, "physics", typeof(CarPhysics));
        byte[] hash = SHA256.HashData(Encoding.UTF8.GetBytes(text.ToString()));
        return Convert.ToHexString(hash, 0, 6).ToLowerInvariant();
    }

    private static void Write(StringBuilder text, string path, object? value, int depth)
    {
        if (value is null)
        {
            text.Append(path).Append("=null\n");
            return;
        }
        if (value is float f)
        {
            text.Append(path).Append('=').Append(f.ToString("R", CultureInfo.InvariantCulture)).Append('\n');
            return;
        }
        if (value is double d)
        {
            text.Append(path).Append('=').Append(d.ToString("R", CultureInfo.InvariantCulture)).Append('\n');
            return;
        }
        Type type = value.GetType();
        if (type.IsPrimitive || type.IsEnum || value is string or decimal)
        {
            text.Append(path).Append('=').Append(Convert.ToString(value, CultureInfo.InvariantCulture)).Append('\n');
            return;
        }
        if (value is Delegate || depth > 8)
            return;
        if (value is IEnumerable items)
        {
            int i = 0;
            foreach (object? item in items)
                Write(text, $"{path}[{i++}]", item, depth + 1);
            return;
        }

        text.Append(path).Append(':').Append(type.Name).Append('\n');
        WriteConstants(text, path, type);
        foreach (PropertyInfo p in type.GetProperties(BindingFlags.Public | BindingFlags.Instance)
                     .Where(p => p.CanRead && p.GetIndexParameters().Length == 0)
                     .OrderBy(p => p.Name, StringComparer.Ordinal))
        {
            object? child;
            try
            {
                child = p.GetValue(value);
            }
            catch (TargetInvocationException)
            {
                continue;
            }
            if (ReferenceEquals(child, value))
                continue;
            Write(text, $"{path}.{p.Name}", child, depth + 1);
        }
    }

    private static void WriteConstants(StringBuilder text, string path, Type type)
    {
        foreach (FieldInfo field in type.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static)
                     .Where(f => f.IsLiteral)
                     .OrderBy(f => f.Name, StringComparer.Ordinal))
            Write(text, $"{path}::{field.Name}", field.GetRawConstantValue(), 0);
    }
}
