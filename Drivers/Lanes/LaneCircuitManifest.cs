using System.Text.Json;
using System.Text.Json.Serialization;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// <c>lanes.json</c>, the front page of what the driver knows about one car
/// on one circuit: which file holds the learned lanes, and the numbers that
/// are one per circuit rather than one per station. Which car they were
/// learned on is the notes' own stamp (<see cref="NotesStamp"/>).
///
/// It lives in the car's pack, with the driver's notes for the circuit
/// (<see cref="DriverNotes"/>), not with the circuit: every factor in it is
/// what this car's physics got wrong about a corner, and another car on the
/// same road needs notes of its own.
/// </summary>
public sealed record LaneCircuitManifest
{
    public const int CurrentFormat = 1;
    public const string FileName = "lanes.json";

    public int Format { get; init; } = CurrentFormat;
    public string Circuit { get; init; } = "";

    /// <summary>The per-station table, relative to the folder.</summary>
    public string Lanes { get; init; } = "lanes.csv";

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

    public static LaneCircuitManifest Parse(string json) =>
        JsonSerializer.Deserialize<LaneCircuitManifest>(json, Json)
        ?? throw new InvalidDataException($"{FileName} is empty.");

    public string ToJson() => JsonSerializer.Serialize(this, Json) + "\n";
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

    /// <summary>The car calibrated on (<see cref="StintegyEVO.Core.Mods.ModCar.Fingerprint"/>).</summary>
    public string Physics { get; init; } = "";
    public string Energy { get; init; } = "";
    public string Tyres { get; init; } = "";
}
