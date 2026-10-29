using System.Text.Json;
using System.Text.Json.Serialization;
using StintegyEVO.Core.Drivers;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// <c>lanes.json</c>, the front page of the driver's folder for one car on
/// one circuit: which file holds the baked lanes. Which car they were baked
/// for is the notes' own stamp (<see cref="NotesStamp"/>).
///
/// It lives in the car's pack, with the driver's folder for the circuit
/// (<see cref="DriverNotes"/>), not with the circuit.
/// </summary>
public sealed record LaneCircuitManifest
{
    public const int CurrentFormat = 1;
    public const string FileName = "lanes.json";

    public int Format { get; init; } = CurrentFormat;
    public string Circuit { get; init; } = "";

    /// <summary>The per-station table, relative to the folder.</summary>
    public string Lanes { get; init; } = "lanes.csv";

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
