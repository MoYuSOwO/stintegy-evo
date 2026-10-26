using System.Text.Json;
using System.Text.Json.Serialization;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// What one driver has learned of one circuit in one car, on top of the car's
/// notes: per forty-metre segment, how much of the noted corner speed,
/// braking and throttle out of the corner this driver takes. One means all
/// of it; a scare in a corner takes some back, and clean laps through it
/// give it back again, at the driver's own pace (Adaptability).
///
/// It only ever holds back from the notes, never past them, and it learns
/// what the notes learn, coarser: whatever the driver learns, the car's
/// notes could have. The same code serves every car and circuit; the
/// numbers belong to the driver, and the game decides whether they are kept
/// (a championship) or thrown away (a one-off race).
/// </summary>
public sealed class LaneFamiliarity
{
    public const float SegmentMeters = 40f;

    /// <summary>The most a driver ever holds back from the notes.</summary>
    public const float Floor = 0.9f;

    // A circuit never driven: how far short of the notes a driver starts,
    // at no adaptability; one with all of it starts on them.
    private const float NewCircuitShortfall = 0.03f;

    // What one scare takes back from the segments that led into it, and a
    // spin or an off, more; the distance back it is charged over, by what
    // the driver was doing: braking reaches back to the marker, a corner to
    // its entry, the throttle only to the apex.
    private const float ScareLesson = 0.015f;
    private const float BigLesson = 0.04f;
    private const float BrakingBackMeters = 120f;
    private const float CornerBackMeters = 80f;
    private const float ThrottleBackMeters = 60f;

    // A clean pass through a segment gives back this much of what is held,
    // at the top and the bottom of the adaptability scale.
    private const float QuickestReturn = 0.25f;
    private const float SlowestReturn = 0.05f;

    [JsonPropertyName("format")] public int Format { get; init; } = 1;
    [JsonPropertyName("lengthMeters")] public float LengthMeters { get; init; }
    [JsonPropertyName("laps")] public int Laps { get; set; }
    [JsonPropertyName("corner")] public float[] Corner { get; init; } = [];
    [JsonPropertyName("brake")] public float[] Brake { get; init; } = [];
    [JsonPropertyName("traction")] public float[] Traction { get; init; } = [];

    /// <summary>What the driver is doing when something goes wrong.</summary>
    public enum Pedal { Corner, Braking, Throttle }

    /// <summary>A circuit this driver has never driven in this car.</summary>
    public static LaneFamiliarity NewCircuit(float lengthMeters, float adaptability)
    {
        int count = Math.Max(1, (int)MathF.Ceiling(lengthMeters / SegmentMeters));
        float start = 1f - NewCircuitShortfall * (1f - Math.Clamp(adaptability, 0f, 1f));
        return new LaneFamiliarity
        {
            LengthMeters = lengthMeters,
            Corner = Enumerable.Repeat(start, count).ToArray(),
            Brake = Enumerable.Repeat(start, count).ToArray(),
            Traction = Enumerable.Repeat(start, count).ToArray()
        };
    }

    public int Segments => Corner.Length;

    /// <summary>Whether this was learned on a circuit of this length: a different layout starts again.</summary>
    public bool Fits(float lengthMeters) =>
        MathF.Abs(LengthMeters - lengthMeters) < 1f &&
        Corner.Length == Brake.Length && Brake.Length == Traction.Length && Corner.Length > 0;

    public int SegmentOf(float s)
    {
        float wrapped = ((s % LengthMeters) + LengthMeters) % LengthMeters;
        return Math.Min(Segments - 1, (int)(wrapped / SegmentMeters));
    }

    /// <summary>
    /// Something went wrong at <paramref name="s"/>: the segments leading
    /// into it give back some of what they take, of whichever the driver
    /// was doing. A spin or an off teaches more than a slide caught.
    /// </summary>
    public void Scare(float s, Pedal pedal, bool big)
    {
        float lesson = big ? BigLesson : ScareLesson;
        (float[] table, float back) = pedal switch
        {
            Pedal.Braking => (Brake, BrakingBackMeters),
            Pedal.Throttle => (Traction, ThrottleBackMeters),
            _ => (Corner, CornerBackMeters)
        };
        int here = SegmentOf(s);
        int reach = (int)MathF.Ceiling(back / SegmentMeters);
        for (int k = 0; k <= reach; k++)
        {
            int i = ((here - k) % Segments + Segments) % Segments;
            table[i] = MathF.Max(Floor, table[i] - lesson);
        }
    }

    /// <summary>A clean pass through segment <paramref name="i"/>: some of what is held back comes back.</summary>
    public void Clean(int i, float adaptability)
    {
        float share = SlowestReturn + (QuickestReturn - SlowestReturn) * Math.Clamp(adaptability, 0f, 1f);
        Corner[i] += (1f - Corner[i]) * share;
        Brake[i] += (1f - Brake[i]) * share;
        Traction[i] += (1f - Traction[i]) * share;
    }

    private static readonly JsonSerializerOptions Json = new() { WriteIndented = true };

    public string ToJson() => JsonSerializer.Serialize(this, Json);

    public static LaneFamiliarity? FromJson(string json) => JsonSerializer.Deserialize<LaneFamiliarity>(json);
}

/// <summary>
/// Where drivers' learning is kept between sessions: who, in which car, at
/// which circuit. The game chooses whether to read and write it.
/// </summary>
public interface IDriverProgressStore
{
    LaneFamiliarity? Load(string driver, string car, string circuit);
    void Save(string driver, string car, string circuit, LaneFamiliarity familiarity);
}

/// <summary>Drivers' learning as files, <c>{root}/{driver}/{car}/{circuit}.json</c>.</summary>
public sealed class DriverProgressFolder(string root) : IDriverProgressStore
{
    public LaneFamiliarity? Load(string driver, string car, string circuit)
    {
        string path = PathOf(driver, car, circuit);
        return File.Exists(path) ? LaneFamiliarity.FromJson(File.ReadAllText(path)) : null;
    }

    public void Save(string driver, string car, string circuit, LaneFamiliarity familiarity)
    {
        string path = PathOf(driver, car, circuit);
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path, familiarity.ToJson());
    }

    private string PathOf(string driver, string car, string circuit) =>
        Path.Combine(root, Safe(driver), Safe(car), Safe(circuit) + ".json");

    private static string Safe(string name)
    {
        if (string.IsNullOrWhiteSpace(name) || name.IndexOfAny(Path.GetInvalidFileNameChars()) >= 0 || name is "." or "..")
            throw new ArgumentException($"'{name}' cannot be a folder name.", nameof(name));
        return name;
    }
}
