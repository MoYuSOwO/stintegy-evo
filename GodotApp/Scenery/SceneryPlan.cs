using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using System.Text.Json;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// Where the scenery goes, as a file somebody wrote.
///
/// Scenery is authored content. The renderer used to improvise it — a
/// pit building at a hardcoded offset, twenty-eight clusters of trees at
/// evenly spaced stations — which made a circuit's surroundings a property
/// of the code rather than of the circuit. This is the other way round: a
/// plan per track, human-readable and diffable, naming props from a
/// library and saying where each one stands.
///
/// Positions are in the track's own frame — station along the centreline
/// and offset to the side of it — rather than in world coordinates. A
/// circuit whose geometry is re-cut moves every world coordinate with it;
/// a grandstand at "station 300, forty-five metres left" is still beside
/// the same corner afterwards. Height is not authored at all: the ground
/// is where the surface says it is, and a prop that needs to sit above or
/// below it says so as an offset.
///
/// Nothing here reaches the physics. The walls remain the only boundary
/// the car knows, and a tree is a thing to look at.
/// </summary>
public sealed class SceneryPlan
{
    public string Track { get; init; } = string.Empty;

    /// <summary>The circuit's name as people say it, for the screen: "Silverstone".</summary>
    public string Name { get; init; } = string.Empty;

    /// <summary>The circuit's sky, sun, haze and ground colours; defaults where the plan is silent.</summary>
    public SceneryEnvironment Environment { get; init; } = new();

    /// <summary>Which of the circuit furniture to place, how often, and where; defaults where the plan is silent.</summary>
    public SceneryFurniture Furniture { get; init; } = new();
    public IReadOnlyList<SceneryPlacement> Props { get; init; } = [];

    /// <summary>Flat colour laid on the ground: fields, car parks, a paddock.</summary>
    public IReadOnlyList<SceneryPatch> Patches { get; init; } = [];

    /// <summary>Countryside generated around the circuit: a patchwork of fields with hedges between them.</summary>
    public SceneryFarmland? Farmland { get; init; }

    /// <summary>
    /// Every prop the plan asks for, with repeats unrolled: a row of
    /// twelve trees is written once and arrives here twelve times.
    /// </summary>
    public IEnumerable<SceneryPlacement> Placements()
    {
        for (int entry = 0; entry < Props.Count; entry++)
        {
            SceneryPlacement prop = Props[entry];
            if (prop.Along is SceneryAlong along)
            {
                foreach (SceneryPlacement each in Along(prop, along, entry))
                    yield return each;
                continue;
            }
            if (prop.Area is SceneryArea area)
            {
                foreach (SceneryPlacement each in Area(prop, area, entry))
                    yield return each;
                continue;
            }
            int count = Math.Max(1, prop.Repeat?.Count ?? 1);
            for (int i = 0; i < count; i++)
            {
                if (i == 0)
                {
                    // Stripped even on the first of a row: a placement
                    // that still carries its repeat would be unrolled
                    // again by the next reader, and a row of twelve would
                    // become a row of a hundred and forty four.
                    yield return prop with { Repeat = null };
                    continue;
                }
                SceneryRepeat repeat = prop.Repeat!.Value;
                yield return prop with
                {
                    S = prop.S + repeat.StepS * i,
                    D = prop.D + repeat.StepD * i,
                    Yaw = prop.Yaw + repeat.StepYaw * i,
                    Repeat = null
                };
            }
        }
    }

    /// <summary>
    /// Read by hand rather than by a serializer: the file has eight keys
    /// and two of them are spelled the way a person would spell them
    /// ("align": "world", "step_s"), which an attribute-decorated model
    /// would only obscure. Comments and trailing commas are allowed,
    /// because the file is for people.
    /// </summary>
    /// <summary>
    /// A prop every so many metres between two stations, on one side or
    /// both, each nudged a little so a row reads as planted and not drawn
    /// with a ruler. The nudges are seeded by the plan entry, so the same
    /// file grows the same trees every time.
    /// </summary>
    private IEnumerable<SceneryPlacement> Along(SceneryPlacement prop, SceneryAlong along, int entry)
    {
        var random = new Random(Seed(entry));
        float every = MathF.Max(along.Every, 0.5f);
        int count = (int)MathF.Floor(MathF.Max(0f, along.To - along.From) / every) + 1;
        int[] sides = along.Side switch
        {
            "both" => [1, -1],
            "left" => [1],
            "right" => [-1],
            _ => [MathF.Sign(prop.D) == 0 ? 1 : (int)MathF.Sign(prop.D)]
        };
        for (int i = 0; i < count; i++)
        {
            foreach (int side in sides)
            {
                yield return prop with
                {
                    S = along.From + i * every + Jitter(random, along.SJitter),
                    D = side * (MathF.Abs(prop.D) + Jitter(random, along.DJitter)),
                    Yaw = prop.Yaw + Jitter(random, along.YawJitter),
                    Scale = prop.Scale * (1f + Jitter(random, along.ScaleJitter)),
                    Along = null,
                    Repeat = null
                };
            }
        }
    }

    /// <summary>
    /// Props scattered through a band beside the circuit — so many per
    /// hundred square metres, anywhere between two stations and two
    /// offsets. A wood is an area of trees; a car park is an area of cars.
    /// </summary>
    private IEnumerable<SceneryPlacement> Area(SceneryPlacement prop, SceneryArea area, int entry)
    {
        var random = new Random(Seed(entry));
        float length = MathF.Abs(area.SEnd - area.SStart);
        float width = MathF.Abs(area.DOuter - area.DInner);
        int count = (int)MathF.Round(length * width / 100f * MathF.Max(area.PerHundredSquareMetres, 0f));
        for (int i = 0; i < count; i++)
        {
            float scale = area.ScaleMin + (float)random.NextDouble() * (area.ScaleMax - area.ScaleMin);
            yield return prop with
            {
                S = area.SStart + (float)random.NextDouble() * (area.SEnd - area.SStart),
                D = area.DInner + (float)random.NextDouble() * (area.DOuter - area.DInner),
                Yaw = area.RandomYaw ? (float)random.NextDouble() * MathF.Tau : prop.Yaw,
                AlignToTrack = !area.RandomYaw && prop.AlignToTrack,
                Scale = prop.Scale * scale,
                Area = null,
                Repeat = null
            };
        }
    }

    private int Seed(int entry) => unchecked(Track.GetHashCode(StringComparison.Ordinal) * 397 ^ (entry + 1) * 7919);

    private static float Jitter(Random random, float amount) =>
        amount <= 0f ? 0f : ((float)random.NextDouble() * 2f - 1f) * amount;

    public static SceneryPlan Parse(string json)
    {
        JsonDocumentOptions options = new()
        {
            CommentHandling = JsonCommentHandling.Skip,
            AllowTrailingCommas = true
        };
        using JsonDocument document = JsonDocument.Parse(json, options);
        JsonElement root = document.RootElement;
        List<SceneryPlacement> props = [];
        if (root.TryGetProperty("props", out JsonElement placed))
        {
            foreach (JsonElement prop in placed.EnumerateArray())
                props.Add(ReadPlacement(prop));
        }
        List<SceneryPatch> patches = [];
        if (root.TryGetProperty("patches", out JsonElement laid))
        {
            foreach (JsonElement patch in laid.EnumerateArray())
            {
                (float s0, float s1) = Pair(patch, "s", 0f, 0f);
                (float d0, float d1) = Pair(patch, "d", 0f, 0f);
                patches.Add(new SceneryPatch(
                    Color: patch.TryGetProperty("color", out JsonElement color) ? color.GetString() ?? "#808080" : "#808080",
                    SStart: s0, SEnd: s1, DInner: d0, DOuter: d1));
            }
        }
        SceneryFarmland? farmland = null;
        if (root.TryGetProperty("farmland", out JsonElement farm))
        {
            List<string> palette = [];
            if (farm.TryGetProperty("palette", out JsonElement colours))
                foreach (JsonElement colour in colours.EnumerateArray())
                    palette.Add(colour.GetString() ?? "#9fb26b");
            farmland = new SceneryFarmland(
                Inner: Number(farm, "from", 180f),
                Outer: Number(farm, "to", 900f),
                Field: Number(farm, "field", 130f),
                Angle: Number(farm, "angle", 0.3f),
                Meadow: Number(farm, "meadow", 0.25f),
                Palette: palette,
                Hedge: farm.TryGetProperty("hedge", out JsonElement hedge) ? hedge.ValueKind == JsonValueKind.Number ? hedge.GetRawText() : hedge.GetString() ?? "" : "",
                Tree: farm.TryGetProperty("tree", out JsonElement tree) ? tree.ValueKind == JsonValueKind.Number ? tree.GetRawText() : tree.GetString() ?? "" : "");
        }
        return new SceneryPlan
        {
            Track = root.TryGetProperty("track", out JsonElement track)
                ? track.GetString() ?? string.Empty
                : string.Empty,
            Props = props,
            Patches = patches,
            Farmland = farmland,
            Name = root.TryGetProperty("name", out JsonElement name) ? name.GetString() ?? string.Empty : string.Empty,
            Environment = root.TryGetProperty("environment", out JsonElement look)
                ? SceneryEnvironment.Read(look)
                : new SceneryEnvironment(),
            Furniture = root.TryGetProperty("furniture", out JsonElement furniture)
                ? SceneryFurniture.Read(furniture)
                : new SceneryFurniture()
        };
    }

    private static SceneryPlacement ReadPlacement(JsonElement prop)
    {
        SceneryRepeat? repeat = null;
        if (prop.TryGetProperty("repeat", out JsonElement block))
        {
            repeat = new SceneryRepeat(
                Count: (int)Number(block, "count", 1f),
                StepS: Number(block, "step_s", 0f),
                StepD: Number(block, "step_d", 0f),
                StepYaw: Number(block, "step_yaw", 0f)
            );
        }
        SceneryAlong? along = null;
        if (prop.TryGetProperty("along", out JsonElement row))
        {
            along = new SceneryAlong(
                From: Number(row, "from", 0f),
                To: Number(row, "to", 0f),
                Every: Number(row, "every", 10f),
                Side: row.TryGetProperty("side", out JsonElement side) ? side.GetString() ?? "given" : "given",
                SJitter: Number(row, "s_jitter", 0f),
                DJitter: Number(row, "d_jitter", 0f),
                YawJitter: Number(row, "yaw_jitter", 0f),
                ScaleJitter: Number(row, "scale_jitter", 0f));
        }
        SceneryArea? area = null;
        if (prop.TryGetProperty("area", out JsonElement zone))
        {
            (float s0, float s1) = Pair(zone, "s", 0f, 0f);
            (float d0, float d1) = Pair(zone, "d", 0f, 0f);
            (float k0, float k1) = Pair(zone, "scale", 1f, 1f);
            area = new SceneryArea(
                SStart: s0, SEnd: s1, DInner: d0, DOuter: d1,
                PerHundredSquareMetres: Number(zone, "per_100m2", 1f),
                ScaleMin: k0, ScaleMax: k1,
                RandomYaw: !(zone.TryGetProperty("yaw", out JsonElement yaw) &&
                             string.Equals(yaw.GetString(), "track", StringComparison.Ordinal)));
        }
        return new SceneryPlacement(
            Along: along,
            Area: area,
            Prop: PropName(prop),
            S: Number(prop, "s", 0f),
            D: Number(prop, "d", 0f),
            Yaw: Number(prop, "yaw", 0f),
            Scale: Number(prop, "scale", 1f),
            Height: Number(prop, "height", 0f),
            AlignToTrack: !(
                prop.TryGetProperty("align", out JsonElement align) &&
                string.Equals(align.GetString(), "world", StringComparison.Ordinal)
            ),
            Repeat: repeat
        );
    }

    /// <summary>
    /// How the plan named its prop, as written: a number stays a number
    /// ("2"), a name stays a name ("pine"), a path stays a path. JSON has
    /// two ways to write a number and a person will use both, so
    /// <c>"prop": 2</c> and <c>"prop": "2"</c> are the same prop.
    /// </summary>
    private static string PropName(JsonElement prop)
    {
        if (!prop.TryGetProperty("prop", out JsonElement name))
            return string.Empty;
        return name.ValueKind switch
        {
            JsonValueKind.Number => name.GetRawText(),
            JsonValueKind.String => name.GetString() ?? string.Empty,
            _ => string.Empty
        };
    }

    /// <summary>A two-number array, "s": [1200, 1600]; one number means both ends.</summary>
    private static (float, float) Pair(JsonElement owner, string name, float a, float b)
    {
        if (!owner.TryGetProperty(name, out JsonElement value))
            return (a, b);
        if (value.ValueKind == JsonValueKind.Number)
        {
            float only = (float)value.GetDouble();
            return (only, only);
        }
        if (value.ValueKind == JsonValueKind.Array && value.GetArrayLength() >= 2)
            return ((float)value[0].GetDouble(), (float)value[1].GetDouble());
        return (a, b);
    }

    private static float Number(JsonElement owner, string name, float fallback) =>
        owner.TryGetProperty(name, out JsonElement value) &&
        value.ValueKind == JsonValueKind.Number
            ? (float)value.GetDouble()
            : fallback;

    /// <summary>
    /// The plan as a file, written by hand rather than by the serializer,
    /// so that a plan a person edits and a plan the editor bridge writes
    /// look the same in a diff: one prop a line, numbers rounded to the
    /// centimetre and the milliradian, no repeats invented.
    /// </summary>
    public string ToJson()
    {
        StringBuilder text = new();
        text.Append("{\n  \"track\": \"").Append(Track).Append("\",\n");
        text.Append("  \"props\": [\n");
        for (int i = 0; i < Props.Count; i++)
        {
            SceneryPlacement prop = Props[i];
            // A number goes back out as a number, which is how a person
            // writes one and how the catalogue reads.
            bool numbered = int.TryParse(prop.Prop, out _);
            text.Append("    { \"prop\": ");
            text.Append(numbered ? prop.Prop : $"\"{prop.Prop}\"").Append(", ");
            text.Append("\"s\": ").Append(Round(prop.S)).Append(", ");
            text.Append("\"d\": ").Append(Round(prop.D)).Append(", ");
            text.Append("\"yaw\": ").Append(Round(prop.Yaw, 3)).Append(", ");
            text.Append("\"scale\": ").Append(Round(prop.Scale, 3));
            if (prop.Height != 0f)
                text.Append(", \"height\": ").Append(Round(prop.Height));
            if (!prop.AlignToTrack)
                text.Append(", \"align\": \"world\"");
            if (prop.Repeat is SceneryRepeat repeat)
            {
                text.Append(", \"repeat\": { \"count\": ").Append(repeat.Count);
                text.Append(", \"step_s\": ").Append(Round(repeat.StepS));
                text.Append(", \"step_d\": ").Append(Round(repeat.StepD));
                text.Append(", \"step_yaw\": ").Append(Round(repeat.StepYaw, 3));
                text.Append(" }");
            }
            text.Append(" }").Append(i + 1 < Props.Count ? ",\n" : "\n");
        }
        text.Append("  ]\n}\n");
        return text.ToString();
    }

    /// <summary>
    /// The circuit a plan file belongs to. A circuit's own plan is
    /// <c>&lt;id&gt;/scenery.json</c> and belongs to its folder; a mod's is
    /// <c>scenery/&lt;id&gt;.json</c> and is named for its track.
    /// </summary>
    public static string TrackOf(string planPath)
    {
        string[] parts = planPath.Replace('\\', '/').Split('/', StringSplitOptions.RemoveEmptyEntries);
        string name = parts.Length > 0 ? parts[^1] : planPath;
        if (name == FileName && parts.Length > 1)
            return parts[^2];
        int dot = name.LastIndexOf('.');
        return dot > 0 ? name[..dot] : name;
    }

    /// <summary>A circuit's own plan, in its folder.</summary>
    public const string FileName = "scenery.json";

    private static string Round(float value, int digits = 2) =>
        MathF.Round(value, digits).ToString(CultureInfo.InvariantCulture);
}

/// <summary>
/// One prop, standing at a station: which prop, where along and across the
/// circuit, which way it faces, how big, and whether its facing is
/// measured from the road or from the world.
///
/// <see cref="Prop"/> is whichever of the three ways the plan named it —
/// a catalogue number, a name in the library, or a path to somebody's own
/// asset. It is carried as written and resolved when it is loaded, so a
/// plan that names a file this build has never seen still reads.
/// </summary>
public readonly record struct SceneryPlacement(
    string Prop,
    float S,
    float D,
    float Yaw = 0f,
    float Scale = 1f,
    float Height = 0f,
    bool AlignToTrack = true,
    SceneryRepeat? Repeat = null,
    SceneryAlong? Along = null,
    SceneryArea? Area = null,
    System.Numerics.Vector3? World = null
);

/// <summary>Every <c>Every</c> metres from <c>From</c> to <c>To</c>, on the given side, left, right or both.</summary>
public readonly record struct SceneryAlong(
    float From, float To, float Every, string Side,
    float SJitter, float DJitter, float YawJitter, float ScaleJitter);

/// <summary>So many per hundred square metres between two stations and two offsets.</summary>
public readonly record struct SceneryArea(
    float SStart, float SEnd, float DInner, float DOuter,
    float PerHundredSquareMetres, float ScaleMin, float ScaleMax, bool RandomYaw);

/// <summary>
/// A patchwork of fields from <c>Inner</c> to <c>Outer</c> metres off the
/// circuit: a grid of <c>Field</c>-metre parcels turned by <c>Angle</c>, each
/// a crop from the palette or, a <c>Meadow</c> share of the time, left as
/// grass; with a hedge prop along the edges and now and then a tree in one.
/// </summary>
public sealed record SceneryFarmland(
    float Inner, float Outer, float Field, float Angle, float Meadow,
    IReadOnlyList<string> Palette, string Hedge, string Tree);

/// <summary>A flat colour on the ground between two stations and two offsets.</summary>
public readonly record struct SceneryPatch(string Color, float SStart, float SEnd, float DInner, float DOuter);

/// <summary>
/// A row of the same prop, so that an avenue of trees is one line in the
/// file rather than thirty. Each step is added to the one before it.
/// </summary>
public readonly record struct SceneryRepeat(
    int Count,
    float StepS = 0f,
    float StepD = 0f,
    float StepYaw = 0f
);
