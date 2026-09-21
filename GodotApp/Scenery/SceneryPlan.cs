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
    public IReadOnlyList<SceneryPlacement> Props { get; init; } = [];

    /// <summary>
    /// Every prop the plan asks for, with repeats unrolled: a row of
    /// twelve trees is written once and arrives here twelve times.
    /// </summary>
    public IEnumerable<SceneryPlacement> Placements()
    {
        foreach (SceneryPlacement prop in Props)
        {
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
        return new SceneryPlan
        {
            Track = root.TryGetProperty("track", out JsonElement track)
                ? track.GetString() ?? string.Empty
                : string.Empty,
            Props = props
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
        return new SceneryPlacement(
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
    /// The circuit a plan file belongs to, from its own name: plans live
    /// beside each other and are named for their track.
    /// </summary>
    public static string TrackOf(string planPath)
    {
        string name = planPath;
        int slash = name.LastIndexOf('/');
        if (slash >= 0)
            name = name[(slash + 1)..];
        int dot = name.LastIndexOf('.');
        return dot > 0 ? name[..dot] : name;
    }

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
    SceneryRepeat? Repeat = null
);

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
