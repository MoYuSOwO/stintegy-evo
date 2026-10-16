using System.Collections.Generic;
using System.Text.Json;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// The circuit furniture the renderer places from the track's own shape —
/// fence, marshal posts, distance boards, start gantry, flags — and what a
/// plan may say about it: switch any of it off, change how often it comes,
/// or name where it stands. Left unsaid, it is placed as it always was.
/// Sides are the plan's own: positive (left of the direction of travel) or
/// negative, written <c>"left"</c> / <c>"right"</c>.
/// </summary>
public sealed record SceneryFurniture
{
    public bool Fence { get; init; } = true;
    public float FenceSpacing { get; init; } = 6f;

    public bool Marshals { get; init; } = true;
    public float MarshalSpacing { get; init; } = 310f;

    /// <summary>Marshal posts at these stations instead of every <see cref="MarshalSpacing"/> metres.</summary>
    public IReadOnlyList<(float S, int Side)> MarshalsAt { get; init; } = [];

    public bool Boards { get; init; } = true;

    /// <summary>Find braking zones from the geometry; off, only <see cref="BoardsAt"/> get boards.</summary>
    public bool BoardsAuto { get; init; } = true;

    /// <summary>Corners to put 150/100/50 boards before, as the station the braking ends at.</summary>
    public IReadOnlyList<(float S, int Side)> BoardsAt { get; init; } = [];

    public bool Gantry { get; init; } = true;

    /// <summary>Where the gantry spans the road; unset, a few metres before the start line.</summary>
    public float? GantryS { get; init; }

    public bool Flags { get; init; } = true;
    public float FlagRange { get; init; } = 140f;
    public float FlagSpacing { get; init; } = 20f;

    public static SceneryFurniture Read(JsonElement block)
    {
        var d = new SceneryFurniture();
        JsonElement fence = Part(block, "fence"), marshals = Part(block, "marshals"),
            boards = Part(block, "boards"), gantry = Part(block, "gantry"), flags = Part(block, "flags");
        return new SceneryFurniture
        {
            Fence = On(fence, d.Fence),
            FenceSpacing = Number(fence, "spacing", d.FenceSpacing),
            Marshals = On(marshals, d.Marshals),
            MarshalSpacing = Number(marshals, "spacing", d.MarshalSpacing),
            MarshalsAt = Places(marshals),
            Boards = On(boards, d.Boards),
            BoardsAuto = Bool(boards, "auto", d.BoardsAuto),
            BoardsAt = Places(boards),
            Gantry = On(gantry, d.Gantry),
            GantryS = gantry.ValueKind == JsonValueKind.Object && gantry.TryGetProperty("s", out JsonElement s) && s.ValueKind == JsonValueKind.Number
                ? (float)s.GetDouble()
                : null,
            Flags = On(flags, d.Flags),
            FlagRange = Number(flags, "range", d.FlagRange),
            FlagSpacing = Number(flags, "spacing", d.FlagSpacing),
        };
    }

    /// <summary>A part may be written as an object of settings, or just true / false.</summary>
    private static JsonElement Part(JsonElement block, string name) =>
        block.TryGetProperty(name, out JsonElement part) ? part : default;

    private static bool On(JsonElement part, bool fallback) => part.ValueKind switch
    {
        JsonValueKind.True => true,
        JsonValueKind.False => false,
        JsonValueKind.Object => Bool(part, "on", fallback),
        _ => fallback
    };

    private static bool Bool(JsonElement part, string name, bool fallback) =>
        part.ValueKind == JsonValueKind.Object && part.TryGetProperty(name, out JsonElement value) &&
        value.ValueKind is JsonValueKind.True or JsonValueKind.False
            ? value.GetBoolean()
            : fallback;

    private static float Number(JsonElement part, string name, float fallback) =>
        part.ValueKind == JsonValueKind.Object && part.TryGetProperty(name, out JsonElement value) &&
        value.ValueKind == JsonValueKind.Number
            ? (float)value.GetDouble()
            : fallback;

    private static IReadOnlyList<(float S, int Side)> Places(JsonElement part)
    {
        var places = new List<(float, int)>();
        if (part.ValueKind != JsonValueKind.Object || !part.TryGetProperty("at", out JsonElement list) ||
            list.ValueKind != JsonValueKind.Array)
            return places;
        foreach (JsonElement item in list.EnumerateArray())
        {
            float s = item.TryGetProperty("s", out JsonElement at) && at.ValueKind == JsonValueKind.Number ? (float)at.GetDouble() : 0f;
            int side = item.TryGetProperty("side", out JsonElement which) &&
                       string.Equals(which.GetString(), "right", System.StringComparison.Ordinal) ? -1 : 1;
            places.Add((s, side));
        }
        return places;
    }
}
