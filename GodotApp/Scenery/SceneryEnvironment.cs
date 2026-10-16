using System.Collections.Generic;
using System.Text.Json;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// How a circuit looks, as opposed to what stands round it: its sky, its
/// sun, its haze and the colours of its ground and barriers. Read from the
/// <c>environment</c> block of the circuit's scenery plan; every key is
/// optional, and one left out keeps the default, so a plan says only what
/// makes its circuit different — a Mediterranean sun, a darker forest green.
///
/// No Godot in here: the plan is a file format, read and tested without the
/// engine. The renderer turns it into engine colours (<c>CircuitLook</c>).
/// </summary>
public sealed record SceneryEnvironment
{
    public SceneryColour SkyTop { get; init; } = SceneryColour.FromHtml("#6f9cc6");
    public SceneryColour SkyHorizon { get; init; } = SceneryColour.FromHtml("#d8dfd8");
    public SceneryColour SkyGround { get; init; } = SceneryColour.FromHtml("#8d9d6e");

    /// <summary>Degrees below the horizon the sun shines down from, and the compass it comes from.</summary>
    public float SunPitch { get; init; } = 40f;
    public float SunYaw { get; init; } = -35f;
    public SceneryColour SunColor { get; init; } = SceneryColour.FromHtml("#ffe9c2");
    public float SunEnergy { get; init; } = 1.15f;
    public float AmbientEnergy { get; init; } = 0.75f;

    public SceneryColour FogColor { get; init; } = SceneryColour.FromHtml("#d3dbd6");
    public float FogDensity { get; init; } = 0.00042f;

    public SceneryColour Road { get; init; } = SceneryColour.FromHtml("#535b5d");
    public SceneryColour Grass { get; init; } = SceneryColour.FromHtml("#89976c");
    public SceneryColour Plain { get; init; } = SceneryColour.FromHtml("#93a372");

    /// <summary>The two bands of the mown run-off.</summary>
    public IReadOnlyList<SceneryColour> Mown { get; init; } = [SceneryColour.FromHtml("#9cb071"), SceneryColour.FromHtml("#8ca265")];

    /// <summary>Barrier faces on the straights, alternating.</summary>
    public IReadOnlyList<SceneryColour> Barrier { get; init; } = [SceneryColour.FromHtml("#e6e1d1"), SceneryColour.FromHtml("#d8d3c2")];

    /// <summary>The tyre wall on the outside of a tight corner, alternating.</summary>
    public IReadOnlyList<SceneryColour> TyreWall { get; init; } = [SceneryColour.FromHtml("#4a5254"), SceneryColour.FromHtml("#565e60")];

    public SceneryColour NearHills { get; init; } = SceneryColour.FromHtml("#869a64");
    public SceneryColour FarHills { get; init; } = SceneryColour.FromHtml("#8fa39c");
    public int Clouds { get; init; } = 34;

    public static SceneryEnvironment Read(JsonElement block)
    {
        var fallback = new SceneryEnvironment();
        return new SceneryEnvironment
        {
            SkyTop = Colour(block, "sky_top", fallback.SkyTop),
            SkyHorizon = Colour(block, "sky_horizon", fallback.SkyHorizon),
            SkyGround = Colour(block, "sky_ground", fallback.SkyGround),
            SunPitch = Number(block, "sun_pitch", fallback.SunPitch),
            SunYaw = Number(block, "sun_yaw", fallback.SunYaw),
            SunColor = Colour(block, "sun_color", fallback.SunColor),
            SunEnergy = Number(block, "sun_energy", fallback.SunEnergy),
            AmbientEnergy = Number(block, "ambient_energy", fallback.AmbientEnergy),
            FogColor = Colour(block, "fog_color", fallback.FogColor),
            FogDensity = Number(block, "fog_density", fallback.FogDensity),
            Road = Colour(block, "road", fallback.Road),
            Grass = Colour(block, "grass", fallback.Grass),
            Plain = Colour(block, "plain", fallback.Plain),
            Mown = Colours(block, "mown", fallback.Mown),
            Barrier = Colours(block, "barrier", fallback.Barrier),
            TyreWall = Colours(block, "tyre_wall", fallback.TyreWall),
            NearHills = Colour(block, "hills_near", fallback.NearHills),
            FarHills = Colour(block, "hills_far", fallback.FarHills),
            Clouds = (int)Number(block, "clouds", fallback.Clouds),
        };
    }

    private static SceneryColour Colour(JsonElement owner, string name, SceneryColour fallback) =>
        owner.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.String
            ? SceneryColour.FromHtml(value.GetString() ?? "")
            : fallback;

    private static IReadOnlyList<SceneryColour> Colours(JsonElement owner, string name, IReadOnlyList<SceneryColour> fallback)
    {
        if (!owner.TryGetProperty(name, out JsonElement value) || value.ValueKind != JsonValueKind.Array)
            return fallback;
        var colours = new List<SceneryColour>();
        foreach (JsonElement item in value.EnumerateArray())
            if (item.ValueKind == JsonValueKind.String)
                colours.Add(SceneryColour.FromHtml(item.GetString() ?? ""));
        return colours.Count > 0 ? colours : fallback;
    }

    private static float Number(JsonElement owner, string name, float fallback) =>
        owner.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.Number
            ? (float)value.GetDouble()
            : fallback;
}

/// <summary>A colour as the plan writes it, red, green, blue and alpha from zero to one.</summary>
public readonly record struct SceneryColour(float R, float G, float B, float A = 1f)
{
    /// <summary>
    /// <c>#rgb</c>, <c>#rrggbb</c> or <c>#rrggbbaa</c>, with or without the
    /// hash. Anything else is not a colour this file can mean, and throws.
    /// </summary>
    public static SceneryColour FromHtml(string html)
    {
        string hex = html.StartsWith('#') ? html[1..] : html;
        if (hex.Length == 3)
            hex = string.Concat(hex[0], hex[0], hex[1], hex[1], hex[2], hex[2]);
        if (hex.Length is not (6 or 8))
            throw new System.FormatException($"Not a colour: \"{html}\"");
        float Channel(int i) => int.Parse(hex.Substring(i, 2), System.Globalization.NumberStyles.HexNumber) / 255f;
        return new SceneryColour(Channel(0), Channel(2), Channel(4), hex.Length == 8 ? Channel(6) : 1f);
    }
}
