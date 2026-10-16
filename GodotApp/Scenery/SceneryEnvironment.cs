using System.Collections.Generic;
using System.Text.Json;
using Godot;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// How a circuit looks, as opposed to what stands round it: its sky, its
/// sun, its haze and the colours of its ground and barriers. Read from the
/// <c>environment</c> block of the circuit's scenery plan; every key is
/// optional, and one left out keeps the default, so a plan says only what
/// makes its circuit different — a Mediterranean sun, a darker forest green.
/// </summary>
public sealed record SceneryEnvironment
{
    public Color SkyTop { get; init; } = Color.FromHtml("#6f9cc6");
    public Color SkyHorizon { get; init; } = Color.FromHtml("#d8dfd8");
    public Color SkyGround { get; init; } = Color.FromHtml("#8d9d6e");

    /// <summary>Degrees below the horizon the sun shines down from, and the compass it comes from.</summary>
    public float SunPitch { get; init; } = 40f;
    public float SunYaw { get; init; } = -35f;
    public Color SunColor { get; init; } = Color.FromHtml("#ffe9c2");
    public float SunEnergy { get; init; } = 1.15f;
    public float AmbientEnergy { get; init; } = 0.75f;

    public Color FogColor { get; init; } = Color.FromHtml("#d3dbd6");
    public float FogDensity { get; init; } = 0.00042f;

    public Color Road { get; init; } = Color.FromHtml("#535b5d");
    public Color Grass { get; init; } = Color.FromHtml("#89976c");
    public Color Plain { get; init; } = Color.FromHtml("#93a372");

    /// <summary>The two bands of the mown run-off.</summary>
    public IReadOnlyList<Color> Mown { get; init; } = [Color.FromHtml("#9cb071"), Color.FromHtml("#8ca265")];

    /// <summary>Barrier faces on the straights, alternating.</summary>
    public IReadOnlyList<Color> Barrier { get; init; } = [Color.FromHtml("#e6e1d1"), Color.FromHtml("#d8d3c2")];

    /// <summary>The tyre wall on the outside of a tight corner, alternating.</summary>
    public IReadOnlyList<Color> TyreWall { get; init; } = [Color.FromHtml("#4a5254"), Color.FromHtml("#565e60")];

    public Color NearHills { get; init; } = Color.FromHtml("#869a64");
    public Color FarHills { get; init; } = Color.FromHtml("#8fa39c");
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

    private static Color Colour(JsonElement owner, string name, Color fallback) =>
        owner.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.String
            ? Color.FromHtml(value.GetString() ?? "")
            : fallback;

    private static IReadOnlyList<Color> Colours(JsonElement owner, string name, IReadOnlyList<Color> fallback)
    {
        if (!owner.TryGetProperty(name, out JsonElement value) || value.ValueKind != JsonValueKind.Array)
            return fallback;
        var colours = new List<Color>();
        foreach (JsonElement item in value.EnumerateArray())
            if (item.ValueKind == JsonValueKind.String)
                colours.Add(Color.FromHtml(item.GetString() ?? ""));
        return colours.Count > 0 ? colours : fallback;
    }

    private static float Number(JsonElement owner, string name, float fallback) =>
        owner.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.Number
            ? (float)value.GetDouble()
            : fallback;
}
