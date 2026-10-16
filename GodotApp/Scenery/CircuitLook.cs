using System.Collections.Generic;
using System.Linq;
using Godot;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// A circuit's <see cref="SceneryEnvironment"/> in engine colours, for the
/// views. The plan stays free of Godot so it can be read and tested without
/// the engine; this is where it meets the renderer.
/// </summary>
public sealed record CircuitLook
{
    public Color SkyTop { get; init; }
    public Color SkyHorizon { get; init; }
    public Color SkyGround { get; init; }
    public float SunPitch { get; init; }
    public float SunYaw { get; init; }
    public Color SunColor { get; init; }
    public float SunEnergy { get; init; }
    public float AmbientEnergy { get; init; }
    public Color FogColor { get; init; }
    public float FogDensity { get; init; }
    public Color Road { get; init; }
    public Color Grass { get; init; }
    public Color Plain { get; init; }
    public IReadOnlyList<Color> Mown { get; init; } = [];
    public IReadOnlyList<Color> Barrier { get; init; } = [];
    public IReadOnlyList<Color> TyreWall { get; init; } = [];
    public Color NearHills { get; init; }
    public Color FarHills { get; init; }
    public int Clouds { get; init; }

    public static CircuitLook From(SceneryEnvironment e) => new()
    {
        SkyTop = Engine(e.SkyTop),
        SkyHorizon = Engine(e.SkyHorizon),
        SkyGround = Engine(e.SkyGround),
        SunPitch = e.SunPitch,
        SunYaw = e.SunYaw,
        SunColor = Engine(e.SunColor),
        SunEnergy = e.SunEnergy,
        AmbientEnergy = e.AmbientEnergy,
        FogColor = Engine(e.FogColor),
        FogDensity = e.FogDensity,
        Road = Engine(e.Road),
        Grass = Engine(e.Grass),
        Plain = Engine(e.Plain),
        Mown = e.Mown.Select(Engine).ToArray(),
        Barrier = e.Barrier.Select(Engine).ToArray(),
        TyreWall = e.TyreWall.Select(Engine).ToArray(),
        NearHills = Engine(e.NearHills),
        FarHills = Engine(e.FarHills),
        Clouds = e.Clouds,
    };

    private static Color Engine(SceneryColour c) => new(c.R, c.G, c.B, c.A);
}
