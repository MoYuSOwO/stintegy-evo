using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.GodotApp.Interop;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// Scenery that did not ship with the game.
///
/// The library inside the project works because the editor imported it:
/// every model there was turned into an engine resource before the build,
/// and <c>res://</c> serves what the importer left. A player adding a tree
/// has none of that. There is no editor in a shipped game, no import step,
/// and no way to put anything inside <c>res://</c> at all.
///
/// So a mod's models are read as files, at runtime, by the parsers that
/// exist in a shipped build:
///
/// <list type="bullet">
/// <item><b>glTF</b> — <c>.glb</c> and <c>.gltf</c> — through
/// <see cref="GltfDocument"/>, which is the engine's own runtime reader
/// and needs nothing imported. This is the format to tell modders about:
/// every tool exports it, and it carries its materials with it.</item>
/// <item><b>Godot's own</b> — <c>.tscn</c>, <c>.scn</c>, <c>.res</c> —
/// which load at runtime because they are already engine formats. Useful
/// for a mod built against the project; no use to somebody with a model
/// from Blender.</item>
/// </list>
///
/// Anything else — <c>.obj</c>, <c>.fbx</c>, <c>.dae</c> — works inside
/// the project, where the editor imports it, and does not work from a mod
/// folder, where there is no importer to do it. That asymmetry is a
/// property of the engine rather than a decision made here, and it is why
/// the modding instructions say glTF.
///
/// A mod folder looks like this, and every part of it is optional:
///
/// <code>
///   mods/greener-silverstone/
///     props/oak.glb
///     scenery/silverstone.json      # "prop": "props/oak.glb"
/// </code>
///
/// Paths inside a mod's plan are relative to that mod's own folder, so a
/// mod can be moved, renamed or zipped without rewriting anything. Nothing
/// a mod provides can reach the simulation: scenery is drawn and nothing
/// else knows it is there.
/// </summary>
public static class SceneryMods
{
    /// <summary>
    /// Where mods live. <c>user://</c> is the writable directory a shipped
    /// game has on every platform — the place a player can actually put
    /// files. STINTEGY_MODS points somewhere else, which is how this gets
    /// tested and how a developer works on one.
    /// </summary>
    public static string Root =>
        System.Environment.GetEnvironmentVariable("STINTEGY_MODS") is string named
        && named.Length > 0
            ? named
            : "user://mods";

    /// <summary>Every mod folder present, in a stable order.</summary>
    public static IReadOnlyList<string> Installed()
    {
        using DirAccess? root = DirAccess.Open(Root);
        if (root is null)
            return [];
        List<string> mods = [];
        foreach (string name in root.GetDirectories())
            mods.Add($"{Root}/{name}");
        mods.Sort(StringComparer.Ordinal);
        return mods;
    }

    /// <summary>
    /// What the mods add to one circuit, in the order the mods are listed.
    /// A mod that says nothing about this circuit contributes nothing.
    /// </summary>
    public static IEnumerable<(SceneryPlan Plan, string Folder)> PlansFor(string track)
    {
        foreach (string mod in Installed())
        {
            string path = $"{mod}/scenery/{track}.json";
            if (!Godot.FileAccess.FileExists(path))
                continue;
            using var file = Godot.FileAccess.Open(path, Godot.FileAccess.ModeFlags.Read);
            if (file is null)
                continue;
            SceneryPlan plan;
            try
            {
                plan = SceneryPlan.Parse(file.GetAsText());
            }
            catch (Exception error)
            {
                GD.PushWarning($"scenery: {path} could not be read -- {error.Message}");
                continue;
            }
            yield return (plan, mod);
        }
    }

    /// <summary>A mod's model, read from its folder (<see cref="RuntimeModels"/>).</summary>
    public static PackedScene? LoadModel(string path) => RuntimeModels.Load(path, "scenery");
}
