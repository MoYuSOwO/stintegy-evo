using System.IO;
using Godot;
using StintegyEVO.Core.Mods;

namespace StintegyEVO.GodotApp.Interop;

/// <summary>
/// Models from outside the project -- a mod's trees, a car's body -- read at
/// runtime by the parsers a shipped game has.
///
/// The library inside the project works because the editor imported it:
/// every model there was turned into an engine resource before the build,
/// and <c>res://</c> serves what the importer left. A mod has none of that.
/// There is no editor in a shipped game and no import step, so a mod's
/// models are read from its own files, packed or not, by the engine's
/// runtime glTF reader (<see cref="GltfDocument"/>), which needs nothing
/// imported. That is why a mod's models are glTF: <c>.obj</c>, <c>.fbx</c>
/// and the rest work inside the project, where the editor imports them, and
/// cannot work from a mod, where there is no importer to do it.
/// </summary>
public static class RuntimeModels
{
    /// <summary>
    /// A model in the project: Godot's own formats through the ordinary
    /// loader, glTF through the runtime reader. Null for a file that is
    /// neither, or that will not parse -- a bad model costs a prop or a
    /// car its looks, not the player their race.
    /// </summary>
    public static PackedScene? Load(string path, string what)
    {
        if (!Godot.FileAccess.FileExists(path))
            return null;
        string extension = Extension(path);
        if (extension is "tscn" or "scn" or "res")
            return ResourceLoader.Load<PackedScene>(path);
        if (extension is not ("glb" or "gltf"))
        {
            GD.PushWarning($"{what}: {path} is a .{extension}, which only the editor can import; use glTF (.glb/.gltf)");
            return null;
        }
        GltfDocument document = new();
        GltfState state = new();
        Error read = document.AppendFromFile(path, state);
        return read == Error.Ok ? Pack(document, state)
            : Warn($"{what}: {path} would not parse as glTF ({read})");
    }

    /// <summary>
    /// A model a mod carries, at <paramref name="path"/> in its files: a
    /// <c>.glb</c>, or a <c>.gltf</c> with everything inside it.
    /// </summary>
    public static PackedScene? Load(IModResources mod, string path, string what)
    {
        if (!mod.Exists(path))
            return Warn($"{what}: {mod.Location} has no {path}");
        if (Extension(path) is not ("glb" or "gltf"))
            return Warn($"{what}: {path} in {mod.Location} is not glTF (.glb/.gltf), the one format a mod's models can be");
        byte[] bytes;
        try
        {
            bytes = mod.ReadAllBytes(path);
        }
        catch (IOException error)
        {
            return Warn($"{what}: {path} in {mod.Location} could not be read -- {error.Message}");
        }
        GltfDocument document = new();
        GltfState state = new();
        Error read = document.AppendFromBuffer(bytes, "", state);
        return read == Error.Ok ? Pack(document, state)
            : Warn($"{what}: {path} in {mod.Location} would not parse as glTF ({read})");
    }

    private static PackedScene? Pack(GltfDocument document, GltfState state)
    {
        Node? scene = document.GenerateScene(state);
        if (scene is null)
            return null;
        PackedScene packed = new();
        Error packing = packed.Pack(scene);
        scene.QueueFree();
        return packing == Error.Ok ? packed : null;
    }

    private static string Extension(string path) => path[(path.LastIndexOf('.') + 1)..].ToLowerInvariant();

    private static PackedScene? Warn(string message)
    {
        GD.PushWarning(message);
        return null;
    }
}
