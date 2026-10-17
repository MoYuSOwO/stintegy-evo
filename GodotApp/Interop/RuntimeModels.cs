using Godot;

namespace StintegyEVO.GodotApp.Interop;

/// <summary>
/// Models from outside the project -- a scenery mod's trees, a car pack's
/// car -- read at runtime by the parsers a shipped game has. Why only glTF
/// and Godot's own formats: see <see cref="Scenery.SceneryMods"/>.
/// </summary>
public static class RuntimeModels
{
    /// <summary>
    /// A model read from disk rather than from the project: glTF through
    /// the engine's runtime reader, Godot's own formats through the
    /// ordinary loader. Null for a file that is neither, or that will not
    /// parse — a bad model costs a mod its tree or its car its looks,
    /// not the player their race.
    /// </summary>
    public static PackedScene? Load(string path, string what)
    {
        if (!Godot.FileAccess.FileExists(path))
            return null;

        string extension = path[(path.LastIndexOf('.') + 1)..].ToLowerInvariant();
        if (extension is "tscn" or "scn" or "res")
            return ResourceLoader.Load<PackedScene>(path);
        if (extension is not ("glb" or "gltf"))
        {
            GD.PushWarning(
                $"{what}: {path} is a .{extension}, which only the editor " +
                "can import; a mod's models have to be glTF (.glb/.gltf)"
            );
            return null;
        }

        GltfDocument document = new();
        GltfState state = new();
        Error read = document.AppendFromFile(path, state);
        if (read != Error.Ok)
        {
            GD.PushWarning($"{what}: {path} would not parse as glTF ({read})");
            return null;
        }
        Node? scene = document.GenerateScene(state);
        if (scene is null)
            return null;
        PackedScene packed = new();
        if (packed.Pack(scene) != Error.Ok)
            return null;
        scene.QueueFree();
        return packed;
    }
}
