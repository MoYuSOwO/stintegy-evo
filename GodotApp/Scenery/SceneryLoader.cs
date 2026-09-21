using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.GodotApp.LowPoly;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// Puts a plan's props on the ground.
///
/// A plan names a prop in one of three ways, and the three are the point
/// of this class:
///
/// <list type="bullet">
/// <item><b>By catalogue number.</b> <c>"prop": 2</c> is whatever
/// <see cref="CataloguePath"/> says number two is. The numbers are the
/// stable handle on the props that ship with the game: rename the file,
/// re-model the tree, and every plan that asked for a 2 still gets a
/// grandstand.</item>
/// <item><b>By name.</b> <c>"prop": "pine"</c> is <c>pine.glb</c> or
/// <c>pine.tscn</c> in <see cref="LibraryPath"/>. Nothing registers it —
/// a prop exists because its file does.</item>
/// <item><b>By its own path.</b> <c>"prop": "res://Assets/MyTrack/oak.glb"</c>
/// or a <c>user://</c> path is loaded as given, which is how a circuit
/// brings scenery nobody else has. Nothing needs to be added to the
/// library, and nothing needs a number.</item>
/// </list>
///
/// A prop the library does not have is reported once and skipped. A
/// circuit missing a tree is a circuit missing a tree; a circuit that
/// refuses to load because somebody renamed a model is a broken game.
///
/// Repeated props of a single mesh are drawn as one instanced batch, so an
/// avenue of forty trees costs the renderer one draw rather than forty.
/// Props that are whole scenes — a grandstand with its own lights and
/// materials — are instanced as scenes, which is what they are for.
///
/// None of this touches the simulation. The plan is read after the
/// circuit is built, the nodes hang off the presentation tree, and the
/// walls remain the only boundary anything physical knows about.
/// </summary>
public sealed partial class SceneryLoader : Node3D
{
    public const string LibraryPath = "res://Assets/Scenery";

    /// <summary>
    /// The numbered props: which file each of the game's own prop numbers
    /// means. A plan that asks for a number gets whatever this says, so
    /// the default scenery can be re-cut without rewriting the circuits
    /// that use it.
    /// </summary>
    public const string CataloguePath = "res://Assets/Scenery/catalogue.json";

    /// <summary>
    /// How many of the same single-mesh prop it takes before they are
    /// drawn as one batch instead of separate nodes.
    /// </summary>
    private const int BatchFrom = 8;

    private readonly Dictionary<string, Prop?> _library = [];
    private Dictionary<string, string>? _catalogue;

    /// <summary>
    /// Builds a plan's scenery under this node. The surface is what puts
    /// each prop on the ground: a plan says where along and across the
    /// circuit a thing stands, never how high.
    /// </summary>
    public void Build(SceneryPlan plan, TrackSurfaceGeometry surface)
    {
        ArgumentNullException.ThrowIfNull(plan);
        ArgumentNullException.ThrowIfNull(surface);

        Dictionary<string, List<Transform3D>> batched = [];
        int placed = 0, missing = 0;
        foreach (SceneryPlacement prop in plan.Placements())
        {
            Prop? found = Resolve(prop.Prop);
            if (found is null)
            {
                missing++;
                continue;
            }
            Transform3D where = Stand(prop, surface);
            if (found.Value.Mesh is not null)
            {
                if (!batched.TryGetValue(prop.Prop, out List<Transform3D>? group))
                    batched[prop.Prop] = group = [];
                group.Add(where);
            }
            else
            {
                Node3D node = found.Value.Scene!.Instantiate<Node3D>();
                node.Transform = where;
                node.Name = $"{prop.Prop}_{placed}";
                AddChild(node);
            }
            placed++;
        }

        foreach ((string prop, List<Transform3D> group) in batched)
        {
            Mesh mesh = Resolve(prop)!.Value.Mesh!;
            if (group.Count < BatchFrom)
            {
                for (int i = 0; i < group.Count; i++)
                {
                    AddChild(new MeshInstance3D
                    {
                        Name = $"{prop}_{i}",
                        Mesh = mesh,
                        Transform = group[i]
                    });
                }
                continue;
            }
            MultiMesh batch = new()
            {
                TransformFormat = MultiMesh.TransformFormatEnum.Transform3D,
                Mesh = mesh,
                InstanceCount = group.Count
            };
            for (int i = 0; i < group.Count; i++)
                batch.SetInstanceTransform(i, group[i]);
            AddChild(new MultiMeshInstance3D { Name = $"{prop}_batch", Multimesh = batch });
        }

        GD.Print(
            $"SCENERY {plan.Track}: {placed} props" +
            (missing > 0 ? $", {missing} not in the library" : "") +
            $"; library {LibraryPath}"
        );
    }

    /// <summary>
    /// One prop, under a node of the caller's choosing: what the edit mode
    /// uses to show what it has just placed, without rebuilding a plan.
    /// </summary>
    /// <summary>
    /// Where a prop stands: along the centreline to its station, out to
    /// the side by its offset, on the ground the surface describes, turned
    /// either with the road or with the world.
    /// </summary>
    public static Transform3D Stand(
        SceneryPlacement prop, TrackSurfaceGeometry surface
    )
    {
        float s = surface.Track.WrapS(prop.S);
        var sample = surface.Track.Sample(s);
        var position = sample.Center + sample.Normal * prop.D;
        float height = MathF.Abs(prop.D) <= sample.HalfWidth
            ? surface.Height(s, prop.D)
            : surface.MeadowHeight(position.X, position.Y);
        float facing = prop.AlignToTrack
            ? -MathF.Atan2(sample.Tangent.Y, sample.Tangent.X) + prop.Yaw
            : prop.Yaw;
        Basis basis = new Basis(Vector3.Up, facing)
            .Scaled(Vector3.One * MathF.Max(prop.Scale, 0.01f));
        return new Transform3D(
            basis,
            new Vector3(position.X, height + prop.Height, position.Y)
        );
    }

    /// <summary>
    /// The prop's file: a catalogue number, a name in the library, or a
    /// path of its own. Looked up once and remembered, including the
    /// misses, so a plan with fifty of the same tree reads one file.
    /// </summary>
    private Prop? Resolve(string prop)
    {
        if (_library.TryGetValue(prop, out Prop? found))
            return found;

        Prop? scene = Load(prop);
        if (scene is null)
        {
            GD.PushWarning(
                $"scenery: nothing to load for prop '{prop}' -- not a " +
                $"catalogue number, not in {LibraryPath}, not a path"
            );
        }
        _library[prop] = scene;
        return scene;
    }

    private Prop? Load(string prop)
    {
        // A number is the game's own prop, through the catalogue, so that
        // the number outlives the file it currently points at.
        if (int.TryParse(prop, out _))
        {
            string? named = Catalogue().GetValueOrDefault(prop);
            return named is null ? null : ByName(named);
        }
        // A path is somebody's own asset, loaded exactly as written.
        if (prop.Contains("://", StringComparison.Ordinal) ||
            prop.Contains('/', StringComparison.Ordinal))
        {
            return At(prop);
        }
        return ByName(prop);
    }

    /// <summary>
    /// Every format the library takes, in the order they are tried.
    /// Godot's own scene formats load as they are; everything else — glTF,
    /// Wavefront, COLLADA, FBX — is whatever the editor imported it as,
    /// which is a scene for some and a bare mesh for others. Both are
    /// accepted, so the answer to "can it load my model" does not depend
    /// on which of the two the importer chose.
    /// </summary>
    private static readonly string[] Extensions =
        [".tscn", ".scn", ".glb", ".gltf", ".obj", ".dae", ".fbx", ".blend", ".res"];

    private static Prop? ByName(string name)
    {
        foreach (string extension in Extensions)
        {
            if (At($"{LibraryPath}/{name}{extension}") is Prop prop)
                return prop;
        }
        return null;
    }

    /// <summary>
    /// A prop at a path, however the importer left it: a scene, or a mesh
    /// with nothing around it.
    /// </summary>
    private static Prop? At(string path)
    {
        if (!ResourceLoader.Exists(path))
            return null;
        Resource? resource = ResourceLoader.Load(path);
        return resource switch
        {
            PackedScene scene => SingleMesh(scene) is Mesh only
                ? new Prop(null, only)   // batch it: it is one mesh anyway
                : new Prop(scene, null),
            Mesh mesh => new Prop(null, mesh),
            _ => null
        };
    }

    /// <summary>
    /// A loaded prop: a scene to instance, or a mesh to batch. Never both,
    /// and a mesh is preferred wherever a scene turns out to be one.
    /// </summary>
    private readonly record struct Prop(PackedScene? Scene, Mesh? Mesh);

    private Dictionary<string, string> Catalogue()
    {
        if (_catalogue is not null)
            return _catalogue;
        _catalogue = [];
        if (!Godot.FileAccess.FileExists(CataloguePath))
            return _catalogue;
        try
        {
            using var file = Godot.FileAccess.Open(
                CataloguePath, Godot.FileAccess.ModeFlags.Read
            );
            foreach ((string number, string name) in
                     SceneryCatalogue.Parse(file.GetAsText()))
            {
                _catalogue[number] = name;
            }
        }
        catch (Exception error)
        {
            GD.PushWarning(
                $"scenery: {CataloguePath} could not be read -- {error.Message}"
            );
        }
        return _catalogue;
    }

    /// <summary>
    /// The mesh of a prop that is nothing but a mesh, or null for a prop
    /// with a scene's worth of parts. Only the first kind can be batched.
    /// </summary>
    private static Mesh? SingleMesh(PackedScene scene)
    {
        SceneState state = scene.GetState();
        if (state.GetNodeCount() != 1)
            return null;
        if (state.GetNodeType(0) != "MeshInstance3D")
            return null;
        for (int i = 0; i < state.GetNodePropertyCount(0); i++)
        {
            if (state.GetNodePropertyName(0, i) == "mesh")
                return state.GetNodePropertyValue(0, i).As<Mesh>();
        }
        return null;
    }
}
