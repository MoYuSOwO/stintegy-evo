using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.GodotApp.LowPoly;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// Puts a plan's props on the ground.
///
/// The library is a directory of Godot scenes and models: drop
/// <c>oak.glb</c> into <see cref="LibraryPath"/> and a plan may stand an
/// "oak" beside any corner. Nothing is registered, compiled in or listed
/// anywhere else — a prop exists because its file does.
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
    /// How many of the same single-mesh prop it takes before they are
    /// drawn as one batch instead of separate nodes.
    /// </summary>
    private const int BatchFrom = 8;

    private readonly Dictionary<string, PackedScene?> _library = [];

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
            PackedScene? scene = Resolve(prop.Prop);
            if (scene is null)
            {
                missing++;
                continue;
            }
            Transform3D where = Stand(prop, surface);
            if (SingleMesh(scene) is not null)
            {
                if (!batched.TryGetValue(prop.Prop, out List<Transform3D>? group))
                    batched[prop.Prop] = group = [];
                group.Add(where);
            }
            else
            {
                Node3D node = scene.Instantiate<Node3D>();
                node.Transform = where;
                node.Name = $"{prop.Prop}_{placed}";
                AddChild(node);
            }
            placed++;
        }

        foreach ((string prop, List<Transform3D> group) in batched)
        {
            Mesh mesh = SingleMesh(Resolve(prop)!)!;
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
    public void Show(
        SceneryPlacement prop, TrackSurfaceGeometry surface, Node3D under
    )
    {
        PackedScene? scene = Resolve(prop.Prop);
        if (scene is null)
            return;
        Node3D node = scene.Instantiate<Node3D>();
        node.Transform = Stand(prop, surface);
        under.AddChild(node);
    }

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
    /// The prop's file, if the library has one: a Godot scene or a model,
    /// by that name. Looked up once and remembered, including the misses.
    /// </summary>
    private PackedScene? Resolve(string prop)
    {
        if (_library.TryGetValue(prop, out PackedScene? found))
            return found;

        PackedScene? scene = null;
        foreach (string extension in new[] { ".tscn", ".scn", ".glb", ".gltf" })
        {
            string path = $"{LibraryPath}/{prop}{extension}";
            if (!ResourceLoader.Exists(path))
                continue;
            scene = ResourceLoader.Load<PackedScene>(path);
            if (scene is not null)
                break;
        }
        if (scene is null)
            GD.PushWarning($"scenery: no prop named '{prop}' in {LibraryPath}");
        _library[prop] = scene;
        return scene;
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
