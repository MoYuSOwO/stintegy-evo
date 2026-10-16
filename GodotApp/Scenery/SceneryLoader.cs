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
    public void Build(SceneryPlan plan, TrackSurfaceGeometry surface) =>
        Build(plan, surface, null);

    /// <summary>
    /// The same, for a plan that came from a mod folder: its prop paths
    /// are relative to that folder, so the mod can be moved or renamed
    /// without rewriting a line of it.
    /// </summary>
    public void Build(SceneryPlan plan, TrackSurfaceGeometry surface, string? folder)
    {
        ArgumentNullException.ThrowIfNull(plan);
        ArgumentNullException.ThrowIfNull(surface);

        Dictionary<string, List<Transform3D>> batched = [];
        int placed = 0, missing = 0, kept = 0;
        IEnumerable<SceneryPlacement> placements = plan.Placements();
        if (plan.Farmland is SceneryFarmland farm)
            placements = System.Linq.Enumerable.Concat(placements, BuildFarmland(plan, farm, surface));
        foreach (SceneryPlacement prop in placements)
        {
            // Scenery never stands on the road or the run-off: a wood
            // scattered across a band that clips a corner loses the trees
            // that would have been inside the barrier, and nothing else.
            if (InsideBarrier(prop, surface))
            {
                kept++;
                continue;
            }
            Prop? found = Resolve(prop.Prop, folder);
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
                group.Add(where * found.Value.Local);
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
            Mesh mesh = Resolve(prop, folder)!.Value.Mesh!;
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

        BuildPatches(plan, surface);

        GD.Print(
            $"SCENERY {plan.Track}: {placed} props" +
            (missing > 0 ? $", {missing} not in the library" : "") +
            (kept > 0 ? $", {kept} kept off the circuit" : "") +
            (plan.Patches.Count > 0 ? $", {plan.Patches.Count} ground patches" : "") +
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
    /// <summary>Clearance kept between the barrier and anything standing behind it.</summary>
    public const float BarrierClearanceMeters = 2.5f;

    /// <summary>
    /// Whether a placement would stand inside the barrier of any part of
    /// the circuit — not only the part it was placed beside. A lap folds
    /// back on itself, and a wood three hundred metres off one straight can
    /// be thirty from another; the nearest station decides, from anywhere
    /// on the map.
    /// </summary>
    public static bool InsideBarrier(SceneryPlacement prop, TrackSurfaceGeometry surface)
    {
        // Overhead structures are authored with a height and may span the road.
        if (prop.Height > 0f)
            return false;
        System.Numerics.Vector2 point;
        if (prop.World is Vector3 world)
            point = new System.Numerics.Vector2(world.X, world.Z);
        else
        {
            var own = surface.Track.Sample(surface.Track.WrapS(prop.S));
            point = own.Center + own.Normal * prop.D;
        }
        (float s, float d) = surface.NearestStation(point);
        var sample = surface.Track.Sample(s);
        float buffer = d >= 0f ? sample.LeftBufferWidth : sample.RightBufferWidth;
        return MathF.Abs(d) < sample.HalfWidth + buffer + BarrierClearanceMeters;
    }

    /// <summary>
    /// Ground colour laid on the meadow in the circuit's frame, a hand's
    /// breadth above it so the two do not fight, and never inside the
    /// barrier: a field that is drawn across a corner stops at the fence.
    /// </summary>
    /// <summary>
    /// Lays the farmland and returns its hedges and trees to be placed like
    /// any other props. The parcels are a turned grid in the world, not
    /// bands that follow the road, because fields were there before the
    /// circuit; each is cut into small cells that sit on the meadow, and
    /// the cells too near the circuit — of any part of it — are left as
    /// grass, so the patchwork stops short of the fence.
    /// </summary>
    private List<SceneryPlacement> BuildFarmland(SceneryPlan plan, SceneryFarmland farm, TrackSurfaceGeometry surface)
    {
        var props = new List<SceneryPlacement>();
        if (farm.Palette.Count == 0)
            return props;
        var random = new Random(unchecked(plan.Track.GetHashCode(StringComparison.Ordinal) * 31 + 17));
        Vector3 min = LowPolyMesh.V(surface.Minimum), max = LowPolyMesh.V(surface.Maximum);
        Vector2 center = new((min.X + max.X) * 0.5f, (min.Z + max.Z) * 0.5f);
        float reach = new Vector2(max.X - min.X, max.Z - min.Z).Length() * 0.5f + farm.Outer;
        float field = MathF.Max(farm.Field, 30f);
        int n = (int)MathF.Ceiling(reach / field);
        Vector2 u = new(MathF.Cos(farm.Angle), MathF.Sin(farm.Angle)), v = new(-u.Y, u.X);
        var mesh = new LowPolyMesh();
        const int split = 4;

        float Clear(Vector2 p)
        {
            (float s, float d) = surface.NearestStation(new System.Numerics.Vector2(p.X, p.Y));
            return MathF.Abs(d);
        }
        Vector3 Ground(Vector2 p) => new(p.X, surface.MeadowHeight(p.X, p.Y) + 0.05f, p.Y);

        for (int i = -n; i < n; i++)
        {
            for (int j = -n; j < n; j++)
            {
                // Parcels vary in size a little: the grid is nudged per parcel.
                float w = field * (0.8f + 0.4f * (float)random.NextDouble());
                float h = field * (0.8f + 0.4f * (float)random.NextDouble());
                Vector2 origin = center + u * (i * field) + v * (j * field);
                bool meadow = random.NextDouble() < farm.Meadow;
                Color color = Color.FromHtml(farm.Palette[random.Next(farm.Palette.Count)]);
                bool any = false;
                for (int a = 0; a < split; a++)
                {
                    for (int b = 0; b < split; b++)
                    {
                        Vector2 p0 = origin + u * (w * a / split) + v * (h * b / split);
                        Vector2 p1 = origin + u * (w * (a + 1) / split) + v * (h * b / split);
                        Vector2 p2 = origin + u * (w * (a + 1) / split) + v * (h * (b + 1) / split);
                        Vector2 p3 = origin + u * (w * a / split) + v * (h * (b + 1) / split);
                        Vector2 mid = (p0 + p2) * 0.5f;
                        float clear = Clear(mid);
                        if (clear < farm.Inner || clear > farm.Outer)
                            continue;
                        any = true;
                        if (!meadow)
                            mesh.GroundQuad(Ground(p0), Ground(p1), Ground(p2), Ground(p3), color);
                    }
                }
                if (!any)
                    continue;
                // Hedges along two edges of every parcel make the grid of them.
                if (!string.IsNullOrEmpty(farm.Hedge))
                {
                    foreach ((Vector2 from, Vector2 dir, float length) in new[] { (origin, u, w), (origin, v, h) })
                    {
                        for (float x = 5f; x < length - 5f; x += 10f)
                        {
                            Vector2 p = from + dir * x;
                            float clear = Clear(p);
                            if (clear < farm.Inner || clear > farm.Outer)
                                continue;
                            props.Add(new SceneryPlacement(farm.Hedge, 0f, 0f) with { World = new Vector3(p.X, 0f, p.Y), Yaw = -MathF.Atan2(dir.Y, dir.X) });
                        }
                    }
                }
                if (!string.IsNullOrEmpty(farm.Tree) && random.NextDouble() < 0.35)
                {
                    Vector2 p = origin + u * (w * (float)random.NextDouble()) + v * (h * 0.02f);
                    if (Clear(p) >= farm.Inner)
                        props.Add(new SceneryPlacement(farm.Tree, 0f, 0f) with { World = new Vector3(p.X, 0f, p.Y), Yaw = (float)random.NextDouble() * MathF.Tau, Scale = 0.8f + 0.5f * (float)random.NextDouble() });
                }
            }
        }
        var instance = mesh.Instance("Farmland");
        instance.CastShadow = GeometryInstance3D.ShadowCastingSetting.Off;
        AddChild(instance);
        return props;
    }

    private void BuildPatches(SceneryPlan plan, TrackSurfaceGeometry surface)
    {
        if (plan.Patches.Count == 0)
            return;
        var mesh = new LowPolyMesh();
        const float cell = 8f;
        foreach (SceneryPatch patch in plan.Patches)
        {
            Color color = Color.FromHtml(patch.Color);
            int ns = Math.Max(1, (int)MathF.Ceiling(MathF.Abs(patch.SEnd - patch.SStart) / cell));
            int nd = Math.Max(1, (int)MathF.Ceiling(MathF.Abs(patch.DOuter - patch.DInner) / cell));
            for (int i = 0; i < ns; i++)
            {
                float s0 = patch.SStart + (patch.SEnd - patch.SStart) * i / ns;
                float s1 = patch.SStart + (patch.SEnd - patch.SStart) * (i + 1) / ns;
                for (int j = 0; j < nd; j++)
                {
                    float d0 = patch.DInner + (patch.DOuter - patch.DInner) * j / nd;
                    float d1 = patch.DInner + (patch.DOuter - patch.DInner) * (j + 1) / nd;
                    var corner = new SceneryPlacement(string.Empty, (s0 + s1) * 0.5f, (d0 + d1) * 0.5f);
                    if (InsideBarrier(corner, surface))
                        continue;
                    Vector3 Ground(float s, float d)
                    {
                        float wrapped = surface.Track.WrapS(s);
                        var sample = surface.Track.Sample(wrapped);
                        var p = sample.Center + sample.Normal * d;
                        return new Vector3(p.X, surface.MeadowHeight(p.X, p.Y) + 0.06f, p.Y);
                    }
                    // Far out on the inside of a bend the band folds back over
                    // itself; a cell whose edge runs against the road there is
                    // one of the folded ones, and is left out.
                    Vector3 along = Ground(s1, d1) - Ground(s0, d1);
                    Vector3 road = Ground(s1, 0f) - Ground(s0, 0f);
                    if (along.Dot(road) < 0.25f * along.Length() * road.Length())
                        continue;
                    mesh.GroundQuad(Ground(s0, d0), Ground(s1, d0), Ground(s1, d1), Ground(s0, d1), color);
                }
            }
        }
        var instance = mesh.Instance("Patches");
        instance.CastShadow = GeometryInstance3D.ShadowCastingSetting.Off;
        AddChild(instance);
    }

    public static Transform3D Stand(
        SceneryPlacement prop, TrackSurfaceGeometry surface
    )
    {
        if (prop.World is Vector3 world)
        {
            // Placed in the world rather than the circuit's frame: generated
            // countryside, which does not care where the road goes.
            Basis turned = new Basis(Vector3.Up, prop.Yaw).Scaled(Vector3.One * MathF.Max(prop.Scale, 0.01f));
            return new Transform3D(turned, new Vector3(world.X, surface.MeadowHeight(world.X, world.Z) + prop.Height, world.Z));
        }
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
    private Prop? Resolve(string prop, string? folder)
    {
        string key = folder is null ? prop : $"{folder}\u0000{prop}";
        if (_library.TryGetValue(key, out Prop? found))
            return found;

        Prop? scene = Load(prop, folder);
        if (scene is null)
        {
            GD.PushWarning(
                $"scenery: nothing to load for prop '{prop}' -- not a " +
                $"catalogue number, not in {LibraryPath}, not a path"
            );
        }
        _library[key] = scene;
        return scene;
    }

    private Prop? Load(string prop, string? folder)
    {
        // A mod's plan names files inside its own folder, and those files
        // were never imported -- there is no editor in a shipped game --
        // so they are read from disk by the runtime reader instead. A mod
        // that asks for a catalogue number or a library name is asking for
        // the game's own prop and falls through to it: a mod may put a
        // tree where the circuit has none without shipping a tree.
        if (folder is not null && (
                prop.Contains('/', StringComparison.Ordinal) ||
                prop.Contains('.', StringComparison.Ordinal)))
        {
            string path = prop.Contains("://", StringComparison.Ordinal)
                ? prop
                : $"{folder}/{prop}";
            PackedScene? model = SceneryMods.LoadModel(path);
            if (model is null)
                return null;
            return SingleMesh(model) is (Mesh only, Transform3D local)
                ? Prop.Batched(only, local)
                : MergedMesh(model) is Mesh merged
                    ? Prop.Batched(merged)
                    : new Prop(model, null);
        }

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
            PackedScene scene => SingleMesh(scene) is (Mesh only, Transform3D local)
                ? Prop.Batched(only, local)   // batch it: it is one mesh anyway
                : MergedMesh(scene) is Mesh merged
                    ? Prop.Batched(merged)    // several meshes and nothing else: one mesh now
                    : new Prop(scene, null),
            Mesh mesh => Prop.Batched(mesh),
            _ => null
        };
    }

    /// <summary>
    /// A loaded prop: a scene to instance, or a mesh to batch. Never both,
    /// and a mesh is preferred wherever a scene turns out to be one.
    /// </summary>
    /// <summary>
    /// A loaded prop: a scene to instance, or one mesh to batch together
    /// with the transform its node gave it. The pine's cone is lifted half
    /// its height by its node; batching the bare mesh dropped that and left
    /// every batched tree half in the ground.
    /// </summary>
    private readonly record struct Prop(PackedScene? Scene, Mesh? Mesh, Transform3D Local)
    {
        public Prop(PackedScene? scene, Mesh? mesh) : this(scene, mesh, Transform3D.Identity) { }

        /// <summary>
        /// A batched prop, re-cut flat-shaded: a prop made from Godot's
        /// smooth primitives, or modelled smooth by a modder, still sits in
        /// the circuit's faceted style.
        /// </summary>
        public static Prop Batched(Mesh mesh, Transform3D local) => new(null, LowPolyMesh.Faceted(mesh), local);

        public static Prop Batched(Mesh mesh) => Batched(mesh, Transform3D.Identity);
    }

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
    /// <summary>
    /// A prop made of a few meshes and nothing else — a trunk and a crown,
    /// walls and a roof — as one mesh with a surface per part, so a wood of
    /// them is one batch instead of two thousand nodes. Anything that is
    /// more than meshes (a script, a light, an animation) stays a scene.
    /// </summary>
    private static Mesh? MergedMesh(PackedScene scene)
    {
        Node root = scene.Instantiate();
        try
        {
            if (root is not Node3D || root.GetScript().VariantType != Variant.Type.Nil)
                return null;
            var merged = new ArrayMesh();
            bool ok = Collect(root, Transform3D.Identity, merged, isRoot: true);
            return ok && merged.GetSurfaceCount() > 0 ? merged : null;
        }
        finally
        {
            root.Free();
        }
    }

    private static bool Collect(Node node, Transform3D parent, ArrayMesh merged, bool isRoot)
    {
        if (node.GetScript().VariantType != Variant.Type.Nil)
            return false;
        Transform3D here = node is Node3D spatial && !isRoot ? parent * spatial.Transform : parent;
        if (node is MeshInstance3D instance)
        {
            if (instance.Mesh is not Mesh mesh)
                return false;
            for (int i = 0; i < mesh.GetSurfaceCount(); i++)
            {
                Godot.Collections.Array arrays = mesh.SurfaceGetArrays(i);
                var vertices = arrays[(int)Mesh.ArrayType.Vertex].AsVector3Array();
                for (int v = 0; v < vertices.Length; v++)
                    vertices[v] = here * vertices[v];
                arrays[(int)Mesh.ArrayType.Vertex] = vertices;
                if (arrays[(int)Mesh.ArrayType.Normal].VariantType != Variant.Type.Nil)
                {
                    var normals = arrays[(int)Mesh.ArrayType.Normal].AsVector3Array();
                    for (int v = 0; v < normals.Length; v++)
                        normals[v] = (here.Basis * normals[v]).Normalized();
                    arrays[(int)Mesh.ArrayType.Normal] = normals;
                }
                // Tangents would need the same turn; the props here have none worth keeping.
                arrays[(int)Mesh.ArrayType.Tangent] = default;
                merged.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);
                Material? material = instance.GetSurfaceOverrideMaterial(i) ?? mesh.SurfaceGetMaterial(i)
                    ?? (mesh as PrimitiveMesh)?.Material;
                if (material is not null)
                    merged.SurfaceSetMaterial(merged.GetSurfaceCount() - 1, material);
            }
        }
        else if (!isRoot && node.GetType() != typeof(Node3D))
        {
            return false;
        }
        foreach (Node child in node.GetChildren())
        {
            if (!Collect(child, here, merged, isRoot: false))
                return false;
        }
        return true;
    }

    private static (Mesh, Transform3D)? SingleMesh(PackedScene scene)
    {
        SceneState state = scene.GetState();
        if (state.GetNodeCount() != 1)
            return null;
        if (state.GetNodeType(0) != "MeshInstance3D")
            return null;
        Mesh? mesh = null;
        Transform3D local = Transform3D.Identity;
        for (int i = 0; i < state.GetNodePropertyCount(0); i++)
        {
            string name = state.GetNodePropertyName(0, i);
            if (name == "mesh")
                mesh = state.GetNodePropertyValue(0, i).As<Mesh>();
            else if (name == "transform")
                local = state.GetNodePropertyValue(0, i).As<Transform3D>();
        }
        return mesh is null ? null : (mesh, local);
    }

}
