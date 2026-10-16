using System;
using Godot;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>
/// Everything past the circuit's own ground: a plain the meadow sits on, two
/// rings of hills on the horizon and clouds overhead.
///
/// None of it is the circuit. It is there so that the edge of the world is
/// somewhere the camera does not find: the meadow is a rectangle a few
/// hundred metres past the barrier, and beyond it there used to be nothing
/// but the background colour. Distance does the rest — the fog folds the far
/// ring into the sky, so it reads as country going on rather than as a wall.
/// </summary>
public partial class Landscape3D : Node3D
{
    private static readonly Color Cloud = Color.FromHtml("#f4f2ea");
    private Color Plain, NearHill, FarRidge;
    private int _cloudCount;

    private Node3D _clouds = null!;
    private Vector3 _drift = new(1.6f, 0f, 0.7f);
    private float _cloudSpan;
    private Vector3 _center;

    public void Initialize(TrackSurfaceGeometry surface, Scenery.CircuitLook look)
    {
        Plain = look.Plain;
        NearHill = look.NearHills;
        FarRidge = look.FarHills;
        _cloudCount = look.Clouds;
        Vector3 min = LowPolyMesh.V(surface.Minimum), max = LowPolyMesh.V(surface.Maximum);
        _center = (min + max) * 0.5f;
        // The meadow reaches this far past the circuit (CircuitView3D's terrain margin).
        float halfDiagonal = new Vector2(max.X - min.X, max.Z - min.Z).Length() * 0.5f + 650f;
        float ground = min.Y - 1.5f;
        var random = new Random(unchecked((int)(halfDiagonal * 31f)));

        BuildPlain(ground);
        BuildHills(halfDiagonal + 250f, ground, random);
        BuildClouds(halfDiagonal, ground, random);
    }

    public void ShowClouds(bool show)
    {
        if (_clouds is not null)
            _clouds.Visible = show;
    }

    public override void _Process(double delta)
    {
        if (_clouds is null)
            return;
        // Clouds cross the circuit slowly and come back round the far side.
        foreach (Node child in _clouds.GetChildren())
        {
            if (child is not Node3D cloud)
                continue;
            Vector3 p = cloud.Position + _drift * (float)delta;
            if (p.X - _center.X > _cloudSpan) p.X -= 2f * _cloudSpan;
            if (p.Z - _center.Z > _cloudSpan) p.Z -= 2f * _cloudSpan;
            cloud.Position = p;
        }
    }

    private void BuildPlain(float ground)
    {
        var mesh = new LowPolyMesh();
        const float half = 22000f;
        Vector3 c = new(_center.X, ground, _center.Z);
        mesh.GroundQuad(c + new Vector3(-half, 0, -half), c + new Vector3(half, 0, -half),
            c + new Vector3(half, 0, half), c + new Vector3(-half, 0, half), Plain);
        var instance = mesh.Instance("Plain");
        instance.CastShadow = GeometryInstance3D.ShadowCastingSetting.Off;
        AddChild(instance);
    }

    /// <summary>
    /// Two rings: low green hills close in and a taller, bluer ridge behind
    /// them. Heights come from a few sines summed, so the skyline rolls
    /// instead of zig-zagging.
    /// </summary>
    private void BuildHills(float radius, float ground, Random random)
    {
        const int segments = 128;
        float phaseA = (float)random.NextDouble() * Mathf.Tau, phaseB = (float)random.NextDouble() * Mathf.Tau;
        float Height(float angle, float scale, float baseHeight) =>
            baseHeight + scale * (0.55f + 0.25f * MathF.Sin(3f * angle + phaseA) + 0.15f * MathF.Sin(7f * angle + phaseB) + 0.05f * MathF.Sin(17f * angle));

        Vector3 Ring(float angle, float r, float y) =>
            new(_center.X + MathF.Cos(angle) * r, y, _center.Z + MathF.Sin(angle) * r);

        var near = new LowPolyMesh();
        var far = new LowPolyMesh();
        for (int i = 0; i < segments; i++)
        {
            float a = i * Mathf.Tau / segments, b = (i + 1) * Mathf.Tau / segments;
            Color shade = NearHill.Lightened(0.04f * MathF.Sin(i * 0.9f));

            // Near ring: up from the plain to a crest, down the far side.
            Vector3 innerA = Ring(a, radius, ground), innerB = Ring(b, radius, ground);
            Vector3 crestA = Ring(a, radius + 260f, ground + Height(a, 110f, 25f));
            Vector3 crestB = Ring(b, radius + 260f, ground + Height(b, 110f, 25f));
            Vector3 backA = Ring(a, radius + 520f, ground), backB = Ring(b, radius + 520f, ground);
            near.Quad(innerB, innerA, crestA, crestB, shade);
            near.Quad(crestB, crestA, backA, backB, shade.Darkened(0.08f));

            // Far ridge, behind and higher, coloured towards the haze.
            Vector3 ridgeA = Ring(a, radius + 1400f, ground + Height(a + 1.3f, 260f, 90f));
            Vector3 ridgeB = Ring(b, radius + 1400f, ground + Height(b + 1.3f, 260f, 90f));
            Vector3 footA = Ring(a, radius + 700f, ground), footB = Ring(b, radius + 700f, ground);
            Vector3 endA = Ring(a, radius + 2200f, ground), endB = Ring(b, radius + 2200f, ground);
            far.Quad(footB, footA, ridgeA, ridgeB, FarRidge);
            far.Quad(ridgeB, ridgeA, endA, endB, FarRidge.Darkened(0.06f));
        }
        foreach ((LowPolyMesh mesh, string name) in new[] { (near, "NearHills"), (far, "FarRidge") })
        {
            var instance = mesh.Instance(name);
            instance.CastShadow = GeometryInstance3D.ShadowCastingSetting.Off;
            AddChild(instance);
        }
    }

    /// <summary>
    /// A few dozen low clouds, each a handful of flattened faceted blobs, at
    /// heights a camera at the trackside sees above the hills.
    /// </summary>
    private void BuildClouds(float spread, float ground, Random random)
    {
        _clouds = new Node3D { Name = "Clouds" };
        AddChild(_clouds);
        _cloudSpan = spread + 1500f;
        for (int k = 0; k < _cloudCount; k++)
        {
            var mesh = new LowPolyMesh();
            int blobs = 3 + random.Next(4);
            float size = 50f + (float)random.NextDouble() * 70f;
            for (int j = 0; j < blobs; j++)
            {
                Vector3 offset = new(((float)random.NextDouble() - 0.5f) * size * 2.2f, (float)random.NextDouble() * size * 0.25f,
                    ((float)random.NextDouble() - 0.5f) * size * 1.2f);
                Blob(mesh, offset, size * (0.55f + (float)random.NextDouble() * 0.5f), 0.45f, Cloud.Darkened(0.03f * j));
            }
            var cloud = mesh.Instance($"Cloud_{k}");
            cloud.CastShadow = GeometryInstance3D.ShadowCastingSetting.Off;
            cloud.Position = new Vector3(
                _center.X + ((float)random.NextDouble() * 2f - 1f) * _cloudSpan,
                ground + 520f + (float)random.NextDouble() * 380f,
                _center.Z + ((float)random.NextDouble() * 2f - 1f) * _cloudSpan);
            _clouds.AddChild(cloud);
        }
    }

    /// <summary>A coarse faceted sphere, squashed: six around, three down.</summary>
    private static void Blob(LowPolyMesh mesh, Vector3 center, float radius, float squash, Color color)
    {
        const int sides = 6, rings = 3;
        Vector3 P(int i, int j)
        {
            float lat = Mathf.Pi * j / rings - Mathf.Pi * 0.5f;
            float lon = Mathf.Tau * i / sides + (j % 2) * Mathf.Pi / sides;
            return center + new Vector3(MathF.Cos(lat) * MathF.Cos(lon) * radius, MathF.Sin(lat) * radius * squash, MathF.Cos(lat) * MathF.Sin(lon) * radius);
        }
        for (int j = 0; j < rings; j++)
        {
            for (int i = 0; i < sides; i++)
            {
                Vector3 a = P(i, j), b = P(i + 1, j), c = P(i + 1, j + 1), d = P(i, j + 1);
                // Outward faces: the mesh helper winds clockwise for Godot from these.
                mesh.Triangle(a, d, c, color);
                mesh.Triangle(a, c, b, color);
            }
        }
    }
}
