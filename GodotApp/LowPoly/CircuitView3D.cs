using System;
using Godot;
using StintegyEVO.Core.Track;
using NVector2 = System.Numerics.Vector2;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class CircuitView3D : Node3D
{
    public TrackSurfaceGeometry Surface { get; private set; } = null!;
    private static readonly Color Road = Color.FromHtml("#535b5d"), Runoff = Color.FromHtml("#b4ab8e"), Grass = Color.FromHtml("#89976c");
    public void Initialize(TrackData track)
    {
        Surface = new(track);
        BuildTerrain();
        int segments = (int)MathF.Ceiling(track.LengthMeters / 3f);
        float step = track.LengthMeters / segments;
        // Chunked geometry keeps the long circuit cullable in the follow camera.
        for (int begin = 0; begin < segments; begin += 80)
        {
            var mesh = new LowPolyMesh();
            for (int i = begin; i < Math.Min(begin + 80, segments); i++)
            {
                float a = i * step, b = (i + 1) * step;
                var sa = track.Sample(a); var sb = track.Sample(b);
                for (int lane = 0; lane < 6; lane++)
                    Strip(mesh, a, b, Mathf.Lerp(-sa.HalfWidth, sa.HalfWidth, lane / 6f), Mathf.Lerp(-sa.HalfWidth, sa.HalfWidth, (lane + 1) / 6f),
                        Mathf.Lerp(-sb.HalfWidth, sb.HalfWidth, lane / 6f), Mathf.Lerp(-sb.HalfWidth, sb.HalfWidth, (lane + 1) / 6f), Road, 0.025f);
                foreach (int side in new[] { -1, 1 })
                {
                    float ea = side * sa.HalfWidth, eb = side * sb.HalfWidth;
                    float wa = sa.HalfWidth + (side == 1 ? sa.LeftBufferWidth : sa.RightBufferWidth);
                    float wb = sb.HalfWidth + (side == 1 ? sb.LeftBufferWidth : sb.RightBufferWidth);
                    Strip(mesh, a, b, ea, side * wa, eb, side * wb, Runoff, 0f);
                    Strip(mesh, a, b, ea, ea + side * 0.14f, eb, eb + side * 0.14f, LowPolyMesh.Ivory, 0.045f);
                    // Kerbs appear only where there is room and the road actually turns.
                    if (MathF.Abs(sa.RefCurvature) > 0.0012f)
                        Strip(mesh, a, b, ea + side * 0.18f, ea + side * 1.1f, eb + side * 0.18f, eb + side * 1.1f,
                            i % 2 == 0 ? LowPolyMesh.Vermilion : LowPolyMesh.Ivory, 0.065f);
                    Wall(mesh, a, b, side * wa, side * wb);
                    Verge(mesh, a, b, side * wa, side * wb, side);
                }
            }
            AddChild(mesh.Instance($"Road_{begin}"));
        }
        BuildMarkings();
        var scenery = new CircuitScenery(); AddChild(scenery); scenery.Build(Surface);
    }
    public Vector3 Point(float s, float d, float lift = 0f) => LowPolyMesh.V(Surface.Point(s, d)) + Vector3.Up * lift;
    private void Strip(LowPolyMesh mesh, float a, float b, float a0, float a1, float b0, float b1, Color color, float lift)
        => mesh.GroundQuad(Point(a, a0, lift), Point(a, a1, lift), Point(b, b1, lift), Point(b, b0, lift), color);
    private void Wall(LowPolyMesh mesh, float a, float b, float da, float db)
    {
        float side = MathF.Sign(da), thickness = 0.35f;
        Vector3 p = Point(a, da), q = Point(b, db), r = Point(b, db + side * thickness), s = Point(a, da + side * thickness);
        var up = Vector3.Up * 0.85f; var col = Color.FromHtml("#dedbc9");
        mesh.GroundQuad(p + up, q + up, r + up, s + up, col);
        if (side > 0) { mesh.Quad(p, q, q + up, p + up, col); mesh.Quad(r, s, s + up, r + up, col); }
        else { mesh.Quad(q, p, p + up, q + up, col); mesh.Quad(s, r, r + up, s + up, col); }
    }
    private void Verge(LowPolyMesh mesh, float a, float b, float da, float db, int side)
    {
        Vector3 p = Point(a, da), q = Point(b, db);
        Vector3 r = Point(b, db + side * 18f), t = Point(a, da + side * 18f);
        r.Y = Surface.Height(b, 0f) - 2.6f; t.Y = Surface.Height(a, 0f) - 2.6f;
        mesh.GroundQuad(p, q, r, t, Grass);
    }
    private void BuildMarkings()
    {
        var mesh = new LowPolyMesh(); var track = Surface.Track;
        var start = track.Sample(track.StartingLineS);
        int cols = (int)MathF.Ceiling(start.Width / 0.7f);
        for (int row = 0; row < 2; row++) for (int col = 0; col < cols; col++)
        {
            float d0 = -start.HalfWidth + col * start.Width / cols, d1 = -start.HalfWidth + (col + 1) * start.Width / cols;
            Strip(mesh, track.StartingLineS + row * 0.7f, track.StartingLineS + (row + 1) * 0.7f, d0, d1, d0, d1,
                (row + col) % 2 == 0 ? LowPolyMesh.Ivory : LowPolyMesh.Ink, 0.07f);
        }
        for (int i = 1; i <= track.StartingGridCount; i++)
        {
            var grid = track.Grids[i]; var sample = track.Sample(grid.S);
            float d = NVector2.Dot(grid.Position - sample.Center, sample.Normal);
            Strip(mesh, grid.S - 2.2f, grid.S + 2.2f, d - 1.15f, d - 1.04f, d - 1.15f, d - 1.04f, LowPolyMesh.Ivory, 0.06f);
            Strip(mesh, grid.S - 2.2f, grid.S + 2.2f, d + 1.04f, d + 1.15f, d + 1.04f, d + 1.15f, LowPolyMesh.Ivory, 0.06f);
            Strip(mesh, grid.S + 2.1f, grid.S + 2.2f, d - 1.15f, d + 1.15f, d - 1.15f, d + 1.15f, LowPolyMesh.Ivory, 0.06f);
        }
        AddChild(mesh.Instance("GridAndFinish"));
    }
    private void BuildTerrain()
    {
        var min = Surface.Minimum; var max = Surface.Maximum;
        const float spacing = 18f, margin = 650f;
        int nx = (int)MathF.Ceiling((max.X - min.X + margin * 2) / spacing), nz = (int)MathF.Ceiling((max.Z - min.Z + margin * 2) / spacing);
        var vertices = new Vector3[nx + 1, nz + 1];
        for (int x = 0; x <= nx; x++) for (int z = 0; z <= nz; z++)
        {
            float px = min.X - margin + x * spacing, pz = min.Z - margin + z * spacing;
            float height = Surface.MeadowHeight(px, pz);
            vertices[x, z] = new(px, height, pz);
        }
        for (int bx = 0; bx < nx; bx += 16) for (int bz = 0; bz < nz; bz += 16)
        {
            var mesh = new LowPolyMesh();
            for (int x = bx; x < Math.Min(bx + 16, nx); x++) for (int z = bz; z < Math.Min(bz + 16, nz); z++)
            {
                // Restrained field-sized variation rather than random rainbow triangles.
                float tint = 0.018f * MathF.Sin(x * 0.35f + z * 0.2f);
                var color = Grass.Lightened(tint);
                mesh.GroundQuad(vertices[x, z], vertices[x + 1, z], vertices[x + 1, z + 1], vertices[x, z + 1], color);
            }
            var instance = mesh.Instance($"Meadow_{bx}_{bz}"); instance.CastShadow = GeometryInstance3D.ShadowCastingSetting.Off; AddChild(instance);
        }
    }
}
