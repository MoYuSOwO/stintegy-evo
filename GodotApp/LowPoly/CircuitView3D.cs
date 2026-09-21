using System;
using Godot;
using StintegyEVO.Core.Track;
using NVector2 = System.Numerics.Vector2;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class CircuitView3D : Node3D
{
    public TrackSurfaceGeometry Surface { get; private set; } = null!;
    private static readonly Color Road = Color.FromHtml("#535b5d"), Runoff = Color.FromHtml("#b4ab8e"), Grass = Color.FromHtml("#89976c");
    public void Initialize(TrackData track) => Initialize(track, null);

    /// <summary>
    /// Builds the circuit, and its scenery when a plan is named: the road
    /// comes from the domain, the trees and grandstands from a file
    /// somebody wrote. A plan that is not there is not an error — a
    /// circuit with no scenery is a circuit with no scenery.
    /// </summary>
    public void Initialize(TrackData track, string? sceneryPlanPath)
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
                    // Run-off begins where the kerb ends, because the kerb is
                    // the first 0.6 m of the buffer and not a thing beside it
                    // (SurfaceGrip: racing surface to the line, kerb for
                    // KerbWidthMeters past it, then the buffer's own grip).
                    // The narrowest buffer there is equals the kerb, so a
                    // street circuit renders as kerb and then wall, with no
                    // run-off in between, which is what the grammar says.
                    float kerbA = ea + side * SurfaceGrip.KerbWidthMeters;
                    float kerbB = eb + side * SurfaceGrip.KerbWidthMeters;
                    if (wa - sa.HalfWidth > SurfaceGrip.KerbWidthMeters + 1e-3f)
                        Strip(mesh, a, b, kerbA, side * wa, kerbB, side * wb, Runoff, 0f);
                    // The white line is the last of the racing surface, not
                    // the first of the run-off: painted inside its own edge.
                    Strip(mesh, a, b, ea - side * 0.12f, ea, eb - side * 0.12f, eb, LowPolyMesh.Ivory, 0.03f);
                    Wall(mesh, a, b, side * wa, side * wb);
                    Verge(mesh, a, b, side * wa, side * wb, side);
                }
            }
            AddChild(mesh.Instance($"Road_{begin}"));
        }
        BuildKerbs();
        BuildMarkings();
        BuildScenery(sceneryPlanPath);
    }

    private void BuildScenery(string? planPath)
    {
        if (string.IsNullOrWhiteSpace(planPath))
            return;
        if (!Godot.FileAccess.FileExists(planPath))
            return;
        var scenery = new Scenery.SceneryLoader { Name = "Scenery" };
        AddChild(scenery);
        try
        {
            using var file = Godot.FileAccess.Open(planPath, Godot.FileAccess.ModeFlags.Read);
            var plan = Scenery.SceneryPlan.Parse(file.GetAsText());
            scenery.Build(plan, Surface);
        }
        catch (Exception error)
        {
            // A malformed plan costs the circuit its scenery and nothing
            // else: the road is the domain's, and it is already built.
            GD.PushWarning($"scenery: {planPath} could not be read -- {error.Message}");
        }
    }

    /// <summary>
    /// The kerb, everywhere, on both sides.
    ///
    /// The domain has one grammar for every edge of every circuit: racing
    /// surface up to the white line, then <see cref="SurfaceGrip.KerbWidthMeters"/>
    /// of kerb, then the buffer — and the narrowest buffer permitted is the
    /// kerb itself, so there is no case anywhere of a line with no kerb
    /// behind it. The renderer used to paint one only where the road turned
    /// hard enough, in a width it chose for itself, which drew a circuit
    /// that the physics does not agree exists.
    ///
    /// The stripes are a metre each, which is what makes it read as a kerb
    /// rather than a red line, and they are drawn in their own pass because
    /// the road's own segments are three metres long. The strip is flat:
    /// the grip ramp across it is real and priced in the physics, but a
    /// profile is not something a car here ever drives over.
    /// </summary>
    private void BuildKerbs()
    {
        TrackData track = Surface.Track;
        const float stripe = 1f;
        int stripes = (int)MathF.Ceiling(track.LengthMeters / stripe);
        float step = track.LengthMeters / stripes;
        for (int begin = 0; begin < stripes; begin += 240)
        {
            var mesh = new LowPolyMesh();
            for (int i = begin; i < Math.Min(begin + 240, stripes); i++)
            {
                float a = i * step, b = (i + 1) * step;
                float halfA = track.Sample(a).HalfWidth, halfB = track.Sample(b).HalfWidth;
                Color color = i % 2 == 0 ? LowPolyMesh.Vermilion : LowPolyMesh.Ivory;
                foreach (int side in new[] { -1, 1 })
                {
                    Strip(
                        mesh, a, b,
                        side * halfA, side * (halfA + SurfaceGrip.KerbWidthMeters),
                        side * halfB, side * (halfB + SurfaceGrip.KerbWidthMeters),
                        color, 0.035f
                    );
                }
            }
            AddChild(mesh.Instance($"Kerb_{begin}"));
        }
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
    /// <summary>
    /// The ground from the barrier out to where the meadow takes over.
    ///
    /// Its outer rim is put at exactly the height the meadow has at that
    /// point, rather than at a constant drop below the road: the two
    /// surfaces then meet instead of one hanging over the other, which is
    /// what a strip of sky under the grass at the edge of a climbing
    /// section was. The meadow itself runs unbroken beneath all of this —
    /// under the road as well — so nothing here can leave a hole.
    /// </summary>
    private void Verge(LowPolyMesh mesh, float a, float b, float da, float db, int side)
    {
        Vector3 p = Point(a, da), q = Point(b, db);
        Vector3 r = Point(b, db + side * TrackSurfaceGeometry.VergeMeters);
        Vector3 t = Point(a, da + side * TrackSurfaceGeometry.VergeMeters);
        r.Y = Surface.MeadowHeight(r.X, r.Z);
        t.Y = Surface.MeadowHeight(t.X, t.Z);
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
