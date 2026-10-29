using System;
using Godot;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Track;
using StintegyEVO.GodotApp.Interop;
using NVector2 = System.Numerics.Vector2;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class CircuitView3D : Node3D
{
    public TrackSurfaceGeometry Surface { get; private set; } = null!;
    /// <summary>
    /// The circuit's own look — ground, run-off and barrier colours, and the
    /// sky and sun the race view lights it by — from its scenery plan's
    /// environment block, or the defaults where there is none.
    /// </summary>
    public Scenery.CircuitLook Look { get; private set; } = Scenery.CircuitLook.From(new Scenery.SceneryEnvironment());

    /// <summary>The plan this circuit was built with, if it had one.</summary>
    public Scenery.SceneryPlan? Plan { get; private set; }

    private const float CornerBarrierCurvature = 1f / 160f;
    public void Initialize(TrackData track) => Initialize(track, null);

    /// <summary>
    /// Builds the circuit, and its scenery when a plan is named: the road
    /// comes from the domain, the trees and grandstands from a file
    /// somebody wrote. A plan that is not there is not an error — a
    /// circuit with no scenery is a circuit with no scenery.
    /// </summary>
    public void Initialize(TrackData track, ModCircuit? circuit)
    {
        Surface = new(track);
        _registered = circuit;
        // Read first: the plan says what colour the ground is before any of it is built.
        Plan = circuit is null ? null : ReadPlan(circuit.Resources, ModResources.Join(circuit.Folder, Scenery.SceneryPlan.FileName));
        Look = Scenery.CircuitLook.From(Plan?.Environment ?? new Scenery.SceneryEnvironment());
        BuildTerrain();
        var landscape = new Landscape3D { Name = "Landscape" };
        AddChild(landscape);
        landscape.Initialize(Surface, Look);
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
                        Mathf.Lerp(-sb.HalfWidth, sb.HalfWidth, lane / 6f), Mathf.Lerp(-sb.HalfWidth, sb.HalfWidth, (lane + 1) / 6f), Look.Road, 0.025f);
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
                        Strip(mesh, a, b, kerbA, side * wa, kerbB, side * wb, Look.Mown[(i / 2) % Look.Mown.Count], 0f);
                    // The white line is the last of the racing surface, not
                    // the first of the run-off: painted inside its own edge.
                    Strip(mesh, a, b, ea - side * 0.12f, ea, eb - side * 0.12f, eb, LowPolyMesh.Ivory, 0.03f);
                    Wall(mesh, a, b, side * wa, side * wb, BarrierFace(track, i, a, side, wa));
                }
            }
            AddChild(mesh.Instance($"Road_{begin}"));
        }
        BuildKerbs();
        BuildMarkings();
        var dressing = new TrackDressing3D { Name = "Dressing" };
        AddChild(dressing);
        dressing.Initialize(this, Plan?.Furniture ?? new Scenery.SceneryFurniture());
        BuildScenery();
    }

    // A plan in a mod's files, read whether the mod is packed or not.
    private static Scenery.SceneryPlan? ReadPlan(IModResources mod, string path)
    {
        if (!mod.Exists(path))
            return null;
        try
        {
            return Scenery.SceneryPlan.Parse(mod.ReadAllText(path));
        }
        catch (Exception error)
        {
            // A malformed plan costs the circuit its scenery and its look,
            // and nothing else: the road is the domain's.
            GD.PushWarning($"scenery: {path} in {mod.Location} could not be read -- {error.Message}");
            return null;
        }
    }

    private ModCircuit? _registered;

    private void BuildScenery()
    {
        if (_registered is not { } circuit)
            return;
        var scenery = new Scenery.SceneryLoader { Name = "Scenery" };
        AddChild(scenery);
        try
        {
            if (Plan is not null)
                scenery.Build(Plan, Surface, new Scenery.SceneryFiles(circuit.Resources, circuit.Folder));
            // Then what other mods add to it, each from its own files, after
            // the circuit's own and in the order the mods load. It can no
            // more touch the simulation than the circuit's own can.
            foreach (ModSceneryOverlay overlay in GodotMods.Registry.SceneryFor(circuit.Id))
            {
                if (ReadPlan(overlay.Resources, overlay.File) is not { } plan)
                    continue;
                int slash = overlay.File.LastIndexOf('/');
                scenery.Build(plan, Surface, new Scenery.SceneryFiles(overlay.Resources, slash < 0 ? "" : overlay.File[..slash]));
            }
        }
        catch (Exception error)
        {
            GD.PushWarning($"scenery: {circuit.Id}'s scenery could not be built -- {error.Message}");
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
    private void Wall(LowPolyMesh mesh, float a, float b, float da, float db, Color face)
    {
        float side = MathF.Sign(da), thickness = 0.35f;
        Vector3 p = Point(a, da), q = Point(b, db), r = Point(b, db + side * thickness), s = Point(a, da + side * thickness);
        var up = Vector3.Up * 0.85f; var col = Color.FromHtml("#dedbc9");
        mesh.GroundQuad(p + up, q + up, r + up, s + up, col);
        if (side > 0) { mesh.Quad(p, q, q + up, p + up, face); mesh.Quad(r, s, s + up, r + up, col); }
        else { mesh.Quad(q, p, p + up, q + up, face); mesh.Quad(s, r, r + up, s + up, col); }
    }

    /// <summary>
    /// What the road-facing side of this three-metre panel of barrier is
    /// painted as: a tyre wall's belt on the outside of a corner tight
    /// enough to need one, otherwise advertising boards in twelve-metre runs.
    /// </summary>
    private Color BarrierFace(TrackData track, int segment, float s, int side, float wallOffset)
    {
        if (MathF.Abs(track.Sample(s).Curvature) > CornerBarrierCurvature && IsOutside(s, side, wallOffset))
            return Look.TyreWall[segment % Look.TyreWall.Count];
        return Look.Barrier[(segment / 4 + (side > 0 ? 3 : 0)) % Look.Barrier.Count];
    }

    /// <summary>
    /// Whether this side is the outside of the bend here. Measured, not
    /// derived from a sign convention: the outside edge of an arc is the
    /// longer one.
    /// </summary>
    public bool IsOutside(float s, int side, float offset)
    {
        float own = (Point(s + 8f, side * offset) - Point(s - 8f, side * offset)).Length();
        float other = (Point(s + 8f, -side * offset) - Point(s - 8f, -side * offset)).Length();
        return own > other;
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
    /// <summary>
    /// The ground off the road, as one sheet that meets the run-off at the
    /// barrier and eases out to the meadow (TerrainMesh): no strip of verge
    /// laid over another, and nothing over the road.
    /// </summary>
    private void BuildTerrain()
    {
        const float cell = 18f, margin = 650f;
        foreach (var ((cx, cz), triangles) in TerrainMesh.Build(Surface, cell, margin, chunkCells: 16))
        {
            if (triangles.Count == 0)
                continue;
            var mesh = new LowPolyMesh();
            for (int t = 0; t + 2 < triangles.Count; t += 3)
            {
                Vector3 a = LowPolyMesh.V(triangles[t]), b = LowPolyMesh.V(triangles[t + 1]), c = LowPolyMesh.V(triangles[t + 2]);
                // Restrained field-sized variation rather than random rainbow triangles.
                Vector3 middle = (a + b + c) / 3f;
                float tint = 0.018f * MathF.Sin(middle.X / cell * 0.35f + middle.Z / cell * 0.2f);
                mesh.GroundTriangle(a, b, c, Look.Grass.Lightened(tint));
            }
            var instance = mesh.Instance($"Terrain_{cx}_{cz}");
            instance.CastShadow = GeometryInstance3D.ShadowCastingSetting.Off;
            AddChild(instance);
        }
    }
}
