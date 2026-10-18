using System;
using System.Linq;
using Godot;
using StintegyEVO.Core.Track;
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
    public void Initialize(TrackData track, string? sceneryPlanPath)
    {
        Surface = new(track);
        // Read first: the plan says what colour the ground is before any of it is built.
        Plan = ReadPlan(sceneryPlanPath);
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
                    // The pit lane lies over the run-off where it crosses it; the grass is not drawn under it.
                    if (wa - sa.HalfWidth > SurfaceGrip.KerbWidthMeters + 1e-3f
                        && !OnPitLane(a, (kerbA + side * wa) * 0.5f) && !OnPitLane(b, (kerbB + side * wb) * 0.5f))
                        Strip(mesh, a, b, kerbA, side * wa, kerbB, side * wb, Look.Mown[(i / 2) % Look.Mown.Count], 0f);
                    // The white line is the last of the racing surface, not
                    // the first of the run-off: painted inside its own edge.
                    Strip(mesh, a, b, ea - side * 0.12f, ea, eb - side * 0.12f, eb, LowPolyMesh.Ivory, 0.03f);
                    // Where the pit lane runs through the barrier's line, at
                    // its two ends, there is no barrier: there is a road.
                    if (!OnPitLane(a, side * wa) && !OnPitLane(b, side * wb))
                        Wall(mesh, a, b, side * wa, side * wb, BarrierFace(track, i, a, side, wa));
                }
            }
            AddChild(mesh.Instance($"Road_{begin}"));
        }
        BuildKerbs();
        BuildMarkings();
        BuildPitLane();
        var dressing = new TrackDressing3D { Name = "Dressing" };
        AddChild(dressing);
        dressing.Initialize(this, Plan?.Furniture ?? new Scenery.SceneryFurniture());
        BuildScenery(sceneryPlanPath);
    }

    private static Scenery.SceneryPlan? ReadPlan(string? planPath)
    {
        if (string.IsNullOrWhiteSpace(planPath) || !Godot.FileAccess.FileExists(planPath))
            return null;
        try
        {
            using var file = Godot.FileAccess.Open(planPath, Godot.FileAccess.ModeFlags.Read);
            return Scenery.SceneryPlan.Parse(file.GetAsText());
        }
        catch (Exception error)
        {
            // A malformed plan costs the circuit its scenery and its look,
            // and nothing else: the road is the domain's.
            GD.PushWarning($"scenery: {planPath} could not be read -- {error.Message}");
            return null;
        }
    }

    private void BuildScenery(string? planPath)
    {
        if (string.IsNullOrWhiteSpace(planPath))
            return;
        string track = Scenery.SceneryPlan.TrackOf(planPath);
        var scenery = new Scenery.SceneryLoader { Name = "Scenery" };
        AddChild(scenery);
        try
        {
            if (Plan is not null)
                scenery.Build(Plan, Surface);
            // Then whatever the player has added. A mod's scenery is drawn
            // after the circuit's own, from the mod's own folder, and can
            // no more touch the simulation than the circuit's own can.
            foreach ((Scenery.SceneryPlan plan, string folder) in
                     Scenery.SceneryMods.PlansFor(track))
            {
                scenery.Build(plan, Surface, folder);
            }
        }
        catch (Exception error)
        {
            GD.PushWarning($"scenery: {planPath} could not be built -- {error.Message}");
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
                    // None where the pit lane leaves or joins the track: there the edge is a road.
                    if (OnPitLane(a, side * (halfA + SurfaceGrip.KerbWidthMeters * 0.5f))
                        || OnPitLane(b, side * (halfB + SurfaceGrip.KerbWidthMeters * 0.5f)))
                        continue;
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
    /// <summary>Whether the point <paramref name="d"/> to the side of the track at <paramref name="s"/> is on the pit lane.</summary>
    public bool OnPitLane(float s, float d)
    {
        if (Surface.Pit is not { } pit)
            return false;
        TrackSample sample = Surface.Track.Sample(s);
        return pit.Contains(sample.Center + sample.Normal * d);
    }

    /// <summary>Whether a point of the world, in the view's axes, is on the track or its run-off, barrier to barrier.</summary>
    private bool OnCircuit(Vector3 point) => OnCircuit(point, 0.5f);

    private bool OnCircuit(Vector3 point, float margin)
    {
        (float s, float d) = Surface.Project(new NVector2(point.X, point.Z));
        return MathF.Abs(d) <= Surface.BarrierOffset(s, d >= 0f ? 1 : -1) + margin;
    }

    /// <summary>Whether a point of the world, in the view's axes, is on the pit lane.</summary>
    public bool OnPitLane(Vector3 point) => Surface.Pit?.Contains(new NVector2(point.X, point.Z)) ?? false;

    /// <summary>
    /// The pit lane: its surface, out of the road at one end and back into
    /// it at the other, edged in white; the limit lines across it; a box
    /// painted where each car stops; and a garage behind each box.
    /// </summary>
    private void BuildPitLane()
    {
        if (Surface.Pit is not { } pit)
            return;
        var mesh = new LowPolyMesh();
        Vector3 At(float u, float d, float lift) => LowPolyMesh.V(Surface.PitPoint(u, d)) + Vector3.Up * lift;
        void Band(float u0, float u1, float d0, float d1, Color color, float lift) =>
            mesh.GroundQuad(At(u0, d0, lift), At(u0, d1, lift), At(u1, d1, lift), At(u1, d0, lift), color);

        // Under the track's own road where the two overlap, so the lane
        // seems to come out of the track's edge rather than lie across it.
        const float surface = 0.018f, paint = 0.022f;
        int pieces = (int)MathF.Ceiling(pit.LengthMeters / 2f);
        float step = pit.LengthMeters / pieces;
        for (int i = 0; i < pieces; i++)
        {
            float a = i * step, b = (i + 1) * step;
            float ha = pit.Sample(a).HalfWidth, hb = pit.Sample(b).HalfWidth;
            for (int lane = 0; lane < 4; lane++)
            {
                float f0 = lane / 4f, f1 = (lane + 1) / 4f;
                mesh.GroundQuad(
                    At(a, Mathf.Lerp(-ha, ha, f0), surface), At(a, Mathf.Lerp(-ha, ha, f1), surface),
                    At(b, Mathf.Lerp(-hb, hb, f1), surface), At(b, Mathf.Lerp(-hb, hb, f0), surface),
                    Look.Road);
            }
            foreach (int side in new[] { -1, 1 })
                mesh.GroundQuad(
                    At(a, side * ha, paint), At(a, side * (ha - 0.15f), paint),
                    At(b, side * (hb - 0.15f), paint), At(b, side * hb, paint),
                    LowPolyMesh.Ivory);
        }

        // The line between the fast lane and the working lane, dashed, where
        // the lane is both: the working lane is the side the boxes are on,
        // a box in the middle of it.
        if (pit.Boxes.Count > 0)
        {
            PitBox first = pit.Boxes[0];
            PitLaneSample at = pit.Sample(first.U);
            float offset = NVector2.Dot(first.Position - at.Center, at.Normal);
            float widest = at.Width;
            for (float u = 0f; u + 3f < pit.LengthMeters; u += 6f)
            {
                float half = pit.Sample(u).HalfWidth;
                if (half * 2f < widest - 0.05f)
                    continue;
                float divider = 2f * offset - MathF.Sign(offset) * half;
                Band(u, u + 3f, divider - 0.06f, divider + 0.06f, LowPolyMesh.Ivory, paint);
            }
        }

        // The limit lines, the whole width across.
        foreach (float u in new[] { pit.SpeedLimitFromU, pit.SpeedLimitToU })
        {
            float h = pit.Sample(u).HalfWidth;
            Band(u - 0.3f, u + 0.3f, -h, h, LowPolyMesh.Ivory, paint);
        }

        // Each box: its outline in the working lane, and its garage behind it.
        var garages = new LowPolyMesh();
        var fronts = new System.Collections.Generic.List<(float From, float To, int Side)>();
        Color wall = Color.FromHtml("#c9c4b4"), door = Color.FromHtml("#39444a"), roof = Color.FromHtml("#8d969a");
        foreach (PitBox box in pit.Boxes)
        {
            PitLaneSample at = pit.Sample(box.U);
            float d = NVector2.Dot(box.Position - at.Center, at.Normal);
            float across = 1.4f, along = 2.8f;
            Band(box.U - along, box.U + along, d - across, d - across + 0.12f, LowPolyMesh.Vermilion, paint);
            Band(box.U - along, box.U + along, d + across - 0.12f, d + across, LowPolyMesh.Vermilion, paint);
            Band(box.U + along - 0.12f, box.U + along, d - across, d + across, LowPolyMesh.Vermilion, paint);

            // The garage opens onto the lane's edge on the boxes' side. As
            // deep as the ground behind it allows, and never over the
            // circuit: a garage with no room even at its shallowest is not built.
            float side = MathF.Sign(d), front = at.HalfWidth + 0.05f;
            float half = 6.6f, height = 6f;
            Vector3 p0 = At(box.U - half, side * front, 0f), p1 = At(box.U + half, side * front, 0f);
            Vector3 p2 = default, p3 = default;
            bool fits = false;
            foreach (float depth in new[] { 14f, 11f, 8f, 5f })
            {
                p2 = At(box.U + half, side * (front + depth), 0f);
                p3 = At(box.U - half, side * (front + depth), 0f);
                if (fits = !new[] { p0, p1, p2, p3, (p0 + p2) * 0.5f, (p1 + p3) * 0.5f }.Any(OnCircuit))
                    break;
            }
            if (!fits)
                continue;
            fronts.Add((box.U - half, box.U + half, (int)side));
            float floor = MathF.Min(MathF.Min(p0.Y, p1.Y), MathF.Min(p2.Y, p3.Y)) - 0.5f;
            Vector3 b0 = new(p0.X, floor, p0.Z), b1 = new(p1.X, floor, p1.Z), b2 = new(p2.X, floor, p2.Z), b3 = new(p3.X, floor, p3.Z);
            float top = MathF.Max(p0.Y, p1.Y) + height;
            Vector3 t0 = new(p0.X, top, p0.Z), t1 = new(p1.X, top, p1.Z), t2 = new(p2.X, top, p2.Z), t3 = new(p3.X, top, p3.Z);
            Vector3 middle = (t0 + t1 + t2 + t3 + b0 + b1 + b2 + b3) / 8f;
            // Wound so every face looks outward, whichever side of the lane the garages are on.
            void Face(Vector3 a, Vector3 b, Vector3 c, Vector3 e, Color color)
            {
                if ((b - a).Cross(c - a).Dot((a + c) * 0.5f - middle) < 0f) garages.Quad(e, c, b, a, color);
                else garages.Quad(a, b, c, e, color);
            }
            // The front: the door to the lane, and the wall above it.
            Vector3 d0 = new(p0.X, MathF.Max(p0.Y, p1.Y) + 4.2f, p0.Z), d1 = new(p1.X, d0.Y, p1.Z);
            Face(b0, b1, d1, d0, door);
            Face(d0, d1, t1, t0, wall);
            Face(b0, b3, t3, t0, wall);
            Face(b3, b2, t2, t3, wall);
            Face(b2, b1, t1, t2, wall);
            garages.GroundQuad(t0, t1, t2, t3, roof);
        }
        // Walled both sides, with no run-off: on the track's side the
        // track's own barrier is the pit wall, so the lane's wall is drawn
        // only where it is not beside the circuit, and not where the lane is
        // joining the road; on the other side, wall except where a garage
        // opens onto the lane.
        var walls = new LowPolyMesh();
        Color concrete = Color.FromHtml("#dedbc9");
        int panels = (int)MathF.Ceiling(pit.LengthMeters / 3f);
        float panel = pit.LengthMeters / panels;
        for (int i = 0; i < panels; i++)
        {
            float a = i * panel, b = (i + 1) * panel;
            foreach (int side in new[] { -1, 1 })
            {
                float ea = side * pit.Sample(a).HalfWidth, eb = side * pit.Sample(b).HalfWidth;
                Vector3 p = At(a, ea, 0f), q = At(b, eb, 0f);
                Vector3 r = At(b, eb + side * 0.3f, 0f), s = At(a, ea + side * 0.3f, 0f);
                if (OnCircuit(s, 1f) || OnCircuit(r, 1f))
                    continue;
                if (fronts.Exists(f => f.Side == side && b > f.From && a < f.To))
                    continue;
                var up = Vector3.Up * 1.1f;
                walls.GroundQuad(p + up, q + up, r + up, s + up, concrete);
                if (side > 0) { walls.Quad(p, q, q + up, p + up, concrete); walls.Quad(r, s, s + up, r + up, concrete); }
                else { walls.Quad(q, p, p + up, q + up, concrete); walls.Quad(s, r, r + up, s + up, concrete); }
            }
        }
        AddChild(mesh.Instance("PitLane"));
        AddChild(walls.Instance("PitWalls"));
        AddChild(garages.Instance("PitGarages"));
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
