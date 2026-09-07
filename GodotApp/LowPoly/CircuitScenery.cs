using System;
using Godot;
using StintegyEVO.Core.Track;
using NVector2 = System.Numerics.Vector2;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class CircuitScenery : Node3D
{
    public void Build(TrackSurfaceGeometry surface)
    {
        var pit = new LowPolyMesh();
        pit.Box(new(0, -1f, 0), new(116, 2.4f, 15), Color.FromHtml("#b9b5a0"));
        pit.Box(new(0, 3, 0), new(116, 6, 15), Color.FromHtml("#d4cfba"));
        pit.Box(new(0, 6.2f, 0), new(120, 0.7f, 18), LowPolyMesh.Ivory);
        pit.Box(new(0, 4.6f, -7.55f), new(108, 1.5f, 0.15f), Color.FromHtml("#475b60"));
        for (int i = 0; i < 14; i++)
        {
            float x = -51 + i * 7.8f;
            pit.Box(new(x, 1.5f, -7.6f), new(6.3f, 2.8f, 0.2f), LowPolyMesh.Ink);
            pit.Box(new(x, 3.1f, -7.8f), new(6.7f, 0.35f, 0.3f), i % 3 == 0 ? LowPolyMesh.Vermilion : LowPolyMesh.Ivory);
        }
        Place(pit.Instance("PitBuilding"), surface, surface.Track.StartingLineS - 50f, -47f);
        for (int section = 0; section < 4; section++)
        {
            var stand = new LowPolyMesh();
            for (int row = 0; row < 6; row++)
                stand.Box(new(0, 0.5f + row * 0.65f, row * 1.5f), new(62, 0.8f, 1.4f), row % 2 == 0 ? Color.FromHtml("#7c8787") : Color.FromHtml("#a4aaa0"));
            stand.Box(new(0, 7.4f, 4f), new(67, 0.4f, 13), LowPolyMesh.Ivory);
            foreach (float x in new[] { -29f, 0f, 29f })
                stand.Box(new(x, 3.7f, 9f), new(0.5f, 7.4f, 0.5f), LowPolyMesh.Ink);
            Place(stand.Instance($"Grandstand_{section}"), surface, section == 0 ? -50f : surface.Track.LengthMeters * (0.15f + section * 0.2f), 35f);
        }
        var gantry = new LowPolyMesh();
        float width = surface.Track.Sample(0).Width;
        gantry.Box(new(0, 6.7f, 0), new(1.4f, 1.7f, width + 3), LowPolyMesh.Ink);
        foreach (int side in new[] { -1, 1 })
            gantry.Box(new(0, 3.2f, side * (width / 2 + 1.2f)), new(0.7f, 6.4f, 0.7f), LowPolyMesh.Ivory);
        Place(gantry.Instance("TimingBridge"), surface, 0, 0);
        var bridgeText = new Label3D
        {
            Text = "S T I N T E G Y",
            FontSize = 72,
            PixelSize = 0.007f,
            Modulate = LowPolyMesh.Ivory,
            OutlineSize = 0,
            Position = new Vector3(-0.73f, 6.7f, 0),
            Rotation = new Vector3(0, -Mathf.Pi / 2, 0)
        };
        GetNode<Node3D>("TimingBridge").AddChild(bridgeText);
        BuildTrees(surface);
    }
    private void Place(Node3D node, TrackSurfaceGeometry surface, float s, float d)
    {
        var sample = surface.Track.Sample(s); var pos = sample.Center + sample.Normal * d;
        float h = d == 0f ? surface.Height(s, 0f) : surface.MeadowHeight(pos.X, pos.Y) - 0.15f;
        node.Position = new(pos.X, h, pos.Y);
        node.Rotation = new(0, -MathF.Atan2(sample.Tangent.Y, sample.Tangent.X), 0);
        AddChild(node);
    }
    private void BuildTrees(TrackSurfaceGeometry surface)
    {
        var tree = new LowPolyMesh();
        tree.Cone(Vector3.Zero, 0.3f, 0.22f, 2.8f, Color.FromHtml("#75664e"), 5);
        tree.Cone(new(0, 1.5f, 0), 2.1f, 0.9f, 2.4f, Color.FromHtml("#526e52"), 7);
        tree.Cone(new(0, 3.3f, 0), 1.7f, 0.15f, 2.6f, Color.FromHtml("#72845b"), 7);
        var shared = tree.Build(); var random = new Random(901);
        // Spatial clusters are separately culled; individual trees have no process callback.
        for (int cluster = 0; cluster < 28; cluster++)
        {
            float s = surface.Track.LengthMeters * (cluster + 0.3f) / 28f;
            var multimesh = new MultiMesh { TransformFormat = MultiMesh.TransformFormatEnum.Transform3D, Mesh = shared, InstanceCount = 9 };
            for (int i = 0; i < 9; i++)
            {
                var sample = surface.Track.Sample(s + i * 9f);
                float side = cluster % 2 == 0 ? 1 : -1;
                var pos = sample.Center + sample.Normal * (side * (65f + (float)random.NextDouble() * 45f));
                var pose = surface.Track.Project(pos);
                if (MathF.Abs(pose.D) < pose.Sample.HalfWidth + pose.Sample.LeftBufferWidth + 8f)
                {
                    // Avoid placing decorative trees on a neighboring section of track.
                    multimesh.SetInstanceTransform(i, new Transform3D(Basis.Identity.Scaled(Vector3.One * 0.001f), new Vector3(pos.X, -100f, pos.Y)));
                    continue;
                }
                float h = surface.MeadowHeight(pos.X, pos.Y) - 0.2f;
                float scale = 0.8f + (float)random.NextDouble() * 0.65f;
                var basis = new Basis(Vector3.Up, (float)random.NextDouble() * Mathf.Tau).Scaled(Vector3.One * scale);
                multimesh.SetInstanceTransform(i, new Transform3D(basis, new Vector3(pos.X, h, pos.Y)));
            }
            AddChild(new MultiMeshInstance3D { Name = $"Trees_{cluster}", Multimesh = multimesh });
        }
    }
}
