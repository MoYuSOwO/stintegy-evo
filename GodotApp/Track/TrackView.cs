using System;
using Godot;
using StintegyEVO.GodotApp.Interop;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.Track;

[Tool]
public partial class TrackView : Node2D
{
    [ExportGroup("Palette")]
    [Export] public Color BackgroundColor { get; set; } = Color.FromHtml("#b8e994");
    [Export] public Color RoadColor { get; set; } = Color.FromHtml("#dfe4ea");
    [Export] public Color BufferColor { get; set; } = Color.FromHtml("#e8d3b9");
    [Export] public Color WallColor { get; set; } = Color.FromHtml("#576574");
    [Export] public Color ReferenceLineColor { get; set; } = Color.FromHtml("#65c4ff80");
    [Export] public Color StartingGridColor { get; set; } = Color.FromHtml("#fbfcf8cb");

    [ExportGroup("Dimensions")]
    [Export] public float WallThickness { get; set; } = 1.2f;
    [Export] public float FinishLineSquareSize { get; set; } = 0.5f;
    [Export] public float StartingGridLineWidth { get; set; } = 0.35f;

    private TrackData? _track;

    public void Initialize(TrackData track)
    {
        _track = track ?? throw new ArgumentNullException(nameof(track));
        Rebuild();
    }

    private void Rebuild()
    {
        foreach (Node child in GetChildren())
            child.QueueFree();

        if (_track == null || _track.LengthMeters < 2f)
            return;

        CreateBackground();
        DrawBuffers(_track);
        DrawRoad(_track);
        DrawWalls(_track);
        DrawReferenceLine(_track);
        DrawFinishLine(_track);
        DrawStartingGrids(_track);
    }

    private void CreateBackground()
    {
        const float size = 50000f;
        ColorRect background = new()
        {
            Color = BackgroundColor,
            Position = new Vector2(-size * 0.5f, -size * 0.5f),
            Size = new Vector2(size, size),
            MouseFilter = Control.MouseFilterEnum.Ignore,
            ZIndex = -100
        };
        AddChild(background);
        MoveChild(background, 0);
    }

    private void DrawRoad(TrackData track)
    {
        int segments = SegmentCount(track);
        Vector2[] vertices = new Vector2[(segments + 1) * 2];
        for (int i = 0; i <= segments; i++)
        {
            TrackSample sample = SampleAt(track, i, segments);
            vertices[i * 2] = sample.LeftEdge.ToGodot();
            vertices[i * 2 + 1] = sample.RightEdge.ToGodot();
        }
        AddChild(CreateStrip(vertices, RoadColor, 1));
    }

    private void DrawBuffers(TrackData track)
    {
        int segments = SegmentCount(track);
        Vector2[] left = new Vector2[(segments + 1) * 2];
        Vector2[] right = new Vector2[(segments + 1) * 2];
        for (int i = 0; i <= segments; i++)
        {
            TrackSample sample = SampleAt(track, i, segments);
            left[i * 2] = sample.LeftSpace.ToGodot();
            left[i * 2 + 1] = sample.LeftEdge.ToGodot();
            right[i * 2] = sample.RightEdge.ToGodot();
            right[i * 2 + 1] = sample.RightSpace.ToGodot();
        }
        AddChild(CreateStrip(left, BufferColor, 0));
        AddChild(CreateStrip(right, BufferColor, 0));
    }

    private void DrawWalls(TrackData track)
    {
        int segments = SegmentCount(track);
        Vector2[] left = new Vector2[(segments + 1) * 2];
        Vector2[] right = new Vector2[(segments + 1) * 2];
        for (int i = 0; i <= segments; i++)
        {
            TrackSample sample = SampleAt(track, i, segments);
            var leftOuter = sample.LeftSpace + sample.Normal * WallThickness;
            var rightOuter = sample.RightSpace - sample.Normal * WallThickness;
            left[i * 2] = leftOuter.ToGodot();
            left[i * 2 + 1] = sample.LeftSpace.ToGodot();
            right[i * 2] = sample.RightSpace.ToGodot();
            right[i * 2 + 1] = rightOuter.ToGodot();
        }
        AddChild(CreateStrip(left, WallColor, 3));
        AddChild(CreateStrip(right, WallColor, 3));
    }

    private void DrawReferenceLine(TrackData track)
    {
        int segments = SegmentCount(track);
        Vector2[] points = new Vector2[segments];
        for (int i = 0; i < segments; i++)
            points[i] = SampleAt(track, i, segments).RefPosition.ToGodot();

        AddChild(new Line2D
        {
            Points = points,
            Width = 0.35f,
            DefaultColor = ReferenceLineColor,
            Antialiased = true,
            Closed = true,
            ZIndex = 2
        });
    }

    private void DrawFinishLine(TrackData track)
    {
        TrackSample sample = track.Sample(track.StartingLineS);
        int rows = 2;
        int columns = Math.Max(2, Mathf.CeilToInt(sample.Width / FinishLineSquareSize));
        float squareWidth = sample.Width / columns;
        Node2D parent = new() { ZIndex = 4 };
        AddChild(parent);

        for (int row = 0; row < rows; row++)
        {
            for (int column = 0; column < columns; column++)
            {
                float d0 = -sample.HalfWidth + column * squareWidth;
                float d1 = d0 + squareWidth;
                float s0 = row * FinishLineSquareSize;
                float s1 = (row + 1) * FinishLineSquareSize;
                var p00 = sample.Center + sample.Normal * d0 + sample.Tangent * s0;
                var p10 = sample.Center + sample.Normal * d1 + sample.Tangent * s0;
                var p11 = sample.Center + sample.Normal * d1 + sample.Tangent * s1;
                var p01 = sample.Center + sample.Normal * d0 + sample.Tangent * s1;
                parent.AddChild(new Polygon2D
                {
                    Polygon = [p00.ToGodot(), p10.ToGodot(), p11.ToGodot(), p01.ToGodot()],
                    Color = (row + column) % 2 == 0 ? Colors.White : Colors.Black,
                    Antialiased = true
                });
            }
        }
    }

    private void DrawStartingGrids(TrackData track)
    {
        Node2D parent = new() { ZIndex = 4 };
        AddChild(parent);

        for (int gridPosition = 1; gridPosition <= track.StartingGridCount; gridPosition++)
        {
            Grid grid = track.Grids[gridPosition];
            TrackSample sample = track.Sample(grid.S);
            Vector2 center = grid.Position.ToGodot();
            Vector2 halfWidth = sample.Normal.ToGodot() * (TrackGridConfig.GridWidth * 0.5f);
            Vector2 halfLength = sample.Tangent.ToGodot() * (TrackGridConfig.GridLength * 0.5f);
            parent.AddChild(new Line2D
            {
                Points =
                [
                    center - halfLength + halfWidth,
                    center + halfLength + halfWidth,
                    center + halfLength - halfWidth,
                    center - halfLength - halfWidth,
                    center - halfLength + halfWidth
                ],
                Width = StartingGridLineWidth,
                DefaultColor = StartingGridColor,
                Antialiased = true,
                JointMode = Line2D.LineJointMode.Sharp,
                BeginCapMode = Line2D.LineCapMode.Box,
                EndCapMode = Line2D.LineCapMode.Box
            });
        }
    }

    private static int SegmentCount(TrackData track)
    {
        return Math.Max(2, Mathf.CeilToInt(track.LengthMeters));
    }

    private static TrackSample SampleAt(TrackData track, int index, int segmentCount)
    {
        return track.Sample(track.LengthMeters * index / segmentCount);
    }

    private static MeshInstance2D CreateStrip(Vector2[] vertices, Color color, int zIndex)
    {
        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = vertices;
        ArrayMesh mesh = new();
        mesh.AddSurfaceFromArrays(Mesh.PrimitiveType.TriangleStrip, arrays);
        return new MeshInstance2D
        {
            Mesh = mesh,
            Modulate = color,
            ZIndex = zIndex
        };
    }
}
