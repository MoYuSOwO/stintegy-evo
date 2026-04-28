using System;
using Godot;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Nodes.Track;

[Tool]
public partial class TrackRenderer : Node2D
{
    [ExportGroup("Palette")]
    [Export] public Color BgColor = Color.FromHtml("#b8e994");
    [Export] public Color RoadColor = Color.FromHtml("#dfe4ea");
    [Export] public Color BufferColor = Color.FromHtml("#e8d3b9");
    [Export] public Color WallColor = Color.FromHtml("#576574");
    [Export] public Color RacingLineColor = Color.FromHtml("#91c788e4");
    [Export] public Color GridColor = Color.FromHtml("#fbfcf8cb");

    [ExportGroup("Value")]
    [Export] public float wallThickness = 2.4f;
    [Export] public float racingLineThickness = 1.0f;
    [Export] public float finishLineSquareSize = 0.5f;
    [Export] public float startingLineThickness = 0.5f;

    private TrackData? _track;
    public TrackData Track
    {
        get
        {
            if (_track == null)
                throw new ArgumentNullException("_track", "TrackData have not initialized!");
            return _track;
        }
    }

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

    public void Init(TrackData track)
    {
        _track = track;
        BuildTrackVisuals();
    }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

	public void BuildTrackVisuals()
    {
        foreach (Node child in GetChildren())
        {
            child.QueueFree();
        }

        if (Track.Length < 2) return;

        // Set full-screen background (grass)
        CreateDynamicBackground();

        // Draw buffer zones (z-index 0)
        DrawBufferZones(Track);

        // Draw the main body of the track (z-index 1, 2)
        DrawRoad(Track);
        DrawStartingLine(Track);
        DrawStartingGrids(Track);

        // Draw physical walls (visual outline + physics Collision) (z-index 3)
        GenerateWalls(Track);

        // Draw racing lines (z-index 4)
        DrawRacingLine(Track);
    }

    private void CreateDynamicBackground()
    {
        ColorRect bgRect = new()
        {
            Color = BgColor
        };
        float hugeSize = 50000f;
        bgRect.Size = new Vector2(hugeSize, hugeSize);
        bgRect.Position = new Vector2(-hugeSize / 2f, -hugeSize / 2f);
        bgRect.ZIndex = -100;
        AddChild(bgRect);
        MoveChild(bgRect, 0);
    }

    private void DrawRoad(TrackData trackData)
    {
        int n = trackData.Length;
        Vector2[] vertices = new Vector2[(n + 1) * 2];

        for (int i = 0; i <= n; i++)
        {
            int idx = i % n;
            vertices[i * 2] = trackData[idx].LeftEdge;
            vertices[i * 2 + 1] = trackData[idx].RightEdge;
        }

        MeshInstance2D roadMesh = CreateFlatMesh(vertices, RoadColor, 1);
        AddChild(roadMesh);
    }

    private void DrawStartingLine(TrackData trackData)
    {
        var config = trackData.GridConfig;
        var node = trackData[config.StartingLineIdx];
        
        // Two rows of alternating black and white squares
        int rows = 2;
        float totalWidth = node.HalfWidth * 2;
        int cols = Mathf.CeilToInt(totalWidth / finishLineSquareSize);
        float actualSqWidth = totalWidth / cols;

        Node2D finishLineParent = new() { ZIndex = 2 };
        AddChild(finishLineParent);

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                bool isWhite = (r + c) % 2 == 0;
                Color color = isWhite ? Colors.White : Colors.Black;

                float xOffsetStart = -node.HalfWidth + c * actualSqWidth;
                float xOffsetEnd = xOffsetStart + actualSqWidth;
                float yOffsetStart = r * finishLineSquareSize;
                float yOffsetEnd = (r + 1) * finishLineSquareSize;

                Vector2[] quad =
                [
                    node.GetOffsetPos(xOffsetStart) + node.Tangent * yOffsetStart,
                    node.GetOffsetPos(xOffsetEnd) + node.Tangent * yOffsetStart,
                    node.GetOffsetPos(xOffsetEnd) + node.Tangent * yOffsetEnd,
                    node.GetOffsetPos(xOffsetStart) + node.Tangent * yOffsetEnd
                ];

                finishLineParent.AddChild(CreatePolygon(quad, color));
            }
        }
    }

    private void DrawStartingGrids(TrackData trackData)
    {
        var config = trackData.GridConfig;
        Node2D gridsParent = new() { ZIndex = 4 };
        AddChild(gridsParent);

        for (int i = 1; i <= config.GridCount; i++)
        {
            var grid = trackData.Grids[i];
            Vector2 center = grid.Position;
            var node = trackData[grid.Index];

            float w = TrackGridConfig.GridWidth;
            float l = TrackGridConfig.GridLength;

            Vector2 halfWidth = node.Normal * (w * 0.5f);
            Vector2 halfLength = node.Tangent * (l * 0.5f);

            Vector2[] linePoints =
            [
                center - halfLength + halfWidth,
                center + halfLength + halfWidth,
                center + halfLength - halfWidth,
                center - halfLength - halfWidth
            ];

            Line2D gridLine = new()
            {
                Width = startingLineThickness,
                DefaultColor = GridColor,
                ZIndex = 2,
                Antialiased = true,
                JointMode = Line2D.LineJointMode.Sharp,
                BeginCapMode = Line2D.LineCapMode.Box,
                EndCapMode = Line2D.LineCapMode.Box
            };

            foreach (var pt in linePoints)
            {
                gridLine.AddPoint(pt);
            }

            gridsParent.AddChild(gridLine);
        }
    }

    private void DrawBufferZones(TrackData trackData)
    {
        int n = trackData.Length;
        Vector2[] leftVertices = new Vector2[(n + 1) * 2];
        Vector2[] rightVertices = new Vector2[(n + 1) * 2];

        for (int i = 0; i <= n; i++)
        {
            int idx = i % n;
            TrackPoint node = trackData[idx];

            Vector2 leftWall = node.LeftEdge + node.Normal * node.LeftBufferWidth;
            leftVertices[i * 2] = leftWall;
            leftVertices[i * 2 + 1] = node.LeftEdge;

            Vector2 rightWall = node.RightEdge - node.Normal * node.RightBufferWidth;
            rightVertices[i * 2] = node.RightEdge;
            rightVertices[i * 2 + 1] = rightWall;
        }

        AddChild(CreateFlatMesh(leftVertices, BufferColor, 0));
        AddChild(CreateFlatMesh(rightVertices, BufferColor, 0));
    }

    private void GenerateWalls(TrackData trackData)
    {
        int n = trackData.Length;
        
        Vector2[] leftWallVertices = new Vector2[(n + 1) * 2];
        Vector2[] rightWallVertices = new Vector2[(n + 1) * 2];
        StaticBody2D physicsWalls = new();

        for (int i = 0; i <= n; i++)
        {
            int idx = i % n;
            TrackPoint node = trackData[idx];

            Vector2 leftWallInner = node.LeftEdge + node.Normal * node.LeftBufferWidth;
            Vector2 rightWallInner = node.RightEdge - node.Normal * node.RightBufferWidth;
            
            Vector2 leftWallOuter = leftWallInner + node.Normal * wallThickness;
            Vector2 rightWallOuter = rightWallInner - node.Normal * wallThickness;

            leftWallVertices[i * 2] = leftWallOuter; 
            leftWallVertices[i * 2 + 1] = leftWallInner;

            rightWallVertices[i * 2] = rightWallInner;
            rightWallVertices[i * 2 + 1] = rightWallOuter;

            if (i < n)
            {
                int next = (i + 1) % n;
                TrackPoint nextNode = trackData[next];
                Vector2 nextLeftInner = nextNode.LeftEdge + nextNode.Normal * nextNode.LeftBufferWidth;
                Vector2 nextRightInner = nextNode.RightEdge - nextNode.Normal * nextNode.RightBufferWidth;

                physicsWalls.AddChild(new CollisionShape2D { Shape = new SegmentShape2D { A = leftWallInner, B = nextLeftInner } });
                physicsWalls.AddChild(new CollisionShape2D { Shape = new SegmentShape2D { A = rightWallInner, B = nextRightInner } });
            }
        }
        AddChild(physicsWalls);

        AddChild(CreateFlatMesh(leftWallVertices, WallColor, 3));
        AddChild(CreateFlatMesh(rightWallVertices, WallColor, 3));
    }

    private void DrawRacingLine(TrackData trackData)
    {
        Line2D line = new() 
        { 
            Width = racingLineThickness,
            DefaultColor = RacingLineColor,
            ZIndex = 4,
            Antialiased = true,
            JointMode = Line2D.LineJointMode.Round,
            Closed = true
        };
        for (int i = 0; i < trackData.Length; i++)
        {
            line.AddPoint(trackData[i].Optimal);
        }
        AddChild(line);
    }

    private static Polygon2D CreatePolygon(Vector2[] points, Color color)
    {
        return new Polygon2D
        {
            Polygon = points,
            Color = color,
            Antialiased = true
        };
    }

    private static MeshInstance2D CreateFlatMesh(Vector2[] vertices, Color color, int zIndex)
    {
        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = vertices;

        ArrayMesh arrMesh = new();
        arrMesh.AddSurfaceFromArrays(Mesh.PrimitiveType.TriangleStrip, arrays);

        return new MeshInstance2D {
            Mesh = arrMesh,
            Modulate = color,
            ZIndex = zIndex
        };
    }
}
