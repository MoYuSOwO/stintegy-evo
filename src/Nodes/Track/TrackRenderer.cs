using System;
using Godot;
using PloyRacing.Core.Track;

namespace PloyRacing.Nodes.Track;

[Tool]
public partial class TrackRenderer : Node2D
{
    [ExportGroup("Palette (配色)")]
    [Export] public Color BgColor = Color.FromHtml("#b8e994");
    [Export] public Color RoadColor = Color.FromHtml("#dfe4ea");
    [Export] public Color BufferColor = Color.FromHtml("#e8d3b9");
    [Export] public Color WallColor = Color.FromHtml("#576574");
    [Export] public Color RacingLineColor = Color.FromHtml("#91c788e4");
    [Export] public Color GridColor = Color.FromHtml("#fbfcf8cb");

    [ExportGroup("Value (数值)")]
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
        // 每次重新生成前，清理旧的渲染节点
        foreach (Node child in GetChildren())
        {
            child.QueueFree();
        }

        if (Track.Nodes == null || Track.NodeCounts < 2) return;

        // 1. 设置全屏背景（草地）
        CreateDynamicBackground();

        // 2. 绘制缓冲区 (层级 0)
        DrawBufferZones(Track);

        // 3. 绘制赛道主体 (层级 1, 2)
        DrawRoad(Track);
        DrawStartingLine(Track);
        DrawStartingGrids(Track);

        // 4. 绘制物理墙壁 (视觉描边 + 真实的 Collision) (层级 3)
        GenerateWalls(Track);

        // 5. 绘制赛车线 (层级 4)
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
        var nodes = trackData.Nodes;
        int n = nodes.Length;
        Vector2[] vertices = new Vector2[(n + 1) * 2];

        for (int i = 0; i <= n; i++)
        {
            int idx = i % n;
            vertices[i * 2] = nodes[idx].LeftEdge;
            vertices[i * 2 + 1] = nodes[idx].RightEdge;
        }

        // 极简风不需要UV，只要顶点！
        MeshInstance2D roadMesh = CreateFlatMesh(vertices, RoadColor, 1);
        AddChild(roadMesh);
    }

    private void DrawStartingLine(TrackData trackData)
    {
        var config = trackData.GridConfig;
        var node = trackData.Nodes[config.StartingLineIdx];
        
        // 终点线通常由两排黑白交替方块组成
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

                // 计算每个方格的四个顶点
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
            // 获取发车位中心点
            Vector2 center = trackData.GridPosToVector2(i);
            
            // 为了确定朝向，我们需要找到对应的 Node 索引
            int nodeIdx = config.FirstGridIdx - config.GridStepDist * (i - 1);
            if (nodeIdx < 0) nodeIdx += trackData.NodeCounts;
            var node = trackData.Nodes[nodeIdx];

            // 框的实际尺寸 (稍微留白)
            float w = TrackGridConfig.GridWidth;
            float l = TrackGridConfig.GridLength;

            // node.Tangent 是车头前方，node.Normal 是车身左侧
            Vector2 halfWidth = node.Normal * (w * 0.5f);
            Vector2 halfLength = node.Tangent * (l * 0.5f);

            // 组装线段的 4 个顶点 (底边开口，不闭合)
            // 顺序：左后 -> 左前 -> 右前 -> 右后
            Vector2[] linePoints =
            [
                center - halfLength + halfWidth, // 1. 左后方起点
                center + halfLength + halfWidth, // 2. 延伸到左前方
                center + halfLength - halfWidth, // 3. 横跨到右前方 (车头那条线)
                center - halfLength - halfWidth  // 4. 退回到右后方终点
            ];

            // 使用 Line2D 画出赛道上的“白线”
            Line2D gridLine = new()
            {
                Width = startingLineThickness,
                DefaultColor = GridColor, // 珍珠白
                ZIndex = 2,
                Antialiased = true,
                JointMode = Line2D.LineJointMode.Sharp, // 让框框的角是硬直角，而不是圆角
                BeginCapMode = Line2D.LineCapMode.Box,
                EndCapMode = Line2D.LineCapMode.Box
            };

            // 将顶点加入线段
            foreach (var pt in linePoints)
            {
                gridLine.AddPoint(pt);
            }

            gridsParent.AddChild(gridLine);
        }
    }

    private void DrawBufferZones(TrackData trackData)
    {
        var nodes = trackData.Nodes;
        int n = nodes.Length;
        Vector2[] leftVertices = new Vector2[(n + 1) * 2];
        Vector2[] rightVertices = new Vector2[(n + 1) * 2];

        for (int i = 0; i <= n; i++)
        {
            int idx = i % n;
            TrackNode node = nodes[idx];

            Vector2 leftWall = node.LeftEdge + node.Normal * node.LeftBuffer;
            leftVertices[i * 2] = leftWall;
            leftVertices[i * 2 + 1] = node.LeftEdge;

            Vector2 rightWall = node.RightEdge - node.Normal * node.RightBuffer;
            rightVertices[i * 2] = node.RightEdge;
            rightVertices[i * 2 + 1] = rightWall;
        }

        AddChild(CreateFlatMesh(leftVertices, BufferColor, 0));
        AddChild(CreateFlatMesh(rightVertices, BufferColor, 0));
    }

    private void GenerateWalls(TrackData trackData)
    {
        var nodes = trackData.Nodes;
        int n = nodes.Length;
        
        Vector2[] leftWallVertices = new Vector2[(n + 1) * 2];
        Vector2[] rightWallVertices = new Vector2[(n + 1) * 2];
        StaticBody2D physicsWalls = new();

        for (int i = 0; i <= n; i++)
        {
            int idx = i % n;
            TrackNode node = nodes[idx];

            // 墙内侧 (贴着缓冲区)
            Vector2 leftWallInner = node.LeftEdge + node.Normal * node.LeftBuffer;
            Vector2 rightWallInner = node.RightEdge - node.Normal * node.RightBuffer;
            
            // 墙外侧 (向外推展出一个厚度，形成Mesh)
            Vector2 leftWallOuter = leftWallInner + node.Normal * wallThickness;
            Vector2 rightWallOuter = rightWallInner - node.Normal * wallThickness;

            // 填充左墙网格点
            leftWallVertices[i * 2] = leftWallOuter; 
            leftWallVertices[i * 2 + 1] = leftWallInner;

            // 填充右墙网格点
            rightWallVertices[i * 2] = rightWallInner;
            rightWallVertices[i * 2 + 1] = rightWallOuter;

            // 添加物理碰撞盒 (除最后一个闭合点外)
            if (i < n)
            {
                int next = (i + 1) % n;
                TrackNode nextNode = nodes[next];
                Vector2 nextLeftInner = nextNode.LeftEdge + nextNode.Normal * nextNode.LeftBuffer;
                Vector2 nextRightInner = nextNode.RightEdge - nextNode.Normal * nextNode.RightBuffer;

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
        for (int i = 0; i < trackData.NodeCounts; i++)
        {
            line.AddPoint(trackData.Nodes[i].GetOffsetPos(trackData.OptimalLines[i]));
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
