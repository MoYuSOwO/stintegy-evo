using Godot;

namespace StintegyEVO.Nodes.Race;

public partial class FrameTimeMonitor : CanvasLayer
{
    private const float DefaultUpdateInterval = 0.75f;

    private readonly Label label = new()
    {
        Position = new Vector2(12f, 12f),
        ZIndex = 1000
    };

    private double elapsed;
    private float renderSumMs;
    private float renderMaxMs;
    private int renderFrames;
    private float physicsSumMs;
    private float physicsMaxMs;
    private int physicsFrames;
    private int gc0;
    private int gc1;
    private int gc2;

    [Export] public float UpdateIntervalSeconds { get; set; } = DefaultUpdateInterval;

    public override void _Ready()
    {
        Layer = 100;
        label.AddThemeColorOverride("font_color", Colors.White);
        label.AddThemeColorOverride("font_shadow_color", Colors.Black);
        label.AddThemeConstantOverride("shadow_offset_x", 1);
        label.AddThemeConstantOverride("shadow_offset_y", 1);
        AddChild(label);
        gc0 = System.GC.CollectionCount(0);
        gc1 = System.GC.CollectionCount(1);
        gc2 = System.GC.CollectionCount(2);
    }

    public override void _Process(double delta)
    {
        float frameMs = (float)(delta * 1000.0);
        renderSumMs += frameMs;
        renderMaxMs = Mathf.Max(renderMaxMs, frameMs);
        renderFrames++;
        elapsed += delta;

        if (elapsed >= Mathf.Max(UpdateIntervalSeconds, 0.1f))
            Refresh();
    }

    public override void _PhysicsProcess(double delta)
    {
        float frameMs = (float)(delta * 1000.0);
        physicsSumMs += frameMs;
        physicsMaxMs = Mathf.Max(physicsMaxMs, frameMs);
        physicsFrames++;
    }

    private void Refresh()
    {
        float renderAvg = renderFrames > 0 ? renderSumMs / renderFrames : 0f;
        float physicsAvg = physicsFrames > 0 ? physicsSumMs / physicsFrames : 0f;
        float processMs = (float)Performance.GetMonitor(Performance.Monitor.TimeProcess) * 1000f;
        float physicsProcessMs = (float)Performance.GetMonitor(Performance.Monitor.TimePhysicsProcess) * 1000f;
        int drawCalls = Mathf.RoundToInt((float)Performance.GetMonitor(Performance.Monitor.RenderTotalDrawCallsInFrame));
        int renderObjects = Mathf.RoundToInt((float)Performance.GetMonitor(Performance.Monitor.RenderTotalObjectsInFrame));
        int primitives = Mathf.RoundToInt((float)Performance.GetMonitor(Performance.Monitor.RenderTotalPrimitivesInFrame));
        int objects = Mathf.RoundToInt((float)Performance.GetMonitor(Performance.Monitor.ObjectCount));
        int newGc0 = System.GC.CollectionCount(0);
        int newGc1 = System.GC.CollectionCount(1);
        int newGc2 = System.GC.CollectionCount(2);
        string cameraText = BuildCameraText();

        label.Text =
            $"FPS {Engine.GetFramesPerSecond():0}  monitor {Performance.GetMonitor(Performance.Monitor.TimeFps):0}\n" +
            $"Render avg {renderAvg:0.00} ms  max {renderMaxMs:0.00} ms\n" +
            $"Physics avg {physicsAvg:0.00} ms  max {physicsMaxMs:0.00} ms\n" +
            $"Frame CPU {processMs:0.00} ms  physics script {physicsProcessMs:0.00} ms\n" +
            $"Draw calls {drawCalls}  objects {renderObjects}  primitives {primitives}\n" +
            $"{cameraText}\n" +
            $"Godot objects {objects}  GC +{newGc0 - gc0}/{newGc1 - gc1}/{newGc2 - gc2}";

        elapsed = 0.0;
        renderSumMs = 0f;
        renderMaxMs = 0f;
        renderFrames = 0;
        physicsSumMs = 0f;
        physicsMaxMs = 0f;
        physicsFrames = 0;
        gc0 = newGc0;
        gc1 = newGc1;
        gc2 = newGc2;
    }

    private string BuildCameraText()
    {
        Camera2D? camera = GetViewport().GetCamera2D();
        if (camera == null)
            return "Camera none";

        Vector2 viewportSize = GetViewport().GetVisibleRect().Size;
        Vector2 zoom = camera.Zoom;
        Vector2 worldView = new(
            zoom.X == 0f ? 0f : viewportSize.X / zoom.X,
            zoom.Y == 0f ? 0f : viewportSize.Y / zoom.Y
        );

        return $"Game camera zoom {zoom.X:0.00},{zoom.Y:0.00}  view {worldView.X:0}x{worldView.Y:0}  pos {camera.GlobalPosition.X:0},{camera.GlobalPosition.Y:0}";
    }
}
