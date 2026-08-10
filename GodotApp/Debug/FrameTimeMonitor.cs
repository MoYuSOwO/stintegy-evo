using Godot;

namespace StintegyEVO.GodotApp.Debug;

public partial class FrameTimeMonitor : CanvasLayer
{
    private readonly Label _label = new()
    {
        Position = new Vector2(12f, 12f),
        ZIndex = 1000
    };
    private double _elapsed;
    private double _frameIntervalTotalMs;
    private float _maximumFrameIntervalMs;
    private int _frameIntervalSamples;
    private double _coreStepTotalMs;
    private float _maximumCoreStepMs;
    private int _coreStepSamples;

    public override void _Ready()
    {
        Layer = 100;
        _label.AddThemeColorOverride("font_color", Colors.White);
        _label.AddThemeColorOverride("font_shadow_color", Colors.Black);
        _label.AddThemeConstantOverride("shadow_offset_x", 1);
        _label.AddThemeConstantOverride("shadow_offset_y", 1);
        AddChild(_label);
    }

    public override void _Process(double delta)
    {
        _elapsed += delta;
        float frameIntervalMs = (float)delta * 1000f;
        _frameIntervalTotalMs += frameIntervalMs;
        _maximumFrameIntervalMs = Mathf.Max(
            _maximumFrameIntervalMs,
            frameIntervalMs
        );
        _frameIntervalSamples++;
        if (_elapsed < 0.75)
            return;

        double averageFrameIntervalMs = _frameIntervalSamples > 0
            ? _frameIntervalTotalMs / _frameIntervalSamples
            : 0.0;
        double averageCoreStepMs = _coreStepSamples > 0
            ? _coreStepTotalMs / _coreStepSamples
            : 0.0;
        _label.Text =
            $"FPS {Engine.GetFramesPerSecond():0}  |  " +
            $"Frame {averageFrameIntervalMs:0.00}/{_maximumFrameIntervalMs:0.00} ms  |  " +
            $"Core {averageCoreStepMs:0.00}/{_maximumCoreStepMs:0.00} ms";
        _elapsed = 0.0;
        _frameIntervalTotalMs = 0.0;
        _maximumFrameIntervalMs = 0f;
        _frameIntervalSamples = 0;
        _coreStepTotalMs = 0.0;
        _maximumCoreStepMs = 0f;
        _coreStepSamples = 0;
    }

    public void RecordCoreStep(double milliseconds)
    {
        if (!double.IsFinite(milliseconds) || milliseconds < 0.0)
            return;

        _coreStepTotalMs += milliseconds;
        _maximumCoreStepMs = Mathf.Max(
            _maximumCoreStepMs,
            (float)milliseconds
        );
        _coreStepSamples++;
    }
}
