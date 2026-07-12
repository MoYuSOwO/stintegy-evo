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
    private float _maximumFrameMs;

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
        _maximumFrameMs = Mathf.Max(_maximumFrameMs, (float)delta * 1000f);
        if (_elapsed < 0.75)
            return;

        _label.Text = $"FPS {Engine.GetFramesPerSecond():0}\nFrame max {_maximumFrameMs:0.00} ms";
        _elapsed = 0.0;
        _maximumFrameMs = 0f;
    }
}
