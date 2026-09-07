using System;
using System.Linq;
using Godot;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class RaceHud3D : CanvasLayer
{
    private readonly Label _clock = new(), _driver = new(), _speed = new(), _strategy = new(), _stats = new();
    private readonly Button[] _cameras = new Button[3];
    private readonly Button[] _rows = new Button[8];
    private readonly int[] _rowCars = new int[8];
    private readonly CircuitMap _map = new();
    private Button _pause = null!;
    private RaceView3D _race = null!;
    private static readonly Color Paper = Color.FromHtml("#eeeadef5"), Ink = Color.FromHtml("#25312f"), Muted = Color.FromHtml("#647269");
    public void Initialize(RaceView3D race)
    {
        _race = race; Layer = 50;
        var root = new Control { Name = "Interface", MouseFilter = Control.MouseFilterEnum.Ignore };
        AddChild(root); root.SetAnchorsAndOffsetsPreset(Control.LayoutPreset.FullRect);
        var font = new SystemFont { FontNames = ["Avenir Next", "DejaVu Sans"], FontWeight = 500 };
        root.Theme = new Theme { DefaultFont = font, DefaultFontSize = 14 };
        var header = Panel(root, Control.LayoutPreset.TopWide, new Vector2(20, 18), new Vector2(-20, 78));
        var title = Label(header, "STINTEGY", 21, new(18, 9)); title.AddThemeConstantOverride("outline_size", 0);
        Label(header, "S I L V E R S T O N E   /   SOLO PRACTICE", 10, new(19, 37), Muted);
        _clock.Position = new(358, 19); _clock.AddThemeFontSizeOverride("font_size", 17); _clock.AddThemeColorOverride("font_color", Ink); header.AddChild(_clock);
        var controls = new HBoxContainer { Position = new(-529, 12), Size = new(510, 36), AnchorLeft = 1, AnchorRight = 1 };
        controls.AddThemeConstantOverride("separation", 5); header.AddChild(controls);
        string[] names = ["1  CHASE", "2  SIDE", "3  OVERVIEW"];
        for (int i = 0; i < 3; i++)
        {
            int mode = i + 1; var b = Button(names[i]); b.CustomMinimumSize = new(116, 36); b.Pressed += () => race.SetCameraMode(mode); controls.AddChild(b); _cameras[i] = b;
        }
        _pause = Button("Ⅱ PAUSE"); _pause.CustomMinimumSize = new(104, 36); _pause.Pressed += race.TogglePause; controls.AddChild(_pause);
        var order = Panel(root, Control.LayoutPreset.TopLeft, new(20, 100), new(239, 100 + 74 + Math.Min(_rows.Length, race.Simulation.Cars.Count) * 28));
        Label(order, "RUNNING ORDER", 11, new(15, 12), Muted);
        Label(order, "POS     CAR                      LAP", 10, new(15, 37), Muted);
        for (int i = 0; i < _rows.Length; i++)
        {
            int row = i; var b = Button(""); b.Position = new(8, 59 + i * 28); b.Size = new(203, 27); b.Alignment = HorizontalAlignment.Left;
            b.AddThemeFontSizeOverride("font_size", 12); b.Pressed += () => race.SelectCar(_rowCars[row]); order.AddChild(b); _rows[i] = b;
        }
        var bottom = Panel(root, Control.LayoutPreset.BottomLeft, new(20, -141), new(527, -48));
        _driver.Position = new(17, 12); _driver.AddThemeFontSizeOverride("font_size", 13); _driver.AddThemeColorOverride("font_color", Ink); bottom.AddChild(_driver);
        _speed.Position = new(16, 32); _speed.AddThemeFontSizeOverride("font_size", 36); _speed.AddThemeColorOverride("font_color", Ink); bottom.AddChild(_speed);
        _strategy.Position = new(189, 15); _strategy.AddThemeFontSizeOverride("font_size", 12); _strategy.AddThemeColorOverride("font_color", Ink); bottom.AddChild(_strategy);
        var mapPanel = Panel(root, Control.LayoutPreset.BottomRight, new(-237, -231), new(-20, -48));
        Label(mapPanel, "CIRCUIT     /     5.891 KM", 10, new(14, 10), Muted);
        _map.Position = new(10, 29); _map.Size = new(195, 144); mapPanel.AddChild(_map); _map.Initialize(race.Surface, race.Simulation.Cars);
        var footer = Label(root, "← →  CAR     SCROLL  ZOOM     RIGHT DRAG  ORBIT     SPACE  PAUSE     H  HIDE UI", 11, new(24, -28), Ink);
        footer.AnchorTop = 1; footer.AnchorBottom = 1;
        _stats.AnchorLeft = 1; _stats.AnchorRight = 1; _stats.AnchorTop = 1; _stats.AnchorBottom = 1; _stats.Position = new(-360, -28); _stats.Size = new(336, 20);
        _stats.HorizontalAlignment = HorizontalAlignment.Right; _stats.AddThemeFontSizeOverride("font_size", 11); _stats.AddThemeColorOverride("font_color", Ink); root.AddChild(_stats);
    }
    public void Refresh(double coreMs)
    {
        var sim = _race.Simulation; var car = sim.Cars[_race.SelectedCarIndex];
        var elapsed = TimeSpan.FromSeconds(sim.RaceTimeSeconds);
        _clock.Text = $"{elapsed.Minutes:00}:{elapsed.Seconds:00}  /  {sim.Cars.Count} {(sim.Cars.Count == 1 ? "CAR" : "CARS")}";
        _driver.Text = $"CAR {_race.SelectedCarIndex + 1:00}    /    LAP {car.Progress.Lap + 1:00}";
        _speed.Text = $"{car.State.Speed * 3.6f:000} km/h";
        float wear = (car.State.FrontLeft.Wear + car.State.FrontRight.Wear + car.State.RearLeft.Wear + car.State.RearRight.Wear) / 4;
        _strategy.Text = $"TYRES   {car.Strategy.TireMode,-10}  Q / E\nPOWER   {car.Strategy.BatteryMode,-10}  A / D\nENERGY  {car.State.BatterySoc * 100:0}%     TYRE LIFE  {(1 - wear) * 100:0}%";
        var ordered = sim.Cars.Select((c, i) => (Car: c, Index: i)).OrderByDescending(x => x.Car.Progress.RaceDistanceMeters).ToArray();
        for (int i = 0; i < _rows.Length; i++)
        {
            _rows[i].Visible = i < ordered.Length;
            if (i >= ordered.Length) continue;
            _rowCars[i] = ordered[i].Index; _rows[i].Text = $"{i + 1:00}      CAR {ordered[i].Index + 1:00}                 {ordered[i].Car.Progress.Lap + 1:00}";
            _rows[i].AddThemeColorOverride("font_color", ordered[i].Index == _race.SelectedCarIndex ? LowPolyMesh.Vermilion : Ink);
        }
        RefreshControls();
        _stats.Text = coreMs > 0
            ? $"{Engine.GetFramesPerSecond():0} FPS  /  SIM {_race.SimulationRate:0.00}x  /  CORE {coreMs:0.0} ms"
            : $"{Engine.GetFramesPerSecond():0} FPS  /  PREPARING LAP";
        _map.Refresh(_race.SelectedCarIndex);
    }
    public void RefreshControls()
    {
        if (_pause == null) return;
        for (int i = 0; i < 3; i++) _cameras[i].AddThemeColorOverride("font_color", _race.CameraMode == i + 1 ? LowPolyMesh.Vermilion : Ink);
        _pause.Text = _race.IsPaused ? "▶ RUN" : "Ⅱ PAUSE";
    }
    private static Panel Panel(Control parent, Control.LayoutPreset preset, Vector2 start, Vector2 end)
    {
        var p = new Panel(); parent.AddChild(p); p.SetAnchorsAndOffsetsPreset(preset);
        p.OffsetLeft = start.X; p.OffsetTop = start.Y; p.OffsetRight = end.X; p.OffsetBottom = end.Y;
        p.AddThemeStyleboxOverride("panel", new StyleBoxFlat { BgColor = Paper, BorderColor = Color.FromHtml("#d2d2c5"), BorderWidthBottom = 1 }); return p;
    }
    private static Label Label(Control parent, string text, int size, Vector2 position, Color? color = null)
    {
        var label = new Label { Text = text, Position = position, MouseFilter = Control.MouseFilterEnum.Ignore };
        label.AddThemeFontSizeOverride("font_size", size); label.AddThemeColorOverride("font_color", color ?? Ink); parent.AddChild(label); return label;
    }
    private static Button Button(string text)
    {
        var b = new Button { Text = text, FocusMode = Control.FocusModeEnum.None, MouseDefaultCursorShape = Control.CursorShape.PointingHand };
        b.AddThemeStyleboxOverride("normal", new StyleBoxFlat { BgColor = new Color(0, 0, 0, 0) });
        b.AddThemeStyleboxOverride("hover", new StyleBoxFlat { BgColor = Color.FromHtml("#d9dccd") });
        b.AddThemeStyleboxOverride("pressed", new StyleBoxFlat { BgColor = Color.FromHtml("#c8cdbd") });
        b.AddThemeColorOverride("font_color", Ink); b.AddThemeColorOverride("font_hover_color", Ink); b.AddThemeColorOverride("font_pressed_color", Ink); b.AddThemeFontSizeOverride("font_size", 12); return b;
    }
}
