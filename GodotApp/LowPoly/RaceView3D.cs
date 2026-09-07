using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using Godot;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class RaceView3D : Node3D
{
    public RaceSimulation Simulation { get; private set; } = null!;
    public TrackSurfaceGeometry Surface => _circuit.Surface;
    public int SelectedCarIndex { get; private set; }
    public bool IsPaused { get; private set; }
    public int CameraMode => _camera.Mode;
    public float RaceSeconds { get; private set; }
    public float SimulationRate => _coreMs > 0 ? MathF.Min(1f, 1000f / (60f * (float)_coreMs)) : 1f;
    private readonly CircuitView3D _circuit = new() { Name = "Circuit" };
    private readonly RaceCameraRig _camera = new() { Name = "CameraRig" };
    private readonly RaceHud3D _hud = new() { Name = "RaceHud" };
    private readonly List<FormulaCarView3D> _cars = [];
    private double _hudElapsed, _coreMs, _sincePose, _sinceStep;
    private Task<double>? _step;
    private readonly Queue<(int Car, int Tire, int Power)> _commands = [];
    private bool _hudDirty;
    private bool _leaving;
    private static readonly string[] Liveries = ["#b93f30", "#e0bb53", "#507f80", "#ece5d1", "#3e6653", "#456583", "#c17b45", "#7a788b", "#77834b", "#393e42"];
    public override async void _Ready()
    {
        var watch = Stopwatch.StartNew();
        var loading = new CanvasLayer();
        var label = new Label { Text = "STINTEGY\n\nPreparing Silverstone…", Position = new Vector2(44, 44) };
        label.AddThemeFontSizeOverride("font_size", 24); loading.AddChild(label); AddChild(loading);
        TrackData track;
        try { track = await Task.Run(TrackFactory.SilverstoneStyleTestTrack); }
        catch (Exception error) { if (!_leaving) label.Text = "Could not prepare circuit.\n" + error.Message; GD.PushError(error.ToString()); return; }
        if (_leaving || !IsInstanceValid(this)) return;
        loading.QueueFree();
        Simulation = new RaceSimulation(track, new RaceEnvironment { AirTempC = 25f, TrackTempC = 35f });
        AddChild(_circuit); _circuit.Initialize(track); CreateLighting();
        var random = new Random(0x5345564F);
        for (int i = 0; i < 1; i++)
        {
            int number = i + 1; var start = track.Grids[number]; var sample = track.Sample(start.S);
            string id = $"grid-{number:D2}";
            var abilities = new DriverAbilities { Pace = Next(random, 84, 96), Consistency = Next(random, 82, 96), CarControl = Next(random, 84, 97), TireManagement = Next(random, 78, 94), Adaptability = Next(random, 82, 96), Reactions = Next(random, 82, 97), Awareness = Next(random, 82, 97), Overtaking = Next(random, 80, 96), Defending = Next(random, 80, 96) };
            var driver = new ReferenceLineDriver(new DriverProfile(id, abilities, (ulong)random.NextInt64(1, long.MaxValue)));
            var car = new RaceCar(id, new CarConfig(), new TireConfig { StartingSurfaceTempC = 86f, StartingCoreTempC = 84f }, driver,
                new CarState { Position = start.Position, Heading = sample.RefHeading, BatterySoc = 0.82f });
            Simulation.AddCar(car);
            var view = new FormulaCarView3D(); AddChild(view); view.Bind(car, Surface, Color.FromHtml(Liveries[i / 2]), number); _cars.Add(view);
        }
        AddChild(_camera); _camera.Initialize(Surface);
        AddChild(_hud); _hud.Initialize(this); _hud.Refresh(0);
        _camera.Update(0, _cars[SelectedCarIndex]);
        GD.Print($"LOWPOLY ready: {_cars.Count} car; circuit={track.LengthMeters:0}m; startup={watch.Elapsed.TotalSeconds:0.00}s");
    }
    private static float Next(Random r, float min, float max) => min + (float)r.NextDouble() * (max - min);
    public override void _ExitTree() => _leaving = true;

    public override void _Process(double delta)
    {
        if (Simulation == null || _cars.Count == 0) return;
        _sincePose += delta; _sinceStep += delta; _hudElapsed += delta;
        // The worker owns Core while stepping. Every Core read below occurs only after
        // completion; camera and map rendering consume copied poses, never live state.
        if (_step is { IsCompleted: true })
        {
            try { _coreMs = _step.GetAwaiter().GetResult(); }
            catch (Exception error) { IsPaused = true; GD.PushError(error.ToString()); }
            _step = null;
            RaceSeconds = Simulation.RaceTimeSeconds;
            foreach (var car in _cars) car.Capture();
            _sincePose = 0; _hudDirty = true;
        }
        if (_step == null)
        {
            while (_commands.TryDequeue(out var command))
            {
                var car = Simulation.Cars[command.Car];
                car.Strategy = new CarStrategy((TireUsageMode)Math.Clamp((int)car.Strategy.TireMode + command.Tire, 1, 5),
                    (BatteryOutputMode)Math.Clamp((int)car.Strategy.BatteryMode + command.Power, 1, 5));
            }
            if (_hudDirty || _hudElapsed >= 0.2) { _hud.Refresh(_coreMs); _hudDirty = false; _hudElapsed = 0; }
            if (!IsPaused && _sinceStep >= 1.0 / 60.0)
            {
                _sinceStep = 0;
                var simulation = Simulation;
                _step = Task.Run(() =>
                {
                    long started = Stopwatch.GetTimestamp();
                    simulation.Step(1f / 60f);
                    return Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                });
            }
        }
        float fraction = IsPaused ? 1f : Math.Clamp((float)(_sincePose / Math.Max(_coreMs / 1000.0, 1.0 / 60.0)), 0f, 1f);
        foreach (var car in _cars) { car.Render(fraction); car.Select(car == _cars[SelectedCarIndex], CameraMode == 1); }
        _camera.Update(delta, _cars[SelectedCarIndex]);
    }
    public void SetCameraMode(int mode) { _camera.SetMode(mode); _hudDirty = true; _hud.RefreshControls(); }
    public void SelectCar(int index) { SelectedCarIndex = (index % _cars.Count + _cars.Count) % _cars.Count; _hudDirty = true; }
    public void TogglePause() { IsPaused = !IsPaused; _hudDirty = true; _hud.RefreshControls(); }
    public override void _UnhandledInput(InputEvent input)
    {
        if (Simulation == null || _cars.Count == 0) return;
        if (input is InputEventMouseButton mouse && mouse.Pressed)
        {
            if (mouse.ButtonIndex == MouseButton.WheelUp) _camera.Zoom(0.88f);
            else if (mouse.ButtonIndex == MouseButton.WheelDown) _camera.Zoom(1.14f);
            else return;
        }
        else if (input is InputEventMouseMotion motion && motion.ButtonMask.HasFlag(MouseButtonMask.Right)) _camera.Orbit(-motion.Relative.X * 0.006f);
        else if (input is InputEventKey key && key.Pressed && !key.Echo)
        {
            switch (key.Keycode)
            {
                case Key.Key1: SetCameraMode(1); break;
                case Key.Key2: SetCameraMode(2); break;
                case Key.Key3: SetCameraMode(3); break;
                case Key.F: SetCameraMode(CameraMode == 1 ? 2 : 1); break;
                case Key.Left: SelectCar(SelectedCarIndex - 1); break;
                case Key.Right: SelectCar(SelectedCarIndex + 1); break;
                case Key.Space: TogglePause(); break;
                case Key.H: _hud.Visible = !_hud.Visible; break;
                case Key.Q: ChangeStrategy(-1, 0); break;
                case Key.E: ChangeStrategy(1, 0); break;
                case Key.A: ChangeStrategy(0, -1); break;
                case Key.D: ChangeStrategy(0, 1); break;
                default: return;
            }
        }
        else return;
        GetViewport().SetInputAsHandled();
    }
    private void ChangeStrategy(int tire, int power)
    {
        _commands.Enqueue((SelectedCarIndex, tire, power));
        _hudDirty = true;
    }
    private void CreateLighting()
    {
        AddChild(new WorldEnvironment
        {
            Environment = new Godot.Environment
            {
                BackgroundMode = Godot.Environment.BGMode.Color,
                BackgroundColor = Color.FromHtml("#c5ccba"),
                AmbientLightSource = Godot.Environment.AmbientSource.Color,
                AmbientLightColor = Color.FromHtml("#c7d5db"),
                AmbientLightEnergy = 0.42f,
                ReflectedLightSource = Godot.Environment.ReflectionSource.Disabled,
                TonemapMode = Godot.Environment.ToneMapper.Filmic,
                TonemapExposure = 0.85f,
            }
        });
        AddChild(new DirectionalLight3D
        {
            Name = "AfternoonSun",
            RotationDegrees = new Vector3(-48, -35, 0),
            LightColor = Color.FromHtml("#fff0ce"),
            LightEnergy = 0.9f,
            ShadowEnabled = true,
            DirectionalShadowMaxDistance = 700f,
            ShadowBias = 0.06f,
            ShadowNormalBias = 1.2f,
            DirectionalShadowMode = DirectionalLight3D.ShadowMode.Parallel4Splits
        });
    }
}
