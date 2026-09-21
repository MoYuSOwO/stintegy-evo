using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using System.IO;
using System.Threading.Tasks;
using Godot;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.GodotApp.Race;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>
/// Low-poly 3D presentation for a RaceSimulation. The standalone scene creates
/// one stationary car without a controller. Hosts may bind an externally
/// assembled simulation before _Ready; this node only renders and steps it.
/// </summary>
/// <remarks>
/// Main scene: Levels/lowpoly.tscn. After IsInitialized, call SetExternalInput on
/// the Godot main thread; commands queue and apply once the physics worker has
/// released the world. While the view owns a simulation the host must not read,
/// mutate or step it from another thread, and controllers (which run on the
/// worker) should use only their DriverContext. Leaving the tree joins the worker
/// and hands the simulation back. STINTEGY_CSV_TELEMETRY=1 (or a path) records
/// physical telemetry after each completed physics task.
/// </remarks>
public partial class RaceView3D : Node3D
{
    public RaceSimulation Simulation { get; private set; } = null!;
    public TrackSurfaceGeometry Surface => _circuit.Surface;
    public int SelectedCarIndex { get; private set; }
    public bool IsPaused { get; private set; }
    public int CameraMode => _camera.Mode;
    public bool IsPreview => !_simulationStarted;
    public bool IsInitialized { get; private set; }
    public float RaceSeconds { get; private set; }
    public float SimulationRate => _coreMs > 0 ? MathF.Min(1f, 1000f / (60f * (float)_coreMs)) : 1f;

    private readonly CircuitView3D _circuit = new() { Name = "Circuit" };
    private readonly RaceCameraRig _camera = new() { Name = "CameraRig" };
    private readonly RaceHud3D _hud = new() { Name = "RaceHud" };
    private readonly List<FormulaCarView3D> _cars = [];
    private double _hudElapsed, _coreMs, _renderTime, _warmup = 0.1;
    private long _completedTicks;
    private readonly FixedStepBudget _budget = new();
    private readonly record struct CarPose(System.Numerics.Vector2 Position, float Heading, float SteerAngle);
    private sealed record StepBatch(double CoreMs, CarPose[][] Frames);
    private Task<StepBatch>? _step;
    private readonly Queue<(int Car, int Tire, int Power)> _commands = [];
    private readonly Queue<(int Car, DriverInput Input)> _externalInputs = [];
    private RaceCsvTelemetryRecorder? _csvTelemetry;
    private bool _simulationStarted;
    private bool _hudDirty;
    private bool _leaving;
    private bool _ready;
    private RaceSimulation? _composedSimulation;
    private static readonly string[] Liveries =
        ["#b93f30", "#e0bb53", "#507f80", "#ece5d1", "#3e6653", "#456583", "#c17b45", "#7a788b", "#77834b", "#393e42"];

    /// <summary>
    /// Supplies a simulation assembled by an external composition root. Call
    /// this before the node enters the scene tree. The simulation owns its cars,
    /// optional drivers and external inputs.
    /// </summary>
    public void BindSimulation(RaceSimulation simulation)
    {
        ArgumentNullException.ThrowIfNull(simulation);
        if (_ready)
            throw new InvalidOperationException("BindSimulation must be called before RaceView3D is ready.");
        if (simulation.Cars.Count == 0)
            throw new ArgumentException("The supplied simulation must contain at least one car.", nameof(simulation));
        _composedSimulation = simulation;
    }

    /// <summary>
    /// Call on the Godot main thread. Queues an already-decided command until
    /// the physics worker yields ownership, and starts the standalone preview.
    /// A later zero command means coasting, not pausing or freezing physics.
    /// </summary>
    public void SetExternalInput(
        int carIndex,
        float desiredCurvature,
        float desiredAccel,
        float frontBrakeBiasOffset = 0f
    )
    {
        if (!IsInitialized)
            throw new InvalidOperationException("Wait for the view to initialize before supplying input.");
        if ((uint)carIndex >= (uint)_cars.Count)
            throw new ArgumentOutOfRangeException(nameof(carIndex));
        if (!float.IsFinite(desiredCurvature) || !float.IsFinite(desiredAccel) || !float.IsFinite(frontBrakeBiasOffset))
            throw new ArgumentOutOfRangeException(nameof(desiredAccel), "Control commands must be finite.");
        _externalInputs.Enqueue((carIndex, new DriverInput(desiredCurvature, desiredAccel, frontBrakeBiasOffset)));
        _simulationStarted = true;
        _hudDirty = true;
    }

    public override async void _Ready()
    {
        _ready = true;
        var watch = Stopwatch.StartNew();
        foreach (var assembly in new[] { typeof(RaceView3D).Assembly, typeof(RaceSimulation).Assembly })
        {
            var config = assembly.GetCustomAttribute<AssemblyConfigurationAttribute>()?.Configuration ?? "unknown";
            var debug = assembly.GetCustomAttribute<DebuggableAttribute>();
            GD.Print($"LOWPOLY assembly: {assembly.GetName().Name}; configuration={config}; jit_optimization_disabled={debug?.IsJITOptimizerDisabled}; path={assembly.Location}");
        }

        var loading = new CanvasLayer();
        var label = new Label { Text = "STINTEGY\n\nPreparing Silverstone…", Position = new Vector2(44, 44) };
        label.AddThemeFontSizeOverride("font_size", 24);
        loading.AddChild(label);
        AddChild(loading);

        TrackData track;
        try
        {
            track = _composedSimulation?.Track ?? await Task.Run(TrackFactory.SilverstoneStyleTestTrack);
        }
        catch (Exception error)
        {
            if (!_leaving)
                label.Text = "Could not prepare circuit.\n" + error.Message;
            GD.PushError(error.ToString());
            return;
        }
        if (_leaving || !IsInstanceValid(this))
            return;

        loading.QueueFree();
        Simulation = _composedSimulation ?? new RaceSimulation(
            track,
            new RaceEnvironment { AirTempC = 25f, TrackTempC = 35f }
        );
        if (_composedSimulation == null)
            AddDefaultCar(track);
        // External composition owns the initial state, including freely coasting
        // cars with zero commands. Only the built-in, unconnected display is idle.
        _simulationStarted = _composedSimulation is not null;
        RaceSeconds = Simulation.RaceTimeSeconds;

        AddChild(_circuit);
        // The circuit is Silverstone here, so its plan is Silverstone's; a
        // scene that builds another one names that one's.
        _circuit.Initialize(track, "res://Levels/scenery/silverstone.json");
        CreateLighting();
        CreateCarViews();
        if (_cars.Count == 0)
        {
            GD.PushError("RaceView3D requires at least one car to present a simulation.");
            return;
        }

        AddChild(_camera);
        _camera.Initialize(Surface);
        AddChild(_hud);
        _hud.Initialize(this);
        _hud.Refresh(0);
        _camera.Update(0, _cars[SelectedCarIndex]);
        string? telemetryPath = System.Environment.GetEnvironmentVariable("STINTEGY_CSV_TELEMETRY");
        if (!string.IsNullOrWhiteSpace(telemetryPath))
        {
            string path = telemetryPath == "1" || telemetryPath.Equals("true", StringComparison.OrdinalIgnoreCase)
                ? ProjectSettings.GlobalizePath("res://.tmp/telemetry.csv")
                : Path.GetFullPath(telemetryPath);
            _csvTelemetry = new RaceCsvTelemetryRecorder(path);
        }
        IsInitialized = true;
        GD.Print($"LOWPOLY ready: {_cars.Count} car; circuit={track.LengthMeters:0}m; mode={(IsPreview ? "preview — no controller" : "externally composed simulation")}; startup={watch.Elapsed.TotalSeconds:0.00}s");
    }

    private void AddDefaultCar(TrackData track)
    {
        Grid start = track.Grids[1];
        TrackSample sample = track.Sample(start.S);
        var car = new RaceCar(
            "display-01",
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 86f,
                StartingCoreTempC = 84f
            },
            state: new CarState
            {
                Position = sample.Center,
                Heading = sample.Heading,
                Speed = 0f,
                Energy = PowertrainState.Filled(0.82f)
            }
        );
        Simulation.AddCar(car);
    }

    private void CreateCarViews()
    {
        for (int i = 0; i < Simulation.Cars.Count; i++)
        {
            var view = new FormulaCarView3D();
            AddChild(view);
            view.Bind(Simulation.Cars[i], Surface, i == 0 ? Color.FromHtml("#4ad6a0") : Color.FromHtml(Liveries[i % Liveries.Length]), i + 1);
            _cars.Add(view);
        }
    }

    public override void _ExitTree()
    {
        _leaving = true;
        IsInitialized = false;
        // Return ownership of an injected simulation before the host reuses it.
        try { _step?.GetAwaiter().GetResult(); }
        catch (Exception error) { GD.PushError(error.ToString()); }
        _step = null;
        _csvTelemetry?.Dispose();
        _csvTelemetry = null;
    }

    public override void _Process(double delta)
    {
        if (!IsInitialized)
            return;

        _hudElapsed += delta;
        if (!IsPaused && _simulationStarted)
            _budget.Advance(delta);
        else
            _budget.Reset();

        // The worker owns Core while stepping. Every Core read below occurs only
        // after completion; camera and map rendering consume copied poses.
        if (_step is { IsCompleted: true })
        {
            try
            {
                var batch = _step.GetAwaiter().GetResult();
                _coreMs = batch.CoreMs;
                foreach (var frame in batch.Frames)
                {
                    double time = ++_completedTicks * FixedStepBudget.StepSeconds;
                    for (int i = 0; i < frame.Length; i++)
                        _cars[i].Capture(time, frame[i].Position, frame[i].Heading, frame[i].SteerAngle);
                }
            }
            catch (Exception error)
            {
                IsPaused = true;
                GD.PushError(error.ToString());
            }
            _step = null;
            RaceSeconds = Simulation.RaceTimeSeconds;
            if (_csvTelemetry is not null)
                foreach (RaceCar car in Simulation.Cars)
                    _csvTelemetry.Write(RaceSeconds, car, Simulation.Track, Simulation.Environment);
        }

        if (_step == null)
        {
            while (_externalInputs.TryDequeue(out var input))
                Simulation.Cars[input.Car].ExternalInput = input.Input;
            while (_commands.TryDequeue(out var command))
            {
                var car = Simulation.Cars[command.Car];
                car.Strategy = new CarStrategy(
                    (TireUsageMode)Math.Clamp((int)car.Strategy.TireMode + command.Tire, 1, TireLadder.Usage.RungCount),
                    car.CarConfig.Powertrain.OutputLadder.Clamp(car.Strategy.PowerRung + command.Power)
                );
            }

            if (_hudDirty || _hudElapsed >= 0.2)
            {
                _hud.Refresh(_coreMs);
                _hudDirty = false;
                _hudElapsed = 0;
            }

            int steps = IsPaused || !_simulationStarted ? 0 : _budget.TakeSteps();
            if (steps > 0)
            {
                var simulation = Simulation;
                _step = Task.Run(() =>
                {
                    long started = Stopwatch.GetTimestamp();
                    var frames = new CarPose[steps][];
                    for (int tick = 0; tick < steps; tick++)
                    {
                        simulation.Step(1f / 60f);
                        var frame = new CarPose[simulation.Cars.Count];
                        for (int i = 0; i < frame.Length; i++)
                        {
                            var car = simulation.Cars[i];
                            frame[i] = new CarPose(car.State.Position, car.State.Heading, car.State.SteerAngleRadians);
                        }
                        frames[tick] = frame;
                    }
                    return new StepBatch(Stopwatch.GetElapsedTime(started).TotalMilliseconds / steps, frames);
                });
            }
        }

        if (!IsPaused)
        {
            double held = Math.Min(_warmup, delta);
            _warmup -= held;
            _renderTime = Math.Min(_renderTime + delta - held, _completedTicks * FixedStepBudget.StepSeconds);
        }
        for (int i = 0; i < _cars.Count; i++)
        {
            _cars[i].Render(_renderTime);
            _cars[i].Select(i == SelectedCarIndex, CameraMode == 2);
        }
        _camera.Update(delta, _cars[SelectedCarIndex]);
    }

    public void SetCameraMode(int mode)
    {
        _camera.SetMode(mode);
        _hudDirty = true;
        _hud.RefreshControls();
    }

    public void SelectCar(int index)
    {
        if (_cars.Count == 0)
            return;
        SelectedCarIndex = (index % _cars.Count + _cars.Count) % _cars.Count;
        _hudDirty = true;
    }

    public void TogglePause()
    {
        IsPaused = !IsPaused;
        _budget.Reset();
        _hudDirty = true;
        _hud.RefreshControls();
    }

    public override void _UnhandledInput(InputEvent input)
    {
        if (Simulation == null || _cars.Count == 0)
            return;
        if (input is InputEventMouseButton mouse && mouse.Pressed)
        {
            if (mouse.ButtonIndex == MouseButton.WheelUp)
                _camera.Zoom(0.88f);
            else if (mouse.ButtonIndex == MouseButton.WheelDown)
                _camera.Zoom(1.14f);
            else
                return;
        }
        else if (input is InputEventMouseMotion motion && motion.ButtonMask.HasFlag(MouseButtonMask.Right))
            _camera.Orbit(-motion.Relative.X * 0.006f);
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
        else
            return;
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
