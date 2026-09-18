using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using Godot;
using StintegyEVO.GodotApp.Car;
using StintegyEVO.GodotApp.Debug;
using StintegyEVO.GodotApp.Interop;
using StintegyEVO.GodotApp.Track;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using GVector2 = Godot.Vector2;
using NVector2 = System.Numerics.Vector2;

namespace StintegyEVO.GodotApp.Race;

/// <summary>
/// Godot's two-dimensional presentation adapter.  It can create a stationary
/// display grid for the standalone scene, or present a RaceSimulation assembled
/// by an external composition root.  It never chooses or implements a driver.
/// </summary>
public partial class RaceView : Node2D
{
    [Export] public TrackView? TrackRenderer { get; set; }
    [Export] public Camera2D? Camera { get; set; }
    [Export] public bool ShowFrameStats { get; set; } = true;
    [Export] public bool ExportCsvTelemetry { get; set; }

    private const int DefaultGridCarCount = 20;
    private const float FollowCameraZoom = 3f;

    private readonly List<CarView> _carViews = [];
    private readonly CarDashboard _dashboard = new();
    private readonly Label _telemetryLabel = new()
    {
        Position = new GVector2(12f, 42f),
        ZIndex = 1000
    };
    private RaceSimulation? _composedSimulation;
    private RaceSimulation? _simulation;
    private RaceCar? _playerCar;
    private RaceCsvTelemetryRecorder? _csvTelemetry;
    private FrameTimeMonitor? _frameTimeMonitor;
    private GVector2 _overviewCameraPosition;
    private GVector2 _overviewCameraZoom = GVector2.One;
    private int _selectedCarIndex;
    private bool _followSelectedCar;
    private bool _ready;

    public RaceSimulation? Simulation => _simulation;

    /// <summary>
    /// Supplies a simulation assembled outside the view. Call this before the
    /// node enters the scene tree; all cars and controllers remain owned by the
    /// caller's composition root.
    /// </summary>
    public void BindSimulation(RaceSimulation simulation)
    {
        ArgumentNullException.ThrowIfNull(simulation);
        if (_ready)
            throw new InvalidOperationException("BindSimulation must be called before RaceView is ready.");
        _composedSimulation = simulation;
    }

    /// <summary>
    /// Thin external-input bridge for hosts and smoke tests. This stores an
    /// already-decided command; it is not a driving policy or fallback driver.
    /// </summary>
    public void SetExternalInput(
        int carIndex,
        float desiredCurvature,
        float desiredAccel,
        float frontBrakeBiasOffset = 0f
    )
    {
        RaceCar car = GetCar(carIndex);
        car.ExternalInput = new DriverInput(
            desiredCurvature,
            desiredAccel,
            frontBrakeBiasOffset
        );
    }

    public override void _Ready()
    {
        _ready = true;
        if (TrackRenderer == null)
            throw new InvalidOperationException("TrackRenderer is not assigned.");

        TrackData track;
        if (_composedSimulation != null)
        {
            _simulation = _composedSimulation;
            track = _simulation.Track;
        }
        else
        {
            track = TrackFactory.SilverstoneStyleTestTrack();
            _simulation = new RaceSimulation(
                track,
                new RaceEnvironment { AirTempC = 25f, TrackTempC = 35f }
            );
            CreateDefaultGrid(track);
        }

        TrackRenderer.Initialize(track);
        ConfigureCamera(track);
        CreateCarViews();
        CreateHud();
        if (ShowFrameStats)
        {
            _frameTimeMonitor = new FrameTimeMonitor();
            AddChild(_frameTimeMonitor);
        }
        StartCsvTelemetryIfRequested();
        RefreshTelemetry();
    }

    public override void _PhysicsProcess(double delta)
    {
        if (_simulation == null)
            return;

        long coreStepStart = Stopwatch.GetTimestamp();
        if (HasControlSource())
        {
            _simulation.Step(Mathf.Min((float)delta, 0.05f));
            _frameTimeMonitor?.RecordCoreStep(
                Stopwatch.GetElapsedTime(coreStepStart).TotalMilliseconds
            );
        }

        if (_csvTelemetry != null && _playerCar != null)
        {
            _csvTelemetry.Write(
                _simulation.RaceTimeSeconds,
                _playerCar,
                _simulation.Track,
                _simulation.Environment
            );
        }

        foreach (CarView view in _carViews)
            view.SyncFromCore();
        UpdateCamera();
        RefreshTelemetry();
    }

    public override void _ExitTree()
    {
        _csvTelemetry?.Dispose();
        _csvTelemetry = null;
    }

    public override void _UnhandledInput(InputEvent inputEvent)
    {
        if (_playerCar == null || inputEvent is not InputEventKey key || !key.Pressed || key.Echo)
            return;

        if (key.Keycode == Key.F)
        {
            _followSelectedCar = !_followSelectedCar;
            UpdateCamera();
            RefreshTelemetry();
            GetViewport().SetInputAsHandled();
            return;
        }

        int carDelta = key.Keycode switch
        {
            Key.Left => -1,
            Key.Right => 1,
            _ => 0
        };
        if (carDelta != 0)
        {
            SelectObservedCar(carDelta);
            GetViewport().SetInputAsHandled();
            return;
        }

        int tireDelta = key.Keycode switch { Key.Q => -1, Key.E => 1, _ => 0 };
        int powerDelta = key.Keycode switch { Key.A => -1, Key.D => 1, _ => 0 };
        if (tireDelta == 0 && powerDelta == 0)
            return;

        int tire = Math.Clamp(
            (int)_playerCar.Strategy.TireMode + tireDelta,
            1,
            TireLadder.Usage.RungCount
        );
        int power = _playerCar.CarConfig.Powertrain.OutputLadder.Clamp(
            _playerCar.Strategy.PowerRung + powerDelta
        );
        _playerCar.Strategy = new CarStrategy((TireUsageMode)tire, power);
        RefreshTelemetry();
        GetViewport().SetInputAsHandled();
    }

    private bool HasControlSource()
    {
        if (_simulation == null)
            return false;
        foreach (RaceCar car in _simulation.Cars)
        {
            if (car.Driver is not null || car.ExternalInput != default)
                return true;
        }
        return false;
    }

    private void CreateDefaultGrid(TrackData track)
    {
        int carCount = Math.Min(DefaultGridCarCount, track.StartingGridCount);
        for (int gridPosition = 1; gridPosition <= carCount; gridPosition++)
        {
            Grid start = track.Grids[gridPosition];
            TrackSample sample = track.Sample(start.S);
            RaceCar car = new(
                $"display-{gridPosition:D2}",
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
            _simulation!.AddCar(car);
        }

        GD.Print($"Default 2D display: cars={carCount}; No controller; vehicles stationary");
    }

    private void CreateCarViews()
    {
        if (_simulation == null)
            return;

        for (int i = 0; i < _simulation.Cars.Count; i++)
        {
            RaceCar car = _simulation.Cars[i];
            CarView view = new();
            view.Bind(car, CarColor(i));
            AddChild(view);
            _carViews.Add(view);
        }

        if (_simulation.Cars.Count > 0)
        {
            _selectedCarIndex = 0;
            _playerCar = _simulation.Cars[0];
        }
    }

    private static Color CarColor(int index)
    {
        if (index == 0)
            return Color.FromHtml("#ff5d73");
        return Color.FromHsv((index * 0.173f) % 1f, 0.68f, 0.95f);
    }

    private void CreateHud()
    {
        CanvasLayer layer = new() { Layer = 90 };
        ColorRect panel = new()
        {
            Position = new GVector2(8f, 36f),
            Size = new GVector2(650f, 166f),
            Color = Color.FromHtml("#111820d8"),
            MouseFilter = Control.MouseFilterEnum.Ignore
        };
        _telemetryLabel.AddThemeColorOverride("font_color", Color.FromHtml("#f3f6f8"));
        _telemetryLabel.AddThemeFontSizeOverride("font_size", 13);
        _telemetryLabel.AddThemeConstantOverride("line_spacing", 1);
        layer.AddChild(panel);
        layer.AddChild(_telemetryLabel);
        AddChild(layer);
    }

    private void RefreshTelemetry()
    {
        if (_simulation == null)
            return;
        if (_playerCar == null)
        {
            _telemetryLabel.Text = "No cars in composed simulation";
            return;
        }

        CarState state = _playerCar.State;
        CarTelemetry telemetry = state.Telemetry;
        float frontTemp = Average(state.FrontLeft.SurfaceTempC, state.FrontRight.SurfaceTempC);
        float rearTemp = Average(state.RearLeft.SurfaceTempC, state.RearRight.SurfaceTempC);
        _dashboard.Refresh(_playerCar.CarConfig, state, _playerCar.Strategy);
        _telemetryLabel.Text =
            $"{_playerCar.Id}  {ControllerStatus(_playerCar)}  |  {state.Speed * 3.6f:0} km/h  |  Lap {_playerCar.Progress.Lap + 1}  Race {_simulation.RaceTimeSeconds:0.0}s  Cars {_simulation.Cars.Count}  Region {_playerCar.Progress.Region}  View {(_followSelectedCar ? "FOLLOW" : "MAP")}\n" +
            $"{StoresReadout()}  |  {ModesReadout()}  |  Air/Track {_simulation.Environment.AirTempC:0}/{_simulation.Environment.TrackTempC:0} C  |  Q/E {TireLadder.Usage.Label.ToLowerInvariant()}  A/D {_playerCar.CarConfig.Powertrain.OutputLadder.Label.ToLowerInvariant()}\n" +
            $"Input curvature/accel {telemetry.Input.DesiredCurvature:+0.000;-0.000;0.000} 1/m  {telemetry.Input.DesiredAccel:+0.00;-0.00;0.00} m/s²  |  Axle F/R {frontTemp:0.0}/{rearTemp:0.0} C\n" +
            $"Lateral use {telemetry.FrontLateralUse:0.00}/{telemetry.RearLateralUse:0.00}  Longitudinal use {telemetry.FrontLongitudinalUse:0.00}/{telemetry.RearLongitudinalUse:0.00}  Over-limit {telemetry.OverLimit:0.00}\n" +
            $"Wheel surf/core/wear  FL {WheelStatus(state.FrontLeft)}  |  FR {WheelStatus(state.FrontRight)}\n" +
            $"                         RL {WheelStatus(state.RearLeft)}  |  RR {WheelStatus(state.RearRight)}\n" +
            $"Slip {state.SideslipAngleRadians * 180f / MathF.PI:+0.0;-0.0;0.0} deg  Slide {telemetry.RearSlideSeverity:0.00}  GL {telemetry.CombinedGripLimiterCutAccel:0.00}  |  Yaw {state.YawRateRadiansPerSecond:+0.00;-0.00;0.00}/{telemetry.ReferenceYawRateRadiansPerSecond:+0.00;-0.00;0.00} rad/s";
    }

    private static string ControllerStatus(RaceCar car) =>
        car.Driver == null ? "No controller" : "Controller attached";

    private string StoresReadout()
    {
        StringBuilder readout = new();
        foreach (DashboardResource resource in _dashboard.Resources)
        {
            if (readout.Length > 0)
                readout.Append("  ");
            readout.Append($"{resource.Label} {resource.Fraction * 100f:0.0}%");
            if (resource.RemainingMassKg > 0f)
                readout.Append($" ({resource.RemainingMassKg:0} kg)");
        }
        if (_dashboard.OutputAvailability < 0.999f)
            readout.Append($"  OUTPUT {_dashboard.OutputAvailability * 100f:0}%");
        return readout.ToString();
    }

    private string ModesReadout()
    {
        StringBuilder readout = new();
        foreach (DashboardMode mode in _dashboard.Modes)
        {
            if (readout.Length > 0)
                readout.Append("  ");
            readout.Append($"{mode.Label} {mode.Rung} {mode.Ordinal}/{mode.RungCount}");
        }
        return readout.ToString();
    }

    private static float Average(float left, float right) => (left + right) * 0.5f;

    private static string WheelStatus(TireState tire) =>
        $"{tire.SurfaceTempC:0}/{tire.CoreTempC:0} C {tire.Wear * 100f:0.0}%";

    private void StartCsvTelemetryIfRequested()
    {
        string? setting = System.Environment.GetEnvironmentVariable("STINTEGY_CSV_TELEMETRY");
        if (!ExportCsvTelemetry && string.IsNullOrWhiteSpace(setting))
            return;

        string path = string.IsNullOrWhiteSpace(setting) || IsEnabledValue(setting)
            ? ProjectSettings.GlobalizePath("res://.tmp/telemetry.csv")
            : Path.GetFullPath(setting);
        _csvTelemetry = new RaceCsvTelemetryRecorder(path);
        GD.Print($"CSV telemetry: {path}");
    }

    private static bool IsEnabledValue(string value) =>
        value.Equals("1", StringComparison.OrdinalIgnoreCase) ||
        value.Equals("true", StringComparison.OrdinalIgnoreCase) ||
        value.Equals("yes", StringComparison.OrdinalIgnoreCase);

    private RaceCar GetCar(int carIndex)
    {
        if (_simulation == null)
            throw new InvalidOperationException("RaceView has not been initialized.");
        if ((uint)carIndex >= (uint)_simulation.Cars.Count)
            throw new ArgumentOutOfRangeException(nameof(carIndex));
        return _simulation.Cars[carIndex];
    }

    private void SelectObservedCar(int delta)
    {
        if (_simulation == null || _simulation.Cars.Count == 0)
            return;

        int count = _simulation.Cars.Count;
        _selectedCarIndex = (_selectedCarIndex + delta) % count;
        if (_selectedCarIndex < 0)
            _selectedCarIndex += count;
        _playerCar = _simulation.Cars[_selectedCarIndex];
        _followSelectedCar = true;
        UpdateCamera();
        RefreshTelemetry();
    }

    private void UpdateCamera()
    {
        if (Camera == null)
            return;

        if (_followSelectedCar && _playerCar != null)
        {
            Camera.Position = _playerCar.State.Position.ToGodot();
            Camera.Zoom = GVector2.One * FollowCameraZoom;
            return;
        }

        Camera.Position = _overviewCameraPosition;
        Camera.Zoom = _overviewCameraZoom;
    }

    private void ConfigureCamera(TrackData track)
    {
        if (Camera == null)
            return;

        NVector2 min = new(float.PositiveInfinity, float.PositiveInfinity);
        NVector2 max = new(float.NegativeInfinity, float.NegativeInfinity);
        const int samples = 128;
        for (int i = 0; i < samples; i++)
        {
            NVector2 point = track.Sample(track.LengthMeters * i / samples).Center;
            min = NVector2.Min(min, point);
            max = NVector2.Max(max, point);
        }
        NVector2 size = max - min;
        GVector2 viewportSize = GetViewportRect().Size;
        const float worldMargin = 60f;
        float horizontalZoom = viewportSize.X / MathF.Max(size.X + worldMargin, 1f);
        float verticalZoom = viewportSize.Y / MathF.Max(size.Y + worldMargin, 1f);
        float zoom = Mathf.Clamp(Mathf.Min(horizontalZoom, verticalZoom), 0.25f, 4f);

        _overviewCameraPosition = ((min + max) * 0.5f).ToGodot();
        _overviewCameraZoom = GVector2.One * zoom;
        UpdateCamera();
    }
}
