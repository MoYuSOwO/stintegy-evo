using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Godot;
using StintegyEVO.GodotApp.Car;
using StintegyEVO.GodotApp.Debug;
using StintegyEVO.GodotApp.Interop;
using StintegyEVO.GodotApp.Track;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using GVector2 = Godot.Vector2;
using NVector2 = System.Numerics.Vector2;

namespace StintegyEVO.GodotApp.Race;

public partial class RaceView : Node2D
{
    [Export] public TrackView? TrackRenderer { get; set; }
    [Export] public Camera2D? Camera { get; set; }
    [Export] public bool ShowFrameStats { get; set; } = true;
    [Export] public bool ExportCsvTelemetry { get; set; }

    private const int DefaultGridCarCount = 20;
    private const int DefaultRosterSeed = 0x5345564F;
    private const float FollowCameraZoom = 3f;

    private readonly List<CarView> _carViews = [];
    private readonly Label _telemetryLabel = new()
    {
        Position = new GVector2(12f, 42f),
        ZIndex = 1000
    };
    private RaceSimulation? _simulation;
    private RaceCar? _playerCar;
    private RaceCsvTelemetryRecorder? _csvTelemetry;
    private FrameTimeMonitor? _frameTimeMonitor;
    private GVector2 _overviewCameraPosition;
    private GVector2 _overviewCameraZoom = GVector2.One;
    private int _selectedCarIndex;
    private bool _followSelectedCar;

    public override void _Ready()
    {
        if (TrackRenderer == null)
            throw new InvalidOperationException("TrackRenderer is not assigned.");

        TrackData track = TrackFactory.SimpleTestTrack();
        _simulation = new RaceSimulation(
            track,
            new RaceEnvironment
            {
                AirTempC = 25f,
                TrackTempC = 35f
            }
        );
        TrackRenderer.Initialize(track);
        ConfigureCamera(track);
        CreateHud();
        CreateDefaultGrid(track);
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
        _simulation.Step(Mathf.Min((float)delta, 0.05f));
        _frameTimeMonitor?.RecordCoreStep(
            Stopwatch.GetElapsedTime(coreStepStart).TotalMilliseconds
        );
        if (_csvTelemetry != null && _playerCar != null)
            _csvTelemetry.Write(
                _simulation.RaceTimeSeconds,
                _playerCar,
                _simulation.Track,
                _simulation.Environment
            );
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
        int batteryDelta = key.Keycode switch { Key.A => -1, Key.D => 1, _ => 0 };
        if (tireDelta == 0 && batteryDelta == 0)
            return;

        int tire = Math.Clamp((int)_playerCar.Strategy.TireMode + tireDelta, 1, 5);
        int battery = Math.Clamp((int)_playerCar.Strategy.BatteryMode + batteryDelta, 1, 5);
        _playerCar.Strategy = new CarStrategy((TireUsageMode)tire, (BatteryOutputMode)battery);
        RefreshTelemetry();
        GetViewport().SetInputAsHandled();
    }

    private RaceCar AddRaceCar(
        string id,
        TrackData track,
        Grid start,
        CarStrategy strategy,
        Color color,
        IRaceDriver? driver = null,
        float initialSpeedMetersPerSecond = 0f
    )
    {
        TrackSample startSample = track.Sample(start.S);
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 86f,
            StartingCoreTempC = 84f
        };
        RaceCar car = new(
            id,
            new CarConfig(),
            tires,
            driver ?? new ReferenceLineDriver(),
            new CarState
            {
                Position = start.Position,
                Heading = startSample.RefHeading,
                Speed = MathF.Max(0f, initialSpeedMetersPerSecond),
                BatterySoc = 0.82f
            }
        )
        {
            Strategy = strategy
        };

        _simulation!.AddCar(car);
        CarView view = new();
        view.Bind(car, color);
        AddChild(view);
        _carViews.Add(view);
        return car;
    }

    private void CreateDefaultGrid(TrackData track)
    {
        Random random = new(DefaultRosterSeed);
        int carCount = Math.Min(DefaultGridCarCount, track.StartingGridCount);
        for (int gridPosition = 1; gridPosition <= carCount; gridPosition++)
        {
            string id = $"grid-{gridPosition:D2}";
            DriverProfile profile = new(
                id,
                CreateDriverAbilities(random),
                (ulong)random.NextInt64(1, long.MaxValue)
            );
            Color color = gridPosition == 1
                ? Color.FromHtml("#ff5d73")
                : Color.FromHsv(
                    (float)random.NextDouble(),
                    0.68f,
                    0.95f
                );
            RaceCar car = AddRaceCar(
                id,
                track,
                track.Grids[gridPosition],
                CarStrategy.Default,
                color,
                new ReferenceLineDriver(profile)
            );
            _playerCar ??= car;
        }

        GD.Print(
            $"Default grid: cars={carCount}, seed={DefaultRosterSeed}, " +
            "strategy=Normal/Normal"
        );
    }

    private static DriverAbilities CreateDriverAbilities(Random random)
    {
        return new DriverAbilities
        {
            Pace = NextRating(random, 84f, 96f),
            Consistency = NextRating(random, 82f, 96f),
            CarControl = NextRating(random, 84f, 97f),
            TireManagement = NextRating(random, 78f, 94f),
            Adaptability = NextRating(random, 82f, 96f),
            Reactions = NextRating(random, 82f, 97f),
            Awareness = NextRating(random, 82f, 97f),
            Overtaking = NextRating(random, 80f, 96f),
            Defending = NextRating(random, 80f, 96f)
        };
    }

    private static float NextRating(Random random, float minimum, float maximum)
    {
        return minimum + (float)random.NextDouble() * (maximum - minimum);
    }

    private void CreateHud()
    {
        CanvasLayer layer = new() { Layer = 90 };
        ColorRect panel = new()
        {
            Position = new GVector2(8f, 36f),
            Size = new GVector2(570f, 148f),
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
        if (_simulation == null || _playerCar == null)
            return;

        var state = _playerCar.State;
        var telemetry = state.Telemetry;
        float frontTemp = (state.FrontLeft.SurfaceTempC + state.FrontRight.SurfaceTempC) * 0.5f;
        float rearTemp = (state.RearLeft.SurfaceTempC + state.RearRight.SurfaceTempC) * 0.5f;
        string trafficStatus = TrafficStatus(_playerCar);
        _telemetryLabel.Text =
            $"{_playerCar.Id}  {state.Speed * 3.6f:0} km/h  |  Lap {_playerCar.Progress.Lap + 1}  Race {_simulation.RaceTimeSeconds:0.0}s  Cars {_simulation.Cars.Count}  Region {_playerCar.Progress.Region}  View {(_followSelectedCar ? "FOLLOW" : "MAP")}\n" +
            $"SOC {state.BatterySoc * 100f:0.0}%  |  Tire {_playerCar.Strategy.TireMode}  Battery {_playerCar.Strategy.BatteryMode}  |  Air/Track {_simulation.Environment.AirTempC:0}/{_simulation.Environment.TrackTempC:0} C  |  Q/E tire  A/D batt\n" +
            $"Axle F/R {frontTemp:0.0}/{rearTemp:0.0} C  |  Lateral use {telemetry.FrontLateralUse:0.00}/{telemetry.RearLateralUse:0.00}\n" +
            $"Wheel surf/core/wear  FL {WheelStatus(state.FrontLeft)}  |  FR {WheelStatus(state.FrontRight)}\n" +
            $"                         RL {WheelStatus(state.RearLeft)}  |  RR {WheelStatus(state.RearRight)}\n" +
            $"Slip {state.SideslipAngleRadians * 180f / MathF.PI:+0.0;-0.0;0.0} deg  Slide {telemetry.RearSlideSeverity:0.00}  TC {telemetry.TractionControlCutAccel:0.00}  |  Yaw {state.YawRateRadiansPerSecond:+0.00;-0.00;0.00}/{telemetry.ReferenceYawRateRadiansPerSecond:+0.00;-0.00;0.00} rad/s\n" +
            trafficStatus;
    }

    private static string TrafficStatus(RaceCar car)
    {
        if (car.Driver is not ReferenceLineDriver driver)
            return "Traffic unavailable";

        ReferenceLineDriverTelemetry telemetry = driver.LastTelemetry;
        if (telemetry.TrafficConstraintKind == TrafficSpeedConstraintKind.None)
            return "Traffic UNCONSTRAINED";

        return
            $"Traffic {telemetry.TrafficConstraintKind.ToString().ToUpperInvariant()} " +
            $"{telemetry.TrafficOpponentId ?? "?"}  |  " +
            $"Gap {telemetry.TrafficCurrentClearanceMeters:0.0} m  |  " +
            $"Plan {telemetry.TrafficConstraintDistanceMeters:0} m @ " +
            $"{telemetry.TrafficTargetSpeedMetersPerSecond * 3.6f:0} km/h";
    }

    private static string WheelStatus(TireState tire)
    {
        return $"{tire.SurfaceTempC:0}/{tire.CoreTempC:0} C {tire.Wear * 100f:0.0}%";
    }

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

    private static bool IsEnabledValue(string value)
    {
        return value.Equals("1", StringComparison.OrdinalIgnoreCase) ||
               value.Equals("true", StringComparison.OrdinalIgnoreCase) ||
               value.Equals("yes", StringComparison.OrdinalIgnoreCase);
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
