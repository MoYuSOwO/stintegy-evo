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
    [Export] public bool EnableTrafficAvoidanceDemo { get; set; } = true;
    [Export] public float TrafficDemoLeadSpeedKph { get; set; } = 45f;
    [Export] public float TrafficDemoCameraZoom { get; set; } = 3f;

    private readonly List<CarView> _carViews = [];
    private readonly Label _telemetryLabel = new()
    {
        Position = new GVector2(12f, 82f),
        ZIndex = 1000
    };
    private RaceSimulation? _simulation;
    private RaceCar? _playerCar;
    private RaceCar? _demoLeadCar;
    private RaceCsvTelemetryRecorder? _csvTelemetry;
    private FrameTimeMonitor? _frameTimeMonitor;

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

        if (EnableTrafficAvoidanceDemo)
        {
            float leadSpeed = MathF.Max(5f, TrafficDemoLeadSpeedKph / 3.6f);
            _demoLeadCar = AddRaceCar(
                id: "pace-car",
                track,
                start: track.Grids[1],
                new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Save),
                Color.FromHtml("#4aa8ff"),
                new SpeedLimitedReferenceDriver(leadSpeed),
                initialSpeedMetersPerSecond: leadSpeed
            );
            _playerCar = AddRaceCar(
                id: "player",
                track,
                start: track.Grids[5],
                new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Normal),
                Color.FromHtml("#ff5d73"),
                initialSpeedMetersPerSecond: 25f
            );
        }
        else
        {
            _playerCar = AddRaceCar(
                id: "player",
                track,
                start: track.Grids[1],
                new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Normal),
                Color.FromHtml("#ff5d73")
            );
        }
        UpdateCamera();
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

    private void CreateHud()
    {
        CanvasLayer layer = new() { Layer = 90 };
        ColorRect panel = new()
        {
            Position = new GVector2(8f, 76f),
            Size = new GVector2(510f, 266f),
            Color = Color.FromHtml("#111820d8"),
            MouseFilter = Control.MouseFilterEnum.Ignore
        };
        _telemetryLabel.AddThemeColorOverride("font_color", Color.FromHtml("#f3f6f8"));
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
        string demoStatus = _demoLeadCar == null
            ? string.Empty
            : $"   Blue {_demoLeadCar.State.Speed * 3.6f:0} km/h";
        _telemetryLabel.Text =
            $"STINTEGYEVO  race {_simulation.RaceTimeSeconds:0.0}s  lap {_playerCar.Progress.Lap + 1}\n" +
            $"Speed {state.Speed * 3.6f:0} km/h   SOC {state.BatterySoc * 100f:0.0}%{demoStatus}\n" +
            $"{trafficStatus}\n" +
            $"Tire {_playerCar.Strategy.TireMode}   Battery {_playerCar.Strategy.BatteryMode}\n" +
            $"Air {_simulation.Environment.AirTempC:0} C   Track {_simulation.Environment.TrackTempC:0} C\n" +
            $"Front {frontTemp:0.0} C   Rear {rearTemp:0.0} C   Use {telemetry.FrontLateralUse:0.00}/{telemetry.RearLateralUse:0.00}\n" +
            $"FL {WheelStatus(state.FrontLeft)}   FR {WheelStatus(state.FrontRight)}\n" +
            $"RL {WheelStatus(state.RearLeft)}   RR {WheelStatus(state.RearRight)}\n" +
            $"Slip {state.SideslipAngleRadians * 180f / MathF.PI:+0.0;-0.0;0.0} deg   Slide {telemetry.RearSlideSeverity:0.00}   TC {telemetry.TractionControlCutAccel:0.00}\n" +
            $"Yaw {state.YawRateRadiansPerSecond:+0.00;-0.00;0.00}/{telemetry.ReferenceYawRateRadiansPerSecond:+0.00;-0.00;0.00} rad/s\n" +
            $"Region {_playerCar.Progress.Region}   Q/E tire  A/D battery";
    }

    private static string TrafficStatus(RaceCar car)
    {
        if (car.Driver is not ReferenceLineDriver driver)
            return "Traffic unavailable";

        ReferenceLineDriverTelemetry telemetry = driver.LastTelemetry;
        if (telemetry.TrafficConstraintKind == TrafficSpeedConstraintKind.None)
            return "Traffic CLEAR   Catch the blue pace car";

        return
            $"Traffic {telemetry.TrafficConstraintKind.ToString().ToUpperInvariant()} " +
            $"{telemetry.TrafficOpponentId ?? "?"}   " +
            $"gap {telemetry.TrafficCurrentClearanceMeters:0.0} m\n" +
            $"Traffic plan {telemetry.TrafficConstraintDistanceMeters:0} m ahead @ " +
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

        Camera.Position = ((min + max) * 0.5f).ToGodot();
        Camera.Zoom = GVector2.One * zoom;
    }

    private void UpdateCamera()
    {
        if (!EnableTrafficAvoidanceDemo || Camera == null || _playerCar == null)
            return;

        Camera.Position = _playerCar.State.Position.ToGodot();
        Camera.Zoom = GVector2.One * MathF.Max(0.25f, TrafficDemoCameraZoom);
    }

    private sealed class SpeedLimitedReferenceDriver(
        float speedLimitMetersPerSecond
    ) : IRaceDriver
    {
        private const float SpeedGain = 2.5f;
        private readonly ReferenceLineDriver _inner = new();
        private readonly float _speedLimitMetersPerSecond = MathF.Max(
            1f,
            speedLimitMetersPerSecond
        );

        public float TireEnergyEfficiency => _inner.TireEnergyEfficiency;

        public void Initialize(in RaceDriverInitContext context)
        {
            _inner.Initialize(in context);
        }

        public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
        {
            DriverInput input = _inner.GetControl(in context, dt);
            float speedLimitedAcceleration =
                SpeedGain *
                (_speedLimitMetersPerSecond - context.Car.State.Speed) +
                context.Car.State.Telemetry.LossAccel;
            return input with
            {
                DesiredAccel = MathF.Min(
                    input.DesiredAccel,
                    speedLimitedAcceleration
                )
            };
        }
    }
}
