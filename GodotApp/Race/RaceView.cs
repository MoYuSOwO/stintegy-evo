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
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Drivers.Learned;
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

    /// <summary>
    /// Whether the trained policy drives, or the scripted grid does.
    ///
    /// The checkpoint on the other side of this switch is a Silverstone
    /// specialist trained alone, and what it is worth is a lap time against
    /// the analytic driver on an empty circuit. So it gets an empty circuit:
    /// one car, no traffic, the same instruction the evaluation used. Twenty
    /// scripted cars in front of it would measure the queue rather than the
    /// policy. Turn this off for the scripted grid this scene has always
    /// had.
    /// </summary>
    [Export] public bool UseLearnedDriver { get; set; } = true;

    private const int DefaultGridCarCount = 20;
    private const int DefaultRosterSeed = 0x5345564F;
    private const float FollowCameraZoom = 3f;

    /// <summary>
    /// The trained actor, exported by Training/python/export_policy.py from
    /// checkpoints/bestalpha002.pt — 325k steps, a 1:40.929 clean lap
    /// against the analytic driver's 1:47.050.
    /// </summary>
    private const string LearnedPolicyPath =
        "res://Assets/Drivers/silverstone-expert.nn";

    private readonly List<CarView> _carViews = [];
    private readonly CarDashboard _dashboard = new();
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
    private LapBoard? _lapBoard;
    private StreamWriter? _learnedTrace;

    public override void _Ready()
    {
        if (TrackRenderer == null)
            throw new InvalidOperationException("TrackRenderer is not assigned.");

        // The circuit the shipped policy was trained on. The scripted grid
        // is happy anywhere; the specialist is not, and a maiden voyage
        // that put it somewhere else would be measuring generalization
        // nobody has claimed.
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
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
        if (UseLearnedDriver)
            CreateLearnedCar(track);
        else
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
        float step = Mathf.Min((float)delta, 0.05f);
        _simulation.Step(step);
        if (_lapBoard != null && _playerCar != null &&
            _lapBoard.Update(_playerCar, _simulation.RaceTimeSeconds, step))
        {
            GD.Print($"Lap {_lapBoard.Laps}: {_lapBoard.Readout()}");
        }
        if (_learnedTrace != null && _playerCar != null &&
            _playerCar.Driver is DirectDriveRaceDriver traced)
        {
            NVector2 at = _playerCar.State.Position;
            _learnedTrace.WriteLine(
                $"{_simulation.RaceTimeSeconds:0.###},{at.X:0.##},{at.Y:0.##}," +
                $"{_playerCar.State.Speed:0.###},{traced.LastAction[0]:0.####}," +
                $"{(_playerCar.Progress.Region == TrackRegion.RacingSurface ? 1 : 0)}"
            );
        }
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
        _learnedTrace?.Dispose();
        _learnedTrace = null;
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

        // Stepped against the ladders this car actually has rather than a
        // remembered five, so a powertrain that offers a different number of
        // settings is steppable the day it is fitted.
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

    private RaceCar AddRaceCar(
        string id,
        TrackData track,
        Grid start,
        CarStrategy strategy,
        Color color,
        IRaceDriver? driver = null,
        float initialSpeedMetersPerSecond = 0f,
        TireConfig? tireConfig = null,
        float chargeFraction = 0.82f
    )
    {
        TrackSample startSample = track.Sample(start.S);
        TireConfig tires = tireConfig ?? new TireConfig
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
                Energy = PowertrainState.Filled(chargeFraction)
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

    /// <summary>
    /// One car, driven by the trained policy on its own clock.
    ///
    /// The car is built to the conditions the evaluation quoted its lap
    /// under rather than to this scene's usual ones: tyres at ninety
    /// degrees, four fifths of a charge, and the Normal/Normal instruction
    /// every evaluated lap was driven on. Those are the training host's
    /// numbers, and a lap time is only comparable to another lap time when
    /// the car underneath it is the same car.
    ///
    /// The clock, by contrast, is deliberately the other one. Training held
    /// the driver externally because there the agent's step boundary is the
    /// decision boundary; a race lets the driver time itself. Same contract,
    /// different owner — and this is the first time anything has run the
    /// internal side of it with a real policy behind it.
    /// </summary>
    private void CreateLearnedCar(TrackData track)
    {
        byte[] weights = Godot.FileAccess.GetFileAsBytes(LearnedPolicyPath);
        if (weights.Length == 0)
        {
            throw new InvalidOperationException(
                $"No policy at {LearnedPolicyPath}. Export one with " +
                "Training/python/export_policy.py, or clear UseLearnedDriver."
            );
        }

        MlpDrivingPolicy policy = MlpDrivingPolicy.FromBytes(weights);
        DirectDriveRaceDriver driver = new(policy);
        RaceCar car = AddRaceCar(
            "learned-01",
            track,
            track.Grids[1],
            new CarStrategy(TireUsageMode.Normal, 3),
            Color.FromHtml("#4ad6a0"),
            driver,
            tireConfig: new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            chargeFraction: 0.8f
        );
        _playerCar = car;
        _followSelectedCar = true;
        _lapBoard = new LapBoard(track.LengthMeters);
        StartLearnedTraceIfRequested(track);

        GD.Print(
            $"Learned driver: {LearnedPolicyPath}, " +
            $"{policy.Network.InputSize} in / {policy.Network.OutputSize} out, " +
            $"{policy.Network.LayerCount} layers, " +
            $"{DirectDriveRaceDriver.DefaultDecisionHz:0} Hz internal clock, " +
            "strategy=Normal/Normal"
        );
    }

    /// <summary>
    /// Where the learned car actually went, if asked for.
    ///
    /// The existing CSV recorder is built around the analytic driver's
    /// planner telemetry and writes nothing for a car that has no planner,
    /// which is every learned car. This writes the four columns a line can
    /// be drawn from, and the track's own edges beside them, so that
    /// "it drives a racing line" is a picture somebody can look at rather
    /// than a claim about a lap time.
    ///
    ///     STINTEGY_LEARNED_TRACE=/tmp/trace.csv godot --headless ...
    /// </summary>
    private void StartLearnedTraceIfRequested(TrackData track)
    {
        string? path =
            System.Environment.GetEnvironmentVariable("STINTEGY_LEARNED_TRACE");
        if (string.IsNullOrWhiteSpace(path))
            return;

        _learnedTrace = new StreamWriter(Path.GetFullPath(path));
        _learnedTrace.WriteLine("time_s,x,y,speed_mps,curvature_cmd,on_track");
        using StreamWriter edges = new(
            Path.ChangeExtension(Path.GetFullPath(path), ".track.csv")
        );
        edges.WriteLine("s_m,left_x,left_y,center_x,center_y,right_x,right_y");
        const int samples = 1200;
        for (int i = 0; i <= samples; i++)
        {
            float s = track.LengthMeters * i / samples;
            TrackSample sample = track.Sample(s);
            NVector2 left = sample.LeftEdge;
            NVector2 right = sample.RightEdge;
            edges.WriteLine(
                $"{s:0.###},{left.X:0.###},{left.Y:0.###}," +
                $"{sample.Center.X:0.###},{sample.Center.Y:0.###}," +
                $"{right.X:0.###},{right.Y:0.###}"
            );
        }
        GD.Print($"Learned trace: {Path.GetFullPath(path)}");
    }

    /// <summary>
    /// Lap times taken the way the evaluation takes them, so that the
    /// number on this HUD and the number in the training log mean the same
    /// thing. A lap is charged the seconds it spent off the racing surface
    /// and the seconds it spent against a barrier; a lap charged nothing is
    /// clean, and only a clean lap is quoted as a lap time.
    /// </summary>
    private sealed class LapBoard(float lapMeters)
    {
        private float _lapStartSeconds;
        private float _offCourseSeconds;
        private float _wallSecondsAtLapStart;
        private int _lastLap = -1;

        public float OffCourseThisLap => _offCourseSeconds;
        public float LastLapSeconds { get; private set; }
        public float LastLapCharged { get; private set; }
        public float BestCleanSeconds { get; private set; } = float.PositiveInfinity;
        public int Laps { get; private set; }
        public int CleanLaps { get; private set; }

        /// <summary>Returns true on the frame a lap is completed.</summary>
        public bool Update(RaceCar car, float raceTimeSeconds, float dt)
        {
            if (car.Progress.Region != TrackRegion.RacingSurface)
                _offCourseSeconds += dt;

            int lap = (int)(car.Progress.RaceDistanceMeters / lapMeters);
            if (lap == _lastLap)
                return false;
            bool completed = _lastLap >= 0;
            if (completed)
            {
                LastLapSeconds = raceTimeSeconds - _lapStartSeconds;
                float wall = car.BoundaryContactSeconds - _wallSecondsAtLapStart;
                LastLapCharged = LastLapSeconds + _offCourseSeconds + wall;
                Laps++;
                if (_offCourseSeconds <= 0f && wall <= 0f)
                {
                    CleanLaps++;
                    BestCleanSeconds = MathF.Min(BestCleanSeconds, LastLapSeconds);
                }
            }
            _lastLap = lap;
            _lapStartSeconds = raceTimeSeconds;
            _offCourseSeconds = 0f;
            _wallSecondsAtLapStart = car.BoundaryContactSeconds;
            return completed;
        }

        public string Readout()
        {
            if (Laps == 0)
                return $"Lap 1 in progress  |  off {_offCourseSeconds:0.00}s";
            return
                $"Last {Clock(LastLapSeconds)} (charged {Clock(LastLapCharged)})  |  " +
                $"Best clean {Clock(BestCleanSeconds)}  |  " +
                $"Clean {CleanLaps}/{Laps}  |  off {_offCourseSeconds:0.00}s";
        }

        private static string Clock(float seconds)
        {
            return float.IsFinite(seconds)
                ? $"{(int)(seconds / 60f)}:{seconds % 60f:00.000}"
                : "--:--.---";
        }
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
        _dashboard.Refresh(_playerCar.CarConfig, state, _playerCar.Strategy);
        _telemetryLabel.Text =
            $"{_playerCar.Id}  {state.Speed * 3.6f:0} km/h  |  Lap {_playerCar.Progress.Lap + 1}  Race {_simulation.RaceTimeSeconds:0.0}s  Cars {_simulation.Cars.Count}  Region {_playerCar.Progress.Region}  View {(_followSelectedCar ? "FOLLOW" : "MAP")}\n" +
            $"{StoresReadout()}  |  {ModesReadout()}  |  Air/Track {_simulation.Environment.AirTempC:0}/{_simulation.Environment.TrackTempC:0} C  |  Q/E {TireLadder.Usage.Label.ToLowerInvariant()}  A/D {_playerCar.CarConfig.Powertrain.OutputLadder.Label.ToLowerInvariant()}\n" +
            $"Axle F/R {frontTemp:0.0}/{rearTemp:0.0} C  |  Lateral use {telemetry.FrontLateralUse:0.00}/{telemetry.RearLateralUse:0.00}\n" +
            $"Wheel surf/core/wear  FL {WheelStatus(state.FrontLeft)}  |  FR {WheelStatus(state.FrontRight)}\n" +
            $"                         RL {WheelStatus(state.RearLeft)}  |  RR {WheelStatus(state.RearRight)}\n" +
            $"Slip {state.SideslipAngleRadians * 180f / MathF.PI:+0.0;-0.0;0.0} deg  Slide {telemetry.RearSlideSeverity:0.00}  TC {telemetry.TractionControlCutAccel:0.00}  |  Yaw {state.YawRateRadiansPerSecond:+0.00;-0.00;0.00}/{telemetry.ReferenceYawRateRadiansPerSecond:+0.00;-0.00;0.00} rad/s\n" +
            trafficStatus;
    }

    private string TrafficStatus(RaceCar car)
    {
        // A learned car has no traffic evaluator to report on — collision
        // avoidance is the policy's own skill — so this line shows what the
        // policy last asked the car for instead, which is the one thing
        // about it that is otherwise invisible.
        if (car.Driver is DirectDriveRaceDriver learned)
        {
            ReadOnlySpan<float> action = learned.LastAction;
            string board = _lapBoard?.Readout() ?? "no lap board";
            return
                $"Policy curvature {action[0]:+0.000;-0.000; 0.000}  " +
                $"accel {action[1]:+0.000;-0.000; 0.000}  |  " +
                $"Region {car.Progress.Region}  |  {board}";
        }

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

    /// <summary>
    /// Every store the car carries and how much of each is left, named by the
    /// powertrain rather than by this panel - so a car with a tank reads out
    /// a tank here, and a hybrid reads out both of its stores, without the
    /// panel being told either exists.
    /// </summary>
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

        // Only worth the space when it is biting, which is when the car is
        // about to feel wrong for a reason the driver cannot otherwise see.
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
            readout.Append(
                $"{mode.Label} {mode.Rung} {mode.Ordinal}/{mode.RungCount}"
            );
        }
        return readout.ToString();
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
