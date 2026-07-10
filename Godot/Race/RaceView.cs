using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.Presentation.Car;
using StintegyEVO.Presentation.Debug;
using StintegyEVO.Presentation.Interop;
using StintegyEVO.Presentation.Track;
using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;
using GVector2 = Godot.Vector2;
using NVector2 = System.Numerics.Vector2;

namespace StintegyEVO.Presentation.Race;

public partial class RaceView : Node2D
{
    [Export] public TrackView? TrackRenderer { get; set; }
    [Export] public Camera2D? Camera { get; set; }
    [Export] public bool ShowFrameStats { get; set; } = true;

    private readonly List<CarView> _carViews = [];
    private readonly Label _telemetryLabel = new()
    {
        Position = new GVector2(12f, 82f),
        ZIndex = 1000
    };
    private RaceSimulation? _simulation;
    private RaceCar? _playerCar;

    public override void _Ready()
    {
        if (TrackRenderer == null)
            throw new InvalidOperationException("TrackRenderer is not assigned.");

        TrackData track = TrackFactory.SimpleTestTrack();
        _simulation = new RaceSimulation(track, new RaceEnvironment { AirTempC = 25f });
        TrackRenderer.Initialize(track);
        ConfigureCamera(track);
        CreateHud();

        _playerCar = AddRaceCar(
            id: "player",
            track,
            start: track.Grids[1],
            new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Normal),
            Color.FromHtml("#ff5d73")
        );
        AddRaceCar(
            id: "opponent",
            track,
            start: track.Grids[2],
            new CarStrategy(TireUsageMode.Push, BatteryOutputMode.Eco),
            Color.FromHtml("#4da3ff")
        );

        if (ShowFrameStats)
            AddChild(new FrameTimeMonitor());
        RefreshTelemetry();
    }

    public override void _PhysicsProcess(double delta)
    {
        if (_simulation == null)
            return;

        _simulation.Step(Mathf.Min((float)delta, 0.05f));
        foreach (CarView view in _carViews)
            view.SyncFromCore();
        RefreshTelemetry();
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
        Color color
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
            new ReferenceLineDriver(),
            new CarState
            {
                Position = start.Position,
                Heading = startSample.RefHeading,
                Speed = 8f,
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
            Size = new GVector2(330f, 146f),
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
        _telemetryLabel.Text =
            $"STINTEGYEVO  race {_simulation.RaceTimeSeconds:0.0}s  lap {_playerCar.Progress.Lap + 1}\n" +
            $"Speed {state.Speed * 3.6f:0} km/h   SOC {state.BatterySoc * 100f:0.0}%\n" +
            $"Tire {_playerCar.Strategy.TireMode}   Battery {_playerCar.Strategy.BatteryMode}\n" +
            $"Front {frontTemp:0.0} C   Rear {rearTemp:0.0} C   Use {telemetry.FrontLateralUse:0.00}/{telemetry.RearLateralUse:0.00}\n" +
            $"Region {_playerCar.Progress.Region}   Q/E tire  A/D battery";
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
}
