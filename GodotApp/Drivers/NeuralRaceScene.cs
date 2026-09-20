using Godot;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.GodotApp.LowPoly;
using System.Threading.Tasks;

namespace StintegyEVO.GodotApp.Drivers;

/// <summary>
/// The scene that puts the baked policy on the circuit.
///
/// It is a composition root and nothing else: it builds Silverstone, fits a
/// car the way the evaluation fits one, gives it the neural controller, and
/// hands the finished simulation to the presentation layer. The physics
/// runs live — what is on screen is the policy driving, not a recording of
/// it having driven.
///
/// The car is fitted the way the scoreboard fits it, because a driver graded
/// under one set of parts and shown under another is a different driver:
/// the combined-grip limiter at full strength, warm tyres at nominal stress,
/// the pit wall's Normal tyre and Normal power, and no perception noise.
/// The pack starts full.
/// </summary>
public partial class NeuralRaceScene : Node3D
{
    /// <summary>
    /// The network at the wheel. A path inside the project, so an exported
    /// build carries it; swap it to drive a different bake.
    /// </summary>
    [Export] public string ModelPath { get; set; } =
        "res://Assets/Drivers/parent3l.onnx";

    /// <summary>
    /// Where the car starts, in metres along the centreline, and how fast.
    /// Zero speed means "work it out from the corner ahead", which is what
    /// a training episode does.
    /// </summary>
    [Export] public float StartMeters { get; set; }
    [Export] public float StartSpeed { get; set; }

    private readonly RaceView3D _view = new() { Name = "RaceView3D" };
    private NeuralDriverController? _controller;

    public override async void _Ready()
    {
        var loading = new CanvasLayer();
        var label = new Label
        {
            Text = "STINTEGY\n\nPreparing Silverstone…",
            Position = new Vector2(44, 44)
        };
        label.AddThemeFontSizeOverride("font_size", 24);
        loading.AddChild(label);
        AddChild(loading);

        TrackData track;
        try
        {
            track = await Task.Run(TrackFactory.SilverstoneStyleTestTrack);
        }
        catch (System.Exception error)
        {
            label.Text = "Could not prepare circuit.\n" + error.Message;
            GD.PushError(error.ToString());
            return;
        }
        if (!IsInstanceValid(this))
            return;
        loading.QueueFree();

        string model = ProjectSettings.GlobalizePath(ModelPath);
        if (!Godot.FileAccess.FileExists(ModelPath) && !System.IO.File.Exists(model))
        {
            GD.PushError(
                $"No driver network at {ModelPath}. Export one with " +
                "Training/python/export_onnx.py."
            );
            return;
        }

        var simulation = new RaceSimulation(
            track,
            new RaceEnvironment { AirTempC = 25f, TrackTempC = 35f }
        );

        // The evaluation's own fitment.
        var config = new CarConfig { CombinedGripLimiterStrength = 1f };
        var tires = new TireConfig
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        var strategy = new CarStrategy(TireUsageMode.Normal, 3);

        _controller = new NeuralDriverController(model, config, tires, strategy);
        float s = track.WrapS(StartMeters);
        TrackSample sample = track.Sample(s);
        float speed = StartSpeed > 0f ? StartSpeed : RollingStart(track, s);
        var car = new RaceCar(
            "neural-01",
            config,
            tires,
            new Driver(
                new DriverProfile("parent3l", new DriverAbilities()),
                _controller
            ),
            new CarState
            {
                Position = sample.Center,
                Heading = sample.Heading,
                Speed = speed,
                Energy = PowertrainState.Filled(1f)
            }
        )
        {
            Strategy = strategy
        };
        simulation.AddCar(car);

        _view.BindSimulation(simulation);
        AddChild(_view);
        GD.Print(
            $"NEURAL: {System.IO.Path.GetFileName(model)} at the wheel; " +
            $"{NeuralDriverController.DecisionHz:0} Hz decisions; " +
            $"rolling start {speed:0.0} m/s at {s:0} m; " +
            "tyre Normal / power Normal; limiter 1.0; pack full"
        );
    }

    /// <summary>
    /// The speed a training episode would have started at here: fast enough
    /// to be driving, slow enough for the corner in the next forty metres.
    /// Mirrors DirectDriveDuelEnvironment.EstimateStartSpeed, which is
    /// private to the training environment — a standing start is a state
    /// the policy has hardly ever been in, and the first thing anybody
    /// watching would see is a car that cannot pull away.
    /// </summary>
    private static float RollingStart(TrackData track, float s)
    {
        float curvature = 0f;
        for (int i = 0; i < 6; i++)
        {
            curvature = Mathf.Max(
                curvature, Mathf.Abs(track.Sample(s + i * 8f).Curvature)
            );
        }
        float safe = Mathf.Sqrt(18f / Mathf.Max(curvature, 0.002f));
        return Mathf.Clamp(safe * 0.75f, 20f, 60f);
    }

    public override void _ExitTree() => _controller?.Dispose();
}
