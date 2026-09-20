using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.GodotApp.Drivers;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// What the policy is fed in a race, against what it was fed in the bake.
///
/// The failure this guards against is the quiet one: an in-game driver that
/// builds its own observation, gets a block subtly wrong, and produces
/// confident nonsense that looks like a bad policy rather than a bad
/// harness. The eval scoreboard's own index bug was exactly this shape.
///
/// So the same car, on the same circuit, at the same station and speed,
/// with the same fitment, is observed twice: once through the training
/// environment, once through the controller the game puts at the wheel.
/// The two vectors are compared block by block. They are not required to
/// be identical — the training environment's own warm-up and its randomised
/// draws are not reproduced here — only to agree channel for channel to
/// within a tolerance that no misplaced block could survive.
/// </summary>
public sealed class NeuralDriverInGameTests
{
    private const string Track = "silverstone";
    private const float StartS = 0f;
    private const float StartSpeed = 42f;
    private const float Charge = 0.8f;
    private const float WarmupSeconds = 1e-6f;

    private static string ModelPath => Path.GetFullPath(
        Path.Combine(
            AppContext.BaseDirectory,
            "..", "..", "..", "..", "..",
            "Assets", "Drivers", "parent3l.onnx"
        )
    );

    private static readonly (string Name, int Offset, int Size)[] Blocks =
    [
        ("geometry", DirectDriveObservation.GeometryOffset, 198),
        ("tyres and battery", DirectDriveObservation.TireAndBatteryOffset, 17),
        ("mode", DirectDriveObservation.ModeOffset, 1),
        ("aero", DirectDriveObservation.AeroOffset, 3),
        ("road and limits", DirectDriveObservation.RoadAndLimitsOffset, 13),
        ("resources and budget", DirectDriveObservation.ResourceAndBudgetOffset, 5),
        ("ego", DirectDriveObservation.EgoOffset, 14),
        ("opponents", DirectDriveObservation.OpponentOffset, 96),
        ("previous frame", DirectDriveObservation.PreviousDynamicOffset, 110),
    ];

    [Fact]
    public void TheGameFeedsThePolicyTheObservationTheBakeDid()
    {
        if (!HasModel())
            return;

        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        RaceSimulation simulation = Build(track, out RaceCar car, out var driver);
        using NeuralDriverController controller = driver;
        // The first driving substep decides before any physics has run, so
        // that observation alone has cold tyre loads; the warm-up hair of
        // time that the training environment also takes puts both sides on
        // the same footing from the second decision on.
        simulation.Step(WarmupSeconds);

        DirectDriveDuelEnvironment environment = new(
            solo: true,
            egoStrategy: new CarStrategy(TireUsageMode.Normal, 3)
        );
        float[] fromTraining = new float[DirectDriveObservation.ObservationSize];
        environment.ResetScenario(
            Track, seed: 1, egoStartS: StartS, forwardGapMeters: 20f,
            startSpeedMetersPerSecond: StartSpeed, fromTraining
        );
        float[] previous = (float[])fromTraining.Clone();

        // Both sides are driven by the same policy: the action the game is
        // holding is handed to the training environment, so the two cars
        // take the same line and any disagreement in the observation is the
        // observation's own rather than two cars drifting apart.
        float[] worst = new float[Blocks.Length];
        for (int step = 0; step < 45; step++)
        {
            // The game steps first, because its decision is taken at the
            // start of the step and is what drove it; the training
            // environment is then given that same action for the same
            // interval, so the two cars stay on one line and a difference
            // in the observation is the observation's own.
            simulation.Step(1f / NeuralDriverController.DecisionHz);
            float[] action =
                [controller.LastCurvatureNorm, controller.LastAccelerationNorm];

            ReadOnlySpan<float> inGame = controller.LastObservation;
            if (step == 0)
            {
                // Still the cold first decision: it was taken before any
                // physics ran, and the beat puts the second one at the start
                // of the next step. Everything after this is a like-for-like
                // reading of a state both sides have integrated.
                environment.Step(action, fromTraining);
                fromTraining.CopyTo(previous, 0);
                continue;
            }
            for (int b = 0; b < Blocks.Length; b++)
            {
                (string name, int offset, int size) = Blocks[b];
                for (int i = 0; i < size; i++)
                {
                    int channel = offset + i;
                    Assert.True(
                        float.IsFinite(inGame[channel]),
                        $"{name} channel {channel} was not finite in the game"
                    );
                    float difference =
                        MathF.Abs(inGame[channel] - previous[channel]);
                    worst[b] = MathF.Max(worst[b], difference);
                    // A tenth of a channel's own O(1) scale. Two readings of
                    // one state agree far closer than this; no block that has
                    // slipped by a channel, and no reading the game has built
                    // for itself, comes anywhere near it.
                    Assert.True(
                        difference < 0.1f,
                        $"step {step}: {name} channel {channel} differs: game " +
                        $"{inGame[channel]:0.0000} vs training " +
                        $"{previous[channel]:0.0000}"
                    );
                }
            }
            environment.Step(action, fromTraining);
            fromTraining.CopyTo(previous, 0);
            if (environment.IsTerminal)
                break;
        }

        for (int b = 0; b < Blocks.Length; b++)
            Console.WriteLine($"{Blocks[b].Name,-22} worst channel gap {worst[b]:0.000000}");
        Assert.True(car.Progress.TotalDistance > 100f, "the game's car barely moved");
    }

    /// <summary>
    /// The policy drives the car for a few seconds and the car is still on
    /// the road, pointing the right way, at a racing speed. This is the
    /// smoke test for the whole chain — observation, network, action, and
    /// the envelope the action is stretched onto — because every way of
    /// getting one of them wrong ends with the car stopped, spinning or in
    /// a wall, and none of them ends with a lap.
    /// </summary>
    [Fact]
    public void ThePolicyDrivesTheCarDownTheRoad()
    {
        if (!HasModel())
            return;

        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        RaceSimulation simulation = Build(track, out RaceCar car, out var driver);
        using NeuralDriverController controller = driver;

        for (int i = 0; i < 150; i++)   // ten seconds at 15 Hz
            simulation.Step(1f / 15f);

        Assert.True(
            controller.Decisions >= 140,
            $"only {controller.Decisions} decisions in ten seconds of racing"
        );
        Assert.Equal(TrackRegion.RacingSurface, car.Progress.Region);
        Assert.InRange(car.State.Speed, 25f, 100f);
        Assert.InRange(car.Progress.TotalDistance, 250f, 900f);
        Assert.False(car.State.Spinning, "the car was spinning after ten seconds");
    }

    private static RaceSimulation Build(
        TrackData track,
        out RaceCar car,
        out NeuralDriverController controller
    )
    {
        RaceSimulation simulation = new(
            track,
            new RaceEnvironment { AirTempC = 25f, TrackTempC = 35f }
        );
        CarConfig config = new() { CombinedGripLimiterStrength = 1f };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        CarStrategy strategy = new(TireUsageMode.Normal, 3);
        controller = new NeuralDriverController(ModelPath, config, tires, strategy);
        TrackSample sample = track.Sample(StartS);
        car = new RaceCar(
            "neural-01",
            config,
            tires,
            new Driver(new DriverProfile("parent3l", new DriverAbilities()), controller),
            new CarState
            {
                Position = sample.Center,
                Heading = sample.Heading,
                Speed = StartSpeed,
                Energy = PowertrainState.Filled(Charge)
            }
        )
        {
            Strategy = strategy
        };
        simulation.AddCar(car);
        return simulation;
    }

    /// <summary>
    /// The network is exported rather than committed, and this xunit has no
    /// runtime skip, so a tree without one says so and passes rather than
    /// failing: the suite is not the place to discover that somebody has
    /// not run the exporter.
    /// </summary>
    private static bool HasModel()
    {
        if (File.Exists(ModelPath))
            return true;
        Console.WriteLine(
            $"no network at {ModelPath}; export one with " +
            "Training/python/export_onnx.py. Skipped."
        );
        return false;
    }
}
