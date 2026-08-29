using System.Globalization;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Drivers.Learned;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.TrainingHost.Environment;

namespace StintegyEVO.TrainingHost;

internal static class Program
{
    public static int Main(string[] args)
    {
        if (args.Length == 1 && args[0] == "--scan")
            return RunReachabilityScan();
        if (args.Length == 1 && args[0] == "--trace")
            return RunTrace();

        TextWriter diagnostics = Console.Error;
        try
        {
            Stream protocolInput = Console.OpenStandardInput();
            Stream protocolOutput = Console.OpenStandardOutput();
            Console.SetOut(diagnostics);
            (
                int batchSize,
                long seedBase,
                string? trackFamily,
                float minimumForwardGapMeters,
                float maximumForwardGapMeters,
                float episodeDurationSeconds,
                CarStrategy opponentStrategy,
                float opponentPace,
                bool solo
            ) =
                ParseOptions(args);
            BatchedTrainingHost host = new(
                batchSize,
                seedBase,
                trackFamily,
                minimumForwardGapMeters,
                maximumForwardGapMeters,
                episodeDurationSeconds,
                opponentStrategy,
                opponentPace,
                solo
            );
            host.Run(protocolInput, protocolOutput, diagnostics);
            return 0;
        }
        catch (Exception exception)
        {
            diagnostics.WriteLine($"Training host failed: {exception.Message}");
            return 1;
        }
    }

    private static (
        int BatchSize,
        long SeedBase,
        string? TrackFamily,
        float MinimumForwardGapMeters,
        float MaximumForwardGapMeters,
        float EpisodeDurationSeconds,
        CarStrategy OpponentStrategy,
        float OpponentPace,
        bool Solo
    ) ParseOptions(string[] args)
    {
        int batchSize = 1;
        long seedBase = 0;
        string? trackFamily = null;
        float minimumForwardGapMeters =
            DirectDriveDuelEnvironment.DefaultMinimumForwardGapMeters;
        float maximumForwardGapMeters =
            DirectDriveDuelEnvironment.DefaultMaximumForwardGapMeters;
        float episodeDurationSeconds =
            DirectDriveDuelEnvironment.DefaultEpisodeDurationSeconds;
        CarStrategy opponentStrategy = CarStrategy.Default;
        float opponentPace = 70f;
        bool solo = false;
        for (int i = 0; i < args.Length; i++)
        {
            string option = args[i];
            if (option == "--solo")
            {
                solo = true;
                continue;
            }
            if (i + 1 >= args.Length)
                throw new ArgumentException($"Missing value for '{option}'.");
            string value = args[++i];
            switch (option)
            {
                case "--batch":
                    if (!int.TryParse(
                            value,
                            NumberStyles.Integer,
                            CultureInfo.InvariantCulture,
                            out batchSize
                        ) || batchSize <= 0)
                    {
                        throw new ArgumentException(
                            "--batch must be a positive integer."
                        );
                    }
                    break;
                case "--seed-base":
                    if (!long.TryParse(
                            value,
                            NumberStyles.Integer,
                            CultureInfo.InvariantCulture,
                            out seedBase
                        ))
                    {
                        throw new ArgumentException(
                            "--seed-base must be a signed 64-bit integer."
                        );
                    }
                    break;
                case "--track":
                    trackFamily = value;
                    break;
                case "--minimum-forward-gap":
                    minimumForwardGapMeters = ParsePositiveFloat(
                        option,
                        value
                    );
                    break;
                case "--maximum-forward-gap":
                    maximumForwardGapMeters = ParsePositiveFloat(
                        option,
                        value
                    );
                    break;
                case "--episode-seconds":
                    episodeDurationSeconds = ParsePositiveFloat(
                        option,
                        value
                    );
                    break;
                case "--opponent-strategy":
                    opponentStrategy = value switch
                    {
                        "normal" => CarStrategy.Default,
                        "protect" => new CarStrategy(
                            TireUsageMode.Protect,
                            BatteryOutputMode.Save
                        ),
                        _ => throw new ArgumentException(
                            "--opponent-strategy must be normal or protect."
                        )
                    };
                    break;
                case "--opponent-pace":
                    opponentPace = ParseFiniteFloat(option, value);
                    if (opponentPace < 0f || opponentPace > 100f)
                    {
                        throw new ArgumentException(
                            "--opponent-pace must be between zero and 100."
                        );
                    }
                    break;
                default:
                    throw new ArgumentException($"Unknown option '{option}'.");
            }
        }
        if (maximumForwardGapMeters < minimumForwardGapMeters)
        {
            throw new ArgumentException(
                "--maximum-forward-gap must not be smaller than " +
                "--minimum-forward-gap."
            );
        }
        return (
            batchSize,
            seedBase,
            trackFamily,
            minimumForwardGapMeters,
            maximumForwardGapMeters,
            episodeDurationSeconds,
            opponentStrategy,
            opponentPace,
            solo
        );
    }

    private static float ParseFiniteFloat(string option, string value)
    {
        if (!float.TryParse(
                value,
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out float parsed
            ) || !float.IsFinite(parsed))
        {
            throw new ArgumentException($"{option} must be a finite number.");
        }
        return parsed;
    }

    private static float ParsePositiveFloat(string option, string value)
    {
        float parsed = ParseFiniteFloat(option, value);
        if (parsed <= 0f)
        {
            throw new ArgumentException(
                $"{option} must be a finite positive number."
            );
        }
        return parsed;
    }

    private static int RunReachabilityScan()
    {
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        string[] tracks = ["silverstone"];
        float[] actions = [-0.4f, -0.25f, 0f, 0.25f, 0.4f];
        float[] pulseDurationsSeconds = [3f, 6f, 30f];
        int passes = 0;
        int contacts = 0;
        int episodes = 0;
        float[] actionBuffer = new float[DirectDriveObservation.ActionSize];

        foreach (string trackFamily in tracks)
        {
            DirectDriveDuelEnvironment trackProbe = new();
            trackProbe.ResetTrack(trackFamily, seed: 0, observation);
            float startS = FindBestPassingWindowStart(
                trackProbe.Simulation.Track
            );
            float attackProgress = MeasureSoloProgress(
                trackProbe.Simulation.Track,
                startS,
                new ReferenceLineDriver(),
                new CarStrategy(
                    TireUsageMode.Attack,
                    BatteryOutputMode.Attack
                )
            );
            float normalProgress = MeasureSoloProgress(
                trackProbe.Simulation.Track,
                startS + 14f,
                new ReferenceLineDriver(
                    profile: DirectDriveDuelEnvironment.TrainingOpponentProfile
                ),
                new CarStrategy(
                    TireUsageMode.Normal,
                    BatteryOutputMode.Normal
                )
            );
            Console.WriteLine(
                $"solo track={trackFamily} attack={attackProgress:0.0} " +
                $"normal={normalProgress:0.0} advantage={attackProgress - normalProgress:0.0}m/30s"
            );
            foreach (float sideAction in actions)
            {
                ReadOnlySpan<float> durations = sideAction == 0f
                    ? [30f]
                    : pulseDurationsSeconds;
                foreach (float pulseDuration in durations)
                {
                    DirectDriveDuelEnvironment environment = new();
                    environment.ResetScenario(
                        trackFamily,
                        seed: 0,
                        startS,
                        forwardGapMeters: 14f,
                        startSpeedMetersPerSecond: 60f,
                        observation
                    );
                    TrainingTerminalReason reason = TrainingTerminalReason.None;
                    int followSteps = 0;
                    int stopSteps = 0;
                    int clearSteps = 0;
                    while (!environment.IsTerminal)
                    {
                        float action = environment.ElapsedSeconds < pulseDuration
                            ? sideAction
                            : 0f;
                        actionBuffer[0] = action;
                        TrainingStepResult result = environment.Step(
                            actionBuffer,
                            observation
                        );
                        reason = result.TerminalReason;
                        TrafficSpeedConstraintKind kind =
                            ((ReferenceLineDriver)environment.Ego.Driver)
                            .LastTelemetry.TrafficConstraintKind;
                        if (kind == TrafficSpeedConstraintKind.Follow)
                            followSteps++;
                        else if (kind == TrafficSpeedConstraintKind.Stop)
                            stopSteps++;
                        else
                            clearSteps++;
                    }

                    episodes++;
                    passes += reason == TrainingTerminalReason.Passed ? 1 : 0;
                    contacts += reason == TrainingTerminalReason.Contact ? 1 : 0;
                    Console.WriteLine(
                        $"track={environment.TrackFamily,-12} s={startS,7:0} " +
                        $"action={sideAction,5:0.00} pulse={pulseDuration,4:0.0}s " +
                        $"result={reason,-7} " +
                        $"lead={environment.SignedLeadDistanceMeters,7:0.0} " +
                        $"min={environment.MinimumSignedLeadDistanceMeters,6:0.0} " +
                        $"lat={environment.MaximumAbsoluteReferenceOffsetMeters,4:0.0} " +
                        $"F/S/C={followSteps}/{stopSteps}/{clearSteps}"
                    );
                }
            }
        }

        Console.WriteLine(
            $"summary episodes={episodes} passes={passes} contacts={contacts}"
        );
        return passes > 0 ? 0 : 1;
    }

    private static float FindBestPassingWindowStart(
        StintegyEVO.Core.Track.TrackData track
    )
    {
        const float windowMeters = 500f;
        const float probeStepMeters = 20f;
        float bestS = 0f;
        float bestScore = float.PositiveInfinity;
        for (float s = 0f; s < track.LengthMeters; s += 10f)
        {
            float maximumCurvature = 0f;
            float totalCurvature = 0f;
            float minimumHalfWidth = float.PositiveInfinity;
            int count = 0;
            for (float ahead = 0f;
                 ahead <= windowMeters;
                 ahead += probeStepMeters)
            {
                StintegyEVO.Core.Track.TrackSample sample = track.Sample(
                    s + ahead
                );
                float curvature = MathF.Abs(sample.RefCurvature);
                maximumCurvature = MathF.Max(maximumCurvature, curvature);
                totalCurvature += curvature;
                minimumHalfWidth = MathF.Min(
                    minimumHalfWidth,
                    sample.HalfWidth
                );
                count++;
            }
            float score = maximumCurvature * 2000f +
                          totalCurvature / Math.Max(count, 1) * 1000f -
                          minimumHalfWidth * 0.01f;
            if (score >= bestScore)
                continue;
            bestScore = score;
            bestS = s;
        }
        return bestS;
    }

    private static float MeasureSoloProgress(
        TrackData track,
        float startS,
        ReferenceLineDriver driver,
        CarStrategy strategy
    )
    {
        TrackSample sample = track.Sample(startS);
        RaceCar car = new(
            "solo",
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            driver,
            new CarState
            {
                Position = sample.RefPosition,
                Heading = sample.RefHeading,
                Speed = 60f,
                BatterySoc = 0.8f
            }
        )
        {
            Strategy = strategy
        };
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        float origin = car.Progress.TotalDistance;
        for (int i = 0; i < 30 * 60; i++)
            simulation.Step(1f / 60f);
        return car.Progress.TotalDistance - origin;
    }

    private static int RunTrace()
    {
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        DirectDriveDuelEnvironment probe = new();
        probe.ResetTrack("silverstone", seed: 0, observation);
        float startS = FindBestPassingWindowStart(probe.Simulation.Track);
        foreach (float forwardGap in new[] { 8f, 10f })
        {
            const float action = 0.35f;
            DirectDriveDuelEnvironment environment = new();
            environment.ResetScenario(
                "silverstone",
                seed: 0,
                startS,
                forwardGapMeters: forwardGap,
                startSpeedMetersPerSecond: 60f,
                observation
            );
            float[] actionBuffer = [action];
            Console.WriteLine(
                $"TRACE action={action:0.00} gap={forwardGap:0.0}"
            );
            int step = 0;
            while (!environment.IsTerminal && environment.ElapsedSeconds < 12f)
            {
                environment.Step(actionBuffer, observation);
                step++;
                if (step % 10 != 0)
                    continue;

                RaceCar ego = environment.Ego;
                RaceCar opponent = environment.Opponent;
                TrackPose pose = environment.Simulation.Track.Project(
                    ego.State.Position
                );
                ReferenceLineDriverTelemetry telemetry =
                    ((ReferenceLineDriver)ego.Driver).LastTelemetry;
                Console.WriteLine(
                    $"t={environment.ElapsedSeconds,4:0.0} " +
                    $"s={pose.S,6:0} gap={environment.SignedLeadDistanceMeters,6:0.0} " +
                    $"v={ego.State.Speed,5:0.0}/{opponent.State.Speed,5:0.0} " +
                    $"lat={pose.D - pose.Sample.RefOffset,5:0.0} " +
                    $"kind={telemetry.TrafficConstraintKind,-6} " +
                    $"vref={telemetry.ReferencePathTargetSpeed,5:0.0} " +
                    $"vtgt={telemetry.TargetSpeed,5:0.0} " +
                    $"loss={telemetry.TrafficTimeLossSeconds,4:0.00} " +
                    $"wake={ego.State.AirVelocityDeficit,4:0.00}/" +
                    $"{ego.State.WakeDownforceLoss,4:0.00}"
                );
            }
        }
        return 0;
    }
}
