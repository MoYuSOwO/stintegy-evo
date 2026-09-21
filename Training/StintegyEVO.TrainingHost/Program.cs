using System.Globalization;
using StintegyEVO.Core.Cars;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;

namespace StintegyEVO.TrainingHost;

internal static class Program
{
    public static int Main(string[] args)
    {
        if (args.Length == 1 && args[0] == "--terminal-speed")
            return PrintTerminalSpeeds();

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
                CarStrategy? egoStrategy,
                float decisionHz,
                float opponentPace,
                bool solo,
                bool randomiseEpisodeStart,
                EpisodeStartDistribution episodeStarts,
                bool hiddenCurriculum,
                float raceKilometres,
                float budgetLambda,
                float budgetGamma,
                bool deltaActions
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
                solo,
                egoStrategy,
                decisionHz,
                randomiseEpisodeStart,
                episodeStarts,
                hiddenCurriculum,
                raceKilometres,
                budgetLambda,
                budgetGamma,
                deltaActions
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
        CarStrategy? EgoStrategy,
        float DecisionHz,
        float OpponentPace,
        bool Solo,
        bool RandomiseEpisodeStart,
        EpisodeStartDistribution EpisodeStarts,
        bool HiddenCurriculum,
        float RaceKilometres,
        float BudgetLambda,
        float BudgetGamma,
        bool DeltaActions
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
        // Solo unless asked for a duel. It used to default to false, which
        // no caller could use because the environment threw on it; now that
        // it would silently put a second car on the road and change the
        // shape of every message, the wheel-to-wheel arm has to be asked
        // for by name.
        bool solo = true;
        CarStrategy? egoStrategy = null;
        float decisionHz = DirectDriveController.DefaultDecisionHz;
        bool randomiseEpisodeStart = false;
        bool hiddenCurriculum = false;
        float raceKilometres = EnergyBudget.DefaultRaceKilometres;
        float budgetLambda = EnergyBudget.DefaultLambda;
        float budgetGamma = EnergyBudget.DefaultGamma;
        // The delta-action pilot: the first action becomes an increment to
        // the steering command rather than the command itself. Off is the
        // world as it stands, bit for bit.
        bool deltaActions = false;
        EpisodeStartDistribution episodeStarts = new();
        for (int i = 0; i < args.Length; i++)
        {
            string option = args[i];
            if (option == "--ego-analytic")
            {
                throw new ArgumentException(
                    "--ego-analytic is retired: the analytic driver left " +
                    "master with the boundary migration."
                );
            }
            if (option == "--solo")
            {
                solo = true;
                continue;
            }
            if (option == "--duel")
            {
                solo = false;
                continue;
            }
            if (option == "--randomise-episode-start")
            {
                randomiseEpisodeStart = true;
                continue;
            }
            if (option == "--hidden-curriculum")
            {
                hiddenCurriculum = true;
                continue;
            }
            if (option == "--delta-actions")
            {
                deltaActions = true;
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
                            PowerOutputMode.Save
                        ),
                        _ => throw new ArgumentException(
                            "--opponent-strategy must be normal or protect."
                        )
                    };
                    break;
                case "--decision-hz":
                    decisionHz = ParsePositiveFloat(option, value);
                    break;
                case "--analytic-hz":
                    throw new ArgumentException(
                        "--analytic-hz is retired with the analytic driver."
                    );
                case "--race-km":
                    raceKilometres = ParsePositiveFloat(option, value);
                    break;
                case "--budget-lambda":
                    budgetLambda = ParseFiniteFloat(option, value);
                    break;
                case "--budget-gamma":
                    budgetGamma = ParsePositiveFloat(option, value);
                    break;
                case "--ego-modes":
                    egoStrategy = ParseModes(option, value);
                    break;
                case "--episode-start-normal-share":
                    episodeStarts = episodeStarts with
                    {
                        RaceNormalShare = ParseUnitFloat(option, value)
                    };
                    break;
                case "--episode-start-wear-max":
                    episodeStarts = episodeStarts with
                    {
                        ExtremeWearMax = ParseUnitFloat(option, value)
                    };
                    break;
                case "--episode-start-temp-min":
                    episodeStarts = episodeStarts with
                    {
                        ExtremeTempMinC = ParseFiniteFloat(option, value)
                    };
                    break;
                case "--episode-start-temp-max":
                    episodeStarts = episodeStarts with
                    {
                        ExtremeTempMaxC = ParseFiniteFloat(option, value)
                    };
                    break;
                case "--episode-start-charge-min":
                    episodeStarts = episodeStarts with
                    {
                        ExtremeChargeMin = ParseUnitFloat(option, value)
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
            egoStrategy,
            decisionHz,
            opponentPace,
            solo,
            randomiseEpisodeStart,
            episodeStarts,
            hiddenCurriculum,
            raceKilometres,
            budgetLambda,
            budgetGamma,
            deltaActions
        );
    }

    /// <summary>
    /// A pit-wall instruction written as "tyre,power", each a rung counted
    /// from one. Evaluation uses it so that a measurement says which
    /// instruction it was taken under instead of leaving it to a seed.
    /// </summary>
    /// <summary>
    /// Prints, as one JSON object, the flat-road terminal speed of the car
    /// the environment builds, at the nominal episode start (fresh tyres at
    /// 90 C, 80% of the pack), for every power rung under the certification
    /// tyre rung and for 5/5. Read by the straight-line certification script;
    /// the car is the environment's own, so the two cannot drift apart.
    /// </summary>
    private static int PrintTerminalSpeeds()
    {
        CarConfig config = new();
        TireConfig tires = new() { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f };
        PowertrainState energy = PowertrainState.Filled(0.8f);
        List<string> entries = [];
        for (int power = 1; power <= 5; power++)
        {
            float speed = TerminalSpeedOnTheFlat(
                config, tires, new CarStrategy((TireUsageMode)3, power), energy
            );
            entries.Add(
                $"\"3,{power}\": {speed.ToString("R", CultureInfo.InvariantCulture)}"
            );
        }
        float attack = TerminalSpeedOnTheFlat(
            config, tires, new CarStrategy((TireUsageMode)5, 5), energy
        );
        entries.Add(
            $"\"5,5\": {attack.ToString("R", CultureInfo.InvariantCulture)}"
        );
        Console.WriteLine("{" + string.Join(", ", entries) + "}");
        return 0;
    }

    /// <summary>
    /// The speed a car settles at on a flat, straight road with the pedal
    /// flat: where the most drive the published envelope allows meets what
    /// the air and rolling resistance take. Returns the upper search bound
    /// if the car is still accelerating there. world-v2 kept this in Core;
    /// with the envelope published, it belongs with the tool that reads it.
    /// </summary>
    internal static float TerminalSpeedOnTheFlat(
        CarConfig config,
        TireConfig tires,
        CarStrategy strategy,
        PowertrainState energy
    )
    {
        const float lower = 1f;
        const float upper = 150f;
        CarState state = new() { Energy = energy };
        state.InstallFreshTires(tires);

        float Surplus(float speed)
        {
            CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
                state, config, tires, strategy, speed, curvature: 0f
            );
            return limits.MaximumDriveAcceleration - limits.LossAcceleration;
        }

        if (Surplus(upper) > 0f)
            return upper;
        if (Surplus(lower) <= 0f)
            return lower;

        float low = lower;
        float high = upper;
        for (int i = 0; i < 48; i++)
        {
            float mid = 0.5f * (low + high);
            if (Surplus(mid) > 0f)
                low = mid;
            else
                high = mid;
        }
        return 0.5f * (low + high);
    }

    private static CarStrategy ParseModes(string option, string value)
    {
        string[] parts = value.Split(',');
        if (parts.Length != 2 ||
            !int.TryParse(parts[0], out int tire) ||
            !int.TryParse(parts[1], out int power) ||
            tire < 1 || tire > 5 || power < 1)
        {
            throw new ArgumentException(
                $"{option} must be written tyre,power with rungs counted " +
                "from one, for example 3,3."
            );
        }
        return new CarStrategy((TireUsageMode)tire, power);
    }

    /// <summary>A share or a fraction: finite and inside the unit
    /// interval, because every one of these is one of those.</summary>
    private static float ParseUnitFloat(string option, string value)
    {
        float parsed = ParseFiniteFloat(option, value);
        if (parsed < 0f || parsed > 1f)
        {
            throw new ArgumentException(
                $"{option} must be between zero and one."
            );
        }
        return parsed;
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
}
