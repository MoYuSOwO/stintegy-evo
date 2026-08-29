using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Drivers.Learned;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Track.RefLines;

namespace StintegyEVO.TrainingHost.Environment;

/// <summary>
/// The direct-drive training environment: the ego car is driven by the
/// learned interface with no behavior layers between policy and vehicle,
/// while the scripted opponent keeps its full analytic stack as sparring
/// partner. Contact is not terminal here — it carries a steward penalty
/// with a crude at-fault reading and the race goes on, exactly as the
/// training plan's safety chapter lays out. One agent step is one policy
/// decision; the action chosen at step t steers the car through step t+1,
/// a deliberate one-tick reaction latency.
/// </summary>
public sealed class DirectDriveDuelEnvironment
{
    public const float AgentStepSeconds =
        1f / DirectDriveRaceDriver.DefaultDecisionHz;
    public const float DefaultEpisodeDurationSeconds = 60f;
    public const float DefaultMinimumForwardGapMeters = 12f;
    public const float DefaultMaximumForwardGapMeters = 28f;

    private const float PassHoldSeconds = 0.5f;
    private const float StalledSpeedMetersPerSecond = 1f;
    private const float StalledHoldSeconds = 2f;
    private const float WarmupStepSeconds = 1e-6f;

    // Placeholder until the wear budgets per tire mode are calibrated from
    // baseline stints; positive infinity disables the penalty while keeping
    // the plumbing and the protocol shape stable.
    private const float WearBudgetPerKilometerDisabled =
        float.PositiveInfinity;

    internal static readonly DriverProfile TrainingOpponentProfile = new(
        "training-opponent",
        new DriverAbilities
        {
            Pace = 70f,
            Consistency = 100f,
            CarControl = 100f,
            TireManagement = 80f,
            Adaptability = 100f,
            Reactions = 100f,
            Awareness = 100f,
            Overtaking = 100f,
            Defending = 100f
        },
        randomSeed: 0x545241494E494E47UL
    );

    private static readonly TrackChoice[] TrainingTracks =
    [
        new("speedway", BuildSpeedwayTrack),
        new("simple-right", () => TrackFactory.SimpleTestTrack(isLeft: false)),
        new("simple-left", () => TrackFactory.SimpleTestTrack(isLeft: true)),
        new("silverstone", TrackFactory.SilverstoneStyleTestTrack),
        new("shanghai", TrackFactory.ShanghaiStyleTestTrack)
    ];

    private static readonly TrackChoice HeldOutTrack = new(
        "sepang",
        TrackFactory.SepangStyleTestTrack
    );

    private readonly ManualDrivingPolicy _manualPolicy = new();
    private readonly float[] _previousAction =
        new float[DirectDriveObservation.ActionSize];
    private readonly float _minimumForwardGapMeters;
    private readonly float _maximumForwardGapMeters;
    private readonly float _episodeDurationSeconds;
    private readonly CarStrategy _opponentStrategy;
    private readonly DriverProfile _opponentProfile;
    private readonly bool _solo;
    private RaceSimulation? _simulation;
    private RaceCar? _ego;
    private DirectDriveRaceDriver? _egoDriver;
    private RaceCar? _opponent;
    private float _elapsedSeconds;
    private float _passHoldSeconds;
    private float _stalledHoldSeconds;
    private float _egoDistanceOrigin;
    private float _opponentDistanceOrigin;
    private float _episodeStartWear;
    private bool _terminal;

    public string TrackFamily { get; private set; } = string.Empty;
    public float EgoStartS { get; private set; }
    public float InitialForwardGapMeters { get; private set; }
    public float ElapsedSeconds => _elapsedSeconds;
    public bool IsTerminal => _terminal;
    public float SignedLeadDistanceMeters => CalculateSignedLeadDistance();
    public float MinimumSignedLeadDistanceMeters { get; private set; }
    public float MaximumAbsoluteReferenceOffsetMeters { get; private set; }
    public RaceSimulation Simulation =>
        _simulation ?? throw new InvalidOperationException("Reset must be called first.");
    public RaceCar Ego =>
        _ego ?? throw new InvalidOperationException("Reset must be called first.");
    public RaceCar Opponent =>
        _opponent ?? throw new InvalidOperationException(
            _solo
                ? "A solo environment has no opponent."
                : "Reset must be called first."
        );

    public DirectDriveDuelEnvironment(
        float minimumForwardGapMeters = DefaultMinimumForwardGapMeters,
        float maximumForwardGapMeters = DefaultMaximumForwardGapMeters,
        float episodeDurationSeconds = DefaultEpisodeDurationSeconds,
        CarStrategy? opponentStrategy = null,
        float opponentPace = 70f,
        bool solo = false
    )
    {
        if (!float.IsFinite(minimumForwardGapMeters) ||
            minimumForwardGapMeters <= 0f)
        {
            throw new ArgumentOutOfRangeException(
                nameof(minimumForwardGapMeters)
            );
        }
        if (!float.IsFinite(maximumForwardGapMeters) ||
            maximumForwardGapMeters < minimumForwardGapMeters)
        {
            throw new ArgumentOutOfRangeException(
                nameof(maximumForwardGapMeters)
            );
        }
        if (!float.IsFinite(episodeDurationSeconds) ||
            episodeDurationSeconds <= 0f)
        {
            throw new ArgumentOutOfRangeException(nameof(episodeDurationSeconds));
        }
        if (!float.IsFinite(opponentPace) ||
            opponentPace < 0f || opponentPace > 100f)
        {
            throw new ArgumentOutOfRangeException(nameof(opponentPace));
        }

        _minimumForwardGapMeters = minimumForwardGapMeters;
        _maximumForwardGapMeters = maximumForwardGapMeters;
        _episodeDurationSeconds = episodeDurationSeconds;
        _opponentStrategy = opponentStrategy ?? CarStrategy.Default;
        _opponentProfile = MathF.Abs(opponentPace - 70f) <= 1e-5f
            ? TrainingOpponentProfile
            : CreateOpponentProfile(opponentPace);
        _solo = solo;
    }

    public void Reset(long seed, Span<float> observation)
    {
        StableRandom random = new(unchecked((ulong)seed));
        TrackChoice choice = TrainingTracks[random.NextInt(TrainingTracks.Length)];
        ResetCore(choice, ref random, observation);
    }

    public void ResetHeldOut(long seed, Span<float> observation)
    {
        StableRandom random = new(unchecked((ulong)seed));
        ResetCore(HeldOutTrack, ref random, observation);
    }

    public void ResetTrack(
        string trackFamily,
        long seed,
        Span<float> observation
    )
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(trackFamily);
        StableRandom random = new(unchecked((ulong)seed));
        ResetCore(FindTrack(trackFamily), ref random, observation);
    }

    public void ResetScenario(
        string trackFamily,
        long seed,
        float egoStartS,
        float forwardGapMeters,
        float startSpeedMetersPerSecond,
        Span<float> observation
    )
    {
        if (!float.IsFinite(egoStartS))
            throw new ArgumentOutOfRangeException(nameof(egoStartS));
        if (!float.IsFinite(forwardGapMeters) || forwardGapMeters <= 0f)
            throw new ArgumentOutOfRangeException(nameof(forwardGapMeters));
        if (!float.IsFinite(startSpeedMetersPerSecond) ||
            startSpeedMetersPerSecond < 0f)
        {
            throw new ArgumentOutOfRangeException(
                nameof(startSpeedMetersPerSecond)
            );
        }

        StableRandom random = new(unchecked((ulong)seed));
        ResetCore(
            FindTrack(trackFamily),
            ref random,
            observation,
            egoStartS,
            forwardGapMeters,
            startSpeedMetersPerSecond
        );
    }

    public TrainingStepResult Step(
        ReadOnlySpan<float> actionValues,
        Span<float> observation
    )
    {
        if (_simulation is null || _ego is null || _egoDriver is null)
            throw new InvalidOperationException("Reset must be called before Step.");
        if (_terminal)
            throw new InvalidOperationException("Reset must be called after a terminal step.");
        EnsureObservationSize(observation);
        if (actionValues.Length != DirectDriveObservation.ActionSize)
        {
            throw new ArgumentException(
                $"Action must contain exactly {DirectDriveObservation.ActionSize} values.",
                nameof(actionValues)
            );
        }

        Span<float> action = stackalloc float[
            DirectDriveObservation.ActionSize
        ];
        for (int i = 0; i < action.Length; i++)
        {
            action[i] = float.IsFinite(actionValues[i])
                ? Math.Clamp(actionValues[i], -1f, 1f)
                : 0f;
        }
        _manualPolicy.SetAction(action);
        float egoDistanceBefore = _ego.Progress.TotalDistance;
        float opponentDistanceBefore =
            _opponent?.Progress.TotalDistance ?? 0f;

        _simulation.Step(AgentStepSeconds);
        _elapsedSeconds += AgentStepSeconds;
        _egoDriver.LastObservation.CopyTo(observation);

        float egoProgress = _ego.Progress.TotalDistance - egoDistanceBefore;
        float opponentProgress = _opponent is null
            ? 0f
            : _opponent.Progress.TotalDistance - opponentDistanceBefore;
        float signedLeadDistance = CalculateSignedLeadDistance();
        MinimumSignedLeadDistanceMeters = MathF.Min(
            MinimumSignedLeadDistanceMeters,
            signedLeadDistance
        );
        TrackPose egoPose = _simulation.Track.Project(_ego.State.Position);
        MaximumAbsoluteReferenceOffsetMeters = MathF.Max(
            MaximumAbsoluteReferenceOffsetMeters,
            MathF.Abs(egoPose.D - egoPose.Sample.RefOffset)
        );
        bool contact = _ego.HitCarThisStep;
        bool egoAtFault = contact && signedLeadDistance > 0f;
        TrainingTerminalReason terminalReason = DetermineTerminalReason(
            signedLeadDistance
        );
        _terminal = terminalReason != TrainingTerminalReason.None;

        float actionMagnitude = 0f;
        float actionDelta = 0f;
        for (int i = 0; i < action.Length; i++)
        {
            actionMagnitude += action[i] * action[i];
            float delta = action[i] - _previousAction[i];
            actionDelta += delta * delta;
            _previousAction[i] = action[i];
        }

        return new TrainingStepResult(
            terminalReason,
            OwnProgressReward: 0.0002f * egoProgress,
            RelativeProgressReward: _solo
                ? 0f
                : 0.1f * (egoProgress - opponentProgress),
            PassReward: terminalReason == TrainingTerminalReason.Passed
                ? 25f
                : 0f,
            ContactPenalty: contact
                ? (egoAtFault ? -20f : -2f)
                : 0f,
            WallPenalty: terminalReason == TrainingTerminalReason.Wall
                ? -30f
                : 0f,
            ActionMagnitudePenalty: -0.001f * actionMagnitude,
            ActionDeltaPenalty: -0.01f * actionDelta,
            TimePenalty: -0.003f,
            TimeoutOutcome: terminalReason == TrainingTerminalReason.Timeout &&
                            !_solo
                ? Math.Clamp(-signedLeadDistance * 0.1f, -8f, 8f)
                : 0f,
            ModeBudgetPenalty: ModeBudgetPenalty(egoProgress)
        );
    }

    private float ModeBudgetPenalty(float egoProgressMeters)
    {
        if (float.IsPositiveInfinity(WearBudgetPerKilometerDisabled) ||
            _ego is null ||
            egoProgressMeters <= 0f)
        {
            return 0f;
        }

        float wearNow = TotalWear(_ego.State);
        float wearDelta = wearNow - _episodeStartWear;
        _episodeStartWear = wearNow;
        float budget = WearBudgetPerKilometerDisabled *
                       egoProgressMeters / 1000f;
        return -MathF.Max(0f, wearDelta - budget) * 100f;
    }

    private static float TotalWear(CarState state) =>
        state.FrontLeft.Wear + state.FrontRight.Wear +
        state.RearLeft.Wear + state.RearRight.Wear;

    private void ResetCore(
        TrackChoice choice,
        ref StableRandom random,
        Span<float> observation,
        float? egoStartS = null,
        float? forwardGapMeters = null,
        float? startSpeedMetersPerSecond = null
    )
    {
        EnsureObservationSize(observation);
        TrackData track = choice.Track.Value;
        TrackFamily = choice.Name;
        EgoStartS = track.WrapS(
            egoStartS ?? random.NextSingle(0f, track.LengthMeters)
        );
        InitialForwardGapMeters =
            forwardGapMeters ?? random.NextSingle(
                _minimumForwardGapMeters,
                _maximumForwardGapMeters
            );
        float startSpeed = startSpeedMetersPerSecond ??
                           EstimateStartSpeed(track, EgoStartS);

        RaceEnvironment raceEnvironment = new()
        {
            AirTempC = random.NextSingle(18f, 32f),
            TrackTempC = random.NextSingle(22f, 45f)
        };
        _simulation = new RaceSimulation(track, raceEnvironment);
        _egoDriver = new DirectDriveRaceDriver(_manualPolicy);
        _ego = CreateCar(
            "training-ego",
            track,
            EgoStartS,
            startSpeed,
            _egoDriver,
            new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Attack)
        );
        _simulation.AddCar(_ego);
        if (_solo)
        {
            _opponent = null;
        }
        else
        {
            _opponent = CreateCar(
                "training-opponent",
                track,
                EgoStartS + InitialForwardGapMeters,
                startSpeed,
                new ReferenceLineDriver(profile: _opponentProfile),
                _opponentStrategy
            );
            _simulation.AddCar(_opponent);
        }

        _manualPolicy.SetAction(stackalloc float[
            DirectDriveObservation.ActionSize
        ]);
        _simulation.Step(WarmupStepSeconds);
        _egoDistanceOrigin = _ego.Progress.TotalDistance;
        _opponentDistanceOrigin = _opponent?.Progress.TotalDistance ?? 0f;
        _egoDriver.LastObservation.CopyTo(observation);
        Array.Clear(_previousAction);
        _episodeStartWear = TotalWear(_ego.State);
        MinimumSignedLeadDistanceMeters = InitialForwardGapMeters;
        MaximumAbsoluteReferenceOffsetMeters = 0f;
        _elapsedSeconds = 0f;
        _passHoldSeconds = 0f;
        _stalledHoldSeconds = 0f;
        _terminal = false;
    }

    private TrainingTerminalReason DetermineTerminalReason(
        float signedLeadDistance
    )
    {
        // Contact is deliberately not terminal: it is priced by the steward
        // penalty and the race continues, the way real incidents do.
        if (Ego.LastBoundaryContact.HasValue)
            return TrainingTerminalReason.Wall;

        if (_opponent is not null)
        {
            float fullClearance =
                Ego.Collision.HalfLengthMeters +
                _opponent.Collision.HalfLengthMeters;
            if (signedLeadDistance <= -fullClearance)
                _passHoldSeconds += AgentStepSeconds;
            else
                _passHoldSeconds = 0f;
            if (_passHoldSeconds + 1e-6f >= PassHoldSeconds)
                return TrainingTerminalReason.Passed;
        }

        if (Ego.State.Speed < StalledSpeedMetersPerSecond)
            _stalledHoldSeconds += AgentStepSeconds;
        else
            _stalledHoldSeconds = 0f;
        if (_stalledHoldSeconds + 1e-6f >= StalledHoldSeconds)
            return TrainingTerminalReason.Stalled;

        return _elapsedSeconds + 1e-6f >= _episodeDurationSeconds
            ? TrainingTerminalReason.Timeout
            : TrainingTerminalReason.None;
    }

    private float CalculateSignedLeadDistance() =>
        _simulation is null || _ego is null || _opponent is null
            ? 0f
            : InitialForwardGapMeters +
              (_opponent.Progress.TotalDistance - _opponentDistanceOrigin) -
              (_ego.Progress.TotalDistance - _egoDistanceOrigin);

    private static float EstimateStartSpeed(TrackData track, float s)
    {
        float maximumCurvature = 0f;
        for (int i = 0; i < 6; i++)
        {
            maximumCurvature = MathF.Max(
                maximumCurvature,
                MathF.Abs(track.Sample(s + i * 8f).RefCurvature)
            );
        }
        float lateralSafeSpeed = MathF.Sqrt(
            18f / MathF.Max(maximumCurvature, 0.002f)
        );
        return Math.Clamp(lateralSafeSpeed * 0.75f, 20f, 60f);
    }

    private static RaceCar CreateCar(
        string id,
        TrackData track,
        float s,
        float speed,
        IRaceDriver driver,
        CarStrategy strategy
    )
    {
        TrackSample sample = track.Sample(s);
        return new RaceCar(
            id,
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
                Speed = speed,
                BatterySoc = 0.8f
            }
        )
        {
            Strategy = strategy
        };
    }

    private static void EnsureObservationSize(Span<float> observation)
    {
        if (observation.Length != DirectDriveObservation.ObservationSize)
        {
            throw new ArgumentException(
                $"Observation must contain exactly {DirectDriveObservation.ObservationSize} values.",
                nameof(observation)
            );
        }
    }

    private static TrackChoice FindTrack(string trackFamily)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(trackFamily);
        if (string.Equals(
                trackFamily,
                HeldOutTrack.Name,
                StringComparison.Ordinal
            ))
        {
            return HeldOutTrack;
        }

        foreach (TrackChoice choice in TrainingTracks)
        {
            if (string.Equals(
                    trackFamily,
                    choice.Name,
                    StringComparison.Ordinal
                ))
            {
                return choice;
            }
        }
        throw new ArgumentOutOfRangeException(
            nameof(trackFamily),
            $"Unknown training track family '{trackFamily}'."
        );
    }

    private static TrackData BuildSpeedwayTrack() =>
        new TrackBuilder(
                Vector2.Zero,
                startWidth: 20f,
                refLineSolver: CenterLineRefLineSolver.Instance
            )
            .AddStraight(3000f)
            .AddTurn(180f, 400f)
            .AddStraight(3000f)
            .AddTurn(180f, 400f)
            .CloseLoop()
            .Build(new TrackGridConfig());

    private static DriverProfile CreateOpponentProfile(float pace) =>
        new(
            $"training-opponent-{pace:0.##}",
            TrainingOpponentProfile.Abilities with { Pace = pace },
            TrainingOpponentProfile.RandomSeed
        );

    private readonly record struct TrackChoice(
        string Name,
        Lazy<TrackData> Track
    )
    {
        public TrackChoice(string name, Func<TrackData> factory) : this(
            name,
            new Lazy<TrackData>(factory, isThreadSafe: true)
        )
        {
        }
    }

    private struct StableRandom
    {
        private ulong _state;

        public StableRandom(ulong seed)
        {
            _state = seed;
        }

        public int NextInt(int exclusiveMaximum) =>
            (int)(NextUInt64() % (uint)exclusiveMaximum);

        public float NextSingle() =>
            (float)((NextUInt64() >> 40) * (1.0 / (1UL << 24)));

        public float NextSingle(float minimum, float maximum) =>
            minimum + (maximum - minimum) * NextSingle();

        private ulong NextUInt64()
        {
            _state += 0x9E3779B97F4A7C15UL;
            ulong value = _state;
            value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9UL;
            value = (value ^ (value >> 27)) * 0x94D049BB133111EBUL;
            return value ^ (value >> 31);
        }
    }
}

/// <summary>
/// The environment's hand on the wheel: holds the latest external action
/// and hands it to the driver at its decision tick.
/// </summary>
internal sealed class ManualDrivingPolicy : IDrivingPolicy
{
    private readonly float[] _action =
        new float[DirectDriveObservation.ActionSize];

    public void SetAction(ReadOnlySpan<float> action)
    {
        action[..DirectDriveObservation.ActionSize].CopyTo(_action);
    }

    public void Act(ReadOnlySpan<float> observation, Span<float> action)
    {
        _action.CopyTo(action);
    }
}
