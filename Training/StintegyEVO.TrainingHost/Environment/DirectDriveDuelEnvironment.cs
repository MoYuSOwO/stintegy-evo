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

    /// <summary>
    /// Reward per meter of the ego's own progress. The archived offset task
    /// weighted this at a twentieth of this rate because relative progress
    /// carried the signal there; driving itself is now the thing being
    /// learned, so covering ground has to outweigh the per-step clock and
    /// the action regularizers by a clear margin, and in the solo stage it
    /// is the objective outright.
    /// </summary>
    /// <summary>
    /// What a metre of track is worth. Raised fivefold from the rate the
    /// first two rounds used, where a perfect lap earned less than the fixed
    /// costs of driving it and the optimal policy was therefore to crawl.
    /// </summary>
    private const float OwnProgressRate = 0.02f;

    /// <summary>
    /// The penalties the road exacts, all per second and all proportional to
    /// the square of the speed, after the shape Sony used: leaving the track
    /// and scraping a barrier are both things you are doing, priced for as
    /// long as you do them, not events that end the race. Their reward
    /// function ends an episode for nothing but running out of time.
    ///
    /// The barrier is charged by the seconds actually spent against it,
    /// which the simulation accumulates across its own substeps; leaving the
    /// track is charged for the whole step, because where the car is at the
    /// end of one is all the region test can say.
    /// </summary>
    private const float OffCoursePenaltyPerSpeedSquaredSecond = 1e-3f;
    private const float WallPenaltyPerSpeedSquaredSecond = 5e-3f;

    /// <summary>
    /// Sliding, priced by how far past the tyres' limit the car is being
    /// asked to go and how far sideways it has ended up as a result. This
    /// stands where an action-smoothness penalty used to: what wants
    /// discouraging is the car being out of shape, which is a thing the
    /// physics can see, not the policy's hand being unsteady, which is not.
    /// </summary>
    private const float TyreSlipPenaltyPerSecond = 2f;

    private const float TimePenaltyPerSecond = 0.01f;

    /// <summary>
    /// Price per unit of friction-circle usage taken beyond what the pit
    /// wall allotted, per agent step. A tire mode is exactly an instruction
    /// about how much of the tire's grip the driver may spend, and the
    /// physics never enforces it — only battery modes are hardware-capped —
    /// so a learned driver would otherwise drive Attack while the wall
    /// called Protect. Disobeying Protect outright buys on the order of one
    /// percent more distance a lap; this rate prices that at roughly ten
    /// times what it earns, which is what makes obedience the strategy
    /// game's premise rather than a suggestion. Subject to revision once
    /// training shows how the policy actually trades it.
    /// </summary>
    private const float ModeExcessPenaltyRate = 0.1f;

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

    /// <summary>
    /// What the car learns on. Gradient from Silverstone's flat airfield to
    /// the simple layout's seven percent, and lean from a road circuit's two
    /// and a half degrees to Zandvoort's eighteen.
    /// </summary>
    private static readonly TrackChoice[] TrainingTracks =
    [
        new("simple-right", () => TrackFactory.SimpleTestTrack(isLeft: false)),
        new("simple-left", () => TrackFactory.SimpleTestTrack(isLeft: true)),
        new("silverstone", TrackFactory.SilverstoneStyleTestTrack),
        new("shanghai", TrackFactory.ShanghaiStyleTestTrack),
        new("zandvoort", TrackFactory.ZandvoortStyleTestTrack)
    ];

    /// <summary>
    /// What it is tested on, and every one of them asks for something past
    /// the edge of what it was taught rather than between two things it
    /// already knows. Monaco climbs harder than any training track, the
    /// speedway leans further than any of them, and Sepang is neither —
    /// just a circuit it has never seen.
    ///
    /// The old split had Sepang alone, whose one and a half percent and two
    /// and a half degrees both sit comfortably inside the training range.
    /// Passing that says a policy can interpolate, which was never the
    /// question.
    /// </summary>
    private static readonly TrackChoice[] HeldOutTracks =
    [
        new("sepang", TrackFactory.SepangStyleTestTrack),
        new("monaco", TrackFactory.MonacoStyleTestTrack),
        new("speedway", BuildSpeedwayTrack)
    ];

    private static readonly TrackChoice HeldOutTrack = HeldOutTracks[0];

    private readonly ManualDrivingPolicy _manualPolicy = new();
    // The canonical mode-to-grip-allowance mapping, the same one the
    // analytic planner drives to.
    private readonly VehicleSpeedPlanningConfig _planningConfig = new();
    private readonly float _minimumForwardGapMeters;
    private readonly float _maximumForwardGapMeters;
    private readonly float _episodeDurationSeconds;
    private readonly CarStrategy _opponentStrategy;
    /// <summary>
    /// How often the sparring partner rethinks. Ten a second, the same rate
    /// the agent decides at, which is both cheap and appropriately coarse.
    /// </summary>
    private const float OpponentDecisionHz = 10f;

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
    private bool _terminal;

    public string TrackFamily { get; private set; } = string.Empty;
    public float EgoStartS { get; private set; }
    public float InitialForwardGapMeters { get; private set; }
    public CarStrategy EgoStrategy { get; private set; } = CarStrategy.Default;
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

        bool offCourse = _ego.Progress.Region != TrackRegion.RacingSurface;
        float wallSeconds = _ego.BoundaryContactSeconds;
        float speedSquared = _ego.State.Speed * _ego.State.Speed;
        float sliding =
            MathF.Min(MathF.Abs(_ego.State.Telemetry.OverLimit), 1f) *
            MathF.Abs(_ego.State.SideslipAngleRadians);

        return new TrainingStepResult(
            terminalReason,
            // Masked off course, so that cutting a corner cannot pay for
            // itself with the ground it gains.
            OwnProgressReward: offCourse
                ? 0f
                : OwnProgressRate * egoProgress,
            RelativeProgressReward: _solo
                ? 0f
                : 0.1f * (egoProgress - opponentProgress),
            PassReward: terminalReason == TrainingTerminalReason.Passed
                ? 25f
                : 0f,
            ContactPenalty: contact
                ? (egoAtFault ? -20f : -2f)
                : 0f,
            // Priced by how long the car leant on it, not by whether it
            // touched at all. Charging a whole step for a glance leaves a
            // car that has already brushed with no reason to come off the
            // barrier before the step is out.
            WallPenalty:
                -WallPenaltyPerSpeedSquaredSecond * speedSquared * wallSeconds,
            OffCoursePenalty: offCourse
                ? -OffCoursePenaltyPerSpeedSquaredSecond * speedSquared *
                  AgentStepSeconds
                : 0f,
            TyreSlipPenalty:
                -TyreSlipPenaltyPerSecond * sliding * AgentStepSeconds,
            TimePenalty: -TimePenaltyPerSecond * AgentStepSeconds,
            TimeoutOutcome: terminalReason == TrainingTerminalReason.Timeout &&
                            !_solo
                ? Math.Clamp(-signedLeadDistance * 0.1f, -8f, 8f)
                : 0f,
            ModeExcessPenalty: ModeExcessPenalty(),
            // Coming to a halt ends the episode, and an ending that costs
            // nothing is worth more than any lap that risks the wall — so
            // without this the surest way to stop losing points is to stop.
            // A car parked on a race track has retired, and retiring is at
            // least as expensive as running out of road.
            RetirementPenalty: terminalReason == TrainingTerminalReason.Stalled
                ? -30f
                : 0f
        );
    }

    /// <summary>
    /// How far past its allotted share of the friction circle the car is
    /// being driven. The tire mode maps to the same grip fraction the
    /// analytic planner drives to, so the comparison is the instruction
    /// itself rather than a proxy: at Attack the allowance is the whole
    /// circle and no excess is possible, while at Protect anything above
    /// about ninety-six percent is grip the wall did not authorize. The
    /// reading is the last physics substep of the agent step, which
    /// samples rather than integrates the excess; over an episode's
    /// thousands of steps that average is what the policy optimizes.
    /// </summary>
    private float ModeExcessPenalty()
    {
        if (_ego is null)
            return 0f;

        CarTelemetry telemetry = _ego.State.Telemetry;
        float allowance = _planningConfig.GetAccelerationUsage(_ego.Strategy);
        float frontUse = CombinedUse(
            telemetry.FrontLateralUse,
            telemetry.FrontLongitudinalUse
        );
        float rearUse = CombinedUse(
            telemetry.RearLateralUse,
            telemetry.RearLongitudinalUse
        );
        float excess = MathF.Max(frontUse, rearUse) - allowance;
        return excess <= 0f ? 0f : -ModeExcessPenaltyRate * excess;
    }

    /// <summary>
    /// Share of the available friction circle actually being spent, capped
    /// at the whole circle. A request beyond the circle does not buy grip —
    /// it slides — and the physics already charges for that; counting it
    /// here as well would conflate overdriving with disobeying the wall,
    /// and the plan holds that a driver's self-inflicted costs are priced
    /// by lap time, not by penalties.
    /// </summary>
    private static float CombinedUse(float lateral, float longitudinal) =>
        MathF.Min(
            1f,
            MathF.Sqrt(lateral * lateral + longitudinal * longitudinal)
        );

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
        // Every episode draws a pit-wall instruction. Without this the
        // policy would only ever be told Attack and could never learn what
        // the other modes ask of it, however the observation reports them.
        EgoStrategy = new CarStrategy(
            (TireUsageMode)(random.NextInt(5) + 1),
            (BatteryOutputMode)(random.NextInt(5) + 1)
        );

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
            EgoStrategy
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
                new HeldDecisionDriver(
                    new ReferenceLineDriver(profile: _opponentProfile),
                    OpponentDecisionHz
                ),
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
        // Neither contact nor a barrier is terminal: both are priced for as
        // long as they last and the race continues, the way real incidents
        // do. An episode that ends the moment a car brushes something
        // destroys every bit of learning that would have followed, and it
        // teaches a policy that the cheapest race is a short one.
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
        foreach (TrackChoice heldOut in HeldOutTracks)
        {
            if (string.Equals(
                    trackFamily,
                    heldOut.Name,
                    StringComparison.Ordinal
                ))
            {
                return heldOut;
            }
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
