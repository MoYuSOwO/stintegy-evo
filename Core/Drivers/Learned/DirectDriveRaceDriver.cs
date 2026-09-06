using System;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers.Learned;

/// <summary>
/// Who decides when this driver decides.
/// </summary>
public enum DecisionClock
{
    /// <summary>
    /// The driver keeps its own. It decides once every decision period of
    /// simulated time, wherever that happens to fall inside whatever step
    /// the caller is running. This is how a race runs it.
    /// </summary>
    Internal,

    /// <summary>
    /// The caller owns it. The driver looks when it is told to look and
    /// holds what it was told to do until it is told again. This is how
    /// training runs it, because there the agent's step boundary *is* the
    /// decision boundary and a second clock inside the driver can only
    /// disagree with it.
    /// </summary>
    External
}

/// <summary>
/// The learned-driver interface to the car: a policy reads one observation
/// and writes curvature and acceleration, which go to the vehicle with no
/// behavior in between. There is deliberately no traffic evaluator, no
/// racing room clamp, no lateral handover and no following law on this
/// path — collision avoidance is the policy's skill, adjudicated by the
/// stewards in training, exactly as the training plan lays out. The only
/// limits applied are the car's own actuators: maximum curvature request,
/// maximum braking, and the drive limit the powertrain and battery mode
/// physically deliver.
///
/// The analytic planner is not on this path at all. It used to run once a
/// decision to write a coach block into the observation, and it cost
/// three-quarters of the simulation to do it -- for advice that is 4 to 16
/// percent wrong about braking over a crest and leaves up to 81 percent of
/// the grip unused through a compression, which is to say wrong about
/// exactly the ground the road model just added.
///
/// Fifteen decisions a second. The project ran at ten for its whole first
/// era on the strength of Sony finding nothing above ten worth having on
/// Gran Turismo — but that finding belongs to their interface, where a
/// policy commands a steering angle. This one commands a curvature and
/// holds it flat until the next decision, and the car tracks that command
/// through a yaw response of about a hundred and fifty milliseconds, so a
/// hundred-millisecond period leaves barely one and a half corrections
/// inside the plant's own time constant.
///
/// Measured rather than argued. The analytic driver — the same code at
/// every rate, so whatever changes is the rate — was held to a ladder of
/// periods on eleven circuits. Ten hertz is a cliff and not a slope: it
/// spends whole seconds a lap outside the white lines and on two circuits
/// cannot complete a clean lap at all, while fifteen puts every excursion
/// to zero and takes fourteen to twenty-six seconds off the lap. Above
/// fifteen the ladder is flat — thirty and sixty are worth tenths — while
/// the wall clock keeps climbing. Fifteen is the first rung past the
/// cliff, which is where this now sits.
///
/// Evidence: Training/python/frequency_sweep.json.
///
/// <para><b>The decision contract.</b> A decision is two things at one
/// instant: the observation is sampled, and the control it produces is
/// fixed for the whole period that follows. Written out:</para>
///
/// <code>
///   observation sampled at t  →  control held over [t, t + period)
///                             →  next observation sampled at t + period
/// </code>
///
/// <para>Every consumer of this driver obeys that one sentence, which is
/// the point of writing it down. A race lets the driver time itself
/// (<see cref="DecisionClock.Internal"/>); training hands it the clock
/// (<see cref="DecisionClock.External"/>) so that the agent's step and the
/// decision are the same interval by construction. They were not, once:
/// the driver's own clock free-ran against the environment's step, the
/// action arrived three-quarters of the way through the step it was
/// credited to, and the observation handed back was sampled before that
/// action had touched anything — submitting full throttle and full brake
/// from the same state returned two identical observations. Everything
/// downstream, the reward included, was attached to the wrong action.</para>
/// </summary>
public sealed class DirectDriveRaceDriver : IRaceDriver
{
    public const float DefaultDecisionHz = 15f;

    private readonly IDrivingPolicy? _policy;
    private readonly VehicleSpeedPlanningConfig _planningConfig;
    private readonly DecisionClock _clock;
    private readonly DirectDriveObservationBuilder _observationBuilder = new();
    private readonly float[] _observation =
        new float[DirectDriveObservation.ObservationSize];
    private readonly float[] _action =
        new float[DirectDriveObservation.ActionSize];
    private readonly float _decisionPeriodSeconds;
    private float _secondsSinceDecision;
    private bool _hasDecision;
    private DriverInput _heldInput;
    private float _lastCurvatureNorm;
    private float _lastAccelerationNorm;

    // The actuator range as it stood at the instant the observation was
    // sampled. An action is stretched onto these rather than onto whatever
    // the car can do by the time the action arrives, because the
    // observation already told the policy what its throttle was worth --
    // reading the ceiling again later would silently change the deal.
    private float _sampledDriveLimit;
    private float _sampledBrakeLimit;
    private float _sampledMaxCurvature;
    private bool _hasSample;

    public DirectDriveRaceDriver(
        IDrivingPolicy policy,
        VehicleSpeedPlanningConfig? speedPlanningConfig = null,
        float decisionHz = DefaultDecisionHz
    )
        : this(
            policy ?? throw new ArgumentNullException(nameof(policy)),
            speedPlanningConfig,
            decisionHz,
            DecisionClock.Internal
        )
    {
    }

    private DirectDriveRaceDriver(
        IDrivingPolicy? policy,
        VehicleSpeedPlanningConfig? speedPlanningConfig,
        float decisionHz,
        DecisionClock clock
    )
    {
        if (!float.IsFinite(decisionHz) || decisionHz <= 0f)
            throw new ArgumentOutOfRangeException(nameof(decisionHz));
        _policy = policy;
        _planningConfig = speedPlanningConfig ?? new VehicleSpeedPlanningConfig();
        _clock = clock;
        _decisionPeriodSeconds = 1f / decisionHz;
    }

    /// <summary>
    /// A driver that is looked through and steered from outside: the caller
    /// calls <see cref="Observe"/> when it wants the state of the world, and
    /// <see cref="CommitAction"/> with whatever answers it. No policy,
    /// because the thing that would be asked is on the other side of a pipe.
    /// </summary>
    public static DirectDriveRaceDriver ExternallyClocked(
        VehicleSpeedPlanningConfig? speedPlanningConfig = null,
        float decisionHz = DefaultDecisionHz
    )
    {
        return new DirectDriveRaceDriver(
            policy: null,
            speedPlanningConfig,
            decisionHz,
            DecisionClock.External
        );
    }

    public DecisionClock Clock => _clock;

    public float DecisionPeriodSeconds => _decisionPeriodSeconds;

    public ReadOnlySpan<float> LastObservation => _observation;
    public ReadOnlySpan<float> LastAction => _action;

    public void Initialize(in RaceDriverInitContext context)
    {
        _observationBuilder.Reset();
        _secondsSinceDecision = 0f;
        _hasDecision = false;
        _hasSample = false;
        _heldInput = default;
        _lastCurvatureNorm = 0f;
        _lastAccelerationNorm = 0f;
        Array.Clear(_observation);
        Array.Clear(_action);
    }

    public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
    {
        if (_clock == DecisionClock.External)
        {
            // Somebody else is counting. Whatever was last committed is
            // what this car is doing, for every substep of it.
            return _heldInput;
        }

        _secondsSinceDecision += dt;
        if (_hasDecision &&
            _secondsSinceDecision < _decisionPeriodSeconds)
        {
            return _heldInput;
        }

        _secondsSinceDecision = 0f;
        _hasDecision = true;

        Observe(in context);
        Array.Clear(_action);
        _policy!.Act(_observation, _action);
        CommitAction(_action);
        return _heldInput;
    }

    /// <summary>
    /// Samples the world into <see cref="LastObservation"/> and remembers
    /// what the car could do at that instant, so that the action answering
    /// this observation is scaled by the ceilings the observation reported.
    ///
    /// Nothing is decided here and nothing moves. Calling it twice at the
    /// same instant is not the same as calling it once, though: the
    /// observation carries a copy of the previous sample, so a sample is
    /// also a tick of that memory. One sample per decision, always.
    /// </summary>
    public void Observe(in RaceDriverFrameContext context)
    {
        RaceCar car = context.Car;
        CarState state = car.State;

        // Not advice, just the actuator range the policy's [-1, 1] is
        // stretched onto: what this car can pull and push right now.
        float gripAllowance = car.TireConfig.GetAccelerationUsage(car.Strategy);
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            state,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            state.Speed,
            state.Telemetry.ActualCurvature,
            gripAllowance
        );
        _sampledDriveLimit = MathF.Max(
            1f,
            limits.MaximumDriveAcceleration *
            _planningConfig.DriveAccelerationUsage
        );
        _sampledBrakeLimit = MathF.Max(1f, car.CarConfig.MaxBrakeAccel);
        _sampledMaxCurvature = MathF.Max(
            1e-4f,
            car.CarConfig.MaxCurvatureRequest
        );
        _hasSample = true;

        // The brake and steering halves of the action are stretched onto
        // fixed car constants, but the drive half is stretched onto a
        // ceiling that moves with grip, battery and speed. Without this the
        // same +0.5 buys four metres a second squared one tick and two and a
        // half the next, and nothing in the observation says which.
        // Reported as a fraction of the car's own peak so that it stays the
        // same number on a different car.
        float driveCeilingFraction = Math.Clamp(
            _sampledDriveLimit /
            MathF.Max(car.CarConfig.MaxDriveAcceleration, 1e-3f),
            0f,
            1f
        );

        _observationBuilder.Build(
            in context,
            new DirectDriveCarLimits(driveCeilingFraction, gripAllowance),
            _lastCurvatureNorm,
            _lastAccelerationNorm,
            _observation
        );
    }

    /// <summary>
    /// Takes the answer to the last observation and turns it into what the
    /// car is held at until the next decision.
    /// </summary>
    public void CommitAction(ReadOnlySpan<float> action)
    {
        if (action.Length < DirectDriveObservation.ActionSize)
        {
            throw new ArgumentException(
                $"An action is {DirectDriveObservation.ActionSize} values.",
                nameof(action)
            );
        }

        if (!_hasSample)
        {
            throw new InvalidOperationException(
                "An action answers an observation; none has been sampled."
            );
        }

        float curvatureNorm = SanitizeUnit(action[0]);
        float accelerationNorm = SanitizeUnit(action[1]);
        _lastCurvatureNorm = curvatureNorm;
        _lastAccelerationNorm = accelerationNorm;
        // Reported back as what was actually used, not as what was asked
        // for: a policy that sends an infinity should be able to see that
        // the car drove a zero.
        _action[0] = curvatureNorm;
        _action[1] = accelerationNorm;

        _heldInput = new DriverInput(
            curvatureNorm * _sampledMaxCurvature,
            accelerationNorm >= 0f
                ? accelerationNorm * _sampledDriveLimit
                : accelerationNorm * _sampledBrakeLimit
        );
    }

    private static float SanitizeUnit(float value)
    {
        return float.IsFinite(value) ? Math.Clamp(value, -1f, 1f) : 0f;
    }
}
