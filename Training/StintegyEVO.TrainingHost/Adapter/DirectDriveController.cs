using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.TrainingHost.Adapter;

/// <summary>
/// The learned driver's seat on the new contract: an
/// <see cref="IDriverController"/> clocked from outside.
///
/// The training environment owns the clock, because the agent's step is
/// the decision. It calls <see cref="Observe"/> with the frame the
/// simulation froze, sends the vector across the pipe, and answers with
/// <see cref="CommitAction"/>; the simulation then asks this controller for
/// its command on every substep and gets the committed one, held flat until
/// the next decision:
///
/// <code>
///   observation sampled at t  →  control held over [t, t + period)
///                             →  next observation sampled at t + period
/// </code>
///
/// Everything read is in the frozen frame or the car's published
/// parameters, handed in at construction. The action's [-1, 1] is
/// stretched onto the actuator range as it stood when the observation was
/// sampled, using the published performance envelope, so the policy is
/// told what its throttle is worth and then gets exactly that.
/// </summary>
public sealed class DirectDriveController : IDriverController
{
    public const float DefaultDecisionHz = 15f;

    private readonly CarConfig _config;
    private readonly TireConfig _tires;
    private readonly float _curvatureDeltaCap;
    private readonly DirectDriveObservationBuilder _observationBuilder;
    private readonly float[] _observation =
        new float[DirectDriveObservation.ObservationSize];
    private readonly float[] _action =
        new float[DirectDriveObservation.ActionSize];
    private DriverInput _heldInput;
    private float _commandedCurvatureNorm;
    private float _lastCurvatureNorm;
    private float _lastAccelerationNorm;
    private float _sampledDriveLimit;
    private float _sampledBrakeLimit;
    private float _sampledMaxCurvature;
    private bool _hasSample;

    /// <summary>
    /// A seat on the absolute contract, or — for the delta-action pilot —
    /// on the incremental one.
    ///
    /// <paramref name="deltaActions"/> changes what the first action means
    /// and nothing else. Absolute: the number is the curvature the car is
    /// to hold. Incremental: it is how far to move the command this
    /// decision, and the seat carries the command between decisions.
    /// </summary>
    public DirectDriveController(
        CarConfig config,
        TireConfig tires,
        bool deltaActions = false,
        float decisionHz = DefaultDecisionHz
    )
    {
        _config = config ?? throw new ArgumentNullException(nameof(config));
        _tires = tires ?? throw new ArgumentNullException(nameof(tires));
        _observationBuilder = new DirectDriveObservationBuilder(config);
        DeltaActions = deltaActions;
        _curvatureDeltaCap = deltaActions
            ? CurvatureDeltaCap(config, decisionHz)
            : 0f;
    }

    /// <summary>Whether the first action is an increment rather than a command.</summary>
    public bool DeltaActions { get; }

    /// <summary>
    /// The most the steering command may move in one decision, in units of
    /// the normalised command, so that the pilot cannot reach anywhere the
    /// absolute contract could not.
    ///
    /// Calibrated against the car, not chosen: the wheels themselves take
    /// <c>MaxSteerAngleRadians / SteerRateLimitRadiansPerSecond</c> to go
    /// from straight to full lock — 0.76 s on this car — and the command's
    /// whole range is two units wide, so a cap of two units over that many
    /// decisions lets the command cross its range exactly as fast as the
    /// wheels cross half of theirs. The rack is therefore still the
    /// binding constraint everywhere the rack was binding before, which is
    /// what makes this a change of what an action means rather than a
    /// change to what the car can do.
    ///
    /// On the default car at fifteen hertz that is 0.1745 a decision.
    /// </summary>
    public static float CurvatureDeltaCap(CarConfig config, float decisionHz)
    {
        float lockSeconds = MathF.Max(config.MaxSteerAngleRadians, 1e-3f) /
                            MathF.Max(config.SteerRateLimitRadiansPerSecond, 1e-3f);
        float decisions = MathF.Max(lockSeconds * decisionHz, 1f);
        return 2f / decisions;
    }

    public ReadOnlySpan<float> LastObservation => _observation;

    /// <summary>
    /// Remaining charge minus the pit wall's target line at the car's point
    /// in the race, set by whoever owns the race before each
    /// <see cref="Observe"/>. Zero when the instruction carries no line.
    /// The target line is an instruction, not a physical fact, so it comes
    /// from the race's owner rather than from the frame.
    /// </summary>
    public float BudgetDeviation { get; set; }
    public ReadOnlySpan<float> LastAction => _action;

    /// <summary>
    /// The steering command the car is holding, in the action's own units:
    /// the last action under the absolute contract, the integrator under
    /// the incremental one. Read around a commit, its change is how far
    /// the wheel was asked to move by that decision — which is the same
    /// question under either contract, and is what the steering costs are
    /// charged on.
    /// </summary>
    public float CommandedCurvatureNorm => _action[0];

    public void Initialize(in DriverContext context)
    {
        _observationBuilder.Reset();
        _hasSample = false;
        _heldInput = default;
        _commandedCurvatureNorm = 0f;
        _lastCurvatureNorm = 0f;
        _lastAccelerationNorm = 0f;
        Array.Clear(_observation);
        Array.Clear(_action);
    }

    /// <summary>Whatever was last committed, for every substep of it.</summary>
    public DriverInput GetControl(in DriverContext context, float dt) => _heldInput;

    /// <summary>
    /// Samples the frozen frame into <see cref="LastObservation"/> and
    /// remembers what the car could do at that instant.
    ///
    /// Calling it twice at the same instant is not the same as calling it
    /// once: the observation carries a copy of the previous sample, so a
    /// sample is also a tick of that memory. One sample per decision.
    /// </summary>
    public void Observe(in DriverContext context)
    {
        RaceCarSnapshot car = context.Car;
        float gripAllowance = _tires.GetAccelerationUsage(car.Strategy);
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            EnvelopeState(in car),
            _config,
            _tires,
            car.Strategy,
            car.SpeedMetersPerSecond,
            car.Telemetry.ActualCurvature,
            gripAllowance
        );
        _sampledDriveLimit = MathF.Max(1f, limits.MaximumDriveAcceleration);
        _sampledBrakeLimit = MathF.Max(1f, _config.MaxBrakeAccel);
        _sampledMaxCurvature = MathF.Max(1e-4f, _config.MaxCurvatureRequest);
        _hasSample = true;

        // The drive half of the action is stretched onto a ceiling that
        // moves with grip, battery and speed, so the observation says where
        // it stands, as a fraction of this car's own peak.
        float driveCeilingFraction = Math.Clamp(
            _sampledDriveLimit / MathF.Max(_config.MaxDriveAcceleration, 1e-3f),
            0f,
            1f
        );

        _observationBuilder.Build(
            in context,
            new DirectDriveCarLimits(driveCeilingFraction, gripAllowance),
            _lastCurvatureNorm,
            _lastAccelerationNorm,
            BudgetDeviation,
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

        // Absolute: the action is the command. Incremental: the action
        // moves the command, and what the car is given -- and what the
        // observation reports back as the command in hand -- is the
        // integrator, never the increment. A driver has to see the wheel
        // it is holding, the same way it has to see the charge it is
        // being billed for.
        float curvatureNorm;
        if (DeltaActions)
        {
            _commandedCurvatureNorm = Math.Clamp(
                _commandedCurvatureNorm +
                SanitizeUnit(action[0]) * _curvatureDeltaCap,
                -1f,
                1f
            );
            curvatureNorm = _commandedCurvatureNorm;
        }
        else
        {
            curvatureNorm = SanitizeUnit(action[0]);
        }
        float accelerationNorm = SanitizeUnit(action[1]);
        _lastCurvatureNorm = curvatureNorm;
        _lastAccelerationNorm = accelerationNorm;
        _action[0] = curvatureNorm;
        _action[1] = accelerationNorm;

        _heldInput = new DriverInput(
            curvatureNorm * _sampledMaxCurvature,
            accelerationNorm >= 0f
                ? accelerationNorm * _sampledDriveLimit
                : accelerationNorm * _sampledBrakeLimit
        );
    }

    /// <summary>
    /// The part of a car state the published envelope reads, rebuilt from
    /// the frame: speed, the stores, the air state and each tyre's
    /// temperatures, wear and load. A private scratch copy, never a live car.
    /// </summary>
    private static CarState EnvelopeState(in RaceCarSnapshot car)
    {
        float primary = car.Resources.IsDefaultOrEmpty ? 0f : car.Resources[0].Fraction;
        float secondary = car.Resources.IsDefaultOrEmpty || car.Resources.Length < 2
            ? primary
            : car.Resources[1].Fraction;
        CarState state = new()
        {
            Position = car.Position,
            Heading = car.HeadingRadians,
            Speed = car.SpeedMetersPerSecond,
            SideslipAngleRadians = car.SideslipAngleRadians,
            YawRateRadiansPerSecond = car.YawRateRadiansPerSecond,
            Energy = new PowertrainState(primary, secondary),
            AirVelocityDeficit = car.AirVelocityDeficit,
            DownforceVelocityDeficit = car.DownforceVelocityDeficit,
            WakeDownforceLoss = car.WakeDownforceLoss,
            DragReduction = car.DragReduction
        };
        CopyTire(state.FrontLeft, car.FrontLeft);
        CopyTire(state.FrontRight, car.FrontRight);
        CopyTire(state.RearLeft, car.RearLeft);
        CopyTire(state.RearRight, car.RearRight);
        return state;
    }

    private static void CopyTire(TireState target, in TireSnapshot source)
    {
        target.SurfaceTempC = source.SurfaceTempC;
        target.CoreTempC = source.CoreTempC;
        target.Wear = source.Wear;
        target.LoadN = source.LoadN;
        target.SurfaceGrip = source.SurfaceGrip;
    }

    private static float SanitizeUnit(float value) =>
        float.IsFinite(value) ? Math.Clamp(value, -1f, 1f) : 0f;
}
