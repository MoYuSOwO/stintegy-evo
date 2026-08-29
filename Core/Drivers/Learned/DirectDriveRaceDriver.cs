using System;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers.Learned;

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
/// Ten decisions a second, not thirty. Sony tested five to sixty on Gran
/// Turismo and found nothing above ten worth having, and a decision is the
/// expensive part of a step.
/// </summary>
public sealed class DirectDriveRaceDriver : IRaceDriver
{
    public const float DefaultDecisionHz = 10f;

    private readonly IDrivingPolicy _policy;
    private readonly VehicleSpeedPlanningConfig _planningConfig;
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


    public DirectDriveRaceDriver(
        IDrivingPolicy policy,
        VehicleSpeedPlanningConfig? speedPlanningConfig = null,
        float decisionHz = DefaultDecisionHz
    )
    {
        ArgumentNullException.ThrowIfNull(policy);
        if (!float.IsFinite(decisionHz) || decisionHz <= 0f)
            throw new ArgumentOutOfRangeException(nameof(decisionHz));
        _policy = policy;
        _planningConfig = speedPlanningConfig ?? new VehicleSpeedPlanningConfig();
        _decisionPeriodSeconds = 1f / decisionHz;
    }

    public ReadOnlySpan<float> LastObservation => _observation;
    public ReadOnlySpan<float> LastAction => _action;

    public void Initialize(in RaceDriverInitContext context)
    {
        _observationBuilder.Reset();
        _secondsSinceDecision = 0f;
        _hasDecision = false;
        _heldInput = default;
        _lastCurvatureNorm = 0f;
        _lastAccelerationNorm = 0f;
    }

    public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
    {
        _secondsSinceDecision += dt;
        if (_hasDecision &&
            _secondsSinceDecision < _decisionPeriodSeconds)
        {
            return _heldInput;
        }

        _secondsSinceDecision = 0f;
        _hasDecision = true;

        RaceCar car = context.Car;
        CarState state = car.State;
        VehicleSpeedPlanningConfig config = _planningConfig;

        // Not advice, just the actuator range the policy's [-1, 1] is
        // stretched onto: what this car can pull and push right now.
        float gripAllowance = config.GetAccelerationUsage(car.Strategy);
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            state,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            state.Speed,
            state.Telemetry.ActualCurvature,
            gripAllowance
        );
        float driveLimit = MathF.Max(
            1f,
            limits.MaximumDriveAcceleration * config.DriveAccelerationUsage
        );
        float brakeLimit = MathF.Max(1f, car.CarConfig.MaxBrakeAccel);
        float maxCurvature = MathF.Max(
            1e-4f,
            car.CarConfig.MaxCurvatureRequest
        );

        // The brake and steering halves of the action are stretched onto
        // fixed car constants, but the drive half is stretched onto a
        // ceiling that moves with grip, battery and speed. Without this the
        // same +0.5 buys four metres a second squared one tick and two and a
        // half the next, and nothing in the observation says which.
        // Reported as a fraction of the car's own peak so that it stays the
        // same number on a different car.
        float driveCeilingFraction = Math.Clamp(
            driveLimit / MathF.Max(car.CarConfig.MaxDriveAcceleration, 1e-3f),
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
        Array.Clear(_action);
        _policy.Act(_observation, _action);

        float curvatureNorm = SanitizeUnit(_action[0]);
        float accelerationNorm = SanitizeUnit(_action[1]);
        _lastCurvatureNorm = curvatureNorm;
        _lastAccelerationNorm = accelerationNorm;
        _heldInput = new DriverInput(
            curvatureNorm * maxCurvature,
            accelerationNorm >= 0f
                ? accelerationNorm * driveLimit
                : accelerationNorm * brakeLimit
        );
        return _heldInput;
    }

    private static float SanitizeUnit(float value)
    {
        return float.IsFinite(value) ? Math.Clamp(value, -1f, 1f) : 0f;
    }
}
