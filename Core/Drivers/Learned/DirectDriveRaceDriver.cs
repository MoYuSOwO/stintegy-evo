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
/// physically deliver. The analytic planner still runs, but only to write
/// the coach block of the observation.
/// </summary>
public sealed class DirectDriveRaceDriver : IRaceDriver
{
    public const float DefaultDecisionHz = 30f;

    private readonly IDrivingPolicy _policy;
    private readonly VehicleSpeedPlanner _speedPlanner;
    private readonly VehicleSpeedLookahead _coachLookahead = new();
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

    public float StanleyGain { get; init; } = 2f;
    public float StanleySofteningSpeed { get; init; } = 4f;
    public float HeadingGain { get; init; } = 1f;
    public float CurvaturePreviewTimeSeconds { get; init; } = 0.15f;
    public float MaximumCurvaturePreviewMeters { get; init; } = 6f;
    public float CoachSpeedGain { get; init; } = 2.5f;

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
        _speedPlanner = new VehicleSpeedPlanner(speedPlanningConfig);
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
        VehicleSpeedPlanningConfig config = _speedPlanner.Config;
        _speedPlanner.PlanReferenceLookahead(
            _coachLookahead,
            car,
            context.Track,
            context.Pose.S,
            config.SpeedPlanningHorizonMeters,
            config.PathPredictionStepMeters,
            DriverPlanningModifiers.Neutral
        );
        StanleyControlSample coachSteering = StanleyControlLaw.Sample(
            context.Track,
            state.Position,
            state.VelocityHeading,
            state.Speed,
            car.CarConfig.WheelBaseMeters,
            lateralTargetOffsetMeters: 0f,
            StanleyGain,
            StanleySofteningSpeed,
            HeadingGain,
            CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters,
            car.CarConfig.MaxCurvatureRequest
        );
        VehicleSpeedPlanPoint coachSpeedPoint = _coachLookahead.Sample(0f);
        float coachReferenceAcceleration =
            coachSpeedPoint.ReferenceAcceleration;
        if (state.Speed > coachSpeedPoint.TargetSpeed)
        {
            coachReferenceAcceleration = MathF.Min(
                coachReferenceAcceleration,
                0f
            );
        }
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            state,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            state.Speed,
            coachSteering.DesiredCurvature,
            config.GetAccelerationUsage(car.Strategy),
            coachReferenceAcceleration
        );
        float driveLimit = MathF.Max(
            1f,
            limits.MaximumDriveAcceleration * config.DriveAccelerationUsage
        );
        float brakeLimit = MathF.Max(1f, car.CarConfig.MaxBrakeAccel);
        float coachAcceleration = coachReferenceAcceleration +
            limits.LossAcceleration +
            CoachSpeedGain * (coachSpeedPoint.TargetSpeed - state.Speed);
        float maxCurvature = MathF.Max(
            1e-4f,
            car.CarConfig.MaxCurvatureRequest
        );
        float coachCurvatureNorm = Math.Clamp(
            coachSteering.DesiredCurvature / maxCurvature,
            -1f,
            1f
        );
        float coachAccelerationNorm = NormalizeAcceleration(
            coachAcceleration,
            brakeLimit,
            driveLimit
        );

        _observationBuilder.Build(
            in context,
            _coachLookahead,
            coachCurvatureNorm,
            coachAccelerationNorm,
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

    private static float NormalizeAcceleration(
        float acceleration,
        float brakeLimit,
        float driveLimit
    )
    {
        float normalized = acceleration >= 0f
            ? acceleration / driveLimit
            : acceleration / brakeLimit;
        return Math.Clamp(normalized, -1f, 1f);
    }

    private static float SanitizeUnit(float value)
    {
        return float.IsFinite(value) ? Math.Clamp(value, -1f, 1f) : 0f;
    }
}
