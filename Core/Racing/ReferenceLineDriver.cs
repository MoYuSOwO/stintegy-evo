using System;
using System.Diagnostics;
using System.Numerics;
using TheStint.Core.Cars;
using TheStint.Core.Track;
using TheStint.Core.Util;

namespace TheStint.Core.Racing;

/// <summary>
/// Always tracks the global racing line with Stanley steering. When steering
/// feedback adds curvature, a decaying virtual-curvature envelope adjusts the
/// car-specific speed plan without creating a second geometric path.
/// </summary>
public sealed class ReferenceLineDriver : IRaceDriver
{
    private readonly VehicleSpeedPlanner _speedPlanner;
    private VehicleSpeedProfile? _speedProfile;
    private RaceCar? _plannedCar;
    private CarStrategy _plannedStrategy;
    private float _nextGlobalReplanTime;

    public ReferenceLineDriver(VehicleSpeedPlanningConfig? speedPlanningConfig = null)
    {
        _speedPlanner = new VehicleSpeedPlanner(speedPlanningConfig);
    }

    public float SpeedGain { get; init; } = 2.5f;
    public float StanleyGain { get; init; } = 2f;
    public float StanleySofteningSpeed { get; init; } = 4f;
    public float HeadingGain { get; init; } = 1f;
    public float CurvaturePreviewTimeSeconds { get; init; } = 0.15f;
    public float MaximumCurvaturePreviewMeters { get; init; } = 6f;

    public VehicleSpeedProfile? CurrentSpeedProfile => _speedProfile;
    public ReferenceLineDriverTelemetry LastTelemetry { get; private set; }

    public void Initialize(in RaceDriverInitContext context)
    {
        ReplanGlobal(context.Car, context.Track, context.RaceTimeSeconds);
    }

    public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
    {
        EnsureGlobalSpeedProfile(in context);

        RaceCar car = context.Car;
        CarState state = car.State;
        float wheelBase = MathF.Max(car.CarConfig.WheelBaseMeters, 0.5f);
        Vector2 frontAxlePosition = state.Position + state.Forward * (wheelBase * 0.5f);
        TrackPose frontPose = context.Track.Project(frontAxlePosition);
        TrackSample frontSample = frontPose.Sample;
        float curvaturePreviewDistance = MathF.Min(
            state.Speed * CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters
        );
        TrackSample previewSample = context.Track.Sample(
            frontPose.S + curvaturePreviewDistance
        );

        float lateralError = frontPose.D - frontSample.RefOffset;
        float headingError = MathHelper.NormalizeAngle(
            frontSample.RefHeading - state.Heading
        );
        float stanleyCorrection = MathF.Atan(
            StanleyGain * lateralError /
            MathF.Max(StanleySofteningSpeed + state.Speed, 0.1f)
        );
        float feedforwardSteer = MathF.Atan(wheelBase * previewSample.RefCurvature);
        float steeringAngle =
            feedforwardSteer + HeadingGain * headingError + stanleyCorrection;
        float maximumSteeringAngle = MathF.Atan(
            wheelBase * MathF.Max(car.CarConfig.MaxCurvatureRequest, 0f)
        );
        steeringAngle = Math.Clamp(
            steeringAngle,
            -maximumSteeringAngle,
            maximumSteeringAngle
        );
        float desiredCurvature = MathF.Tan(steeringAngle) / wheelBase;

        // Feedforward already represents the global line. Only the signed
        // feedback contribution is carried forward as a decaying correction.
        float curvatureCorrection = desiredCurvature - previewSample.RefCurvature;
        VehicleSpeedProfilePoint globalReference = _speedProfile!.Sample(frontPose.S);
        VehicleSpeedProfilePoint speedReference = globalReference;
        float correctionDecayDistance = 0f;
        float correctionMaximumCurvature = MathF.Abs(desiredCurvature);
        float correctionSpeedPlanningMilliseconds = 0f;

        if (MathF.Abs(curvatureCorrection) >
            _speedPlanner.Config.CurvatureCorrectionActivationThreshold)
        {
            Stopwatch timer = Stopwatch.StartNew();
            CurvatureCorrectionSpeedPlan correctionPlan =
                _speedPlanner.PlanCurvatureCorrection(
                    car,
                    context.Track,
                    _speedProfile,
                    frontPose.S,
                    curvatureCorrection,
                    desiredCurvature
                );
            timer.Stop();
            speedReference = correctionPlan.Current;
            correctionDecayDistance = correctionPlan.DecayDistanceMeters;
            correctionMaximumCurvature = correctionPlan.MaximumAbsoluteCurvature;
            correctionSpeedPlanningMilliseconds = ElapsedMilliseconds(timer);
        }

        float referenceAcceleration = speedReference.ReferenceAcceleration;
        if (state.Speed > speedReference.TargetSpeed)
            referenceAcceleration = MathF.Min(referenceAcceleration, 0f);
        CarPerformanceLimits currentLimits = CarPhysics.EstimatePerformanceLimits(
            state,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            state.Speed,
            desiredCurvature,
            _speedPlanner.Config.LateralGripUsage,
            referenceAcceleration
        );
        // The speed plan stores net vehicle acceleration, while DriverInput asks
        // for axle acceleration before rolling, aero, and cornering losses.
        float lossCompensationAcceleration = currentLimits.LossAcceleration;
        float speedFeedbackAcceleration = SpeedGain *
                                          (speedReference.TargetSpeed - state.Speed);
        float desiredAcceleration = referenceAcceleration +
                                    lossCompensationAcceleration +
                                    speedFeedbackAcceleration;
        desiredAcceleration = Math.Clamp(
            desiredAcceleration,
            -car.CarConfig.MaxBrakeAccel,
            car.CarConfig.MaxDriveAccelRequest
        );

        LastTelemetry = new ReferenceLineDriverTelemetry(
            frontPose.S,
            lateralError,
            headingError,
            frontSample.RefCurvature,
            previewSample.RefCurvature,
            desiredCurvature,
            curvatureCorrection,
            correctionDecayDistance,
            correctionMaximumCurvature,
            correctionSpeedPlanningMilliseconds,
            globalReference.TargetSpeed,
            speedReference.TargetSpeed,
            referenceAcceleration,
            lossCompensationAcceleration,
            speedFeedbackAcceleration,
            desiredAcceleration
        );
        return new DriverInput(desiredCurvature, desiredAcceleration);
    }

    private void EnsureGlobalSpeedProfile(in RaceDriverFrameContext context)
    {
        if (_speedProfile == null ||
            !ReferenceEquals(_plannedCar, context.Car) ||
            _plannedStrategy != context.Car.Strategy ||
            context.RaceTimeSeconds >= _nextGlobalReplanTime)
        {
            ReplanGlobal(context.Car, context.Track, context.RaceTimeSeconds);
        }
    }

    private void ReplanGlobal(RaceCar car, TrackData track, float raceTimeSeconds)
    {
        _speedProfile = _speedPlanner.Plan(car, track);
        _plannedCar = car;
        _plannedStrategy = car.Strategy;
        _nextGlobalReplanTime = raceTimeSeconds + _speedPlanner.Config.ReplanIntervalSeconds;
    }

    private static float ElapsedMilliseconds(Stopwatch stopwatch)
    {
        return (float)(stopwatch.ElapsedTicks * 1000.0 / Stopwatch.Frequency);
    }
}
