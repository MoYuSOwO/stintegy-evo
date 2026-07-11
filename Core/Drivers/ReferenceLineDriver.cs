using System;
using System.Diagnostics;
using System.Numerics;
using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;
using TheStint.Core.Util;

namespace TheStint.Core.Drivers;

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
    private RaceCar? _runtimeCar;
    private DriverPerformanceState? _performance;
    private CarStrategy _plannedStrategy;
    private int _plannedPerformanceRevision = -1;
    private float _nextGlobalReplanTime;
    private float _perceivedSideslip;
    private float _perceivedYawRate;

    public ReferenceLineDriver(
        VehicleSpeedPlanningConfig? speedPlanningConfig = null,
        DriverProfile? profile = null
    )
    {
        _speedPlanner = new VehicleSpeedPlanner(speedPlanningConfig);
        Profile = profile ?? DriverProfile.LegacyBaseline;
    }

    public ReferenceLineDriver(DriverProfile profile) : this(null, profile)
    {
    }

    public float SpeedGain { get; init; } = 2.5f;
    public float StanleyGain { get; init; } = 2f;
    public float StanleySofteningSpeed { get; init; } = 4f;
    public float HeadingGain { get; init; } = 1f;
    public float CurvaturePreviewTimeSeconds { get; init; } = 0.15f;
    public float MaximumCurvaturePreviewMeters { get; init; } = 6f;

    public DriverProfile Profile { get; }
    public float TireEnergyEfficiency => _performance?.TireEnergyEfficiency ?? 1f;
    public VehicleSpeedProfile? CurrentSpeedProfile => _speedProfile;
    public ReferenceLineDriverTelemetry LastTelemetry { get; private set; }

    public void Initialize(in RaceDriverInitContext context)
    {
        ResetPerformanceState(context.Car, context.Pose);
        ReplanGlobal(context.Car, context.Track, context.RaceTimeSeconds);
    }

    public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
    {
        UpdatePerformanceState(in context, dt);
        EnsureGlobalSpeedProfile(in context);

        RaceCar car = context.Car;
        CarState state = car.State;
        float wheelBase = MathF.Max(car.CarConfig.WheelBaseMeters, 0.5f);
        // The path controller follows the velocity direction. Body sideslip is
        // stabilized by the vehicle layer instead of turning into an abrupt
        // Stanley heading correction.
        Vector2 frontAxlePosition = state.Position + state.VelocityForward * (wheelBase * 0.5f);
        TrackPose frontPose = context.Track.Project(frontAxlePosition);
        TrackSample frontSample = frontPose.Sample;
        float curvaturePreviewDistance = MathF.Min(
            state.Speed * CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters
        );
        TrackSample previewSample = context.Track.Sample(
            frontPose.S + curvaturePreviewDistance
        );

        float lateralTargetError = _performance!.LateralTargetErrorMeters;
        float lateralError = frontPose.D - (frontSample.RefOffset + lateralTargetError);
        float headingError = MathHelper.NormalizeAngle(
            frontSample.RefHeading - state.VelocityHeading
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
        float controlCorrection = ApplyCarControl(
            state,
            wheelBase,
            ref desiredCurvature,
            dt,
            out float controlSeverity
        );

        // The front-axle Stanley error contains the normal geometric offset
        // between the car centre and its front axle while cornering. That is
        // useful for steering, but it is not an off-line recovery path and
        // must not lower the speed plan. Build the recovery correction from
        // the car-centre pose instead, then retain any active stability-control
        // correction made above.
        TrackSample centerSample = context.Pose.Sample;
        float centerLateralError =
            context.Pose.D - (centerSample.RefOffset + lateralTargetError);
        float centerHeadingError = MathHelper.NormalizeAngle(
            centerSample.RefHeading - state.VelocityHeading
        );
        float recoveryStanleyCorrection = MathF.Atan(
            StanleyGain * centerLateralError /
            MathF.Max(StanleySofteningSpeed + state.Speed, 0.1f)
        );
        float recoverySteeringAngle = Math.Clamp(
            feedforwardSteer + HeadingGain * centerHeadingError +
            recoveryStanleyCorrection,
            -maximumSteeringAngle,
            maximumSteeringAngle
        );
        float recoveryCurvature = Math.Clamp(
            MathF.Tan(recoverySteeringAngle) / wheelBase + controlCorrection,
            -car.CarConfig.MaxCurvatureRequest,
            car.CarConfig.MaxCurvatureRequest
        );
        float curvatureCorrection =
            recoveryCurvature - previewSample.RefCurvature;
        float perceivedS = frontPose.S + _performance.BrakeMarkerErrorMeters;
        VehicleSpeedProfilePoint globalReference = _speedProfile!.Sample(perceivedS);
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

        float cornerWeight = SmoothStep01(
            MathF.Abs(previewSample.RefCurvature) / 0.01f
        );
        float paceRatio = _performance.PaceEfficiency /
                          MathF.Max(_performance.PlanningPaceEfficiency, 0.8f);
        float localSpeedFactor = Math.Clamp(
            1f + (paceRatio - 1f) * cornerWeight +
            _performance.LocalSpeedErrorFraction * cornerWeight,
            0.97f,
            1.02f
        );
        // The curvature-correction plan anchors its first target to the
        // current speed and carries acceleration separately. Scaling that
        // anchored target below one would look like an overspeed condition
        // and suppress the positive recovery acceleration, most visibly when
        // launching from an offset grid slot.
        if (correctionDecayDistance <= 0f)
        {
            speedReference = speedReference with
            {
                TargetSpeed = speedReference.TargetSpeed * localSpeedFactor
            };
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
            _speedPlanner.Config.GetAccelerationUsage(car.Strategy.TireMode),
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
        float driveAccelerationLimit =
            currentLimits.MaximumDriveAcceleration *
            _speedPlanner.Config.DriveAccelerationUsage *
            Math.Clamp(
                _performance.PaceEfficiency * _performance.EstimatedGripScale,
                0.8f,
                1.05f
            );
        if (desiredAcceleration > 0f && controlSeverity > 0f)
        {
            float retainedDriveAtFullSeverity = Lerp(
                0.25f,
                0.8f,
                _performance.EffectiveControl
            );
            desiredAcceleration *= 1f - controlSeverity *
                                   (1f - retainedDriveAtFullSeverity);
        }
        desiredAcceleration = Math.Clamp(
            desiredAcceleration,
            -car.CarConfig.MaxBrakeAccel * _performance.PaceEfficiency,
            driveAccelerationLimit
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
            driveAccelerationLimit,
            desiredAcceleration,
            Profile.Abilities.Pace,
            Profile.Abilities.Consistency,
            Profile.Abilities.CarControl,
            Profile.Abilities.TireManagement,
            Profile.Abilities.Adaptability,
            _performance.SessionForm,
            _performance.LapForm,
            _performance.SegmentForm,
            _performance.PlanningPace,
            _performance.EffectivePace,
            _performance.PaceEfficiency,
            _performance.EffectiveControl,
            _performance.EffectiveTireManagement,
            _performance.TireEnergyEfficiency,
            _performance.EffectiveAdaptability,
            _performance.ActualGrip,
            _performance.EstimatedGrip,
            _performance.EstimatedGripScale,
            _performance.BrakeMarkerErrorMeters,
            lateralTargetError,
            _performance.LocalSpeedErrorFraction,
            controlSeverity,
            controlCorrection,
            _performance.IsRecovering
        );
        return new DriverInput(desiredCurvature, desiredAcceleration);
    }

    private void EnsureGlobalSpeedProfile(in RaceDriverFrameContext context)
    {
        if (_speedProfile == null ||
            !ReferenceEquals(_plannedCar, context.Car) ||
            _plannedStrategy != context.Car.Strategy ||
            _plannedPerformanceRevision != _performance!.PlanningRevision ||
            context.RaceTimeSeconds >= _nextGlobalReplanTime)
        {
            ReplanGlobal(context.Car, context.Track, context.RaceTimeSeconds);
        }
    }

    private void ReplanGlobal(RaceCar car, TrackData track, float raceTimeSeconds)
    {
        DriverPlanningModifiers modifiers = new(
            _performance?.PlanningPaceEfficiency ?? 1f,
            _performance?.EstimatedGripScale ?? 1f
        );
        _speedProfile = _speedPlanner.Plan(car, track, modifiers);
        _plannedCar = car;
        _plannedStrategy = car.Strategy;
        _plannedPerformanceRevision = _performance?.PlanningRevision ?? 0;
        _nextGlobalReplanTime = raceTimeSeconds + _speedPlanner.Config.ReplanIntervalSeconds;
    }

    private void UpdatePerformanceState(
        in RaceDriverFrameContext context,
        float dt
    )
    {
        if (!ReferenceEquals(_runtimeCar, context.Car) || _performance == null)
            ResetPerformanceState(context.Car, context.Pose);

        float actualGrip = EstimateCurrentLateralGrip(context.Car);
        _performance!.Update(
            context.Car.Progress.Lap,
            context.Pose.S,
            actualGrip,
            dt
        );
    }

    private void ResetPerformanceState(RaceCar car, TrackPose pose)
    {
        _runtimeCar = car;
        _performance = new DriverPerformanceState(Profile, car.Id);
        _performance.Initialize(
            car.Progress.Lap,
            pose.S,
            EstimateCurrentLateralGrip(car)
        );
        _perceivedSideslip = car.State.SideslipAngleRadians;
        _perceivedYawRate = car.State.YawRateRadiansPerSecond;
        _speedProfile = null;
        _plannedCar = null;
        _plannedPerformanceRevision = -1;
    }

    private float ApplyCarControl(
        CarState state,
        float wheelBase,
        ref float desiredCurvature,
        float dt,
        out float severity
    )
    {
        float desiredYawRate = state.Speed * desiredCurvature;
        float actualYawError = state.YawRateRadiansPerSecond - desiredYawRate;
        float sideslipSeverity = Math.Clamp(
            (MathF.Abs(state.SideslipAngleRadians) - 0.03f) / 0.07f,
            0f,
            1f
        );
        float yawSeverity = Math.Clamp(
            (MathF.Abs(actualYawError) - 0.08f) / 0.6f,
            0f,
            1f
        ) * Math.Clamp(
            MathF.Abs(state.SideslipAngleRadians) / 0.04f,
            0f,
            1f
        );
        float rearSlideSeverity = Math.Clamp(
            (state.Telemetry.RearSlideSeverity - 0.15f) / 0.6f,
            0f,
            1f
        );
        severity = MathF.Max(
            sideslipSeverity,
            MathF.Max(yawSeverity, rearSlideSeverity)
        );

        bool unstable = _performance!.IsRecovering
            ? severity > 0.05f
            : severity > 0.15f;
        _performance.UpdateControlEvent(unstable);

        float observationTime = Lerp(
            0.25f,
            0.04f,
            _performance.EffectiveControl
        );
        float response = 1f - MathF.Exp(-MathF.Max(0f, dt) / observationTime);
        _perceivedSideslip = Lerp(
            _perceivedSideslip,
            state.SideslipAngleRadians,
            response
        );
        _perceivedYawRate = Lerp(
            _perceivedYawRate,
            state.YawRateRadiansPerSecond,
            response
        );

        if (!_performance.IsRecovering)
            return 0f;

        float perceivedYawError = _perceivedYawRate - desiredYawRate;
        float correction = (
            0.35f * _perceivedSideslip / MathF.Max(wheelBase, 0.5f) -
            0.25f * perceivedYawError / MathF.Max(state.Speed, 5f)
        ) * _performance.ControlGainScale;
        desiredCurvature = Math.Clamp(
            desiredCurvature + correction,
            -_runtimeCar!.CarConfig.MaxCurvatureRequest,
            _runtimeCar.CarConfig.MaxCurvatureRequest
        );
        return correction;
    }

    private float EstimateCurrentLateralGrip(RaceCar car)
    {
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            car.State,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            speed: 0f,
            curvature: 0f,
            gripUsage: _speedPlanner.Config.GetAccelerationUsage(car.Strategy.TireMode)
        );
        return MathF.Max(limits.LateralAccelerationLimit, 1e-3f);
    }

    private static float SmoothStep01(float value)
    {
        float x = Math.Clamp(value, 0f, 1f);
        return x * x * (3f - 2f * x);
    }

    private static float Lerp(float from, float to, float t) =>
        from + (to - from) * Math.Clamp(t, 0f, 1f);

    private static float ElapsedMilliseconds(Stopwatch stopwatch)
    {
        return (float)(stopwatch.ElapsedTicks * 1000.0 / Stopwatch.Frequency);
    }
}
