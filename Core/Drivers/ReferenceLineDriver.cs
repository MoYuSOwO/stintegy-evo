using System;
using System.Diagnostics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Tracks the global racing line with Stanley steering, predicts the spatial
/// path produced by that same control law from the live vehicle state, joins
/// that prediction back to the racing line, and replans speed over the rolling
/// local horizon every frame.
/// </summary>
public sealed class ReferenceLineDriver : IRaceDriver
{
    private readonly VehicleSpeedPlanner _speedPlanner;
    private readonly StanleyPathPredictor _pathPredictor = new();
    private RaceCar? _runtimeCar;
    private DriverPerformanceState? _performance;
    private StabilityControlState _stabilityControlState;

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
    public VehicleSpeedLookahead CurrentSpeedLookahead =>
        _speedPlanner.CurrentPredictedPathLookahead;
    public VehiclePathPrediction CurrentPathPrediction { get; private set; } = new();
    public ReferenceLineDriverTelemetry LastTelemetry { get; private set; }

    public void Initialize(in RaceDriverInitContext context)
    {
        ResetPerformanceState(context.Car, context.Pose);
    }

    public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
    {
        UpdatePerformanceState(in context, dt);

        RaceCar car = context.Car;
        CarState state = car.State;
        float wheelBase = MathF.Max(car.CarConfig.WheelBaseMeters, 0.5f);
        float lateralTargetError = _performance!.LateralTargetErrorMeters;
        // The path controller follows the velocity direction. Body sideslip is
        // stabilized by the vehicle layer instead of turning into an abrupt
        // Stanley heading correction.
        StanleyControlSample control = StanleyControlLaw.Sample(
            context.Track,
            state.Position,
            state.VelocityHeading,
            state.Speed,
            wheelBase,
            lateralTargetError,
            StanleyGain,
            StanleySofteningSpeed,
            HeadingGain,
            CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters,
            car.CarConfig.MaxCurvatureRequest
        );
        TrackPose frontPose = control.FrontPose;
        TrackSample frontSample = control.FrontSample;
        TrackSample previewSample = control.PreviewSample;
        float lateralError = control.LateralErrorMeters;
        float headingError = control.HeadingErrorRadians;
        float desiredCurvature = control.DesiredCurvature;
        float controlCorrection = ApplyCarControl(
            state,
            wheelBase,
            ref desiredCurvature,
            dt,
            out float controlSeverity
        );

        float curvatureCorrection = desiredCurvature - previewSample.RefCurvature;
        float planningPace = Math.Clamp(
            _performance.PaceEfficiency *
            (1f + _performance.LocalSpeedErrorFraction),
            0.8f,
            1.05f
        );
        DriverPlanningModifiers planningModifiers = new(
            planningPace,
            _performance.EstimatedGripScale,
            _performance.FrontBrakeBiasOffset
        );
        long speedPlanningStartTimestamp = Stopwatch.GetTimestamp();
        VehicleSpeedLookahead referenceLookahead =
            _speedPlanner.PlanReferenceLookahead(
                car,
                context.Track,
                frontPose.S + _performance.BrakeMarkerErrorMeters,
                _speedPlanner.Config.SpeedPlanningHorizonMeters,
                _speedPlanner.Config.PathPredictionStepMeters,
                planningModifiers
            );
        VehicleSpeedPlanPoint referencePathPlan =
            referenceLookahead.Sample(0f);
        long predictionStartTimestamp = Stopwatch.GetTimestamp();
        CurrentPathPrediction = _pathPredictor.Predict(
            car,
            context.Track,
            referenceLookahead,
            lateralTargetError,
            StanleyGain,
            StanleySofteningSpeed,
            HeadingGain,
            CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters,
            _speedPlanner.Config.SpeedPlanningHorizonMeters,
            _speedPlanner.Config.PathPredictionStepMeters,
            _speedPlanner.Config.MinimumDynamicPredictionMeters,
            _speedPlanner.Config.PredictionConvergenceHoldMeters,
            _speedPlanner.Config.PredictionConvergenceLateralErrorMeters,
            _speedPlanner.Config.PredictionConvergenceHeadingErrorRadians,
            _speedPlanner.Config.PredictionConvergenceCurvatureError,
            _speedPlanner.Config.GetAccelerationUsage(car.Strategy),
            desiredCurvature,
            new StabilityPredictionSeed(
                _stabilityControlState,
                _performance.IsRecovering,
                _performance.EffectiveControl,
                _performance.ControlGainScale
            )
        );
        float pathPredictionMilliseconds = ElapsedMilliseconds(
            predictionStartTimestamp
        );
        DynamicPathSpeedPlan dynamicPlan =
            _speedPlanner.PlanPredictedPath(car, CurrentPathPrediction);
        VehicleSpeedPlanPoint speedReference = dynamicPlan.Current;
        float rollingSpeedPlanningMilliseconds = MathF.Max(
            0f,
            ElapsedMilliseconds(speedPlanningStartTimestamp) -
            pathPredictionMilliseconds
        );

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
            _speedPlanner.Config.GetAccelerationUsage(car.Strategy),
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
            CurrentPathPrediction.LengthMeters,
            CurrentPathPrediction.MaximumAbsoluteCommandedCurvature,
            CurrentPathPrediction.TerminalLateralErrorMeters,
            CurrentPathPrediction.DynamicPredictionLengthMeters,
            CurrentPathPrediction.JoinsReferenceLine,
            CurrentPathPrediction.ReferenceLineJoinCurvatureDelta,
            pathPredictionMilliseconds,
            rollingSpeedPlanningMilliseconds,
            referencePathPlan.TargetSpeed,
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
            _performance.FrontBrakeBiasOffset,
            controlSeverity,
            controlCorrection,
            _performance.IsRecovering
        );
        return new DriverInput(
            desiredCurvature,
            desiredAcceleration,
            _performance.FrontBrakeBiasOffset
        );
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
        _stabilityControlState = new StabilityControlState(
            car.State.SideslipAngleRadians,
            car.State.YawRateRadiansPerSecond
        );
    }

    private float ApplyCarControl(
        CarState state,
        float wheelBase,
        ref float desiredCurvature,
        float dt,
        out float severity
    )
    {
        severity = StabilityControlLaw.CalculateSeverity(
            state.Speed,
            state.SideslipAngleRadians,
            state.YawRateRadiansPerSecond,
            state.Telemetry.RearSlideSeverity,
            desiredCurvature
        );

        bool unstable = StabilityControlLaw.IsUnstable(
            severity,
            _performance!.IsRecovering
        );
        _performance.UpdateControlEvent(unstable);
        StabilityControlResult result = StabilityControlLaw.Apply(
            ref _stabilityControlState,
            state.SideslipAngleRadians,
            state.YawRateRadiansPerSecond,
            state.Speed,
            desiredCurvature,
            wheelBase,
            _runtimeCar!.CarConfig.MaxCurvatureRequest,
            _performance.EffectiveControl,
            _performance.ControlGainScale,
            _performance.IsRecovering,
            dt
        );
        desiredCurvature = result.CommandedCurvature;
        return result.CurvatureCorrection;
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
            gripUsage: _speedPlanner.Config.GetAccelerationUsage(car.Strategy)
        );
        return MathF.Max(limits.LateralAccelerationLimit, 1e-3f);
    }

    private static float Lerp(float from, float to, float t) =>
        from + (to - from) * Math.Clamp(t, 0f, 1f);

    private static float ElapsedMilliseconds(long startTimestamp)
    {
        long elapsedTicks = Stopwatch.GetTimestamp() - startTimestamp;
        return (float)(elapsedTicks * 1000.0 / Stopwatch.Frequency);
    }
}
