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
public sealed class ReferenceLineDriver : IRaceDriver, ITrafficMotionPlanSource
{
    private readonly VehicleSpeedPlanner _speedPlanner;
    private readonly StanleyPathPredictor _pathPredictor = new();
    private readonly VehicleSpeedLookahead _referenceSpeedLookahead = new();
    private readonly PathPlanBuffer _currentPlan = new();
    private readonly PathPlanBuffer _handoverProbePlan = new();
    private readonly TrafficMotionPlan _publishedTrafficMotionPlan = new();
    private readonly TacticalManeuverPlanner _tacticalPlanner = new();
    private readonly TrackConstrainedLateralOffset _tacticalOffsetProfile = new();
    private RaceCar? _runtimeCar;
    private DriverPerformanceState? _performance;
    private StabilityControlState _stabilityControlState;
    private TrafficConstraintMemory _trafficMemory;
    private TrafficConflictReport _lastTrafficConflictReport;
    private TacticalIntent _tacticalIntent;
    private bool _hasPreparedFrame;
    private PreparedReferenceLineFrame _preparedFrame;

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

    private const float GravityMetersPerSecondSquared = 9.80665f;

    public float SpeedGain { get; init; } = 2.5f;
    public float StanleyGain { get; init; } = 2f;
    public float StanleySofteningSpeed { get; init; } = 4f;
    public float HeadingGain { get; init; } = 1f;
    public float CurvaturePreviewTimeSeconds { get; init; } = 0.15f;
    public float MaximumCurvaturePreviewMeters { get; init; } = 6f;

    public DriverProfile Profile { get; }
    public float TireEnergyEfficiency => _performance?.TireEnergyEfficiency ?? 1f;
    public float CorneringEfficiency => _performance?.PaceEfficiency ?? 1f;
    public float LimitSettleUse =>
        _performance?.LimitSettleUse ?? float.PositiveInfinity;
    public VehicleSpeedLookahead CurrentSpeedLookahead =>
        _currentPlan.SpeedLookahead;
    public VehiclePathPrediction CurrentPathPrediction => _currentPlan.Path;
    public ReferenceLineDriverTelemetry LastTelemetry { get; private set; }
    internal TrafficConflictReport LastTrafficConflictReport =>
        _lastTrafficConflictReport;
    internal TrafficConflictReport LastTacticalConflictReport =>
        _tacticalPlanner.LastObservedConflictReport;
    internal float LastTacticalOffsetMeters =>
        _tacticalIntent.TargetOffsetMeters;
    internal TacticalManeuverPhase LastTacticalPhase =>
        _tacticalPlanner.Phase;

    public void Initialize(in RaceDriverInitContext context)
    {
        ResetPerformanceState(context.Car, context.Pose);
    }

    public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
    {
        if (!_hasPreparedFrame ||
            !ReferenceEquals(_runtimeCar, context.Car) ||
            _preparedFrame.RaceTimeSeconds != context.RaceTimeSeconds)
        {
            PrepareCurrentFrame(in context, dt);
        }

        PreparedReferenceLineFrame prepared = _preparedFrame;
        RaceCar car = context.Car;
        CarState state = car.State;
        DriverPerformanceState performance = _performance!;
        long trafficPlanningStartTimestamp = Stopwatch.GetTimestamp();
        if (context.HasFrameSnapshot)
        {
            RaceFrameSnapshot frame = context.Frame;
            TrafficAwareSpeedPlan trafficPlan = _speedPlanner.PlanPredictedPath(
                _currentPlan.SpeedLookahead,
                car,
                _currentPlan.Path,
                context.Track,
                in frame,
                context.CarSnapshotIndex,
                in _trafficMemory
            );
            _currentPlan.SpeedPlan = trafficPlan.SpeedPlan;
            _currentPlan.TrafficConstraint = trafficPlan.TrafficConstraint;
            _currentPlan.NextTrafficMemory = trafficPlan.NextTrafficMemory;
            _lastTrafficConflictReport = trafficPlan.ConflictReport;
        }
        else
        {
            _currentPlan.SpeedPlan = _speedPlanner.PlanPredictedPath(
                _currentPlan.SpeedLookahead,
                car,
                _currentPlan.Path,
                context.Track
            );
            _currentPlan.TrafficConstraint = default;
            _currentPlan.NextTrafficMemory = default;
            _lastTrafficConflictReport = default;
        }
        CommitSelectedPlan(_currentPlan);
        DynamicPathSpeedPlan dynamicPlan = _currentPlan.SpeedPlan;
        TrafficSpeedConstraint trafficConstraint =
            _currentPlan.TrafficConstraint;
        VehicleSpeedPlanPoint speedReference = dynamicPlan.Current;
        float rollingSpeedPlanningMilliseconds =
            prepared.BaseSpeedPlanningMilliseconds +
            ElapsedMilliseconds(trafficPlanningStartTimestamp);

        float referenceAcceleration = speedReference.ReferenceAcceleration;
        if (state.Speed > speedReference.TargetSpeed)
            referenceAcceleration = MathF.Min(referenceAcceleration, 0f);
        CarPerformanceLimits currentLimits = CarPhysics.EstimatePerformanceLimits(
            state,
            car.CarConfig,
            car.TireConfig,
            car.Strategy,
            state.Speed,
            prepared.DesiredCurvature,
            _speedPlanner.Config.GetAccelerationUsage(car.Strategy),
            referenceAcceleration,
            frontBrakeBiasOffset: 0f,
            corneringEfficiency: performance.PaceEfficiency
        );
        // The speed plan stores net vehicle acceleration, while DriverInput asks
        // for axle acceleration before rolling, aero, and cornering losses.
        float lossCompensationAcceleration = currentLimits.LossAcceleration;
        // Gravity stands between the axle and the road exactly as the losses
        // do, and it has to be paid for in the same place. Without this the
        // plan may know the hill is there and the car still will not hold the
        // planned speed on it: only the proportional term is left to answer
        // the gradient, and a proportional term answers a standing pull with
        // a standing error.
        float gradeCompensationAcceleration = -RoadAttitudeAt(in prepared)
            .AlongTrackGravity(GravityMetersPerSecondSquared);
        float speedFeedbackAcceleration = SpeedGain *
                                          (speedReference.TargetSpeed - state.Speed);
        float desiredAcceleration = referenceAcceleration +
                                    lossCompensationAcceleration +
                                    gradeCompensationAcceleration +
                                    speedFeedbackAcceleration;
        float driveAccelerationLimit =
            currentLimits.MaximumDriveAcceleration *
            _speedPlanner.Config.DriveAccelerationUsage *
            performance.EstimatedGripScale;
        if (desiredAcceleration > 0f && prepared.ControlSeverity > 0f)
        {
            float retainedDriveAtFullSeverity = Lerp(
                0.25f,
                0.8f,
                performance.EffectiveControl
            );
            desiredAcceleration *= 1f - prepared.ControlSeverity *
                                   (1f - retainedDriveAtFullSeverity);
        }
        desiredAcceleration = Math.Clamp(
            desiredAcceleration,
            -car.CarConfig.MaxBrakeAccel,
            driveAccelerationLimit
        );

        LastTelemetry = new ReferenceLineDriverTelemetry(
            prepared.FrontPose.S,
            prepared.LateralErrorMeters,
            prepared.HeadingErrorRadians,
            prepared.FrontSample.RefCurvature,
            prepared.PreviewSample.RefCurvature,
            prepared.DesiredCurvature,
            prepared.CurvatureCorrection,
            _currentPlan.Path.LengthMeters,
            _currentPlan.Path.MaximumAbsoluteCommandedCurvature,
            _currentPlan.Path.TerminalLateralErrorMeters,
            _currentPlan.Path.DynamicPredictionLengthMeters,
            _currentPlan.Path.JoinsReferenceLine,
            _currentPlan.Path.ReferenceLineJoinCurvatureDelta,
            prepared.PathPredictionMilliseconds,
            rollingSpeedPlanningMilliseconds,
            trafficConstraint.Kind,
            trafficConstraint.OpponentId,
            trafficConstraint.PathDistanceMeters,
            trafficConstraint.TargetSpeedMetersPerSecond,
            trafficConstraint.PredictedConflictTimeSeconds,
            trafficConstraint.CurrentClearanceMeters,
            _lastTrafficConflictReport.EvaluationDistanceMeters,
            _lastTrafficConflictReport.FreeArrivalTimeSeconds,
            _lastTrafficConflictReport.ConstrainedArrivalTimeSeconds,
            _lastTrafficConflictReport.TimeLossSeconds,
            prepared.ReferencePathPlan.TargetSpeed,
            speedReference.TargetSpeed,
            referenceAcceleration,
            lossCompensationAcceleration,
            gradeCompensationAcceleration,
            speedFeedbackAcceleration,
            driveAccelerationLimit,
            desiredAcceleration,
            Profile.Abilities.Pace,
            Profile.Abilities.Consistency,
            Profile.Abilities.CarControl,
            Profile.Abilities.TireManagement,
            Profile.Abilities.Adaptability,
            performance.SessionForm,
            performance.LapForm,
            performance.SegmentForm,
            performance.PlanningPace,
            performance.EffectivePace,
            performance.PaceEfficiency,
            performance.EffectiveControl,
            performance.EffectiveTireManagement,
            performance.TireEnergyEfficiency,
            performance.EffectiveAdaptability,
            performance.ActualGrip,
            performance.EstimatedGrip,
            performance.EstimatedGripScale,
            performance.BrakeMarkerErrorMeters,
            prepared.LateralTargetErrorMeters,
            performance.LocalSpeedErrorFraction,
            performance.FrontBrakeBiasOffset,
            prepared.ControlSeverity,
            prepared.ControlCorrection,
            performance.IsRecovering
        );
        _hasPreparedFrame = false;
        return new DriverInput(
            prepared.DesiredCurvature,
            desiredAcceleration,
            performance.FrontBrakeBiasOffset
        );
    }

    void ITrafficMotionPlanSource.PrepareTrafficMotionPlan(
        in RaceDriverFrameContext context,
        float dt
    )
    {
        PrepareCurrentFrame(in context, dt);
    }

    private void PrepareCurrentFrame(
        in RaceDriverFrameContext context,
        float dt
    )
    {
        UpdatePerformanceState(in context, dt);
        _tacticalIntent = _tacticalPlanner.Update(
            in context,
            in _lastTrafficConflictReport
        );

        RaceCar car = context.Car;
        CarState state = car.State;
        RacingRoomSnapshot racingRoom = context.HasFrameSnapshot
            ? context.Frame.RacingRoom
            : default;
        _tacticalOffsetProfile.UpdateRacingRoomConstraint(
            context.Track,
            context.Pose.S,
            state.Speed,
            in racingRoom,
            car.Id,
            car.Collision.HalfWidthMeters
        );
        DriverPlanningModifiers planningModifiers = new(
            _performance!.PaceEfficiency,
            _performance.EstimatedGripScale *
            (1f + _performance.LocalSpeedErrorFraction),
            _performance.FrontBrakeBiasOffset
        );
        bool needsHandoverPlan =
            _tacticalOffsetProfile.RequiresHandoverPlanning(
                context.Track,
                context.Pose.S,
                _tacticalIntent.TargetOffsetMeters,
                car.Collision.HalfWidthMeters
            );
        if (needsHandoverPlan && _currentPlan.SpeedLookahead.Count > 0)
        {
            ReferenceLineHandoverConstraints handoverConstraints =
                _speedPlanner.CreateHandoverConstraints(
                    car,
                    _currentPlan.SpeedLookahead,
                    planningModifiers,
                    _tacticalIntent.LatestCompletionDistanceMeters
                );
            _tacticalOffsetProfile.UpdateCommittedTarget(
                context.Track,
                context.Pose.S,
                state.Speed,
                _tacticalIntent.TargetOffsetMeters,
                car.Collision.HalfWidthMeters,
                in handoverConstraints
            );
            if (_referenceSpeedLookahead.Count > 0)
            {
                float baselineTargetSpeed =
                    _currentPlan.SpeedLookahead.Sample(0f).TargetSpeed;
                TrackData track = context.Track;
                float tacticalOffset =
                    _tacticalOffsetProfile.CommittedTargetOffsetMeters;
                float executionOffset = _performance.LateralTargetErrorMeters;
                ReferenceLineHandoverSpeedSelector.Refine(
                    _tacticalOffsetProfile,
                    baselineTargetSpeed,
                    () => EvaluateCommittedHandoverTargetSpeed(
                        car,
                        track,
                        _referenceSpeedLookahead,
                        tacticalOffset,
                        executionOffset
                    )
                );
            }
        }
        else
        {
            _tacticalOffsetProfile.UpdateCommittedTarget(
                context.Track,
                context.Pose.S,
                state.Speed,
                _tacticalIntent.TargetOffsetMeters,
                car.Collision.HalfWidthMeters
            );
        }
        float wheelBase = MathF.Max(car.CarConfig.WheelBaseMeters, 0.5f);
        float lateralTargetError = _performance!.LateralTargetErrorMeters;
        // The path controller follows the velocity direction. Body sideslip is
        // stabilized by the vehicle layer instead of turning into an abrupt
        // Stanley heading correction.
        float tacticalOffsetMeters =
            _tacticalOffsetProfile.CommittedTargetOffsetMeters;
        StanleyControlSample control =
            !_tacticalOffsetProfile.HasCommittedProfile
            ? StanleyControlLaw.Sample(
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
            )
            : StanleyControlLaw.Sample(
                context.Track,
                state.Position,
                state.VelocityHeading,
                state.Speed,
                wheelBase,
                car.Collision.HalfWidthMeters,
                _tacticalOffsetProfile,
                tacticalOffsetMeters,
                lateralTargetError,
                StanleyGain,
                StanleySofteningSpeed,
                HeadingGain,
                CurvaturePreviewTimeSeconds,
                MaximumCurvaturePreviewMeters,
                car.CarConfig.MaxCurvatureRequest
            );
        float desiredCurvature = control.DesiredCurvature;
        float controlCorrection = ApplyCarControl(
            state,
            wheelBase,
            ref desiredCurvature,
            dt,
            out float controlSeverity
        );
        float curvatureCorrection =
            desiredCurvature - control.PreviewSample.RefCurvature;
        // Pace goes in untouched, because the car is discounted by exactly
        // this and the plan has to be the lap the car can drive. Misjudging a
        // corner's speed is an error, not a limit, so it sits with the other
        // error rather than being smuggled in as less pace.
        long planningStartTimestamp = Stopwatch.GetTimestamp();
        VehicleSpeedLookahead referenceLookahead =
            _speedPlanner.PlanReferenceLookahead(
                _referenceSpeedLookahead,
                car,
                context.Track,
                control.FrontPose.S + _performance.BrakeMarkerErrorMeters,
                _speedPlanner.Config.SpeedPlanningHorizonMeters,
                _speedPlanner.Config.PathPredictionStepMeters,
                planningModifiers
            );
        VehicleSpeedPlanPoint referencePathPlan =
            referenceLookahead.Sample(0f);
        PredictPathCandidate(
            car,
            context.Track,
            referenceLookahead,
            tacticalOffsetMeters,
            lateralTargetError,
            desiredCurvature,
            _currentPlan,
            out float pathPredictionMilliseconds
        );
        _currentPlan.SpeedPlan = _speedPlanner.PreparePredictedPathForTraffic(
            _currentPlan.SpeedLookahead,
            car,
            _currentPlan.Path,
            context.Track
        );
        _currentPlan.TrafficConstraint = default;
        _currentPlan.NextTrafficMemory = default;
        float baseSpeedPlanningMilliseconds = MathF.Max(
            0f,
            ElapsedMilliseconds(planningStartTimestamp) -
            pathPredictionMilliseconds
        );
        _preparedFrame = new PreparedReferenceLineFrame(
            context.RaceTimeSeconds,
            control.FrontPose,
            control.FrontSample,
            control.PreviewSample,
            control.LateralErrorMeters,
            control.HeadingErrorRadians,
            desiredCurvature,
            curvatureCorrection,
            controlCorrection,
            controlSeverity,
            lateralTargetError,
            referencePathPlan,
            pathPredictionMilliseconds,
            baseSpeedPlanningMilliseconds
        );
        _hasPreparedFrame = true;
    }

    TrafficMotionPlan? ITrafficMotionPlanSource.FreezeTrafficMotionPlan()
    {
        if (!_hasPreparedFrame || _currentPlan.Path.Count < 2)
        {
            _publishedTrafficMotionPlan.Clear();
            return null;
        }

        _publishedTrafficMotionPlan.BuildFrom(
            _currentPlan.Path,
            _currentPlan.SpeedLookahead
        );
        return _publishedTrafficMotionPlan;
    }

    private void PredictPathCandidate(
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead referenceLookahead,
        float tacticalOffsetMeters,
        float executionOffsetMeters,
        float initialCommandedCurvature,
        PathPlanBuffer destination,
        out float pathPredictionMilliseconds
    )
    {
        long predictionStartTimestamp = Stopwatch.GetTimestamp();
        StabilityPredictionSeed stabilitySeed = new(
            _stabilityControlState,
            _performance!.IsRecovering,
            _performance.EffectiveControl,
            _performance.ControlGainScale
        );
        if (!_tacticalOffsetProfile.HasCommittedProfile)
        {
            _pathPredictor.Predict(
                destination.Path,
                car,
                track,
                referenceLookahead,
                executionOffsetMeters,
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
                initialCommandedCurvature,
                stabilitySeed
            );
        }
        else
        {
            _pathPredictor.Predict(
                destination.Path,
                car,
                track,
                referenceLookahead,
                tacticalOffsetMeters,
                _tacticalOffsetProfile,
                executionOffsetMeters,
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
                initialCommandedCurvature,
                stabilitySeed
            );
        }
        pathPredictionMilliseconds = ElapsedMilliseconds(
            predictionStartTimestamp
        );

    }

    private float EvaluateCommittedHandoverTargetSpeed(
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead referenceLookahead,
        float tacticalOffsetMeters,
        float executionOffsetMeters
    )
    {
        CarState state = car.State;
        StanleyControlSample control = StanleyControlLaw.Sample(
            track,
            state.Position,
            state.VelocityHeading,
            state.Speed,
            MathF.Max(car.CarConfig.WheelBaseMeters, 0.5f),
            car.Collision.HalfWidthMeters,
            _tacticalOffsetProfile,
            tacticalOffsetMeters,
            executionOffsetMeters,
            StanleyGain,
            StanleySofteningSpeed,
            HeadingGain,
            CurvaturePreviewTimeSeconds,
            MaximumCurvaturePreviewMeters,
            car.CarConfig.MaxCurvatureRequest
        );
        _pathPredictor.Predict(
            _handoverProbePlan.Path,
            car,
            track,
            referenceLookahead,
            tacticalOffsetMeters,
            _tacticalOffsetProfile,
            executionOffsetMeters,
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
            control.DesiredCurvature
        );
        DynamicPathSpeedPlan speedPlan = _speedPlanner.PlanPredictedPath(
            _handoverProbePlan.SpeedLookahead,
            car,
            _handoverProbePlan.Path,
            track
        );
        return speedPlan.Current.TargetSpeed;
    }

    private void CommitSelectedPlan(PathPlanBuffer selectedPlan)
    {
        _trafficMemory = selectedPlan.NextTrafficMemory;
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
        _trafficMemory.Clear();
        _lastTrafficConflictReport = default;
        _tacticalIntent = TacticalIntent.Keep;
        _tacticalPlanner.Reset();
        _tacticalOffsetProfile.ResetCommittedTarget();
        _hasPreparedFrame = false;
        _preparedFrame = default;
        _publishedTrafficMotionPlan.Clear();
    }

    /// <summary>
    /// The road under the front axle, read at the car's own place across the
    /// width so a cross-section that curves is taken where the car is.
    /// </summary>
    private static RoadAttitude RoadAttitudeAt(
        in PreparedReferenceLineFrame frame
    )
    {
        TrackSample sample = frame.FrontSample;
        if (sample.Grade == 0f &&
            sample.BankSlope == 0f &&
            sample.BankCurvature == 0f)
        {
            return RoadAttitude.Flat;
        }
        return new RoadAttitude(
            sample.Grade,
            sample.BankSlopeAt(frame.FrontPose.D)
        );
    }

    private readonly record struct PreparedReferenceLineFrame(
        float RaceTimeSeconds,
        TrackPose FrontPose,
        TrackSample FrontSample,
        TrackSample PreviewSample,
        float LateralErrorMeters,
        float HeadingErrorRadians,
        float DesiredCurvature,
        float CurvatureCorrection,
        float ControlCorrection,
        float ControlSeverity,
        float LateralTargetErrorMeters,
        VehicleSpeedPlanPoint ReferencePathPlan,
        float PathPredictionMilliseconds,
        float BaseSpeedPlanningMilliseconds
    );

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
            gripUsage: _speedPlanner.Config.GetAccelerationUsage(car.Strategy),
            corneringEfficiency: _performance?.PaceEfficiency ?? 1f
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
