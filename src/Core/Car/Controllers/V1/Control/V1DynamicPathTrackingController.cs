using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using Godot;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;
using StintegyEVO.Core.Car.Controllers.V1.Control.Longitudinal;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Online;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.Control;

public sealed class V1DynamicPathTrackingController : IController, IControllerDebugPaths, IControllerTelemetry
{
    private static readonly ControllerDebugPathStyle ReferenceLineStyle = new(Color.FromHtml("#65c4ff"), 1.0f, 50);
    private static readonly ControllerDebugPathStyle DynamicPathStyle = new(Color.FromHtml("#ffb14a"), 1.6f, 55);
    private static readonly ControllerDebugPathStyle LookaheadStyle = new(Color.FromHtml("#70f078"), 1.2f, 60);
    private const float DynamicPathPlanHorizonMeters = 300.0f;
    private const float DynamicPathSafetyMarginMeters = 0.5f;
    private const float ControllerMaximumSpeedMetersPerSecond = 85.0f;
    private const float ControllerTerminalSpeedMetersPerSecond = float.PositiveInfinity;
    private const float ControllerFrictionUsage = 0.95f;
    private const float DefaultPlanningPeriodSeconds = 0.10f;

    private readonly IRacingLineSolver _solver;
    private readonly SpeedProfilePlanner _speedProfilePlanner;
    private readonly EmpiricalHandlingDiagramLateralController _lateralController;
    private readonly StabilityAwareSpeedController _longitudinalController;
    private readonly float _planningPeriodSeconds;
    private RacingLine? _racingLine;
    private long _racingLineVersion;
    private DynamicPathOfflineGraph? _dynamicGraph;
    private DynamicPathOnlinePathPlanner? _dynamicPathPlanner;
    private DynamicPathOnlinePath? _dynamicPath;
    private long _dynamicPathVersion;
    private SpeedProfile? _speedProfile;
    private LateralControlOutput _lastLateralOutput;
    private StabilityAwareSpeedControlOutput _lastLongitudinalOutput;
    private Vector2 _lastPosition;
    private long _lookaheadVersion;
    private bool _dynamicPathFailureLogged;
    private bool _hasLookahead;
    private float _timeUntilPlanning;
    private float _input;
    private float _steer;
    private float _pathPlanUsec;
    private float _speedPlanUsec;
    private float _lateralUsec;
    private float _longitudinalUsec;

    public V1DynamicPathTrackingController() : this(
        new MinimumCurvatureRacingLineSolver(),
        new SpeedProfilePlanner(CreateControllerSpeedPlanningConfig()),
        new EmpiricalHandlingDiagramLateralController(),
        new StabilityAwareSpeedController(new StabilityAwareSpeedControlConfig
        {
            EnvelopeConfig = CreateControllerSpeedPlanningConfig()
        })
    )
    {
    }

    public V1DynamicPathTrackingController(
        IRacingLineSolver solver,
        SpeedProfilePlanner speedProfilePlanner,
        EmpiricalHandlingDiagramLateralController lateralController,
        StabilityAwareSpeedController longitudinalController,
        float planningPeriodSeconds = DefaultPlanningPeriodSeconds
    )
    {
        if (planningPeriodSeconds < 0.0f)
            throw new ArgumentOutOfRangeException(nameof(planningPeriodSeconds));

        _solver = solver;
        _speedProfilePlanner = speedProfilePlanner;
        _lateralController = lateralController;
        _longitudinalController = longitudinalController;
        _planningPeriodSeconds = planningPeriodSeconds;
    }

    public float Input => _input;
    public float Steer => _steer;
    public float PlanningPeriodSeconds => _planningPeriodSeconds;
    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }
    public int DebugPathLineCount => 3;
    public IEnumerable<string> TelemetryColumns =>
    [
        "v1_path_valid",
        "v1_path_used_previous",
        "v1_path_prefix_samples",
        "v1_path_length",
        "v1_time_path_us",
        "v1_time_speed_us",
        "v1_time_lateral_us",
        "v1_time_longitudinal_us",
        "v1_ctrl_ehd_fit_samples",
        "v1_ctrl_target_speed",
        "v1_ctrl_lookahead_distance",
        "v1_ctrl_lookahead_x",
        "v1_ctrl_lookahead_y",
        "v1_ctrl_lateral_error",
        "v1_ctrl_heading_error",
        "v1_ctrl_beta",
        "v1_ctrl_beta_ref",
        "v1_ctrl_yaw_rate",
        "v1_ctrl_yaw_ref",
        "v1_ctrl_yaw_error",
        "v1_ctrl_stability_risk",
        "v1_ctrl_steer_ff",
        "v1_ctrl_steer_fb",
        "v1_ctrl_steer_stability",
        "v1_ctrl_steer_yaw_damping",
        "v1_ctrl_steer_beta_damping",
        "v1_ctrl_steering_angle",
        "v1_ctrl_target_ay",
        "v1_ctrl_track_offset",
        "v1_ctrl_track_usable_half_width",
        "v1_ctrl_track_boundary_excess",
        "v1_ctrl_track_buffer_excess",
        "v1_ctrl_nearest_index",
        "v1_ctrl_profile_distance",
        "v1_long_target_speed",
        "v1_long_target_accel",
        "v1_long_requested_accel",
        "v1_long_limited_accel",
        "v1_long_max_accel",
        "v1_long_max_decel",
        "v1_long_lateral_demand",
        "v1_long_lateral_speed_limit",
        "v1_long_tracking_risk",
        "v1_long_stability_risk"
    ];

    public void Init(CarLogic carLogic, TrackData track)
    {
        _input = 0.0f;
        _steer = 0.0f;
        _timeUntilPlanning = 0.0f;
        _lateralController.Initialize(carLogic.Config, track);
        _lateralController.Reset();
        InitRacingLine(track);
        InitDynamicPath(track, carLogic.Config);
    }

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
        if (_dynamicGraph == null || _dynamicPathPlanner == null)
        {
            _input = 0.0f;
            _steer = 0.0f;
            return;
        }

        _timeUntilPlanning -= dt;
        if (_speedProfile == null || _dynamicPath == null || _timeUntilPlanning <= 0.0f)
        {
            TryUpdatePlan(track, carSensor, carLogic);
            _timeUntilPlanning = _planningPeriodSeconds;
        }

        if (_speedProfile == null)
        {
            _input = 0.0f;
            _steer = 0.0f;
            _hasLookahead = false;
            return;
        }

        try
        {
            long lateralStart = Stopwatch.GetTimestamp();
            _lastLateralOutput = _lateralController.Control(_speedProfile, carSensor, carLogic.Config, track, dt);
            _lateralUsec = ElapsedUsec(lateralStart);

            long longitudinalStart = Stopwatch.GetTimestamp();
            _lastLongitudinalOutput = _longitudinalController.Control(
                _speedProfile,
                _lastLateralOutput,
                carSensor,
                carLogic,
                track
            );
            _longitudinalUsec = ElapsedUsec(longitudinalStart);

            _steer = _lastLateralOutput.SteeringInput;
            _input = _lastLongitudinalOutput.Input;
            _lastPosition = carSensor.Position;
            _hasLookahead = _lastLateralOutput.NearestProfileIndex >= 0;
            _lookaheadVersion++;
        }
        catch (Exception ex)
        {
            _input = 0.0f;
            _steer = 0.0f;
            _hasLookahead = false;
            if (!_dynamicPathFailureLogged)
            {
                GD.PrintErr($"V1 dynamic path tracking control failed: {ex}");
                _dynamicPathFailureLogged = true;
            }
        }
    }

    public int GetDebugPathPointCount(int lineIndex)
    {
        return lineIndex switch
        {
            0 when _racingLine != null => _racingLine.Count + 1,
            1 when _dynamicPath != null => _dynamicPath.Samples.Length,
            2 when _hasLookahead => 2,
            _ => 0
        };
    }

    public Vector2 GetDebugPathPoint(int lineIndex, int pointIndex)
    {
        return lineIndex switch
        {
            0 when _racingLine != null => _racingLine[pointIndex].Position,
            1 when _dynamicPath != null => _dynamicPath.Samples[pointIndex].Position,
            2 when _hasLookahead && pointIndex == 0 => _lastPosition,
            2 when _hasLookahead => _lastLateralOutput.LookaheadPoint,
            _ => Vector2.Zero
        };
    }

    public ControllerDebugPathStyle GetDebugPathStyle(int lineIndex)
    {
        return lineIndex switch
        {
            1 => DynamicPathStyle,
            2 => LookaheadStyle,
            _ => ReferenceLineStyle
        };
    }

    public long GetDebugPathVersion(int lineIndex)
    {
        return lineIndex switch
        {
            0 when _racingLine != null => _racingLineVersion,
            1 when _dynamicPath != null => _dynamicPathVersion,
            2 when _hasLookahead => _lookaheadVersion,
            _ => -1
        };
    }

    public void AppendTelemetryValues(StringBuilder builder)
    {
        TelemetryCsv.Append(builder, _dynamicPath != null);
        TelemetryCsv.Append(builder, _dynamicPath?.UsedPreviousPath ?? false);
        TelemetryCsv.Append(builder, _dynamicPath?.ConstantPrefixSampleCount ?? 0);
        TelemetryCsv.Append(builder, _dynamicPath?.PhysicalLength ?? 0.0f);
        TelemetryCsv.Append(builder, _pathPlanUsec);
        TelemetryCsv.Append(builder, _speedPlanUsec);
        TelemetryCsv.Append(builder, _lateralUsec);
        TelemetryCsv.Append(builder, _longitudinalUsec);
        TelemetryCsv.Append(builder, _lateralController.Model.FitSampleCount);
        TelemetryCsv.Append(builder, _lastLateralOutput.TargetSpeed);
        TelemetryCsv.Append(builder, _lastLateralOutput.LookaheadDistance);
        TelemetryCsv.Append(builder, _lastLateralOutput.LookaheadPoint.X);
        TelemetryCsv.Append(builder, _lastLateralOutput.LookaheadPoint.Y);
        TelemetryCsv.Append(builder, _lastLateralOutput.LateralError);
        TelemetryCsv.Append(builder, _lastLateralOutput.HeadingError);
        TelemetryCsv.Append(builder, _lastLateralOutput.Beta);
        TelemetryCsv.Append(builder, _lastLateralOutput.BetaReference);
        TelemetryCsv.Append(builder, _lastLateralOutput.YawRate);
        TelemetryCsv.Append(builder, _lastLateralOutput.YawRateReference);
        TelemetryCsv.Append(builder, _lastLateralOutput.YawRateError);
        TelemetryCsv.Append(builder, _lastLateralOutput.StabilityRisk);
        TelemetryCsv.Append(builder, _lastLateralOutput.FeedForwardSteeringAngle);
        TelemetryCsv.Append(builder, _lastLateralOutput.FeedbackSteeringAngle);
        TelemetryCsv.Append(builder, _lastLateralOutput.StabilitySteeringAngle);
        TelemetryCsv.Append(builder, _lastLateralOutput.YawDampingSteeringAngle);
        TelemetryCsv.Append(builder, _lastLateralOutput.BetaDampingSteeringAngle);
        TelemetryCsv.Append(builder, _lastLateralOutput.SteeringAngle);
        TelemetryCsv.Append(builder, _lastLateralOutput.TargetLateralAcceleration);
        TelemetryCsv.Append(builder, _lastLateralOutput.TrackOffset);
        TelemetryCsv.Append(builder, _lastLateralOutput.TrackUsableHalfWidth);
        TelemetryCsv.Append(builder, _lastLateralOutput.TrackBoundaryExcess);
        TelemetryCsv.Append(builder, _lastLateralOutput.TrackBufferBoundaryExcess);
        TelemetryCsv.Append(builder, _lastLateralOutput.NearestProfileIndex);
        TelemetryCsv.Append(builder, _lastLateralOutput.ProfileDistance);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.TargetSpeed);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.TargetAcceleration);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.RequestedAcceleration);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.LimitedAcceleration);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.MaximumAcceleration);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.MaximumDeceleration);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.LateralDemandAcceleration);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.LateralSpeedLimit);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.TrackingRisk);
        TelemetryCsv.Append(builder, _lastLongitudinalOutput.StabilityRisk);
    }

    private void TryUpdatePlan(TrackData track, CarSensor carSensor, CarLogic carLogic)
    {
        try
        {
            long pathStart = Stopwatch.GetTimestamp();
            _dynamicPath = _dynamicPathPlanner!.PlanFromPoseContinuing(
                _dynamicGraph!,
                track,
                carSensor.Position,
                carSensor.Rotation,
                carSensor.LinearVelocity.Length()
            );
            _pathPlanUsec = ElapsedUsec(pathStart);
            _dynamicPathVersion++;

            long speedStart = Stopwatch.GetTimestamp();
            _speedProfile = _speedProfilePlanner.PlanCurrentFrame(_dynamicPath, carSensor, carLogic, track);
            _speedPlanUsec = ElapsedUsec(speedStart);
            _dynamicPathFailureLogged = false;
        }
        catch (Exception ex)
        {
            _pathPlanUsec = 0.0f;
            _speedPlanUsec = 0.0f;
            if (_speedProfile == null)
            {
                _dynamicPath = null;
                _dynamicPathVersion++;
            }

            if (!_dynamicPathFailureLogged)
            {
                GD.PrintErr($"V1 dynamic path planning failed: {ex}");
                _dynamicPathFailureLogged = true;
            }
        }
    }

    private void InitRacingLine(TrackData track)
    {
        try
        {
            _racingLine = _solver.Generate(track);
            _racingLineVersion++;
            GD.Print($"V1 racing line generated: {_racingLine.Count} points.");
        }
        catch (Exception ex)
        {
            GD.PrintErr($"V1 racing line generation failed: {ex}");
            _racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
            _racingLineVersion++;
        }
    }

    private void InitDynamicPath(TrackData track, CarConfig carConfig)
    {
        _dynamicGraph = null;
        _dynamicPath = null;
        _speedProfile = null;
        _dynamicPathVersion++;
        _dynamicPathFailureLogged = false;
        _hasLookahead = false;

        if (_racingLine == null)
            return;

        try
        {
            _dynamicGraph = new DynamicPathOfflineGraphBuilder().Build(
                track,
                _racingLine,
                carConfig,
                new DynamicPathOfflineConfig
                {
                    SafetyMarginMeters = DynamicPathSafetyMarginMeters
                }
            );
            _dynamicPathPlanner = new DynamicPathOnlinePathPlanner(new DynamicPathOnlineConfig
            {
                MinimumPlanHorizonMeters = DynamicPathPlanHorizonMeters,
                StartLayerLookahead = 2
            });
            _dynamicPathPlanner.ResetMemory();

            GD.Print($"V1 dynamic path graph generated: {_dynamicGraph.LayerCount} layers, {_dynamicGraph.EdgeCount} edges.");
        }
        catch (Exception ex)
        {
            GD.PrintErr($"V1 dynamic path graph generation failed: {ex}");
            _dynamicGraph = null;
            _dynamicPathPlanner = null;
            _dynamicPath = null;
            _speedProfile = null;
            _dynamicPathVersion++;
        }
    }

    private static SpeedPlanningConfig CreateControllerSpeedPlanningConfig()
    {
        return new SpeedPlanningConfig
        {
            MaximumSpeedMetersPerSecond = ControllerMaximumSpeedMetersPerSecond,
            TerminalSpeedMetersPerSecond = ControllerTerminalSpeedMetersPerSecond,
            FrictionUsage = ControllerFrictionUsage
        };
    }

    private static float ElapsedUsec(long startTimestamp)
    {
        long endTimestamp = Stopwatch.GetTimestamp();
        return (float)((endTimestamp - startTimestamp) * 1_000_000.0 / Stopwatch.Frequency);
    }
}
