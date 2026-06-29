using System;
using System.Collections.Generic;
using System.Text;
using Godot;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Car.Controllers.V1.GraphBased;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;
using StintegyEVO.Core.Track;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1;

public sealed class V1GraphPurePursuitController : IController, IControllerDebugPaths, IControllerTelemetry
{
    private const float LookaheadMeters = 8f;
    private const float SpeedLookaheadMeters = 2f;
    private const float MaxProfileSpeedMs = 90f;
    private const float MaxProfileLongitudinalAccel = 3.0f;
    private const float MaxProfileLongitudinalDecel = 7.0f;
    private const float MaxProfileLateralAccel = 6.0f;
    private const float InputDeadbandMs = 0.35f;
    private const float ThrottleGain = 0.08f;
    private const float BrakeGain = 0.12f;
    private const float PositiveAccelFeedForwardGain = 0.035f;
    private const float NegativeAccelFeedForwardGain = 0.06f;
    private const float MaxThrottle = 0.35f;
    private const float MaxBrake = 0.55f;
    private const float MaxSteerRatePerSecond = 1.8f;

    private static readonly ControllerDebugPathStyle ReferenceLineStyle = new(Color.FromHtml("#65c4ff"), 1.0f, 50);
    private static readonly ControllerDebugPathStyle SelectedGraphPathStyle = new(Color.FromHtml("#ffd166"), 1.8f, 55);

    private readonly IRacingLineSolver _solver;
    private readonly GraphBasedLocalPlanner _graphPlanner = new();
    private readonly IVelocityProfileSolver _velocityProfileSolver = new FbgaVelocityProfileSolver();
    private readonly GraphOnlineTrajectoryHandler _trajectoryHandler;
    private readonly GraphPlannerConfig _graphPlannerConfig = new()
    {
        HorizonMeters = 120f,
        LateralResolutionMeters = 0.5f,
    };

    private RacingLine? _racingLine;
    private ITrackReferenceLine? _referenceLine;
    private GraphPlannerCache? _graphPlannerCache;
    private GraphPath? _selectedGraphPath;
    private VelocityProfile _selectedVelocityProfile = VelocityProfile.Empty;
    private bool _reportedGraphPathException;
    private long _selectedGraphPathVersion;
    private long _graphPlanUsec;
    private long _velocityProfileUsec;
    private long _graphPlanAllocatedBytes;
    private bool _graphPlanReusedPreviousPath;
    private bool _graphPlanResetPreviousPath;
    private float _graphPlanPreviousPathDistanceMeters;
    private int _graphPlanGc0Delta;
    private int _graphPlanGc1Delta;
    private int _graphPlanGc2Delta;
    private float _targetSpeedMs;
    private float _targetAccelerationMs2;
    private float _input;
    private float _steer;

    public V1GraphPurePursuitController() : this(new MinimumCurvatureRacingLineSolver())
    {
    }

    public V1GraphPurePursuitController(IRacingLineSolver solver)
    {
        _solver = solver;
        _trajectoryHandler = new GraphOnlineTrajectoryHandler(_graphPlanner);
    }

    public float Input => _input;
    public float Steer => _steer;
    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }

    public IEnumerable<string> TelemetryColumns
    {
        get
        {
            yield return "graph_plan_usec";
            yield return "graph_velocity_profile_usec";
            yield return "graph_plan_alloc_bytes";
            yield return "graph_reused_previous_path";
            yield return "graph_reset_previous_path";
            yield return "graph_previous_path_distance";
            yield return "graph_plan_gc0_delta";
            yield return "graph_plan_gc1_delta";
            yield return "graph_plan_gc2_delta";
            yield return "graph_path_points";
            yield return "graph_path_fallback";
            yield return "graph_target_speed_ms";
            yield return "graph_target_accel_ms2";
            yield return "graph_velocity_profile_points";
        }
    }

    public int DebugPathLineCount => _selectedGraphPath == null ? 1 : 2;

    public void Init(CarLogic carLogic, TrackData track)
    {
        try
        {
            _racingLine = _solver.Generate(track);
            GD.Print($"V1 graph pure-pursuit racing line generated: {_racingLine.Count} points.");
        }
        catch (Exception ex)
        {
            GD.PrintErr($"V1 graph pure-pursuit racing line generation failed: {ex}");
            _racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
        }

        _referenceLine = new RacingLineReferenceAdapter(_racingLine);
        _graphPlannerCache = _graphPlanner.BuildCache(track, _referenceLine, _graphPlannerConfig);
        _selectedGraphPath = null;
        _selectedVelocityProfile = VelocityProfile.Empty;
        _trajectoryHandler.Reset();
    }

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
        UpdateSelectedGraphPath(carSensor, carLogic, track);
        UpdateControl(dt, carSensor, carLogic);
    }

    public int GetDebugPathPointCount(int lineIndex)
    {
        return lineIndex switch
        {
            0 => _racingLine == null ? 0 : _racingLine.Count + 1,
            1 => _selectedGraphPath?.Count ?? 0,
            _ => 0
        };
    }

    public Vector2 GetDebugPathPoint(int lineIndex, int pointIndex)
    {
        return lineIndex switch
        {
            0 => _racingLine![pointIndex].Position,
            1 => _selectedGraphPath![pointIndex].Position,
            _ => Vector2.Zero
        };
    }

    public ControllerDebugPathStyle GetDebugPathStyle(int lineIndex)
    {
        return lineIndex == 1 ? SelectedGraphPathStyle : ReferenceLineStyle;
    }

    public long GetDebugPathVersion(int lineIndex)
    {
        return lineIndex switch
        {
            0 => _racingLine == null ? -1 : 1,
            1 => _selectedGraphPath == null ? -1 : _selectedGraphPathVersion,
            _ => -1
        };
    }

    private void UpdateSelectedGraphPath(CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
        if (_referenceLine == null || _graphPlannerCache == null)
            return;

        try
        {
            long allocatedBefore = GC.GetAllocatedBytesForCurrentThread();
            int gc0Before = GC.CollectionCount(0);
            int gc1Before = GC.CollectionCount(1);
            int gc2Before = GC.CollectionCount(2);
            ulong planStart = Time.GetTicksUsec();
            GraphOnlineTrajectory trajectory = _trajectoryHandler.Plan(new GraphPlannerRequest(
                track,
                carSensor.Position,
                carSensor.Rotation,
                _referenceLine,
                _graphPlannerConfig,
                PreviousPath: null,
                Cache: _graphPlannerCache
            ));
            ulong profileStart = Time.GetTicksUsec();
            _selectedGraphPath = trajectory.Path;
            _selectedVelocityProfile = _velocityProfileSolver.Solve(new VelocityProfileRequest(
                _selectedGraphPath,
                carSensor.LinearVelocity.Length(),
                MaxProfileSpeedMs,
                MaxProfileLongitudinalAccel,
                MaxProfileLongitudinalDecel,
                MaxProfileLateralAccel,
                AccelerationEnvelope: new VehicleAccelerationEnvelope(carLogic)
            ));
            ulong profileEnd = Time.GetTicksUsec();
            _graphPlanUsec = (long)(profileStart - planStart);
            _velocityProfileUsec = (long)(profileEnd - profileStart);
            _graphPlanAllocatedBytes = GC.GetAllocatedBytesForCurrentThread() - allocatedBefore;
            _graphPlanReusedPreviousPath = trajectory.ReusedPreviousPath;
            _graphPlanResetPreviousPath = trajectory.ResetPreviousPath;
            _graphPlanPreviousPathDistanceMeters = trajectory.PreviousPathDistanceMeters;
            _graphPlanGc0Delta = GC.CollectionCount(0) - gc0Before;
            _graphPlanGc1Delta = GC.CollectionCount(1) - gc1Before;
            _graphPlanGc2Delta = GC.CollectionCount(2) - gc2Before;
            _selectedGraphPathVersion++;
            _reportedGraphPathException = false;
        }
        catch (Exception ex)
        {
            _selectedGraphPath = null;
            _selectedVelocityProfile = VelocityProfile.Empty;
            _input = 0f;
            _steer = 0f;
            if (_reportedGraphPathException)
                return;

            _reportedGraphPathException = true;
            GD.PrintErr($"V1 graph pure-pursuit path generation failed: {ex}");
        }
    }

    public void AppendTelemetryValues(StringBuilder builder)
    {
        TelemetryCsv.Append(builder, (int)_graphPlanUsec);
        TelemetryCsv.Append(builder, (int)_velocityProfileUsec);
        TelemetryCsv.Append(builder, (int)Math.Min(_graphPlanAllocatedBytes, int.MaxValue));
        TelemetryCsv.Append(builder, _graphPlanReusedPreviousPath);
        TelemetryCsv.Append(builder, _graphPlanResetPreviousPath);
        TelemetryCsv.Append(
            builder,
            float.IsFinite(_graphPlanPreviousPathDistanceMeters) ? _graphPlanPreviousPathDistanceMeters : -1f
        );
        TelemetryCsv.Append(builder, _graphPlanGc0Delta);
        TelemetryCsv.Append(builder, _graphPlanGc1Delta);
        TelemetryCsv.Append(builder, _graphPlanGc2Delta);
        TelemetryCsv.Append(builder, _selectedGraphPath?.Count ?? 0);
        TelemetryCsv.Append(builder, _selectedGraphPath?.IsFallback == true);
        TelemetryCsv.Append(builder, _targetSpeedMs);
        TelemetryCsv.Append(builder, _targetAccelerationMs2);
        TelemetryCsv.Append(builder, _selectedVelocityProfile.Count);
    }

    private void UpdateControl(float dt, CarSensor carSensor, CarLogic carLogic)
    {
        if (_selectedGraphPath == null || _selectedGraphPath.Count <= 1)
        {
            _input = 0f;
            _steer = 0f;
            return;
        }

        Vector2 target = FindLookaheadPoint(_selectedGraphPath, carSensor.Position, LookaheadMeters);
        Vector2 toTarget = target - carSensor.Position;
        float targetDistance = Math.Max(toTarget.Length(), 0.001f);
        float targetAngle = toTarget.Angle();
        float alpha = GeomUtil.NormalizeAngle(targetAngle - carSensor.Rotation);
        float steerAngle = Mathf.Atan2(
            2f * carLogic.Config.Chassis.WheelBase * Mathf.Sin(alpha),
            targetDistance
        );
        float targetSteer = Mathf.Clamp(
            steerAngle / carLogic.Config.Chassis.MaxSteerAngle,
            -1f,
            1f
        );

        float maxSteerDelta = Math.Max(dt, 0f) * MaxSteerRatePerSecond;
        _steer = Mathf.Clamp(targetSteer, _steer - maxSteerDelta, _steer + maxSteerDelta);

        int nearestPathIndex = FindNearestPathIndex(_selectedGraphPath, carSensor.Position);
        int targetSpeedIndex = FindLookaheadPathIndex(_selectedGraphPath, nearestPathIndex, SpeedLookaheadMeters);
        VelocityProfilePoint targetSpeedPoint = _selectedVelocityProfile[targetSpeedIndex];
        VelocityProfilePoint currentVelocityPoint = _selectedVelocityProfile[nearestPathIndex];
        _targetSpeedMs = targetSpeedPoint.TargetSpeed;
        _targetAccelerationMs2 = currentVelocityPoint.TargetAcceleration;

        float speedError = _targetSpeedMs - carSensor.LinearVelocity.Length();
        float speedCommand = 0f;
        if (Mathf.Abs(speedError) <= InputDeadbandMs)
        {
            speedCommand = 0f;
        }
        else if (speedError > 0f)
        {
            speedCommand = speedError * ThrottleGain;
        }
        else
        {
            speedCommand = speedError * BrakeGain;
        }

        float accelCommand = _targetAccelerationMs2 >= 0f
            ? _targetAccelerationMs2 * PositiveAccelFeedForwardGain
            : _targetAccelerationMs2 * NegativeAccelFeedForwardGain;
        float inputCommand = speedCommand + accelCommand;
        if (speedError > InputDeadbandMs && inputCommand < 0f)
            inputCommand = 0f;
        else if (speedError < -InputDeadbandMs && inputCommand > 0f)
            inputCommand = 0f;

        _input = inputCommand >= 0f
            ? Mathf.Clamp(inputCommand, 0f, MaxThrottle)
            : -Mathf.Clamp(-inputCommand, 0f, MaxBrake);
    }

    private static Vector2 FindLookaheadPoint(GraphPath path, Vector2 position, float lookaheadMeters)
    {
        int nearestIndex = 0;
        float nearestDistSq = float.MaxValue;
        for (int i = 0; i < path.Count; i++)
        {
            float distSq = position.DistanceSquaredTo(path[i].Position);
            if (distSq < nearestDistSq)
            {
                nearestDistSq = distSq;
                nearestIndex = i;
            }
        }

        float targetDistance = path[nearestIndex].Distance + lookaheadMeters;
        for (int i = nearestIndex; i < path.Count; i++)
        {
            if (path[i].Distance >= targetDistance)
                return path[i].Position;
        }

        return path[path.Count - 1].Position;
    }

    private static int FindNearestPathIndex(GraphPath path, Vector2 position)
    {
        int nearestIndex = 0;
        float nearestDistSq = float.MaxValue;
        for (int i = 0; i < path.Count; i++)
        {
            float distSq = position.DistanceSquaredTo(path[i].Position);
            if (distSq < nearestDistSq)
            {
                nearestDistSq = distSq;
                nearestIndex = i;
            }
        }

        return nearestIndex;
    }

    private static int FindLookaheadPathIndex(GraphPath path, int nearestIndex, float lookaheadMeters)
    {
        int safeNearestIndex = Math.Clamp(nearestIndex, 0, path.Count - 1);
        float targetDistance = path[safeNearestIndex].Distance + Math.Max(0f, lookaheadMeters);
        int targetIndex = safeNearestIndex;
        for (int i = safeNearestIndex; i < path.Count; i++)
        {
            targetIndex = i;
            if (path[i].Distance >= targetDistance)
                break;
        }

        return targetIndex;
    }
}
