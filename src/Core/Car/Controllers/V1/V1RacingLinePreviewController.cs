using System;
using Godot;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Online;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1;

public sealed class V1RacingLinePreviewController : IController, IControllerDebugPaths
{
    private static readonly ControllerDebugPathStyle ReferenceLineStyle = new(Color.FromHtml("#65c4ff"), 1.0f, 50);
    private static readonly ControllerDebugPathStyle DynamicPathStyle = new(Color.FromHtml("#ffb14a"), 1.6f, 55);
    private const float DynamicPathPlanHorizonMeters = 80.0f;
    private const float DynamicPathSafetyMarginMeters = 0.5f;

    private readonly IRacingLineSolver _solver;
    private RacingLine? _racingLine;
    private long _racingLineVersion;
    private CarConfig? _carConfig;
    private DynamicPathOfflineGraph? _dynamicGraph;
    private DynamicPathOnlinePathPlanner? _dynamicPathPlanner;
    private DynamicPathOnlinePath? _dynamicPath;
    private long _dynamicPathVersion;
    private bool _dynamicPathFailureLogged;

    public V1RacingLinePreviewController() : this(new MinimumCurvatureRacingLineSolver())
    {
    }

    public V1RacingLinePreviewController(IRacingLineSolver solver)
    {
        _solver = solver;
    }

    public void Init(CarLogic carLogic, TrackData track)
    {
        _carConfig = carLogic.Config;
        InitRacingLine(track);
        InitDynamicPath(track, carLogic.Config);
    }

    public float Input => 0.1f;
    public float Steer => 0f;
    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }
    public int DebugPathLineCount => 2;

    public int GetDebugPathPointCount(int lineIndex)
    {
        return lineIndex switch
        {
            0 when _racingLine != null => _racingLine.Count + 1,
            1 when _dynamicPath != null => _dynamicPath.Samples.Length,
            _ => 0
        };
    }

    public Vector2 GetDebugPathPoint(int lineIndex, int pointIndex)
    {
        return lineIndex switch
        {
            0 when _racingLine != null => _racingLine[pointIndex].Position,
            1 when _dynamicPath != null => _dynamicPath.Samples[pointIndex].Position,
            _ => Vector2.Zero
        };
    }

    public ControllerDebugPathStyle GetDebugPathStyle(int lineIndex)
    {
        return lineIndex == 1 ? DynamicPathStyle : ReferenceLineStyle;
    }

    public long GetDebugPathVersion(int lineIndex)
    {
        return lineIndex switch
        {
            0 when _racingLine != null => _racingLineVersion,
            1 when _dynamicPath != null => _dynamicPathVersion,
            _ => -1
        };
    }

    public void InitRacingLine(TrackData track)
    {
        try
        {
            _racingLine = _solver.Generate(track);
            _racingLineVersion++;
            GD.Print($"V1 racing line generated: {_racingLine.Count} points.");
        }
        catch (System.Exception ex)
        {
            GD.PrintErr($"V1 racing line generation failed: {ex}");
            _racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
            _racingLineVersion++;
        }
    }

    public void StartRacingLineGeneration(TrackData track)
    {
        InitRacingLine(track);
        if (_carConfig != null)
            InitDynamicPath(track, _carConfig);
    }

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
        if (_dynamicGraph == null || _dynamicPathPlanner == null)
            return;

        TryUpdateDynamicPath(track, carSensor.Position, carSensor.Rotation, carSensor.LinearVelocity.Length());
    }

    private void InitDynamicPath(TrackData track, CarConfig carConfig)
    {
        _dynamicGraph = null;
        _dynamicPath = null;
        _dynamicPathVersion++;
        _dynamicPathFailureLogged = false;

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
            _dynamicPathVersion++;
        }
    }

    private void TryUpdateDynamicPath(TrackData track, Vector2 position, float heading, float speedMetersPerSecond)
    {
        if (_dynamicGraph == null || _dynamicPathPlanner == null)
            return;

        try
        {
            _dynamicPath = _dynamicPathPlanner.PlanFromPoseContinuing(_dynamicGraph, track, position, heading, speedMetersPerSecond);
            _dynamicPathFailureLogged = false;
            _dynamicPathVersion++;
        }
        catch (Exception ex)
        {
            _dynamicPath = null;
            _dynamicPathVersion++;

            if (!_dynamicPathFailureLogged)
            {
                GD.PrintErr($"V1 dynamic path planning failed: {ex}");
                _dynamicPathFailureLogged = true;
            }
        }
    }
}
