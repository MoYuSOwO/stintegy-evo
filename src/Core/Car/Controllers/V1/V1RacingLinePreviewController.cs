using Godot;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1;

public sealed class V1RacingLinePreviewController : IController, IControllerDebugPaths
{
    private static readonly ControllerDebugPathStyle ReferenceLineStyle = new(Color.FromHtml("#65c4ff"), 1.0f, 50);

    private readonly IRacingLineSolver _solver;
    private RacingLine? _racingLine;
    private long _racingLineVersion;

    public V1RacingLinePreviewController() : this(new MinimumCurvatureRacingLineSolver())
    {
    }

    public V1RacingLinePreviewController(IRacingLineSolver solver)
    {
        _solver = solver;
    }

    public void Init(CarLogic carLogic, TrackData track)
    {
        InitRacingLine(track);
    }

    public float Input => 0f;
    public float Steer => 0f;
    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }
    public int DebugPathLineCount => 1;

    public int GetDebugPathPointCount(int lineIndex)
    {
        return lineIndex == 0 && _racingLine != null ? _racingLine.Count + 1 : 0;
    }

    public Vector2 GetDebugPathPoint(int lineIndex, int pointIndex)
    {
        return lineIndex == 0 && _racingLine != null ? _racingLine[pointIndex].Position : Vector2.Zero;
    }

    public ControllerDebugPathStyle GetDebugPathStyle(int lineIndex)
    {
        return ReferenceLineStyle;
    }

    public long GetDebugPathVersion(int lineIndex)
    {
        return lineIndex == 0 && _racingLine != null ? _racingLineVersion : -1;
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
    }

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }
}
