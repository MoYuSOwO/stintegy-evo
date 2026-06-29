using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public sealed class MinimumCurvatureReferenceLineSolver(IRacingLineSolver? solver = null) : ITrackReferenceLineSolver
{
    private readonly IRacingLineSolver _solver = solver ?? new MinimumCurvatureRacingLineSolver();

    public ITrackReferenceLine Generate(TrackData track)
    {
        return new RacingLineReferenceAdapter(_solver.Generate(track));
    }
}
