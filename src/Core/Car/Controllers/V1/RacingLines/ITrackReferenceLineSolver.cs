using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public interface ITrackReferenceLineSolver
{
    ITrackReferenceLine Generate(TrackData track);
}
