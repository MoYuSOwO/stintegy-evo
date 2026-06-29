using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public interface IRacingLineSolver
{
    RacingLine Generate(TrackData track);
}
