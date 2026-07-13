using System.Collections.Generic;

namespace StintegyEVO.Core.Track.RefLines;

public interface IRefLineSolver
{
    RefLine Generate(IReadOnlyList<RefLineTrackPoint> track);
}
