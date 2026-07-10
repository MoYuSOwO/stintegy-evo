using System.Collections.Generic;

namespace TheStint.Core.Track.RefLines;

public interface IRefLineSolver
{
    RefLine Generate(IReadOnlyList<RefLineTrackPoint> track);
}
