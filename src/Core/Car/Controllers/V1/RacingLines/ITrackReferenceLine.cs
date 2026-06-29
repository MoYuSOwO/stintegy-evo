namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public interface ITrackReferenceLine
{
    TrackReferencePoint GetPoint(int trackIndex);
}
