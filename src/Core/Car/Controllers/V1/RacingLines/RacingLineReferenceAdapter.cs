namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public sealed class RacingLineReferenceAdapter(RacingLine racingLine) : ITrackReferenceLine
{
    public TrackReferencePoint GetPoint(int trackIndex)
    {
        RacingLinePoint point = racingLine[trackIndex];
        return new TrackReferencePoint(
            point.TrackIndex,
            point.Offset,
            point.Position,
            point.Heading,
            point.Curvature
        );
    }
}
