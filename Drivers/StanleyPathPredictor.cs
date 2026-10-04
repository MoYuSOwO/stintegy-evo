using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers;

/// <summary>
/// Lookahead along the baked racing line from the car's station. Curvature
/// is the line's own shape. A join extra is added only while still off line.
/// On-line following uses the baked v*(s) instead; this predictor is the
/// join/line-change path for later racecraft.
/// </summary>
internal sealed class StanleyPathPredictor
{
    public const float HorizonMeters = 360f;
    public const float StepMeters = 8f;
    public const float JoinMeters = 48f;

    public readonly record struct Point(float PathCurvature, float TrackS);

    private readonly Point[] _points =
        new Point[(int)MathF.Ceiling(HorizonMeters / StepMeters) + 1];

    public ReadOnlySpan<Point> Predict(
        RaceCarSnapshot car,
        GeometricRacingLine line,
        DriverTrackView track
    )
    {
        int count = _points.Length;
        float s = car.TrackS;
        GeometricRacingLine.Sample here = line.At(track.Sample, track.WrapS, s);
        float nError = car.TrackD - here.OffsetMeters;
        float headingError = MathHelper.NormalizeAngle(here.Heading - car.VelocityHeadingRadians);
        bool offLine = MathF.Abs(nError) > 0.5f || MathF.Abs(headingError) > 0.08f;
        float joinKappa = offLine
            ? nError * 2f / (JoinMeters * JoinMeters) + headingError / JoinMeters
            : 0f;

        for (int i = 0; i < count; i++)
        {
            float pathKappa = PeakLineCurvature(line, track, s);
            if (offLine && i * StepMeters < JoinMeters)
                pathKappa = GreaterMagnitude(pathKappa, joinKappa);
            _points[i] = new Point(pathKappa, s);
            s = track.WrapS(s + StepMeters);
        }

        return _points.AsSpan(0, count);
    }

    private static float PeakLineCurvature(
        GeometricRacingLine line,
        DriverTrackView track,
        float s
    )
    {
        float best = line.At(track.Sample, track.WrapS, s).PathCurvature;
        float before = line.At(track.Sample, track.WrapS, s - 6f).PathCurvature;
        float after = line.At(track.Sample, track.WrapS, s + 6f).PathCurvature;
        if (MathF.Abs(before) > MathF.Abs(best))
            best = before;
        if (MathF.Abs(after) > MathF.Abs(best))
            best = after;
        return best;
    }

    private static float GreaterMagnitude(float a, float b) =>
        MathF.Abs(a) >= MathF.Abs(b) ? a : b;
}
