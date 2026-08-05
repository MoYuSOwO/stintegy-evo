using System;
using StintegyEVO.Core.Track;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// What running beside the racing line actually costs, as geometry alone.
///
/// Every overtake is bought with this: the car leaves the quickest line, pays
/// for the detour, and hopes to be past before the bill exceeds what being
/// stuck was costing. So the exchange rate decides whether passing is possible
/// at all, and it is worth knowing before any planner is asked to trade in it.
///
/// A line held a fixed distance from the racing line has a curvature in closed
/// form. For offset d in the track normal, curvature k becomes k / (1 - k*d)
/// and each metre of reference line becomes (1 - k*d) metres of offset line.
/// The two effects oppose each other around a lap: the inside of one corner is
/// the outside of the next, so what a corner takes a corner gives back.
/// </summary>
public class OffsetLineCostTests(ITestOutputHelper output)
{
    private const float LateralAccelerationLimit = 16f;
    private const float TopSpeedMetersPerSecond = 80f;
    private const float SampleStepMeters = 1f;

    [Theory]
    [InlineData("Silverstone")]
    [InlineData("Shanghai")]
    [InlineData("Monaco")]
    public void HowMuchDoesRunningBesideTheRacingLineCost(string circuit)
    {
        TrackData track = Circuit(circuit);
        double reference = LapSeconds(track, 0f);

        output.WriteLine($"{circuit}: {track.LengthMeters:F0} m lap");
        output.WriteLine($"  racing line             {reference,7:F3} s");
        output.WriteLine("  offset      lap time     cost");
        foreach (float offset in new[] { 1f, 2f, 3f, 4f, -1f, -2f, -3f, -4f })
        {
            double held = LapSeconds(track, offset);
            output.WriteLine(
                $"  {offset,+5:F1} m   {held,9:F3} s   " +
                $"{(held - reference) / reference,7:P2}");
        }

        double worst = 0d;
        foreach (float offset in new[] { 3f, -3f })
        {
            double held = LapSeconds(track, offset);
            worst = Math.Max(worst, (held - reference) / reference);
        }

        // The bar an overtake has to clear. A pass needs a few seconds beside
        // the line, so if holding that line all lap costs a few per cent, a
        // pass is affordable to any car with a real pace advantage. If it
        // costs far more than that, no planner can make passing pay and the
        // fault is in the geometry, not in the decision.
        Assert.True(
            worst < 0.05d,
            $"holding 3 m off the racing line costs {worst:P2} of lap time, " +
            "which is more than any plausible pace advantage can buy back"
        );
    }

    /// <summary>
    /// Time for one lap of a line held at a constant offset, limited only by
    /// how hard the car can corner. No braking or acceleration model: this is
    /// the geometry's own contribution, which is what is being asked about.
    /// </summary>
    private static double LapSeconds(TrackData track, float offsetMeters)
    {
        double seconds = 0d;
        for (float s = 0f; s < track.LengthMeters; s += SampleStepMeters)
        {
            TrackSample sample = track.Sample(s);
            float curvature = sample.RefCurvature;
            float scale = 1f - curvature * offsetMeters;
            if (scale <= 0.05f)
                scale = 0.05f;

            float offsetCurvature = curvature / scale;
            float limit = MathF.Abs(offsetCurvature) > 1e-6f
                ? MathF.Sqrt(LateralAccelerationLimit / MathF.Abs(offsetCurvature))
                : TopSpeedMetersPerSecond;
            seconds += SampleStepMeters * scale /
                       MathF.Min(limit, TopSpeedMetersPerSecond);
        }
        return seconds;
    }

    private static TrackData Circuit(string name) => name switch
    {
        "Shanghai" => TrackFactory.ShanghaiStyleTestTrack(),
        "Monaco" => TrackFactory.MonacoStyleTestTrack(),
        _ => TrackFactory.SilverstoneStyleTestTrack()
    };
}
