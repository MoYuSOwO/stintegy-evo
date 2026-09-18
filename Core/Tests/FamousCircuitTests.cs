using System;
using System.Numerics;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The four circuits added for the second round of coverage and testing:
/// Baku to teach narrow-between-walls, Spa to examine gradient past the
/// training range, Monza to examine braking from real speed, Interlagos
/// for anticlockwise rhythm. Each pin here is the one property the
/// circuit was brought in for; lose it and the circuit is scenery.
/// </summary>
public sealed class FamousCircuitTests
{
    [Fact]
    public void BakuIsTheNarrowestRoadInTheSet()
    {
        TrackData track = TrackFactory.BakuStyleTestTrack();
        float narrowest = 999f, steepest = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            narrowest = MathF.Min(narrowest, track.Sample(s).Width);
            steepest = MathF.Max(steepest, track.Sample(s + 0.5f).Grade);
        }
        // The castle squeeze, tighter than Monaco's 10.5.
        Assert.InRange(narrowest, 7.2f, 8.2f);
        // Bound for the training set, so its climb must stay under the
        // training maximum of 7.3 percent — Monaco's 8.6 remains a
        // gradient the policy has never seen.
        Assert.True(
            steepest < 0.073f,
            $"Baku climbs at {steepest * 100f:0.0}% and would spoil " +
            "Monaco as a gradient exam"
        );
    }

    [Fact]
    public void SpaClimbsPastTheTrainingRange()
    {
        TrackData track = TrackFactory.SpaStyleTestTrack();
        float steepest = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
            steepest = MathF.Max(steepest, track.Sample(s + 0.5f).Grade);
        // The compression climb: well past the training maximum of 7.3
        // and past Monaco's 8.6, which is what it is held out for.
        Assert.InRange(steepest, 0.10f, 0.14f);
    }

    [Fact]
    public void AllFourCloseAndKeepTheGeometryTheyWereAddedFor()
    {
        (string Name, Func<TrackData> Build)[] tracks =
        [
            ("baku", TrackFactory.BakuStyleTestTrack),
            ("spa", TrackFactory.SpaStyleTestTrack),
            ("monza", TrackFactory.MonzaStyleTestTrack),
            ("interlagos", TrackFactory.InterlagosStyleTestTrack),
        ];

        foreach ((string name, Func<TrackData> build) in tracks)
        {
            TrackData track = build();
            float height = 0f;
            for (int metre = 0; metre < (int)track.LengthMeters; metre++)
                height += track.Sample(metre + 0.5f).Grade;
            Assert.InRange(height, -0.05f, 0.05f);

            TrackSample startSample = track.Sample(0f);
            TrackSample endSample = track.Sample(track.LengthMeters);
            Assert.True(
                Vector2.Distance(startSample.Center, endSample.Center) < 1e-4f,
                $"{name} does not close"
            );
            Assert.InRange(
                MathF.Abs(track.Project(startSample.Center).D),
                0f,
                1e-4f
            );
        }
    }
}
