using System.Numerics;
using TheStint.Core.Track;
using Xunit;

namespace TheStint.Core.Tests;

public sealed class TrackFactoryTests
{
    [Fact]
    public void GrandPrixStyleBenchmarksHaveDistinctExpectedLengths()
    {
        TrackData silverstone = TrackFactory.SilverstoneStyleTestTrack();
        TrackData monaco = TrackFactory.MonacoStyleTestTrack();
        TrackData shanghai = TrackFactory.ShanghaiStyleTestTrack();
        TrackData suzuka = TrackFactory.SuzukaStyleTestTrack();

        Assert.InRange(silverstone.LengthMeters, 5840f, 5920f);
        Assert.InRange(monaco.LengthMeters, 3290f, 3390f);
        Assert.InRange(shanghai.LengthMeters, 5410f, 5500f);
        Assert.InRange(suzuka.LengthMeters, 5760f, 5850f);

        AssertClosedAndGridded(silverstone);
        AssertClosedAndGridded(monaco);
        AssertClosedAndGridded(shanghai);
        AssertClosedAndGridded(suzuka);
    }

    private static void AssertClosedAndGridded(TrackData track)
    {
        Vector2 start = track.Sample(0f).Center;
        Vector2 end = track.Sample(track.LengthMeters - TrackData.StepLength).Center;
        Assert.InRange(Vector2.Distance(start, end), 0f, 2.5f);
        Assert.Equal(30, track.StartingGridCount);
    }
}
