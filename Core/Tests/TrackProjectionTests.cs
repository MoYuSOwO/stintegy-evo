using System;
using System.Numerics;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// TrackData.Project finds the nearest point of the centreline, wherever
/// the position is. The spatial hash it searches is sized from the track's
/// width, not its run-off, and a position in a wide buffer used to find no
/// node at all and project onto the start line: at Zandvoort, a car against
/// the outer wall at (184.8, 415.1) projected to s = 3 with a lateral offset
/// of 9.3 m, 450 m from where it was.
/// </summary>
public sealed class TrackProjectionTests
{
    public static TheoryData<string> Tracks => new()
    {
        "zandvoort", "silverstone", "monaco", "daytona", "spa", "singapore"
    };

    [Theory]
    [MemberData(nameof(Tracks))]
    public void AProjectionIsTheNearestCentrelinePointAcrossTheWholeRunOff(string name)
    {
        TrackData track = Build(name);
        for (float s = 0f; s < track.LengthMeters; s += 7f)
        {
            TrackSample at = track.Sample(s);
            float reach = at.HalfWidth + MathF.Max(at.LeftBufferWidth, at.RightBufferWidth) + 5f;
            foreach (float d in new[] { -reach, -reach * 0.6f, reach * 0.6f, reach })
            {
                Vector2 position = at.Center + at.Normal * d;
                TrackPose pose = track.Project(position);
                float found = (position - pose.Sample.Center).Length();
                float nearest = BruteNearest(track, position);
                Assert.True(
                    found <= nearest + 1.5f,
                    $"{name} s={s:F0} d={d:F1}: projected {found:F1} m away, nearest is {nearest:F1}"
                );
            }
        }
    }

    [Fact]
    public void TheZandvoortRunOffCase()
    {
        TrackData track = Build("zandvoort");
        Vector2 position = new(184.75676f, 415.12936f);
        TrackPose pose = track.Project(position);
        Assert.InRange((position - pose.Sample.Center).Length(), 0f, BruteNearest(track, position) + 1.5f);
    }

    private static float BruteNearest(TrackData track, Vector2 position)
    {
        float best = float.MaxValue;
        for (float s = 0f; s < track.LengthMeters; s += 0.5f)
            best = MathF.Min(best, (position - track.Sample(s).Center).Length());
        return best;
    }

    private static TrackData Build(string name) => name switch
    {
        "zandvoort" => TrackFactory.ZandvoortStyleTestTrack(),
        "silverstone" => TrackFactory.SilverstoneStyleTestTrack(),
        "monaco" => TrackFactory.MonacoStyleTestTrack(),
        "daytona" => TrackFactory.DaytonaStyleTestTrack(),
        "spa" => TrackFactory.SpaStyleTestTrack(),
        "singapore" => TrackFactory.SingaporeStyleTestTrack(),
        _ => throw new ArgumentOutOfRangeException(nameof(name))
    };
}
