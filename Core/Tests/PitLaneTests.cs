using System;
using System.IO;
using System.Linq;
using System.Numerics;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class PitLaneTests
{
    public static TheoryData<string> CircuitsWithPits => new() { "silverstone", "spa" };

    private static (TrackData Track, PitLane Lane) Load(string id)
    {
        TrackData track = CircuitLibrary.Load(id);
        Assert.NotNull(track.PitLane);
        return (track, track.PitLane!);
    }

    /// <summary>The lane starts and ends on the road, running the way the lap does.</summary>
    [Theory]
    [MemberData(nameof(CircuitsWithPits))]
    public void BothEndsAreOnTheRoad(string id)
    {
        (TrackData track, PitLane lane) = Load(id);
        foreach ((float u, float s) in new[] { (0f, lane.EntryS), (lane.LengthMeters, lane.ExitS) })
        {
            PitLaneSample end = lane.Sample(u);
            TrackSample road = track.Sample(s);
            Assert.True(Vector2.Distance(end.Center, road.Center) <= road.HalfWidth, $"{id}: lane end at {u} m is off the road");
            Assert.True(Vector2.Dot(end.Tangent, road.Tangent) > 0.95f, $"{id}: lane end at {u} m does not run with the road");
        }
    }

    /// <summary>Between its ends the lane is behind the barrier, and stays clear of every other part of the lap.</summary>
    [Theory]
    [MemberData(nameof(CircuitsWithPits))]
    public void TheMiddleIsOffTheTrackAndClearOfIt(string id)
    {
        (TrackData track, PitLane lane) = Load(id);
        PitLaneSample middle = lane.Sample(lane.LengthMeters * 0.5f);
        TrackPose pose = track.Project(middle.Center);
        float barrier = pose.Sample.HalfWidth + (pose.D > 0 ? pose.Sample.LeftBufferWidth : pose.Sample.RightBufferWidth);
        Assert.True(MathF.Abs(pose.D) - middle.HalfWidth > barrier, $"{id}: the lane's middle overlaps the run-off");

        // No other stretch of road comes near it: everything more than
        // two hundred metres of lap from either end stays a road and its
        // run-off away.
        float span = track.WrapS(lane.ExitS - lane.EntryS);
        for (float u = 0f; u <= lane.LengthMeters; u += 5f)
        {
            PitLaneSample at = lane.Sample(u);
            for (float s = 0f; s < track.LengthMeters; s += 2f)
            {
                float fromEntry = track.WrapS(s - lane.EntryS);
                if (fromEntry < span + 200f || fromEntry > track.LengthMeters - 200f)
                    continue;
                TrackSample road = track.Sample(s);
                float reach = road.HalfWidth + MathF.Max(road.LeftBufferWidth, road.RightBufferWidth) + at.HalfWidth;
                Assert.True(Vector2.Distance(at.Center, road.Center) > reach, $"{id}: lane at {u:F0} m runs into the track at {s:F0} m");
            }
        }
    }

    /// <summary>
    /// The three questions the lane answers: is a position in it, how far
    /// along, and what is the limit there.
    /// </summary>
    [Theory]
    [MemberData(nameof(CircuitsWithPits))]
    public void ItSaysWhereACarIsAndWhatTheLimitIs(string id)
    {
        (TrackData track, PitLane lane) = Load(id);
        float previous = -1f;
        for (float u = 1f; u < lane.LengthMeters; u += 10f)
        {
            PitLaneSample at = lane.Sample(u);
            Vector2 nearEdge = at.Center + at.Normal * at.HalfWidth * 0.8f;
            Assert.True(lane.Contains(nearEdge), $"{id}: a point on the lane at {u:F0} m is not in it");
            PitLanePose pose = lane.Project(nearEdge);
            Assert.True(pose.U > previous, $"{id}: the way along the lane goes backwards at {u:F0} m");
            previous = pose.U;
        }

        PitLaneSample middle = lane.Sample(lane.LengthMeters * 0.5f);
        Assert.False(lane.Contains(middle.Center + middle.Normal * middle.Width * 2f));
        Assert.False(lane.Contains(track.Sample(lane.ExitS + 500f).Center));
        // Past either end, beside the road, is not in the lane.
        PitLaneSample first = lane.Sample(0f);
        Assert.False(lane.Contains(first.Center - first.Tangent * 5f));

        float? limit = lane.SpeedLimitAt(lane.LengthMeters * 0.5f);
        Assert.NotNull(limit);
        Assert.InRange(limit!.Value, 40f / 3.6f, 80.01f / 3.6f);
        Assert.Null(lane.SpeedLimitAt(0f));
        Assert.Null(lane.SpeedLimitAt(lane.LengthMeters));
    }

    /// <summary>The boxes are on the lane, between the limit lines, in order from the entry.</summary>
    [Theory]
    [MemberData(nameof(CircuitsWithPits))]
    public void TheBoxesAreOnTheLaneInsideTheLimit(string id)
    {
        (_, PitLane lane) = Load(id);
        Assert.NotEmpty(lane.Boxes);
        float previous = 0f;
        foreach (PitBox box in lane.Boxes)
        {
            Assert.True(lane.Contains(box.Position), $"{id}: box {box.Number} is off the lane");
            Assert.NotNull(lane.SpeedLimitAt(box.U));
            Assert.True(box.U > previous, $"{id}: box {box.Number} is out of order");
            previous = box.U;
        }
    }

    /// <summary>A circuit with no pit lane written down loads as it always did.</summary>
    [Fact]
    public void ACircuitWithoutOneLoadsWithout()
    {
        Assert.Null(CircuitLibrary.Manifest("monza").PitLane);
        Assert.Null(CircuitLibrary.Load("monza").PitLane);
        Assert.Null(CircuitLibrary.Load("simple").PitLane);
    }

    /// <summary>A lane that does not begin on the road is refused when the circuit loads.</summary>
    [Fact]
    public void ALaneThatStartsOffTheRoadIsRefused()
    {
        TrackData track = CircuitLibrary.Load("silverstone");
        PitLane lane = track.PitLane!;
        var rows = Enumerable.Range(0, (int)lane.LengthMeters + 1)
            .Select(i => lane.Sample(i))
            .Select(at => new PitLaneFile.Row(at.Center + new Vector2(0f, 60f), at.Z, at.Width))
            .ToArray();
        var moved = new CircuitPitLane
        {
            LengthMeters = rows.Length - 1,
            EntryMeters = (int)lane.EntryS,
            ExitMeters = (int)lane.ExitS,
            SpeedLimitKph = 80f,
            SpeedLimitFromMeters = lane.SpeedLimitFromU,
            SpeedLimitToMeters = lane.SpeedLimitToU,
        };
        Assert.Throws<InvalidDataException>(() => PitLaneFile.Parse(moved, PitLaneFile.Write(rows), track, "moved"));
        // The same rows where they belong are accepted.
        var home = rows.Select(r => r with { Center = r.Center - new Vector2(0f, 60f) }).ToArray();
        Assert.NotNull(PitLaneFile.Parse(moved, PitLaneFile.Write(home), track, "home"));
    }
}
