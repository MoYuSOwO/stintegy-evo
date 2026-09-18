using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// TrackData.Project finds the nearest point of the centreline anywhere a
/// car can put a point: inside the walls, and a body's reach past them. The
/// spatial hash it searches used to be sized from the road's width alone,
/// and a position in a wide run-off found no node and projected onto the
/// start line: at Zandvoort, a car against the outer wall at
/// (184.8, 415.1) projected to s = 3 with a lateral offset of 9.3 m, 450 m
/// from where it was. The index now covers the walled world, and a position
/// beyond that is refused rather than answered wrongly.
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
            var walls = TrackBoundaryResolver.GetWallLimits(at);
            float body = TrackData.BodyReachMeters;
            foreach (float d in new[]
                     {
                         walls.RightWallD - body, walls.RightWallD, walls.RightWallD * 0.5f, 0f,
                         walls.LeftWallD * 0.5f, walls.LeftWallD, walls.LeftWallD + body
                     })
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

    /// <summary>
    /// The escape itself. Zandvoort's walls stand 11 m from the centreline,
    /// further than the old index reached, so the wall resolver's corner
    /// projections failed at the wall, the barrier leaked, and a car held on
    /// a constant turn went through it and ended 25 m outside. Driven the
    /// same way from starts all round the lap, no car may leave the walls.
    /// </summary>
    [Fact]
    public void AZandvoortCarHeldOnATurnStaysInsideTheWalls()
    {
        TrackData track = Build("zandvoort");
        for (float start = 0f; start < track.LengthMeters; start += 150f)
        {
            TrackSample at = track.Sample(start);
            RaceCar car = TestControlFixtures.ExternalCar(
                "turning",
                new CarState
                {
                    Position = at.Center,
                    Heading = at.Heading,
                    Speed = 20f,
                    Energy = PowertrainState.Filled(0.9f)
                },
                new DriverInput(0.0096f, 12f)
            );
            var simulation = new RaceSimulation(track);
            simulation.AddCar(car);
            for (int i = 0; i < 60 * 60; i++)
            {
                simulation.Step(1f / 60f);
                if (i % 60 != 0)
                    continue;
                // Judged without Project, which is the thing under test: the
                // car's centre against the true nearest centreline point and
                // the walls there.
                (float distance, float s) = BruteNearestWithS(track, car.State.Position);
                TrackSample there = track.Sample(s);
                float reach = there.HalfWidth +
                              MathF.Max(there.LeftBufferWidth, there.RightBufferWidth);
                Assert.True(
                    distance <= reach + 1f,
                    $"start s={start:F0}: {distance:F1} m from the centreline at frame {i}, walls at {reach:F1}"
                );
            }
        }
    }

    [Fact]
    public void APositionOutsideTheWorldIsRefused()
    {
        TrackData track = Build("silverstone");
        TrackSample at = track.Sample(1000f);
        float far = TrackBoundaryResolver.GetWallLimits(at).LeftWallD + 200f;
        Vector2 outside = at.Center + at.Normal * far;
        // Only if that point is not itself near another part of the circuit.
        if (BruteNearest(track, outside) > 60f)
            Assert.Throws<InvalidOperationException>(() => track.Project(outside));
    }

    private static (float Distance, float S) BruteNearestWithS(TrackData track, Vector2 position)
    {
        float best = float.MaxValue, at = 0f;
        for (float s = 0f; s < track.LengthMeters; s += 1f)
        {
            float d = (position - track.Sample(s).Center).Length();
            if (d < best) { best = d; at = s; }
        }
        return (best, at);
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
