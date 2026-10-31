using System.Numerics;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// A line drawn from outside the driver, redrawn every quarter second the
/// way a racecraft layer above it will: the car goes where it is drawn, and
/// even a drawing that makes no sense is driven round the lap on the road.
/// </summary>
public sealed class DrawnLineTests
{
    private const float RedrawSeconds = 0.25f;

    [Theory]
    [InlineData(2.5f)]
    [InlineData(-2.5f)]
    public void ALineDrawnToOneSideIsDrivenOnThatSide(float side)
    {
        (LaneLapResult lap, float meanOffset) = Lap((_, _) => side);
        Assert.True(lap.Finished, lap.ToString());
        Assert.Equal(0f, lap.OffTrackSeconds);
        Assert.True(MathF.Sign(meanOffset) == MathF.Sign(side) && MathF.Abs(meanOffset) > 1f,
            $"drawn {side:F1} m aside, driven {meanOffset:F2} m aside on average");
    }

    [Fact]
    public void ALineRedrawnAtRandomIsStillDrivenRoundOnTheRoad()
    {
        // What an untrained network draws: every point anywhere within two
        // metres of the racing line, new each quarter second.
        var random = new Random(7);
        (LaneLapResult lap, _) = Lap((_, _) => (float)(random.NextDouble() * 4.0 - 2.0));
        Assert.True(lap.Finished, lap.ToString());
        Assert.Equal(0f, lap.OffTrackSeconds);
        Assert.Equal(0f, lap.WallSeconds);
    }

    [Fact]
    public void ALineRedrawnAsItWasLeavesNoSeamWhereTheDrawingsJoin()
    {
        // Out towards the edge, where the line gives way to the road, drawn
        // once, and drawn again as it was a quarter second on at 80 m/s.
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneSet lanes = LaneTrackFile.ForCircuit("silverstone", track).LanesFor(track);
        var planned = new float[lanes.Count];
        Array.Fill(planned, 80f);
        var points = new float[DrawnLine.PointMeters.Length];
        Array.Fill(points, 6f);
        const int car = 500, on = 10;
        var once = new DrawnLine(lanes);
        once.Draw(car, 80f, 0f, planned, points, 1f);
        var twice = new DrawnLine(lanes);
        twice.Draw(car, 80f, 0f, planned, points, 1f);
        twice.Draw(car + on, 80f, twice.At(car + on), planned, points, 1f);

        float worst = 0f;
        int worstAt = 0;
        for (int i = car + on; i < car + 150; i++)
        {
            float gap = MathF.Abs(twice.At(i) - once.At(i));
            if (gap > worst)
                (worst, worstAt) = (gap, i);
        }
        Assert.True(worst < 0.02f, $"the redrawn line stands {worst:F3} m off the line drawn once, {(worstAt - car) * lanes.Spacing:F0} m on");
    }

    [Theory]
    [InlineData(1f)]
    [InlineData(-1f)]
    public void ASideLaneMadeInAdvanceIsDrivenAsFastAsTheLaneItself(float side)
    {
        // A lane is a line made in advance; handed over whole as one, it is
        // driven as it is, with no redrawing to come in late where it closes
        // in on the racing line along an edge.
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        LaneId lane = side > 0f ? LaneId.Left : LaneId.Right;
        LaneLapResult held = LaneLapRunner.Run(track, file, onStep: (_, driver, _) =>
        {
            driver.TargetRange = 1f;
            driver.TargetSide = side;
        });
        var made = new float[lanes.Count];
        for (int i = 0; i < made.Length; i++)
            made[i] = lanes.Offset(lane, i) - lanes.Offset(LaneId.Free, i);
        bool handed = false;
        LaneLapResult driven = LaneLapRunner.Run(track, file, onStep: (_, driver, _) =>
        {
            if (!handed)
                driver.DriveLine(0, made);
            handed = true;
        });
        Assert.True(held.Finished && driven.Finished, $"held: {held}\nmade: {driven}");
        Assert.True(driven.FlyingLapSeconds < held.FlyingLapSeconds * 1.01f,
            $"the lane held laps in {held.FlyingLapSeconds:F2} s, made in advance in {driven.FlyingLapSeconds:F2} s");
    }

    // One lap of Silverstone with the line redrawn every quarter second from
    // what each point ahead should be, and how far aside of the racing line
    // the car ran on average.
    private static (LaneLapResult Lap, float MeanOffset) Lap(Func<float, int, float> point)
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var points = new float[DrawnLine.PointMeters.Length];
        float next = 0f, sum = 0f;
        int samples = 0;
        LaneLapResult lap = LaneLapRunner.Run(track, file, onStep: (car, driver, t) =>
        {
            float s = car.Progress.CurrentS;
            int station = (int)MathF.Round(lanes.StationOf(s));
            if (driver.DrivesDrawnLine)
            {
                sum += Vector2.Dot(car.State.Position - lanes.Centre(station), lanes.Normal(station)) - lanes.Offset(LaneId.Free, station);
                samples++;
            }
            if (t < next)
                return;
            next = t + RedrawSeconds;
            for (int k = 0; k < points.Length; k++)
                points[k] = point(s + DrawnLine.PointMeters[k], k);
            driver.DrawLine(points);
        });
        return (lap, sum / Math.Max(1, samples));
    }
}
