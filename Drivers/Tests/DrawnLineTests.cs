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
