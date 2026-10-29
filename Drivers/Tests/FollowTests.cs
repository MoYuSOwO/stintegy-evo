using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// A car catching a slower one on its own line, with nobody drawing it a
/// way round: it comes up behind and stays there, and under orders to
/// follow it keeps the gap it was told.
/// </summary>
public sealed class FollowTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void RacingACarAheadOnItsLineItCatchesItAndDoesNotRunIntoIt()
    {
        (bool hit, float closest, float _) = Catch(followGapSeconds: 0f);
        Assert.False(hit, "ran into the car ahead");
        Assert.True(closest < 15f, $"never came closer than {closest:F1} m: it did not catch the car ahead");
    }

    [Fact]
    public void OrderedToFollowItKeepsTheGapItWasTold()
    {
        const float gap = 0.8f;
        (bool hit, _, float closestSeconds) = Catch(followGapSeconds: gap);
        Assert.False(hit, "ran into the car ahead");
        Assert.True(closestSeconds > 0.5f * gap, $"came within {closestSeconds:F2} s of the car ahead, told to keep {gap:F1} s");
    }

    // Silverstone, a car held to 45 m/s and another 80 m behind it free to
    // go as fast as the road lets it: whether they touched, and how close
    // the one behind came, in metres of gap and, once it had caught up, in
    // seconds.
    private static (bool Hit, float ClosestMeters, float ClosestSeconds) Catch(float followGapSeconds)
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        float start = track.StartingLineS;
        string compound = DevelopmentTyres.SuperId;
        (RaceCar slow, LaneDriver slowDriver) = LaneGrid.Seat(simulation, file, lanes, "slow", StockCar.Build(compound), compound,
            start + 80f, 0f, 45f);
        slowDriver.SpeedCap = 45f;
        (RaceCar fast, LaneDriver fastDriver) = LaneGrid.Seat(simulation, file, lanes, "fast", StockCar.Build(compound), compound,
            start, 0f, 45f);
        fastDriver.FollowGapSeconds = followGapSeconds;

        bool hit = false;
        float closest = float.PositiveInfinity, closestSeconds = float.PositiveInfinity;
        float length = track.LengthMeters;
        for (int i = 0; i < 60 * 60; i++)
        {
            simulation.Step(Dt);
            hit |= fast.HitCarThisStep;
            // Bumper to bumper along the lap.
            float gap = ((slow.Progress.CurrentS - fast.Progress.CurrentS) % length + length) % length - 0.5f * (slow.Collision.LengthMeters + fast.Collision.LengthMeters);
            closest = MathF.Min(closest, gap);
            if (i > 60 * 30)
                closestSeconds = MathF.Min(closestSeconds, gap / MathF.Max(fast.State.Speed, 1f));
        }
        return (hit, closest, closestSeconds);
    }
}
