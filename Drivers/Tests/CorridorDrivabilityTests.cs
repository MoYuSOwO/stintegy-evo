using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class CorridorDrivabilityTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void Gate_MatchesTheHalfMeterMonacoBar()
    {
        Assert.True(TrafficCorridor.PassesDrivableGate(70f, 70f, 69f));
        Assert.True(TrafficCorridor.PassesDrivableGate(68.5f, 70f, 69f));
        Assert.False(TrafficCorridor.PassesDrivableGate(68.4f, 70f, 69f));
        Assert.False(TrafficCorridor.PassesDrivableGate(40f, 75f, 74f));
        Assert.True(TrafficCorridor.PassesDrivableGate(50f, 80f, 49.6f));
    }

    [Fact]
    public void OpeningLeaderOnAStraight_DoesNotDragTheFollower()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        float start = track.StartingLineS;

        (RaceCar leader, LaneDriver leaderDriver) = LaneGrid.Seat(
            simulation, file, lanes, "leader", StockCar.Build(compound), compound,
            start + 6f, 0f, 50f,
            abilities: new DriverAbilities { Pace = 90, Consistency = 95, Focus = 95, Precision = 95 });
        leaderDriver.SpeedCap = 50f;

        (RaceCar follower, LaneDriver followerDriver) = LaneGrid.Seat(
            simulation, file, lanes, "follower", StockCar.Build(compound), compound,
            start, 0f, 48f,
            abilities: new DriverAbilities { Pace = 90, Consistency = 95, Focus = 95, Precision = 95 });
        followerDriver.SpeedCap = 48f;

        float slowest = float.PositiveInfinity;
        bool hit = false;
        for (int step = 0; step < 60 * 3; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || follower.HitCarThisStep;
            if (step > 30)
                slowest = MathF.Min(slowest, follower.State.Speed);
        }

        Assert.False(hit, "opening pair made contact");
        Assert.True(slowest > 44f, $"follower was braked to {slowest:F1} m/s while the leader was pulling away");
    }
}
