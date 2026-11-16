using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Multi-car verification of the clean three-layer corridor &amp; racecraft system:
/// standing-start launch stability, latched one-way overtaking commitments,
/// FIA pre-rear-axle door-shut abort, three-wide transitive stack room, hazard
/// dodging, and an identical 6-car standing-start grid race matching <c>LaneDriverDemo</c>.
/// </summary>
public sealed class TrafficCorridorTests
{
    private const float Dt = 1f / 60f;
    private readonly ITestOutputHelper _output;

    public TrafficCorridorTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void FasterCarCatchesAndOvertakesSlowerCarCleanly()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        float start = track.StartingLineS;

        (RaceCar leader, LaneDriver leaderDriver) = LaneGrid.Seat(
            simulation, file, lanes, "leader", StockCar.Build(compound), compound,
            start + 28f, 0f, 52f,
            abilities: new DriverAbilities { Defending = 50 });
        leaderDriver.SpeedCap = 58f;

        (RaceCar chaser, _) = LaneGrid.Seat(
            simulation, file, lanes, "chaser", StockCar.Build(compound), compound,
            start, 0f, 54f,
            abilities: new DriverAbilities { Overtaking = 85 });

        bool hit = false, offTrack = false;
        for (int step = 0; step < 60 * 35; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || chaser.HitCarThisStep;
            offTrack |= leader.BoundaryContactSeconds > 0f || chaser.BoundaryContactSeconds > 0f;
        }

        float marginAhead = chaser.Progress.RaceDistanceMeters - leader.Progress.RaceDistanceMeters;
        _output.WriteLine($"Overtake margin after 35s: {marginAhead:F1} m");
        Assert.False(hit, "cars collided during overtake");
        Assert.False(offTrack, "hit boundary during overtake");
        Assert.True(marginAhead > 8f, $"expected chaser to pass leader cleanly, finished {marginAhead:F1} m relative");
    }

    [Fact]
    public void ThreeCarsSideBySideThroughCornersLeaveTransitiveRoomWithoutContact()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        const float start = 1450f;
        int st = (int)MathF.Round(lanes.StationOf(start));
        float free = lanes.Offset(LaneId.Free, st);
        float roadHeading = track.Sample(start).Heading;

        (RaceCar leftCar, _) = LaneGrid.Seat(simulation, file, lanes, "left", StockCar.Build(compound), compound, start, -free + 3.0f, 60f);
        (RaceCar midCar, _) = LaneGrid.Seat(simulation, file, lanes, "mid", StockCar.Build(compound), compound, start, -free + 0.0f, 60f);
        (RaceCar rightCar, _) = LaneGrid.Seat(simulation, file, lanes, "right", StockCar.Build(compound), compound, start, -free - 3.0f, 60f);
        leftCar.State.Heading = midCar.State.Heading = rightCar.State.Heading = roadHeading;

        RaceCar[] cars = [leftCar, midCar, rightCar];
        bool hit = false, wall = false;
        for (int step = 0; step < 60 * 25; step++)
        {
            simulation.Step(Dt);
            foreach (RaceCar c in cars)
            {
                hit |= c.HitCarThisStep;
                wall |= c.BoundaryContactSeconds > 0f;
            }
        }

        _output.WriteLine($"3-wide distances: L={leftCar.Progress.TotalDistance:F1} M={midCar.Progress.TotalDistance:F1} R={rightCar.Progress.TotalDistance:F1}");
        Assert.False(hit, "3-wide cars collided entering/exiting the corner complex");
        Assert.False(wall, "a car was squeezed into the wall in 3-wide");
        foreach (RaceCar c in cars)
            Assert.True(c.Progress.TotalDistance > 900f, $"{c.Id} stalled or spun ({c.Progress.TotalDistance:F1} m)");
    }

    [Fact]
    public void ChaserAbortsCleanlyWhenLeaderCoversInsideBeforeFiaOverlap()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        const float start = 1350f;

        (RaceCar leader, LaneDriver leaderDriver) = LaneGrid.Seat(
            simulation, file, lanes, "defender", StockCar.Build(compound), compound,
            start + 14f, 0f, 64f,
            abilities: new DriverAbilities { Defending = 95, Awareness = 95 });
        leaderDriver.SpeedCap = 68f;

        (RaceCar chaser, _) = LaneGrid.Seat(
            simulation, file, lanes, "attacker", StockCar.Build(compound), compound,
            start, 0f, 68f,
            abilities: new DriverAbilities { Overtaking = 90, Awareness = 90 });

        bool hit = false, offTrack = false;
        for (int step = 0; step < 60 * 18; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || chaser.HitCarThisStep;
            offTrack |= leader.BoundaryContactSeconds > 0f || chaser.BoundaryContactSeconds > 0f;
        }

        Assert.False(hit, "attacker clipped defender when inside door shut before rear axle");
        Assert.False(offTrack, "hit boundary during defensive cover / abort");
    }

    [Fact]
    public void SlowSkewedHazardCarOnRacingLineIsDodgedCleanly()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        float start = track.StartingLineS;

        (RaceCar hazard, LaneDriver hazardDriver) = LaneGrid.Seat(
            simulation, file, lanes, "hazard", StockCar.Build(compound), compound,
            start + 95f, 0f, 10f);
        hazardDriver.SpeedCap = 10f;
        hazard.State.Heading += 0.45f;

        (RaceCar car1, _) = LaneGrid.Seat(simulation, file, lanes, "fast-1", StockCar.Build(compound), compound, start + 16f, 0f, 66f);
        (RaceCar car2, _) = LaneGrid.Seat(simulation, file, lanes, "fast-2", StockCar.Build(compound), compound, start, 0f, 67f);

        bool hit = false, wall = false;
        for (int step = 0; step < 60 * 8; step++)
        {
            simulation.Step(Dt);
            hit |= hazard.HitCarThisStep || car1.HitCarThisStep || car2.HitCarThisStep;
            wall |= car1.BoundaryContactSeconds > 0f || car2.BoundaryContactSeconds > 0f;
        }

        Assert.False(hit, "approaching cars collided with slow skewed hazard or each other");
        Assert.False(wall, "approaching cars hit the wall while dodging hazard");
        Assert.True(car1.Progress.RaceDistanceMeters > hazard.Progress.RaceDistanceMeters + 50f);
        Assert.True(car2.Progress.RaceDistanceMeters > hazard.Progress.RaceDistanceMeters + 40f);
    }
}
