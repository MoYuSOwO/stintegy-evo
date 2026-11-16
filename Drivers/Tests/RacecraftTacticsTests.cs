using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class RacecraftTacticsTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void BlueFlag_RequiresThreeQuartersOfALapAndAClosingWindow()
    {
        float length = 5890f;
        float catchSeconds = RacecraftTactics.CatchSeconds(50f);
        Assert.Equal(1.20f, catchSeconds, 3);
        Assert.Equal(1.00f, RacecraftTactics.CatchSeconds(0f), 3);
        Assert.Equal(1.40f, RacecraftTactics.CatchSeconds(100f), 3);
        Assert.True(RacecraftTactics.BlueFlagDue(length, 0f, length, -20f, 20f, 1.2f));
        Assert.False(RacecraftTactics.BlueFlagDue(0.5f * length, 0f, length, -20f, 20f, 1.2f));
        Assert.False(RacecraftTactics.BlueFlagDue(length, 0f, length, -40f, 20f, 1.2f));
        Assert.True(RacecraftTactics.YieldClearanceMeters(1.9f, 1.9f) >= 3f + 1.9f);
    }

    [Fact]
    public void StandingStart_HoldsColumnAndPullsOnlyASlowStarter()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        float start = track.StartingLineS;
        int station = (int)MathF.Round(lanes.StationOf(start));
        float column = 2.2f;

        (RaceCar slow, LaneDriver slowDriver) = LaneGrid.Seat(
            simulation, file, lanes, "slow-start", StockCar.Build(compound), compound,
            start + 8f, column, 0f,
            abilities: new DriverAbilities { Pace = 60, Defending = 40, Boldness = 40 });
        slowDriver.SpeedCap = 18f;

        (RaceCar fast, _) = LaneGrid.Seat(
            simulation, file, lanes, "fast-start", StockCar.Build(compound), compound,
            start, column, 0f,
            abilities: new DriverAbilities { Pace = 95, Overtaking = 90, Boldness = 70, Focus = 95, Precision = 95 });

        float maxSway = 0f;
        float maxSep = 0f;
        bool hit = false;
        for (int step = 0; step < 60 * 8; step++)
        {
            simulation.Step(Dt);
            hit |= slow.HitCarThisStep || fast.HitCarThisStep;
            if (fast.Progress.TotalDistance < 70f)
                maxSway = MathF.Max(maxSway, MathF.Abs(fast.Progress.CurrentD - (lanes.Offset(LaneId.Free, station) + column)));
            maxSep = MathF.Max(maxSep, MathF.Abs(fast.Progress.CurrentD - slow.Progress.CurrentD));
        }

        Assert.False(hit);
        Assert.True(maxSway < 0.35f, $"launch sway {maxSway:F2} m");
        Assert.True(maxSep > 1.5f, $"no pull-out, separation {maxSep:F2} m");
    }

    [Fact]
    public void DraftTow_HoldsThrottleUntilPulloutGap()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        float start = track.StartingLineS;

        (RaceCar leader, LaneDriver leaderDriver) = LaneGrid.Seat(
            simulation, file, lanes, "tow-leader", StockCar.Build(compound), compound,
            start + 40f, 0f, 62f,
            abilities: new DriverAbilities { Pace = 80, Defending = 70, Boldness = 50 });
        leaderDriver.SpeedCap = 64f;

        (RaceCar chaser, _) = LaneGrid.Seat(
            simulation, file, lanes, "tow-chaser", StockCar.Build(compound), compound,
            start, 0f, 66f,
            abilities: new DriverAbilities { Pace = 95, Overtaking = 90, Boldness = 50, Focus = 95, Precision = 95 });

        float slowest = float.PositiveInfinity;
        bool hit = false;
        bool off = false;
        for (int step = 0; step < 60 * 2; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || chaser.HitCarThisStep;
            off |= leader.BoundaryContactSeconds > 0f || chaser.BoundaryContactSeconds > 0f;
            if (step > 15)
                slowest = MathF.Min(slowest, chaser.State.Speed);
        }

        Assert.False(hit);
        Assert.False(off);
        Assert.True(slowest > 63f, $"chaser lifted to {slowest:F1} while still well outside d_pullout");
    }

    [Fact]
    public void OneMoveDefend_MovesInsideOnceAndHoldsToEntry()
    {
        (RaceSimulation simulation, LaneSet lanes, RaceCar leader, RaceCar chaser) =
            PairAt(1350f, leaderAhead: 0f, chaserBehind: 10f, leaderSpeed: 62f, chaserSpeed: 68f,
                leader: new DriverAbilities { Pace = 85, Defending = 95, Awareness = 90, Boldness = 40 },
                chaser: new DriverAbilities { Pace = 80, Overtaking = 40, Boldness = 30 });
        int anchor = (int)MathF.Round(lanes.StationOf(1350f));
        (int insideSign, int entry) = NextInside(lanes, anchor);
        bool hit = false;
        bool sawApproach = false;
        float bestInside = float.NegativeInfinity;
        float rebound = 0f;
        for (int step = 0; step < 60 * 20; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || chaser.HitCarThisStep;
            int st = (int)MathF.Round(lanes.StationOf(leader.Progress.CurrentS));
            int toEntry = lanes.Wrap(entry - st);
            if (toEntry == 0 || toEntry > lanes.Count / 2)
                break;
            sawApproach = true;
            float insidePos = insideSign * leader.Progress.CurrentD;
            if (insidePos > bestInside)
                bestInside = insidePos;
            else
                rebound = MathF.Max(rebound, bestInside - insidePos);
        }
        Assert.False(hit);
        Assert.True(sawApproach, "leader never ran the straight before the entry pin");
        Assert.True(bestInside > 1.5f, $"defender never reached the inside ({bestInside:F2} m)");
        Assert.True(rebound <= 0.5f, $"defender moved back toward the outside by {rebound:F2} m before entry");
    }

    [Fact]
    public void SideBySide_ThreeWideHoldsSlotsWithoutContact()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        const float start = 1450f;
        int st = (int)MathF.Round(lanes.StationOf(start));
        float free = lanes.Offset(LaneId.Free, st);
        float heading = track.Sample(start).Heading;
        (RaceCar left, _) = LaneGrid.Seat(simulation, file, lanes, "slot-left", StockCar.Build(compound), compound, start, -free + 3.0f, 60f);
        (RaceCar mid, _) = LaneGrid.Seat(simulation, file, lanes, "slot-mid", StockCar.Build(compound), compound, start, -free, 60f);
        (RaceCar right, _) = LaneGrid.Seat(simulation, file, lanes, "slot-right", StockCar.Build(compound), compound, start, -free - 3.0f, 60f);
        left.State.Heading = mid.State.Heading = right.State.Heading = heading;
        bool hit = false;
        bool wall = false;
        for (int step = 0; step < 60 * 12; step++)
        {
            simulation.Step(Dt);
            foreach (RaceCar car in new[] { left, mid, right })
            {
                hit |= car.HitCarThisStep;
                wall |= car.BoundaryContactSeconds > 0f;
            }
        }
        Assert.False(hit);
        Assert.False(wall);
    }

    [Fact]
    public void Switchback_ClearsDeepInsideDefenderWithoutSpin()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        const float start = 1680f;
        int st = (int)MathF.Round(lanes.StationOf(start));
        (int insideSign, _) = NextInside(lanes, st);
        float insideOff = insideSign * 4.8f - lanes.Offset(LaneId.Free, st);

        (RaceCar leader, LaneDriver leaderDriver) = LaneGrid.Seat(
            simulation, file, lanes, "pair-leader", StockCar.Build(compound), compound,
            start + 8f, insideOff, 40f,
            abilities: new DriverAbilities { Pace = 65, Defending = 95, Boldness = 80, Focus = 90 });
        leaderDriver.SpeedCap = 40f;

        (RaceCar chaser, LaneDriver chaserDriver) = LaneGrid.Seat(
            simulation, file, lanes, "pair-chaser", StockCar.Build(compound), compound,
            start, 0f, 48f,
            abilities: new DriverAbilities { Pace = 95, Overtaking = 95, Boldness = 75, Focus = 98, Precision = 98 });

        bool hit = false;
        bool wall = false;
        bool everSwitchback = false;
        bool wentAhead = false;
        float chaserSpeedWhenAhead = 0f;
        float leaderSpeedWhenAhead = 0f;
        for (int step = 0; step < 60 * 18; step++)
        {
            simulation.Step(Dt);
            hit |= leader.HitCarThisStep || chaser.HitCarThisStep;
            wall |= leader.BoundaryContactSeconds > 0f || chaser.BoundaryContactSeconds > 0f;
            everSwitchback |= chaserDriver.TacticPhase == RacecraftPhase.Switchback;
            if (!wentAhead && chaser.Progress.RaceDistanceMeters > leader.Progress.RaceDistanceMeters)
            {
                wentAhead = true;
                chaserSpeedWhenAhead = chaser.State.Speed;
                leaderSpeedWhenAhead = leader.State.Speed;
            }
        }
        int spins = ((ICarMotion)chaser.Car).Motion(chaser.State).Spins;
        Assert.True(everSwitchback, "switchback never armed against the deep-inside defender");
        Assert.False(hit);
        Assert.False(wall);
        Assert.Equal(0, spins);
        Assert.True(wentAhead, "chaser never got ahead of the defender");
        Assert.True(chaserSpeedWhenAhead > leaderSpeedWhenAhead,
            $"chaser was not faster on the way by ({chaserSpeedWhenAhead:F1} vs {leaderSpeedWhenAhead:F1})");
    }

    private static (RaceSimulation Simulation, LaneSet Lanes, RaceCar Leader, RaceCar Chaser) PairAt(
        float start, float leaderAhead, float chaserBehind, float leaderSpeed, float chaserSpeed,
        DriverAbilities leader, DriverAbilities chaser)
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        string compound = DevelopmentTyres.SuperId;
        (RaceCar leaderCar, _) = LaneGrid.Seat(
            simulation, file, lanes, "pair-leader", StockCar.Build(compound), compound,
            start + leaderAhead, 0f, leaderSpeed, abilities: leader);
        (RaceCar chaserCar, _) = LaneGrid.Seat(
            simulation, file, lanes, "pair-chaser", StockCar.Build(compound), compound,
            start - chaserBehind, 0f, chaserSpeed, abilities: chaser);
        return (simulation, lanes, leaderCar, chaserCar);
    }

    private static (int InsideSign, int Entry) NextInside(LaneSet lanes, int anchor)
    {
        float[] k = CornerMap.FreeCurvature(lanes);
        List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
        int best = int.MaxValue;
        int sign = 0;
        int entry = anchor;
        for (int i = 0; i < bends.Count; i++)
        {
            CornerMap.Pins pins = CornerMap.PinsOf(lanes, k, bends, i);
            int dist = lanes.Wrap(pins.Entry - anchor);
            if (dist < best)
            {
                best = dist;
                entry = pins.Entry;
                sign = pins.Sign == 0 ? 0 : -pins.Sign;
            }
        }
        return (sign, entry);
    }
}
