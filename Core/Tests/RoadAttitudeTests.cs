using System;
using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Track.RefLines;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Pins the 2.5D road: a climb costs speed, a descent gives it back, and a
/// banked corner both carries part of the cornering load and adds grip
/// while it does so. The car stays a body on a surface — nothing here
/// simulates it as a rigid body in three dimensions.
/// </summary>
public sealed class RoadAttitudeTests
{
    private const float Dt = 1f / 120f;
    private const float Gravity = 9.80665f;

    [Fact]
    public void FlatRoadChangesNothing()
    {
        RoadAttitude flat = RoadAttitude.Flat;
        Assert.Equal(1f, flat.NormalCosine, 6);
        Assert.Equal(0f, flat.AlongTrackGravity(Gravity), 6);
        Assert.Equal(0f, flat.LateralGravityDemand(Gravity), 6);
        Assert.Equal(
            Gravity,
            flat.NormalGravity(Gravity, speedMetersPerSecond: 60f, curvature: 0.01f),
            4
        );
    }

    [Theory]
    [InlineData(0.10f)]
    [InlineData(0.05f)]
    public void ClimbingCostsSpeedAndDescendingReturnsIt(float grade)
    {
        float uphill = RunStraight(new RoadAttitude(grade, 0f));
        float level = RunStraight(RoadAttitude.Flat);
        float downhill = RunStraight(new RoadAttitude(-grade, 0f));

        Assert.True(uphill < level, "a climb must cost speed");
        Assert.True(downhill > level, "a descent must return it");
        // The two should be near mirror images of each other, since the
        // only asymmetry is what the drag does over slightly different
        // speeds.
        float lost = level - uphill;
        float gained = downhill - level;
        Assert.InRange(gained / lost, 0.8f, 1.25f);
    }

    [Fact]
    public void BankingCarriesPartOfTheCorneringLoad()
    {
        // The track normal points right, so a surface rising to the right
        // raises the outside of a left-hand corner.
        RoadAttitude banked = new(0f, 0.3f);
        const float leftHander = 0.02f;

        float demandFlat = 60f * 60f * leftHander;
        float demandBanked = demandFlat +
                             banked.LateralGravityDemand(Gravity);

        Assert.True(
            MathF.Abs(demandBanked) < MathF.Abs(demandFlat),
            "banking into the corner must reduce what the tyres owe"
        );

        const float rightHander = -0.02f;
        float wrongWay = 60f * 60f * rightHander +
                         banked.LateralGravityDemand(Gravity);
        Assert.True(
            MathF.Abs(wrongWay) > MathF.Abs(60f * 60f * rightHander),
            "banking the wrong way must cost the tyres more"
        );
    }

    [Fact]
    public void BankingAddsGripAndItGrowsWithSpeed()
    {
        RoadAttitude banked = new(0f, 0.4f);
        const float leftHander = 0.02f;

        float atRest = banked.NormalGravity(Gravity, 0f, leftHander);
        float slow = banked.NormalGravity(Gravity, 30f, leftHander);
        float fast = banked.NormalGravity(Gravity, 60f, leftHander);

        Assert.True(atRest < Gravity, "tilting the road spills some weight");
        Assert.True(slow > atRest);
        Assert.True(fast > slow);
        // Four times the speed-squared should quadruple the bank's share.
        Assert.InRange(
            (fast - atRest) / (slow - atRest),
            3.6f,
            4.4f
        );
    }

    [Fact]
    public void BankingIsNeverAllowedToUnloadTheCarCompletely()
    {
        // A steep bank leaning the wrong way at speed would drive the
        // normal load negative, which this model does not represent.
        RoadAttitude banked = new(0f, 0.5f);
        float load = banked.NormalGravity(Gravity, 90f, curvature: -0.05f);
        Assert.True(load > 0f);
    }

    [Fact]
    public void ProgressiveBankingRewardsTheHighLine()
    {
        // Daytona's shape: shallow at the apron, steep at the wall.
        TrackSample sample = BuildBankedOval().Sample(600f);
        Assert.True(sample.BankCurvature > 0f);

        float low = sample.BankSlopeAt(-5f);
        float high = sample.BankSlopeAt(5f);
        Assert.True(
            high > low,
            "the outside of a progressively banked corner must be steeper"
        );
    }

    [Fact]
    public void BankedCornersPlanFasterThanFlatOnes()
    {
        TrackData flat = BuildOval(bank: TrackSurface.Flat);
        TrackData banked = BuildOval(
            bank: new TrackSurface(BankSlope: 0.5f)
        );
        // Sample where the corner actually is, rather than guessing a
        // fraction that moves whenever the oval's proportions change.
        float turnS = SharpestPoint(flat);
        Assert.True(MathF.Abs(flat.Sample(turnS).RefCurvature) > 1e-3f);

        float flatLimit = PlannedSpeedAt(flat, turnS);
        float bankedLimit = PlannedSpeedAt(banked, turnS);

        Assert.True(
            bankedLimit > flatLimit * 1.05f,
            $"banked {bankedLimit:0.0} should beat flat {flatLimit:0.0}"
        );
    }

    [Fact]
    public void ABankedOvalIsFasterToDriveThanAFlatOne()
    {
        // The end-to-end check the analytic ones cannot make: a real car,
        // planning and driving itself, over the same oval twice. Every sign
        // in the chain has to agree for this to come out the right way
        // round, which is exactly how the first version's inverted bank was
        // caught.
        float flat = RunSimulated(TrackSurface.Flat);
        float banked = RunSimulated(new TrackSurface(BankSlope: 0.45f));
        float wrongWay = RunSimulated(new TrackSurface(BankSlope: -0.45f));

        Assert.True(
            banked > flat * 1.02f,
            $"banked {banked:0} m should beat flat {flat:0} m"
        );
        Assert.True(
            wrongWay < flat,
            $"a bank leaning out of the corner ({wrongWay:0} m) must cost " +
            $"against flat ({flat:0} m)"
        );
    }

    [Fact]
    public void ClimbShowsUpInASimulatedCar()
    {
        float flatDistance = RunSimulated(TrackSurface.Flat);
        float climbDistance = RunSimulated(new TrackSurface(Grade: 0.08f));
        Assert.True(
            climbDistance < flatDistance,
            $"climbing {climbDistance:0} m should fall short of level " +
            $"{flatDistance:0} m"
        );
    }

    [Fact]
    public void RoadCircuitsAreCrownedOnStraightsAndTiltIntoCorners()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        TrackSample straight = FlattestPoint(track);
        TrackSample corner = track.Sample(SharpestPoint(track));

        // A straight sheds water off both edges and leans neither way.
        Assert.True(straight.BankCurvature < -1e-4f);
        Assert.InRange(MathF.Abs(straight.BankSlope), 0f, 0.01f);

        // A corner tilts into itself, and the tilt is the modest couple of
        // degrees a road circuit actually has rather than a speedway's.
        Assert.True(MathF.Abs(corner.BankSlope) > 0.02f);
        Assert.InRange(MathF.Abs(corner.BankSlope), 0f, 0.05f);
        Assert.Equal(
            MathF.Sign(corner.RefCurvature),
            MathF.Sign(corner.BankSlope)
        );
    }

    [Fact]
    public void EveryGrandPrixCircuitGetsTheSameConstructionModel()
    {
        foreach (TrackData track in new[]
                 {
                     TrackFactory.SilverstoneStyleTestTrack(),
                     TrackFactory.MonacoStyleTestTrack(),
                     TrackFactory.ShanghaiStyleTestTrack(),
                     TrackFactory.SepangStyleTestTrack()
                 })
        {
            TrackSample corner = track.Sample(SharpestPoint(track));
            Assert.True(MathF.Abs(corner.BankSlope) > 0.01f);
            Assert.InRange(MathF.Abs(corner.BankSlope), 0f, 0.05f);
        }
    }

    [Fact]
    public void TheSpeedwayIsBankedFarBeyondAnyRoadCircuit()
    {
        TrackData speedway = TrackFactory.BankedSpeedwayTestTrack();
        TrackData road = TrackFactory.SilverstoneStyleTestTrack();
        TrackSample turn = speedway.Sample(SharpestPoint(speedway));
        TrackSample roadCorner = road.Sample(SharpestPoint(road));

        // About thirty-one degrees at the wall against eighteen at the
        // apron, an order beyond what a Grand Prix corner carries.
        Assert.True(MathF.Abs(turn.BankSlope) > 8f * MathF.Abs(roadCorner.BankSlope));
        float apron = turn.BankSlopeAt(-turn.HalfWidth + 1f);
        float wall = turn.BankSlopeAt(turn.HalfWidth - 1f);
        Assert.True(
            MathF.Abs(wall) > MathF.Abs(apron) * 1.4f,
            $"wall {wall:0.000} should be far steeper than apron {apron:0.000}"
        );
    }

    [Fact]
    public void TheSpeedwayIsQuickerThanTheSameShapeUnbanked()
    {
        float banked = LapDistance(TrackFactory.BankedSpeedwayTestTrack());
        float flat = LapDistance(BuildFlatSpeedway());
        // Measured at about four percent. Less than the corner speed alone
        // would suggest, because the planner claims the demand the bank
        // lifts off the tyres more readily than the load it presses on, and
        // because a short track spends much of its lap accelerating out
        // rather than cornering. The pin is set below that so it fails on a
        // sign or a wiring mistake rather than on tuning.
        Assert.True(
            banked > flat * 1.03f,
            $"banked speedway {banked:0} m should beat the flat one {flat:0} m"
        );
    }

    [Fact]
    public void EveryCircuitReturnsToTheHeightItLeft()
    {
        // The one thing a closed circuit's elevation must do. Integrating
        // the gradient the physics actually reads has to come back to zero,
        // or a car would gain or lose energy on every lap for free.
        foreach ((string name, TrackData track) in NamedCircuits())
        {
            float climb = 0f;
            float step = 1f;
            for (float s = 0f; s < track.LengthMeters; s += step)
                climb += track.Sample(s).Grade * step;
            Assert.InRange(climb, -0.5f, 0.5f);
            Assert.True(name.Length > 0);
        }
    }

    [Fact]
    public void CircuitsCarryTheElevationTheirCharacterCallsFor()
    {
        // Ranked by how much each circuit actually climbs: Monaco far
        // beyond the rest, Silverstone the flat airfield it is.
        float monaco = ElevationRange(TrackFactory.MonacoStyleTestTrack());
        float sepang = ElevationRange(TrackFactory.SepangStyleTestTrack());
        float shanghai = ElevationRange(TrackFactory.ShanghaiStyleTestTrack());
        float silverstone = ElevationRange(
            TrackFactory.SilverstoneStyleTestTrack()
        );

        Assert.True(monaco > sepang, $"monaco {monaco:0.0} vs sepang {sepang:0.0}");
        Assert.True(sepang > shanghai);
        Assert.True(shanghai > silverstone);
        // Monaco's climb out of Sainte Dévote is the steepest thing here
        // and is a real gradient, not a rounding artefact.
        Assert.True(SteepestGrade(TrackFactory.MonacoStyleTestTrack()) > 0.03f);
        Assert.True(
            SteepestGrade(TrackFactory.SilverstoneStyleTestTrack()) < 0.02f
        );
    }

    [Fact]
    public void AClimbCostsAndTheMatchingDescentPaysItBack()
    {
        // End to end on a real circuit: a lap of Monaco against the same
        // layout levelled. What matters is that the two come out close,
        // because a closed lap spends climbing exactly what it recovers
        // descending — a hilly circuit is not a slow one, it is one whose
        // speed is differently distributed. A large gap either way would
        // mean the gradient was leaking energy.
        float hilly = LapDistance(TrackFactory.MonacoStyleTestTrack());
        float level = LapDistance(BuildLevelMonaco());
        Assert.InRange(hilly / level, 0.97f, 1.03f);
    }

    private static IEnumerable<(string, TrackData)> NamedCircuits()
    {
        yield return ("silverstone", TrackFactory.SilverstoneStyleTestTrack());
        yield return ("monaco", TrackFactory.MonacoStyleTestTrack());
        yield return ("shanghai", TrackFactory.ShanghaiStyleTestTrack());
        yield return ("sepang", TrackFactory.SepangStyleTestTrack());
        yield return ("speedway", TrackFactory.BankedSpeedwayTestTrack());
    }

    private static float ElevationRange(TrackData track)
    {
        float height = 0f, lowest = 0f, highest = 0f, step = 1f;
        for (float s = 0f; s < track.LengthMeters; s += step)
        {
            height += track.Sample(s).Grade * step;
            lowest = MathF.Min(lowest, height);
            highest = MathF.Max(highest, height);
        }
        return highest - lowest;
    }

    private static float SteepestGrade(TrackData track)
    {
        float steepest = 0f;
        for (float s = 0f; s < track.LengthMeters; s += 2f)
            steepest = MathF.Max(steepest, MathF.Abs(track.Sample(s).Grade));
        return steepest;
    }

    private static TrackData BuildLevelMonaco() =>
        TrackBuilder.FromClosedCenterline(
                TrackCenterlineData.Monaco, 3_337f, 3f, 3f)
            .WithSurface(TrackSurfaces.RoadCircuit)
            .Build(new TrackGridConfig());

    private static float LapDistance(TrackData track)
    {
        RaceCar car = CreateCar(track, s: 10f, speed: 60f);
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        for (int i = 0; i < 60 * 40; i++)
            simulation.Step(1f / 60f);
        return car.Progress.TotalDistance;
    }

    private static TrackData BuildFlatSpeedway()
    {
        return new TrackBuilder(
                Vector2.Zero,
                startWidth: 15f,
                startLeftBuffer: 6f,
                startRightBuffer: 6f
            )
            .AddStraight(350f)
            .AddTurn(180f, 90f)
            .AddStraight(350f)
            .AddTurn(180f, 90f)
            .CloseLoop()
            .Build(new TrackGridConfig());
    }

    private static TrackSample FlattestPoint(TrackData track)
    {
        TrackSample best = track.Sample(0f);
        float bestCurvature = float.MaxValue;
        for (float s = 0f; s < track.LengthMeters; s += 5f)
        {
            TrackSample sample = track.Sample(s);
            float curvature = MathF.Abs(sample.RefCurvature);
            if (curvature < bestCurvature)
            {
                bestCurvature = curvature;
                best = sample;
            }
        }
        return best;
    }

    private static float SharpestPoint(TrackData track)
    {
        float bestS = 0f;
        float bestCurvature = 0f;
        for (float s = 0f; s < track.LengthMeters; s += 5f)
        {
            float curvature = MathF.Abs(track.Sample(s).RefCurvature);
            if (curvature > bestCurvature)
            {
                bestCurvature = curvature;
                bestS = s;
            }
        }
        return bestS;
    }

    private static float PlannedSpeedAt(TrackData track, float s)
    {
        // Fast enough that the car's own top speed is not the binding
        // constraint, so what comes back is the corner's limit.
        RaceCar car = CreateCar(track, s, speed: 85f);
        VehicleSpeedPlanner planner = new();
        VehicleSpeedLookahead plan = planner.PlanReferenceLookahead(
            new VehicleSpeedLookahead(),
            car,
            track,
            s,
            horizonMeters: 20f,
            stepMeters: 2f,
            DriverPlanningModifiers.Neutral
        );
        return plan.Sample(0f).TargetSpeed;
    }

    private static float RunStraight(RoadAttitude road)
    {
        CarConfig config = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        CarState state = new()
        {
            Position = Vector2.Zero,
            Heading = 0f,
            Speed = 50f,
            BatterySoc = 0.8f
        };
        CarPhysicsStepInput input = new(
            new DriverInput(0f, 0f),
            CarStrategy.Default,
            AirTempC: 25f
        )
        {
            RoadAttitude = road
        };
        for (int i = 0; i < 120; i++)
            CarPhysics.Step(state, config, tires, input, Dt);
        return state.Speed;
    }

    private static float RunSimulated(TrackSurface surface)
    {
        TrackData track = BuildOval(surface);
        RaceCar car = CreateCar(track, s: 10f, speed: 45f);
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        for (int i = 0; i < 60 * 20; i++)
            simulation.Step(1f / 60f);
        return car.Progress.TotalDistance;
    }

    private static TrackData BuildOval(TrackSurface bank)
    {
        return new TrackBuilder(
                Vector2.Zero,
                startWidth: 16f,
                refLineSolver: CenterLineRefLineSolver.Instance
            )
            .WithSurface(context => bank)
            .AddStraight(400f)
            .AddTurn(180f, 50f)
            .AddStraight(400f)
            .AddTurn(180f, 50f)
            .CloseLoop()
            .Build(new TrackGridConfig());
    }

    private static TrackData BuildBankedOval()
    {
        return new TrackBuilder(
                Vector2.Zero,
                startWidth: 16f,
                refLineSolver: CenterLineRefLineSolver.Instance
            )
            .WithSurface(context => new TrackSurface(
                BankSlope: 0.45f,
                BankCurvature: 0.012f
            ))
            .AddStraight(400f)
            .AddTurn(180f, 50f)
            .AddStraight(400f)
            .AddTurn(180f, 50f)
            .CloseLoop()
            .Build(new TrackGridConfig());
    }

    private static RaceCar CreateCar(TrackData track, float s, float speed)
    {
        TrackSample sample = track.Sample(s);
        return new RaceCar(
            "surface-test",
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            new ReferenceLineDriver(),
            new CarState
            {
                Position = sample.RefPosition,
                Heading = sample.RefHeading,
                Speed = speed,
                BatterySoc = 0.8f
            }
        );
    }
}
