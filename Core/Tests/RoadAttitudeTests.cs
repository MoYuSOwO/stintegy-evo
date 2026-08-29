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
        //
        // It closes by construction rather than by tuning: heights are what
        // the circuits are specified in, wrapped round and read with a
        // periodic interpolant, and the gradient is that curve's slope. The
        // slope of a periodic curve integrates to nothing over a period, so
        // the tolerance here is for the summation and not for the model --
        // every circuit lands inside a millimetre.
        foreach ((string name, TrackData track) in NamedCircuits())
        {
            float climb = 0f;
            float step = 1f;
            for (float s = 0f; s < track.LengthMeters; s += step)
                climb += track.Sample(s).Grade * step;
            Assert.InRange(climb, -0.05f, 0.05f);
            Assert.True(name.Length > 0);
        }
    }

    [Fact]
    public void TheStartLineIsNotASeamInTheRoad()
    {
        // Closing the height is necessary but not sufficient: a lap can come
        // back to where it started and still arrive there over a step, if
        // the surface either side of the start line was worked out without
        // reference to the other side. What is asserted is not some absolute
        // smoothness but that the seam is nothing special -- the road across
        // the start line has to be no rougher than the road anywhere else on
        // the same circuit.
        foreach ((string name, TrackData track) in NamedCircuits())
        {
            int metres = (int)track.LengthMeters;
            float worstGrade = 0f;
            float worstBank = 0f;
            for (int s = 0; s < metres - 1; s++)
            {
                worstGrade = MathF.Max(
                    worstGrade,
                    MathF.Abs(track.Sample(s + 1).Grade - track.Sample(s).Grade)
                );
                worstBank = MathF.Max(
                    worstBank,
                    MathF.Abs(
                        track.Sample(s + 1).BankSlope -
                        track.Sample(s).BankSlope
                    )
                );
            }

            TrackSample before = track.Sample(metres - 1);
            TrackSample after = track.Sample(0);
            float seamGrade = MathF.Abs(after.Grade - before.Grade);
            float seamBank = MathF.Abs(after.BankSlope - before.BankSlope);

            Assert.True(
                seamGrade <= worstGrade * 1.5f + 1e-4f,
                $"{name} steps its gradient by {seamGrade:0.00000} across the " +
                $"start line, against {worstGrade:0.00000} at its worst " +
                $"elsewhere"
            );
            Assert.True(
                seamBank <= worstBank * 1.5f + 1e-4f,
                $"{name} steps its bank by {seamBank:0.00000} across the " +
                $"start line, against {worstBank:0.00000} at its worst " +
                $"elsewhere"
            );
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

    [Fact]
    public void TheSimpleLayoutClimbsItsStartStraightAndGivesItBack()
    {
        TrackData track = TrackFactory.SimpleTestTrack();

        // Up the start/finish straight, level across the top through the
        // hairpin and esses, and all of it back down the back straight.
        Assert.True(HeightAt(track, 550f) > 25f, "the start straight must climb");
        Assert.InRange(HeightAt(track, 832f) - HeightAt(track, 550f), -3f, 3f);
        Assert.InRange(HeightAt(track, 1332f), -3f, 3f);
        Assert.InRange(HeightAt(track, track.LengthMeters - 1f), -3f, 3f);

        // Steep enough to be felt: a climb of thirty metres over a
        // five-hundred-metre straight is about six percent.
        Assert.True(track.Sample(275f).Grade > 0.04f);
        Assert.True(track.Sample(1080f).Grade < -0.04f);
    }

    [Fact]
    public void TheSimpleLayoutBanksTheHairpinAndTheLongLeft()
    {
        TrackData track = TrackFactory.SimpleTestTrack();

        float hairpin = MathF.Abs(track.Sample(580f).BankSlope);
        float sweeper = MathF.Abs(track.Sample(1450f).BankSlope);
        float straight = MathF.Abs(track.Sample(275f).BankSlope);

        Assert.True(hairpin > 0.25f, $"hairpin bank {hairpin:0.000}");
        Assert.True(sweeper > 0.18f, $"long left bank {sweeper:0.000}");
        Assert.True(straight < 0.02f, "the straight should stay near level");

        // The bank has to lean the way the corner turns, or it would be
        // fighting the very corner it was built for.
        foreach (float s in new[] { 580f, 1450f })
        {
            TrackSample sample = track.Sample(s);
            Assert.Equal(
                MathF.Sign(sample.RefCurvature),
                MathF.Sign(sample.BankSlope)
            );
        }
    }

    [Fact]
    public void TheSimpleLayoutMarksItsFeaturesWhereTheyActuallyAre()
    {
        // The surface is placed by distances written beside the layout. If
        // anyone edits the layout without moving them, the bank lands on a
        // straight and this catches it.
        TrackData track = TrackFactory.SimpleTestTrack();
        Assert.True(MathF.Abs(track.Sample(580f).RefCurvature) > 0.02f);
        Assert.True(MathF.Abs(track.Sample(1450f).RefCurvature) > 0.005f);
        Assert.True(MathF.Abs(track.Sample(275f).RefCurvature) < 0.002f);
        Assert.True(MathF.Abs(track.Sample(1080f).RefCurvature) < 0.002f);
    }

    [Fact]
    public void ThePlanBrakesEarlierForACornerItIsDescendingInto()
    {
        // The plan has to know the road falls away, or it brakes for the
        // corner as though the approach were level and arrives too fast.
        // Same layout three times, differing only in what the road does on
        // the way in.
        float level = PlannedApproachSpeed(0f);
        float downhill = PlannedApproachSpeed(-0.08f);
        float uphill = PlannedApproachSpeed(0.08f);

        Assert.True(
            downhill < level,
            $"descending {downhill:0.0} should be planned slower than level {level:0.0}"
        );
        Assert.True(
            uphill > level,
            $"climbing {uphill:0.0} should be planned faster than level {level:0.0}"
        );
    }

    /// <summary>
    /// How fast the plan says the car may be forty metres from a hairpin,
    /// on an approach with the given gradient. Forty metres because these
    /// cars stop hard enough that a braking zone is short: sampled from
    /// further out the answer is the car's top speed and says nothing
    /// about braking at all.
    /// </summary>
    private static float PlannedApproachSpeed(float grade)
    {
        const float approach = 300f;
        TrackData track = new TrackBuilder(
                Vector2.Zero,
                startWidth: 16f,
                startLeftBuffer: 5f,
                startRightBuffer: 5f
            )
            .AddStraight(approach)
            .AddTurn(180f, 40f)
            .AddStraight(approach)
            .AddTurn(180f, 40f)
            .CloseLoop()
            .WithSurface(context => new TrackSurface(
                // Falls away over the approach and climbs back on the far
                // side, so the lap still closes.
                Grade: context.DistanceMeters < approach
                    ? grade
                    : -grade * approach /
                      MathF.Max(context.LapLengthMeters - approach, 1f)
            ))
            .Build(new TrackGridConfig());

        RaceCar car = CreateCar(track, s: 10f, speed: 70f);
        VehicleSpeedPlanner planner = new();
        VehicleSpeedLookahead plan = planner.PlanReferenceLookahead(
            new VehicleSpeedLookahead(),
            car,
            track,
            startS: 10f,
            horizonMeters: 320f,
            stepMeters: 2f,
            DriverPlanningModifiers.Neutral
        );
        return plan.Sample(approach - 50f).TargetSpeed;
    }

    private static float HeightAt(TrackData track, float target)
    {
        float height = 0f;
        for (float s = 0f; s < target; s += 1f)
            height += track.Sample(s).Grade;
        return height;
    }

    private static IEnumerable<(string, TrackData)> NamedCircuits()
    {
        yield return ("silverstone", TrackFactory.SilverstoneStyleTestTrack());
        yield return ("monaco", TrackFactory.MonacoStyleTestTrack());
        yield return ("shanghai", TrackFactory.ShanghaiStyleTestTrack());
        yield return ("sepang", TrackFactory.SepangStyleTestTrack());
        yield return ("speedway", TrackFactory.BankedSpeedwayTestTrack());
        yield return ("simple", TrackFactory.SimpleTestTrack());
        yield return ("simple-left", TrackFactory.SimpleTestTrack(isLeft: true));
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

    [Fact]
    public void NoCircuitTurnsItsBankOverInASingleMetre()
    {
        // Banking is run in and out over tens of metres on a real road. It
        // cannot reverse under a car in one, and a surface that does is not
        // a road but a step in the lateral force balance.
        //
        // The two that did: the simple layout flipped seventeen degrees of
        // added bank to seventeen the other way between one node and the
        // next, where the hairpin handed over to the esses; and the speedway
        // put a reverse-banked node at each turn exit, where the curvature
        // it was built from dithered across zero on the straight. Both came
        // of reading a bank's direction from the sign of a noisy, stepped
        // curvature rather than from how committed the corner is. The worst
        // any circuit manages now is 0.135 per metre.
        foreach ((string name, TrackData track) in NamedCircuits())
        {
            float worst = 0f;
            float worstAtS = 0f;
            int metres = (int)track.LengthMeters;
            for (int s = 0; s < metres; s++)
            {
                float change = MathF.Abs(
                    track.Sample(s + 1).BankSlope - track.Sample(s).BankSlope
                );
                if (change > worst)
                {
                    worst = change;
                    worstAtS = s;
                }
            }

            Assert.True(
                worst < 0.2f,
                $"{name} changes its bank by {worst:0.000} per metre at " +
                $"s={worstAtS:0}, which is a step rather than a transition"
            );
        }
    }

    [Fact]
    public void ThePlanReadsTheBankWhereTheCarWillActuallyBe()
    {
        // A progressively banked corner is not one surface but a range of
        // them, and the reference line is only one place across it. A plan
        // that always reads the bank at the reference line prices a corner
        // the car is not driving: run high on Daytona's banking and the plan
        // has to see the steeper road, or the whole point of the high line
        // is invisible to it.
        TrackData track = BuildBankedOval();
        float turnS = SharpestPoint(track);

        float high = PlannedSpeedOnLine(track, turnS, offsetMeters: 5f);
        float low = PlannedSpeedOnLine(track, turnS, offsetMeters: -5f);

        Assert.True(
            high > low * 1.02f,
            $"the steeper high line ({high:0.0} m/s) should plan faster " +
            $"than the shallow apron ({low:0.0} m/s)"
        );
    }

    /// <summary>
    /// Plans a path that holds one fixed offset from the centreline through
    /// a corner. Both lines are given the same commanded curvature on
    /// purpose, so the only thing that can separate them is the road each
    /// one is standing on.
    /// </summary>
    private static float PlannedSpeedOnLine(
        TrackData track,
        float turnS,
        float offsetMeters
    )
    {
        const int points = 21;
        const float step = 2f;
        float startS = turnS - points * step * 0.5f;
        RaceCar car = CreateCar(track, startS, speed: 85f);

        VehiclePathPrediction path = new();
        path.Reset(points);
        for (int i = 0; i < points; i++)
        {
            float s = startS + i * step;
            TrackSample sample = track.Sample(s);
            path.Add(new VehiclePathPredictionPoint(
                i * step,
                sample.Center + sample.Normal * offsetMeters,
                MathF.Atan2(sample.Tangent.Y, sample.Tangent.X),
                s,
                0f,
                sample.RefCurvature,
                sample.RefCurvature,
                0f,
                sample.RefCurvature,
                85f
            ));
        }

        VehicleSpeedPlanner planner = new();
        return planner
            .PlanPredictedPath(new VehicleSpeedLookahead(), car, path, track)
            .Current.TargetSpeed;
    }

    [Fact]
    public void TheControllerPaysForTheGradientItIsDrivingOn()
    {
        // The plan knowing about the hill is only half of it. The controller
        // asks the axle for a number and the road adds its pull afterwards,
        // so unless the gradient is answered on the way out, the only thing
        // left to answer it is the proportional speed term — and a
        // proportional term meets a standing pull with a standing error, so
        // the car climbs slower than the plan it is obeying. Measured over
        // twenty seconds of an eight percent oval, paying for it is worth
        // 1.8% of the distance covered.
        float expected = GravityMetersPerSecondSquared * 0.08f /
                         MathF.Sqrt(1f + 0.08f * 0.08f);

        // On a level road the term has to vanish outright: anything else
        // would move every result on every flat circuit.
        Assert.InRange(GradeCompensation(TrackSurface.Flat), -1e-4f, 1e-4f);
        Assert.InRange(
            GradeCompensation(new TrackSurface(Grade: 0.08f)),
            expected * 0.98f,
            expected * 1.02f
        );
        Assert.InRange(
            GradeCompensation(new TrackSurface(Grade: -0.08f)),
            -expected * 1.02f,
            -expected * 0.98f
        );
    }

    private const float GravityMetersPerSecondSquared = 9.80665f;

    /// <summary>
    /// What the driver adds to the axle request to answer the road, averaged
    /// over a settled run.
    /// </summary>
    private static float GradeCompensation(TrackSurface surface)
    {
        TrackData track = BuildOval(surface);
        RaceCar car = CreateCar(track, s: 10f, speed: 45f);
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        ReferenceLineDriver driver = (ReferenceLineDriver)car.Driver;
        double total = 0d;
        int samples = 0;
        for (int i = 0; i < 60 * 20; i++)
        {
            simulation.Step(1f / 60f);
            if (i < 60 * 5)
                continue;
            total += driver.LastTelemetry.GradeCompensationAcceleration;
            samples++;
        }
        return (float)(total / Math.Max(samples, 1));
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
