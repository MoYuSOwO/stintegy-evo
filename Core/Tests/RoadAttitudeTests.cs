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
        Assert.Equal(0f, flat.AlongTrackGravity(Gravity, 60f), 6);
        Assert.Equal(0f, flat.LateralGravityDemand(Gravity, 60f), 6);
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
        // What matters is the share of the grip a corner uses up, not the
        // raw force: resolving the demand onto the surface shrinks it a
        // little whichever way the road leans, so comparing forces alone
        // would call an adverse bank an improvement.
        RoadAttitude banked = new(0f, 0.3f);
        const float speed = 60f;

        // The track normal points right, so a surface rising to the right
        // raises the outside of a left-hand corner.
        Assert.True(
            GripShare(banked, speed, 0.02f) < GripShare(RoadAttitude.Flat, speed, 0.02f),
            "banking into the corner must leave the tyres less to do"
        );
        Assert.True(
            GripShare(banked, speed, -0.02f) > GripShare(RoadAttitude.Flat, speed, -0.02f),
            "banking the wrong way must cost the tyres more"
        );
    }

    /// <summary>
    /// How much of what the road is pressing into the tyres a corner spends.
    /// </summary>
    private static float GripShare(
        RoadAttitude road,
        float speed,
        float curvature
    )
    {
        float demand = road.CurvatureDemandScale * speed * speed * curvature +
                       road.LateralGravityDemand(Gravity, speed);
        return MathF.Abs(demand) /
               road.NormalGravity(Gravity, speed, curvature);
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
    public void ABankedStretchCanStraddleTheStartLine()
    {
        // Banking has no closing condition the way elevation does -- it is a
        // value across the road, not a rate along it, so nothing integrates
        // round a lap and nothing has to come back to zero. What it does
        // have to be is continuous at the seam, and a stretch measured along
        // a number line rather than round a lap is not: its ramp gets cut
        // off at the start line. Today's layouts keep their banked corners
        // away from the line, so this held by luck of where the corners are
        // rather than by construction.
        const float lap = 1000f;
        const float blend = 40f;

        // A corner from 960 m round through the line to 80 m.
        float Weight(float s) =>
            TrackSurfaces.SectionWeight(s, 960f, 1080f, blend, lap);

        // Full weight right across the line, and matching either side of it.
        Assert.Equal(1f, Weight(0f), 3);
        Assert.Equal(1f, Weight(20f), 3);
        Assert.Equal(1f, Weight(980f), 3);

        // Easing in over the forty metres before it and out over the forty
        // after, both of which cross the line, and nothing beyond either.
        Assert.InRange(Weight(940f), 0.4f, 0.6f);
        Assert.InRange(Weight(100f), 0.4f, 0.6f);
        Assert.Equal(0f, Weight(920f), 3);
        Assert.Equal(0f, Weight(120f), 3);
        Assert.Equal(0f, Weight(500f), 3);

        // And no step anywhere round the lap, the seam included.
        float previous = Weight(lap - 1f);
        for (float s = 0f; s < lap; s += 1f)
        {
            float here = Weight(s);
            Assert.True(
                MathF.Abs(here - previous) < 0.05f,
                $"the stretch steps by {MathF.Abs(here - previous):0.000} at s={s:0}"
            );
            previous = here;
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
    public void ACrestTakesWeightOffTheCarAndACompressionPutsItOn()
    {
        // Following a road that bends upward takes force beyond holding the
        // car up, and the tarmac is what supplies it -- which is why a
        // compression is where a car can carry impossible speed, and why a
        // brake pedal means less at the top of a hill. Both scale with the
        // square of the speed, so neither arrives gently.
        const float gravity = 9.80665f;
        RoadAttitude crest = new(0f, 0f, -1f / 500f);
        RoadAttitude compression = new(0f, 0f, 1f / 500f);

        // Sixty metres a second over a five-hundred-metre crest asks for
        // 7.2 of the 9.8 the car has, and it keeps the rest.
        Assert.Equal(
            gravity - 3600f / 500f,
            crest.NormalGravity(gravity, 60f, 0f),
            2
        );
        Assert.Equal(
            gravity + 3600f / 500f,
            compression.NormalGravity(gravity, 60f, 0f),
            2
        );

        // Standing still the road's shape is worth nothing at all: this is
        // a speed-squared effect or it is nothing.
        Assert.Equal(gravity, crest.NormalGravity(gravity, 0f, 0f), 4);

        // And the car is never allowed to be lifted clean off, because a
        // model that keeps the car on the surface has nothing to say about
        // what happens when it is not.
        RoadAttitude brow = new(0f, 0f, -1f / 40f);
        Assert.Equal(
            gravity * RoadAttitude.MinimumNormalShare,
            brow.NormalGravity(gravity, 60f, 0f),
            4
        );
    }

    [Fact]
    public void ThePlanSlowsOverACrestAndPressesOnThroughACompression()
    {
        // Sampled at the top of the hill and at the bottom of it, because
        // those are the two places where the road is momentarily level and
        // what is left is purely how it bends. Anywhere else the gradient
        // would be answering as well.
        TrackData hilly = RollingOval(heightMetres: 12f);
        TrackData flat = RollingOval(heightMetres: 0f);

        Assert.InRange(hilly.Sample(SummitMetres).Grade, -0.005f, 0.005f);
        Assert.InRange(hilly.Sample(DipMetres).Grade, -0.005f, 0.005f);
        Assert.True(hilly.Sample(SummitMetres).VerticalRate < -1e-4f);
        Assert.True(hilly.Sample(DipMetres).VerticalRate > 1e-4f);

        float level = PlannedSpeedAt(flat, SummitMetres);
        float overACrest = PlannedSpeedAt(hilly, SummitMetres);
        float throughACompression = PlannedSpeedAt(hilly, DipMetres);

        Assert.True(
            overACrest < level * 0.985f,
            $"over a crest {overACrest:0.0} should be planned below the same " +
            $"corner on the level, {level:0.0}"
        );
        Assert.True(
            throughACompression > level * 1.015f,
            $"through a compression {throughACompression:0.0} should beat " +
            $"the same corner on the level, {level:0.0}"
        );
    }

    // Halfway round each of the oval's two turns.
    private const float SummitMetres = 400f + MathF.PI * 90f / 2f;
    private const float DipMetres = SummitMetres + 400f + MathF.PI * 90f;

    /// <summary>
    /// One oval with a hill over it, the top of the hill in the middle of
    /// one turn and the bottom in the middle of the other, so a corner can
    /// be compared against the same corner on the level. Passing zero gives
    /// the identical layout dead flat, which is the control.
    /// </summary>
    private static TrackData RollingOval(float heightMetres)
    {
        const float lap = 2f * 400f + 2f * MathF.PI * 90f;
        return new TrackBuilder(
                Vector2.Zero,
                startWidth: 16f,
                refLineSolver: CenterLineRefLineSolver.Instance
            )
            .AddStraight(400f)
            .AddTurn(180f, 90f)
            .AddStraight(400f)
            .AddTurn(180f, 90f)
            .CloseLoop()
            .WithSurface(TrackElevation.ProfileByDistance([
                (SummitMetres - 0.25f * lap, 0f),
                (SummitMetres, heightMetres),
                (SummitMetres + 0.25f * lap, 0f),
                (DipMetres, -heightMetres)
            ]))
            .Build(new TrackGridConfig());
    }

    [Fact]
    public void ThePlanBrakesEarlierForACornerItIsDescendingInto()
    {
        // Where the plan lifts, not how fast it is somewhere. A car coming
        // down the hill is quicker all the way along the approach -- it has
        // been accelerating the whole way -- so its speed at any fixed point
        // says nothing. What the gradient decides is the braking point.
        float level = BrakingPointMetresBeforeTheCorner(0f);
        float downhill = BrakingPointMetresBeforeTheCorner(-0.08f);
        float uphill = BrakingPointMetresBeforeTheCorner(0.08f);

        Assert.True(
            downhill > level + 2f,
            $"descending should brake {downhill:0} m out, further than the " +
            $"{level:0} m a level approach needs"
        );
        Assert.True(
            uphill < level - 1f,
            $"climbing should brake {uphill:0} m out, later than the " +
            $"{level:0} m a level approach needs"
        );
    }

    /// <summary>
    /// How far before the hairpin the plan stops accelerating, on an
    /// approach of the given gradient. One gradient the whole way round, so
    /// the road never bends in the vertical plane and nothing but the
    /// gradient separates the three runs; the lap does not close in height
    /// and is not meant to.
    /// </summary>
    private static float BrakingPointMetresBeforeTheCorner(float grade)
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
            .WithSurface(context => new TrackSurface(Grade: grade))
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

        float peak = 0f;
        float peakAt = 0f;
        for (float d = 0f; d < approach; d += 1f)
        {
            float speed = plan.Sample(d).TargetSpeed;
            if (speed <= peak)
                continue;
            peak = speed;
            peakAt = d;
        }
        return approach - peakAt;
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
            Energy = PowertrainState.Filled(0.8f)
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
                Energy = PowertrainState.Filled(0.8f)
            }
        );
    }
}
