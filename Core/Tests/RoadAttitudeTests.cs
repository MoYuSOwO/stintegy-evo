using System;
using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Pins the physical 2.5D road: centreline curvature, grade, banking,
/// elevation closure, and the force resolution that consumes those values.
/// No planner-generated line or driving controller participates.
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
            MathF.Sign(corner.Curvature),
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
                MathF.Sign(sample.Curvature),
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
        Assert.True(MathF.Abs(track.Sample(580f).Curvature) > 0.02f);
        Assert.True(MathF.Abs(track.Sample(1450f).Curvature) > 0.005f);
        Assert.True(MathF.Abs(track.Sample(275f).Curvature) < 0.002f);
        Assert.True(MathF.Abs(track.Sample(1080f).Curvature) < 0.002f);
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

    /// <summary>
    /// The straightest piece of road, judged by the road.
    ///
    /// This used to ask the racing line, and the racing line is the wrong
    /// witness: a minimum-curvature line runs through an inflection —
    /// curvature exactly zero — in the middle of a corner, where the road
    /// is banked and is supposed to be. The cross slope is written from
    /// the centreline's curvature, so that is what has to be asked here
    /// for the question to be about the same road the answer is about.
    /// The window is the one the surface layer itself reads over.
    /// </summary>
    private static TrackSample FlattestPoint(TrackData track)
    {
        const int half = 8;
        TrackSample best = track.Sample(0f);
        float bestCurvature = float.MaxValue;
        int length = (int)track.LengthMeters;
        for (int s = 0; s < length; s += 5)
        {
            float turn = 0f;
            for (int step = -half; step < half; step++)
            {
                Vector2 a = track.Sample((s + step + length) % length).Tangent;
                Vector2 b =
                    track.Sample((s + step + 1 + length) % length).Tangent;
                turn += MathHelper.NormalizeAngle(
                    MathF.Atan2(b.Y, b.X) - MathF.Atan2(a.Y, a.X)
                );
            }
            float curvature = MathF.Abs(turn) / (2f * half);
            if (curvature < bestCurvature)
            {
                bestCurvature = curvature;
                best = track.Sample(s);
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
            float curvature = MathF.Abs(track.Sample(s).Curvature);
            if (curvature > bestCurvature)
            {
                bestCurvature = curvature;
                bestS = s;
            }
        }
        return bestS;
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

    private static TrackData BuildBankedOval()
    {
        return new TrackBuilder(
                Vector2.Zero,
                startWidth: 16f
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
}
