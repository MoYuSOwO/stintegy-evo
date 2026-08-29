using System;
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
            .WithSurface(_ => bank)
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
            .WithSurface(_ => new TrackSurface(
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
