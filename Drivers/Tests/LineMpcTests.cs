using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using Xunit;
using StintegyEVO.Drivers.Lanes;

namespace StintegyEVO.Drivers.Tests;

public sealed class LineMpcTests
{
    [Fact]
    public void LeftOfTheLineClosesTheErrorOnTheRealPlant()
    {
        (TrackData track, GeometricRacingLine line, LineSpeedProfile profile, CarConfig car, TireConfig tires) =
            SimplePlan();
        GeometricRacingLine.Sample at = line.At(track.Sample, track.WrapS, 40f);
        TrackSample road = track.Sample(40f);
        CarState state = OnTrack(
            road.Center + road.Normal * (at.OffsetMeters + 2f),
            at.Heading,
            40f,
            tires
        );
        var mpc = new LineMpc();
        LineMpc.Command cmd = mpc.Solve(
            state,
            car,
            tires,
            CarStrategy.Default,
            track.Sample,
            track.WrapS,
            track.Project,
            line,
            profile,
            _ => 0.88f,
            0.95f,
            30f
        );
        float before = MathF.Abs(track.Project(state.Position).D - at.OffsetMeters);
        CarState hold = state.Clone();
        CarState follow = state.Clone();
        for (int i = 0; i < 24; i++)
        {
            CarPhysics.Step(
                hold, car, tires,
                new CarPhysicsStepInput(new DriverInput(CarPhysics.EstimateSteering(hold, car, tires, RoadAttitude.Flat).AngleFor(at.PathCurvature), 0f), CarStrategy.Default, 25f),
                1f / 60f
            );
            CarPhysics.Step(
                follow, car, tires,
                new CarPhysicsStepInput(new DriverInput(CarPhysics.EstimateSteering(follow, car, tires, RoadAttitude.Flat).AngleFor(cmd.Curvature), cmd.Accel), CarStrategy.Default, 25f),
                1f / 60f
            );
        }

        float nLine = line.At(track.Sample, track.WrapS, track.Project(follow.Position).S).OffsetMeters;
        float after = MathF.Abs(track.Project(follow.Position).D - nLine);
        float held = MathF.Abs(
            track.Project(hold.Position).D -
            line.At(track.Sample, track.WrapS, track.Project(hold.Position).S).OffsetMeters
        );
        Assert.True(
            after < before || after < held,
            $"error before {before:F2} after {after:F2} held {held:F2} k={cmd.Curvature:F4}"
        );
    }

    [Fact]
    public void FasterThanThePlanAsksToBrake()
    {
        (TrackData track, GeometricRacingLine line, LineSpeedProfile profile, CarConfig car, TireConfig tires) =
            SimplePlan();
        GeometricRacingLine.Sample at = line.At(track.Sample, track.WrapS, 40f);
        float vStar = profile.At(track.WrapS, 40f);
        CarState state = OnTrack(at.Position, at.Heading, vStar + 8f, tires);
        var mpc = new LineMpc();
        LineMpc.Command cmd = mpc.Solve(
            state,
            car,
            tires,
            CarStrategy.Default,
            track.Sample,
            track.WrapS,
            track.Project,
            line,
            profile,
            _ => 0.88f,
            0.95f,
            30f
        );
        Assert.True(cmd.Accel < 0f, $"expected braking, ax={cmd.Accel:F2} v*={vStar:F1}");
    }

    private static (
        TrackData Track,
        GeometricRacingLine Line,
        LineSpeedProfile Profile,
        CarConfig Car,
        TireConfig Tires
    ) SimplePlan()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        GeometricRacingLine line = RuleLineAssets.LoadOrBake(RuleTrackPack.Simple, track, 0.95f);
        LineSpeedProfile profile = LineSpeedProfile.FromLine(
            line,
            track,
            includeRoadCurvature: false,
            windowMeters: 4f
        );
        var car = new CarConfig();
        var tires = DevelopmentTyres.Super;
        var state = new CarState { Energy = PowertrainState.Filled(0.9f) };
        foreach (TireState tire in new[] { state.FrontLeft, state.FrontRight, state.RearLeft, state.RearRight })
        {
            tire.SurfaceTempC = 90f;
            tire.CoreTempC = 90f;
        }

        profile.Recompute(state, car, tires, CarPhysics.GripUsage(CarStrategy.Default), 0.88f);
        return (track, line, profile, car, tires);
    }

    private static CarState OnTrack(Vector2 position, float heading, float speed, TireConfig tires)
    {
        CarState state = new()
        {
            Position = position,
            Heading = heading,
            Speed = speed,
            Energy = PowertrainState.Filled(0.9f)
        };
        state.InstallFreshTires();
        state.WarmTires(90f, 90f);
        foreach (TireState tire in new[] { state.FrontLeft, state.FrontRight, state.RearLeft, state.RearRight })
        {
            tire.SurfaceTempC = 90f;
            tire.CoreTempC = 90f;
        }

        return state;
    }
}
