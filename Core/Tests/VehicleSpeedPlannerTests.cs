using System;
using System.Numerics;
using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;
using Xunit;

namespace TheStint.Core.Tests;

public sealed class VehicleSpeedPlannerTests
{
    [Fact]
    public void ProfileBrakesBeforeTightCorners()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedProfile profile = new VehicleSpeedPlanner().Plan(car, track);

        float minimumSpeed = float.PositiveInfinity;
        float maximumSpeed = 0f;
        bool hasBraking = false;
        for (int i = 0; i < profile.Count; i++)
        {
            minimumSpeed = MathF.Min(minimumSpeed, profile[i].TargetSpeed);
            maximumSpeed = MathF.Max(maximumSpeed, profile[i].TargetSpeed);
            hasBraking |= profile[i].ReferenceAcceleration < -0.5f;
        }

        Assert.True(maximumSpeed - minimumSpeed > 5f);
        Assert.True(hasBraking);
    }

    [Fact]
    public void ProfileUsesEachCarsStrategyAndCondition()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar protect = CreateCar(
            track,
            new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Save)
        );
        RaceCar attack = CreateCar(
            track,
            new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Attack)
        );
        VehicleSpeedPlanner planner = new();

        VehicleSpeedProfile protectProfile = planner.Plan(protect, track);
        VehicleSpeedProfile attackProfile = planner.Plan(attack, track);

        Assert.True(AverageSpeed(attackProfile) > AverageSpeed(protectProfile));
    }

    [Fact]
    public void ReferenceAccelerationMatchesAdjacentProfileSpeed()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedProfile profile = new VehicleSpeedPlanner().Plan(car, track);

        int index = FindModerateAccelerationPoint(profile, track);
        Assert.True(index >= 0, "profile should contain a non-saturated acceleration segment");

        int next = (index + 1) % profile.Count;
        TrackSample sample = track.Sample(index * profile.StepLengthMeters);
        TrackSample nextSample = track.Sample(next * profile.StepLengthMeters);
        float distance = Vector2.Distance(sample.RefPosition, nextSample.RefPosition);
        float startSpeed = profile[index].TargetSpeed;
        float targetSpeed = profile[next].TargetSpeed;
        float expected = (targetSpeed * targetSpeed - startSpeed * startSpeed) /
                         (2f * distance);

        Assert.InRange(
            MathF.Abs(profile[index].ReferenceAcceleration - expected),
            0f,
            1e-4f
        );
    }

    [Fact]
    public void CurvatureCorrectionEnvelopeLowersSpeedOnlyForExtraCurvature()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedPlanner planner = new();
        VehicleSpeedProfile global = planner.Plan(car, track);
        float s = track.Grids[1].S;

        CurvatureCorrectionSpeedPlan baseline = planner.PlanCurvatureCorrection(
            car,
            track,
            global,
            s,
            curvatureCorrection: 0f,
            commandedCurvature: 0f
        );
        CurvatureCorrectionSpeedPlan corrected = planner.PlanCurvatureCorrection(
            car,
            track,
            global,
            s,
            curvatureCorrection: 0.08f,
            commandedCurvature: 0.08f
        );

        Assert.True(corrected.Current.TargetSpeed < baseline.Current.TargetSpeed);
        Assert.True(corrected.MaximumAbsoluteCurvature >= 0.08f);
    }

    [Fact]
    public void CurvatureCorrectionUsesInstantaneousAccelerationAtLaunch()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        car.State.Speed = 0f;
        VehicleSpeedPlanner planner = new();
        VehicleSpeedProfile global = planner.Plan(car, track);

        CurvatureCorrectionSpeedPlan plan = planner.PlanCurvatureCorrection(
            car,
            track,
            global,
            track.Grids[1].S,
            curvatureCorrection: 0.08f,
            commandedCurvature: 0.08f
        );
        float segmentAverage = (
            plan.NextTargetSpeed * plan.NextTargetSpeed -
            plan.Current.TargetSpeed * plan.Current.TargetSpeed
        ) / (2f * plan.FirstSegmentLengthMeters);
        float fullAvailableDrive = car.CarConfig.GetBatteryForceAccelLimit(
            car.Strategy.BatteryMode
        );

        Assert.InRange(fullAvailableDrive - plan.Current.ReferenceAcceleration, 0f, 0.02f);
        Assert.True(plan.Current.ReferenceAcceleration > segmentAverage);
    }

    [Fact]
    public void AttackBatteryLaunchIsNotCappedAtFourMetersPerSecondSquared()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(
            track,
            new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Attack)
        );
        car.State.Speed = 0f;
        VehicleSpeedPlanner planner = new();
        VehicleSpeedProfile global = planner.Plan(car, track);

        CurvatureCorrectionSpeedPlan plan = planner.PlanCurvatureCorrection(
            car,
            track,
            global,
            track.Grids[1].S,
            curvatureCorrection: 0.08f,
            commandedCurvature: 0.08f
        );
        float fullAvailableDrive = car.CarConfig.GetBatteryForceAccelLimit(
            car.Strategy.BatteryMode
        );

        Assert.True(plan.Current.ReferenceAcceleration > 4f);
        Assert.InRange(fullAvailableDrive - plan.Current.ReferenceAcceleration, 0f, 0.02f);
    }

    [Fact]
    public void DriveAccelerationUsageScalesCalculatedLaunchCapability()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        car.State.Speed = 0f;
        VehicleSpeedPlanner planner = new(
            new VehicleSpeedPlanningConfig { DriveAccelerationUsage = 0.8f }
        );
        VehicleSpeedProfile global = planner.Plan(car, track);

        CurvatureCorrectionSpeedPlan plan = planner.PlanCurvatureCorrection(
            car,
            track,
            global,
            track.Grids[1].S,
            curvatureCorrection: 0.08f,
            commandedCurvature: 0.08f
        );
        float expected = car.CarConfig.GetBatteryForceAccelLimit(
            car.Strategy.BatteryMode
        ) * 0.8f;

        Assert.InRange(MathF.Abs(plan.Current.ReferenceAcceleration - expected), 0f, 0.02f);
    }

    private static RaceCar CreateCar(TrackData track, CarStrategy strategy)
    {
        TrackSample start = track.Sample(track.Grids[1].S);
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        RaceCar car = new(
            "planner-test",
            new CarConfig(),
            tires,
            new ReferenceLineDriver(),
            new CarState
            {
                Position = start.RefPosition,
                Heading = start.RefHeading,
                Speed = 20f,
                BatterySoc = 0.8f
            }
        )
        {
            Strategy = strategy
        };
        return car;
    }

    private static float AverageSpeed(VehicleSpeedProfile profile)
    {
        float sum = 0f;
        for (int i = 0; i < profile.Count; i++)
            sum += profile[i].TargetSpeed;
        return sum / Math.Max(profile.Count, 1);
    }

    private static int FindModerateAccelerationPoint(VehicleSpeedProfile profile, TrackData track)
    {
        for (int i = 0; i < profile.Count; i++)
        {
            float command = profile[i].ReferenceAcceleration;
            float curvature = MathF.Abs(track.Sample(i * profile.StepLengthMeters).RefCurvature);
            if (command > 0.4f && command < 3.5f && curvature < 0.01f)
                return i;
        }
        return -1;
    }
}
