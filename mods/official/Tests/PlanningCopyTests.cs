using System;
using Official.SingleTrack;
using StintegyEVO.Core.Cars;
using Xunit;

namespace Official.SingleTrack.Tests;

/// <summary>
/// A driver's planning copy: started from what the car was read doing
/// (<see cref="ICarMotion.Assume"/>), it reads back the same and drives on
/// as the car does.
/// </summary>
public sealed class PlanningCopyTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void ACopyStartedFromTheCarsReadingDrivesOnAsTheCarDoes()
    {
        var car = new SingleTrackCar(new CarConfig(), new TireConfig());
        var air = new CarAir(0.2f, 0.15f, 0.1f);
        CarState state = Fresh(car);
        // Into a corner off the throttle, behind another car: the wheels
        // winding on, the seat loaded, the air thin.
        for (int i = 0; i < 90; i++)
            car.Step(state, Input(new DriverInput(0.04f, i < 45 ? 2f : -6f)) with { Air = air }, Dt);
        CarMotion read = car.Motion(state);
        Assert.NotEqual(0f, read.WheelAngleRadians);
        Assert.NotEqual(0f, read.Curvature);
        Assert.NotEqual(read.LongitudinalAccel, read.StepLongitudinalAccel);

        CarState plan = Fresh(car);
        plan.Position = state.Position;
        plan.Heading = state.Heading;
        plan.SideslipAngleRadians = state.SideslipAngleRadians;
        plan.YawRateRadiansPerSecond = state.YawRateRadiansPerSecond;
        plan.Speed = state.Speed;
        car.Assume(plan, read, air);

        CarMotion copied = car.Motion(plan);
        Assert.Equal(read.WheelAngleRadians, copied.WheelAngleRadians);
        Assert.Equal(read.LongitudinalAccel, copied.LongitudinalAccel);
        Assert.Equal(read.LateralAccel, copied.LateralAccel);
        Assert.Equal(read.Curvature, copied.Curvature);
        Assert.Equal(read.StepLongitudinalAccel, copied.StepLongitudinalAccel);

        // A copy that was not told where the car was drives elsewhere.
        CarState blind = Fresh(car);
        blind.Position = state.Position;
        blind.Heading = state.Heading;
        blind.SideslipAngleRadians = state.SideslipAngleRadians;
        blind.YawRateRadiansPerSecond = state.YawRateRadiansPerSecond;
        blind.Speed = state.Speed;

        var next = Input(new DriverInput(0.04f, -6f));
        for (int i = 0; i < 30; i++)
        {
            car.Step(state, next, Dt);
            car.Step(plan, next, Dt);
            car.Step(blind, next, Dt);
        }
        float drift = (plan.Position - state.Position).Length();
        float blindDrift = (blind.Position - state.Position).Length();
        Assert.True(drift < blindDrift / 10f, $"the copy drifted {drift:F4} m from the car, the blind copy {blindDrift:F4} m");
    }

    [Fact]
    public void AssumingLeavesTheCarsHistoryAndTheAirWhenNoneIsGiven()
    {
        var car = new SingleTrackCar(new CarConfig(), new TireConfig());
        CarState plan = Fresh(car);
        car.Step(plan, Input(new DriverInput(0f, 0f)) with { Air = new CarAir(0.3f, 0.2f, 0.1f) }, Dt);
        float steadyBefore = car.EstimateCorneringLimit(plan, CarStrategy.Default, 50f);
        int spins = car.Motion(plan).Spins;

        car.Assume(plan, car.Motion(plan) with { Spins = spins + 3, Spinning = true });

        Assert.Equal(spins, car.Motion(plan).Spins);
        Assert.False(car.Motion(plan).Spinning);
        Assert.Equal(steadyBefore, car.EstimateCorneringLimit(plan, CarStrategy.Default, 50f));
    }

    private static CarState Fresh(SingleTrackCar car)
    {
        var state = new CarState { Speed = 45f };
        car.InstallFreshTires(state);
        car.Fill(state, PowertrainState.Filled(0.6f));
        car.WarmTires(state, 90f, 90f);
        return state;
    }

    private static CarStepInput Input(DriverInput input) => new(input, CarStrategy.Default, 25f);
}
