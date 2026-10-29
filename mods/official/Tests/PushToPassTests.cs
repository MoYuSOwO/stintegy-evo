using System;
using System.Numerics;
using Official.SingleTrack;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace Official.SingleTrack.Tests;

/// <summary>
/// Push-to-pass: when it may run (the button, the cap, the cooldown, the
/// budget, race control's lock) and what it does (more power, the generator
/// covering half of it, never cheaper on the pack than not pushing).
/// </summary>
public sealed class PushToPassTests
{
    private const float Dt = 1f / 60f;
    private static readonly PushToPassRules Rules = new();

    private static PushToPassState Hold(PushToPassState state, bool held, float seconds)
    {
        for (int i = 0; i < (int)MathF.Round(seconds / Dt); i++)
            state = PushToPassState.Advance(state, Rules, held, Dt);
        return state;
    }

    [Fact]
    public void HoldingTheButtonPushesForFifteenSecondsAndNoMore()
    {
        PushToPassState state = PushToPassState.Advance(default, Rules, true, Dt);
        Assert.True(state.Active);
        state = Hold(state, true, 14.9f);
        Assert.True(state.Active);
        state = Hold(state, true, 0.2f);
        Assert.False(state.Active);
        Assert.Equal(15f, state.UsedSeconds, 2);

        // Still held long past the cooldown: the cap does not chain into a second push.
        state = Hold(state, true, 60f);
        Assert.False(state.Active);
        Assert.Equal(15f, state.UsedSeconds, 2);
    }

    [Fact]
    public void LettingGoEndsThePushAndTheNextWaitsThirtySeconds()
    {
        PushToPassState state = Hold(default, true, 4f);
        state = PushToPassState.Advance(state, Rules, false, Dt);
        Assert.False(state.Active);
        Assert.Equal(4f, state.UsedSeconds, 2);

        state = Hold(state, true, 29.5f);
        Assert.False(state.Active);
        Assert.Equal(4f, state.UsedSeconds, 2);

        state = Hold(state, true, 0.6f);
        Assert.True(state.Active);
    }

    [Fact]
    public void TheRaceHasTwoHundredSecondsAndThenNone()
    {
        PushToPassState state = default;
        for (int push = 0; push < 20; push++)
        {
            state = Hold(state, true, 16f);
            state = Hold(state, false, 31f);
        }
        Assert.Equal(200f, state.UsedSeconds, 2);
        Assert.Equal(0f, state.RemainingSeconds(Rules));
        state = Hold(state, true, 1f);
        Assert.False(state.Active);
    }

    [Fact]
    public void ALockHoldsThePushOffAndEndsOneThatIsRunning()
    {
        PushToPassState locked = Hold(new PushToPassState { Locked = true }, true, 2f);
        Assert.False(locked.Active);
        Assert.Equal(0f, locked.UsedSeconds);

        PushToPassState running = Hold(default, true, 3f);
        running.Locked = true;
        running = PushToPassState.Advance(running, Rules, true, Dt);
        Assert.False(running.Active);
        Assert.Equal(Rules.CooldownSeconds, running.CooldownSeconds, 2);
    }

    [Fact]
    public void CrossingTheLineLiftsRaceControlsLock()
    {
        TrackData track = new TrackBuilder(Vector2.Zero, startWidth: 12f)
            .AddStraight(600f).AddTurn(180f, 40f).AddStraight(600f).AddTurn(180f, 40f)
            .CloseLoop().Build(new TrackGridConfig());
        TrackSample start = track.Sample(track.StartingLineS - 15f);
        RaceCar car = TestControlFixtures.ExternalCar(
            "grid",
            new CarState { Position = start.Center, Heading = start.Heading, Speed = 20f }.WithEnergy(PowertrainState.Filled(0.9f)),
            new DriverInput(0f, 2f));
        car.Strategy = car.Strategy.WithButton(0, true);
        var simulation = new RaceSimulation(track);
        simulation.AddCar(car);
        car.HoldButtonsUntilLine();

        simulation.Step(0.25f);
        Assert.True(car.State.Chassis().PushToPass.Locked);
        Assert.False(car.State.Chassis().PushToPass.Active);

        for (int i = 0; i < 120 && car.Progress.RaceDistanceMeters < 1f; i++)
            simulation.Step(Dt);
        simulation.Step(Dt);
        Assert.False(car.State.Chassis().PushToPass.Locked);
        Assert.True(car.State.Chassis().PushToPass.Active);
        Assert.True(simulation.CaptureFrame()[0].Buttons[0].Running);
    }

    [Fact]
    public void APushIsTwelvePerCentMorePowerWherePowerIsTheLimit()
    {
        var powertrain = RangeExtenderPowertrain.Default;
        PowertrainState energy = PowertrainState.Filled(0.8f);
        // Fast enough that power, not the chassis, sets the limit.
        float plain = powertrain.DriveAccelerationLimit(energy, powertrain.OutputLadder.DefaultRung, 70f, 950f, 100f);
        float pushed = powertrain.DriveAccelerationLimit(energy, powertrain.OutputLadder.DefaultRung, 70f, 950f, 100f, pushToPass: true);
        Near.Equal(1.12f, pushed / plain);
    }

    /// <summary>
    /// The generator answers half of what the motors draw above the race
    /// limit, so the pack pays the other half: a push at full power always
    /// costs more charge, and more fuel, than full power without one.
    /// </summary>
    [Fact]
    public void TheGeneratorCoversHalfOfThePushAndThePackTheRest()
    {
        var powertrain = RangeExtenderPowertrain.Default;
        PowertrainState energy = PowertrainState.Filled(0.8f);
        const float speed = 70f, mass = 950f, dt = 1f;
        float raceAccel = powertrain.RaceDrivePowerLimitWatts / (mass * speed);
        float pushAccel = powertrain.PushToPassDrivePowerWatts / (mass * speed);

        PowertrainState flatOut = powertrain.Settle(energy, raceAccel, 0f, speed, mass, dt).Energy;
        PowertrainState pushed = powertrain.Settle(energy, pushAccel, 0f, speed, mass, dt, pushToPass: true).Energy;
        // The same power without the push flag: what the motors' extra draw costs with no generator help.
        PowertrainState unaided = powertrain.Settle(energy, pushAccel, 0f, speed, mass, dt).Energy;

        float extraDraw = (powertrain.PushToPassDrivePowerWatts - powertrain.RaceDrivePowerLimitWatts) /
                          powertrain.BatteryDriveEfficiency;
        float packJoules = powertrain.BatteryCapacityJoules;
        Near.Equal(extraDraw * dt / packJoules, flatOut.Primary - unaided.Primary, abs: 0f);
        Near.Equal(0.5f * extraDraw * dt / packJoules, flatOut.Primary - pushed.Primary, abs: 0f);
        Assert.True(pushed.Secondary < flatOut.Secondary, "the generator burns fuel for its share");

        // And a push that never reaches the race limit asks nothing extra of the generator.
        PowertrainState halfThrottle = powertrain.Settle(energy, raceAccel * 0.5f, 0f, speed, mass, dt, pushToPass: true).Energy;
        PowertrainState halfPlain = powertrain.Settle(energy, raceAccel * 0.5f, 0f, speed, mass, dt).Energy;
        Assert.Equal(halfPlain, halfThrottle);
    }
}
