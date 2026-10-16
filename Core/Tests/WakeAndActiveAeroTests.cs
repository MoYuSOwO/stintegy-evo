using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class WakeAndActiveAeroTests
{
    private static readonly Lazy<TrackData> Loop = new(() =>
        new TrackBuilder(
                Vector2.Zero,
                startWidth: 12f
            )
            .AddStraight(1400f)
            .AddTurn(180f, 40f)
            .AddStraight(1400f)
            .AddTurn(180f, 40f)
            .CloseLoop()
            .Build(new TrackGridConfig())
    );

    /// <summary>
    /// The active aero is push-to-pass's, and nobody else's: it runs while a
    /// push runs and stops when the push does.
    /// </summary>
    [Fact]
    public void ActiveAeroRunsOnlyWhileAPushRuns()
    {
        TrackData track = Loop.Value;
        RaceSimulation simulation = new(track);
        RaceCar car = CreateCar(track, 100f, "solo");
        simulation.AddCar(car);

        simulation.Step(1f / 60f);
        Assert.Equal(0f, car.State.ActiveAero);

        car.Strategy = car.Strategy with { PushToPass = true };
        simulation.Step(1f / 60f);
        Assert.True(car.State.PushToPass.Active);
        Assert.Equal(1f, car.State.ActiveAero);
        Assert.Equal(1f, simulation.CaptureFrame()[0].ActiveAero);

        car.Strategy = car.Strategy with { PushToPass = false };
        simulation.Step(1f / 60f);
        Assert.Equal(0f, car.State.ActiveAero);
    }

    [Fact]
    public void DownforceRecoveryIsCappedAtCleanAir()
    {
        CarConfig config = new();
        CarState cleanAir = new();
        CarState inWake = new()
        {
            AirVelocityDeficit = 0.08f,
            WakeDownforceLoss = 0.05f
        };
        CarState inWakeAssisted = new()
        {
            AirVelocityDeficit = 0.08f,
            WakeDownforceLoss = 0.05f,
            ActiveAero = 1f
        };
        CarState cleanAirAssisted = new() { ActiveAero = 1f };

        float clean = CarPhysics.EffectiveDownforceAccelPerSpeedSquared(
            cleanAir,
            config
        );
        float wake = CarPhysics.EffectiveDownforceAccelPerSpeedSquared(
            inWake,
            config
        );
        float wakeAssisted = CarPhysics.EffectiveDownforceAccelPerSpeedSquared(
            inWakeAssisted,
            config
        );
        float cleanAssisted = CarPhysics.EffectiveDownforceAccelPerSpeedSquared(
            cleanAirAssisted,
            config
        );

        Assert.True(wake < wakeAssisted);
        Assert.True(wakeAssisted < clean);
        Assert.Equal(clean, cleanAssisted, precision: 6);
    }

    /// <summary>
    /// The active aero trims to dirty air and does nothing about drag. On a
    /// car whose powertrain has nothing extra to give, in clean air, a push
    /// changes nothing at all: same ground covered to the millimetre.
    /// </summary>
    [Fact]
    public void ActiveAeroIsNotADragReduction()
    {
        Assert.Equal(SoloDistance(push: false), SoloDistance(push: true), 3);
    }

    [Fact]
    public void WakeDownforceLossMatchesThePublishedShape()
    {
        // The decay lengths are calibrated against the published post-2022
        // downforce-loss figures: about 18 % at ten metres of clear gap and
        // about 4 % at twenty. This pins the constants to that claim so a
        // future retune cannot silently break the shape the comment cites.
        float lossAtTen = TotalDownforceLossAtGap(10f);
        float lossAtTwenty = TotalDownforceLossAtGap(20f);

        Assert.InRange(lossAtTen, 0.12f, 0.20f);
        Assert.InRange(lossAtTwenty, 0.015f, 0.05f);
        Assert.True(
            lossAtTen / MathF.Max(lossAtTwenty, 1e-6f) >= 3.5f,
            $"the loss must fall several-fold from 10 m to 20 m; " +
            $"got {lossAtTen:0.000} -> {lossAtTwenty:0.000}"
        );
    }

    [Fact]
    public void TheTwoFacesOfTheWakePartCompanyWithDistance()
    {
        // Forty metres back on a straight: the body still sits in slowed air
        // (the tow that makes long-straight slipstreaming a real tactic),
        // while the wings and floor have climbed out of the risen wake and
        // the downforce penalty is essentially gone.
        RaceCar chaser = ChaserAtGap(40f, new CarConfig());

        Assert.True(
            chaser.State.AirVelocityDeficit > 0.015f,
            $"the tow must survive at 40 m; got {chaser.State.AirVelocityDeficit:0.0000}"
        );
        float metAir = 1f - chaser.State.DownforceVelocityDeficit;
        float downforceLoss =
            1f - metAir * metAir * (1f - chaser.State.WakeDownforceLoss);
        Assert.True(
            downforceLoss < 0.01f,
            $"the downforce penalty must be gone at 40 m; got {downforceLoss:0.0000}"
        );
    }

    [Fact]
    public void AnInsensitiveCarShrugsOffDirtyAirButKeepsTheTow()
    {
        RaceCar chaser = ChaserAtGap(
            8f,
            new CarConfig { DirtyAirSensitivity = 0f }
        );

        Assert.True(
            chaser.State.AirVelocityDeficit > 0.04f,
            "the drag relief of a tow is universal"
        );
        Assert.Equal(0f, chaser.State.DownforceVelocityDeficit);
        Assert.Equal(0f, chaser.State.WakeDownforceLoss);
    }

    private static float TotalDownforceLossAtGap(float bodyGapMeters)
    {
        RaceCar chaser = ChaserAtGap(bodyGapMeters, new CarConfig());
        float metAir = 1f - chaser.State.DownforceVelocityDeficit;
        return 1f - metAir * metAir * (1f - chaser.State.WakeDownforceLoss);
    }

    private static RaceCar ChaserAtGap(float bodyGapMeters, CarConfig chaserConfig)
    {
        TrackData track = Loop.Value;
        RaceSimulation simulation = new(track);
        RaceCar leader = CreateCar(track, 100f, "lead");
        RaceCar chaser = CreateCar(track, 100f, "chase", chaserConfig);
        float centerDistance = bodyGapMeters +
                               (leader.Collision.LengthMeters +
                                chaser.Collision.LengthMeters) * 0.5f;
        TrackSample chaserSample = track.Sample(200f);
        TrackSample leaderSample = track.Sample(200f + centerDistance);
        chaser.State.Position = chaserSample.Center;
        chaser.State.Heading = chaserSample.Heading;
        leader.State.Position = leaderSample.Center;
        leader.State.Heading = leaderSample.Heading;
        simulation.AddCar(leader);
        simulation.AddCar(chaser);

        simulation.Step(1f / 60f);

        return chaser;
    }

    private static float SoloDistance(bool push)
    {
        TrackData track = Loop.Value;
        RaceSimulation simulation = new(track);
        RaceCar car = CreateCar(track, 40f, "solo", new CarConfig { Powertrain = ElectricPowertrain.Default });
        car.Strategy = car.Strategy with { PushToPass = push };
        simulation.AddCar(car);
        for (int frame = 0; frame < 12 * 60; frame++)
            simulation.Step(1f / 60f);
        Assert.Equal(push, car.State.PushToPass.Active);
        return car.Progress.RaceDistanceMeters;
    }

    private static RaceCar CreateCar(
        TrackData track,
        float s,
        string id,
        CarConfig? config = null
    )
    {
        TrackSample sample = track.Sample(s);
        return TestControlFixtures.ExternalCar(
            id,
            new CarState
            {
                Position = sample.Center,
                Heading = sample.Heading,
                Speed = 45f,
                Energy = PowertrainState.Filled(0.9f)
            },
            new DriverInput(0f, 2f),
            config
        );
    }
}
