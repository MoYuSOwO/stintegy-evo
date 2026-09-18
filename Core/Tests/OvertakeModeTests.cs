using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class OvertakeModeTests
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

    [Fact]
    public void NobodyCarriesTheModeOffTheGrid()
    {
        TrackData track = Loop.Value;
        float line = track.StartingLineS;
        RaceSimulation simulation = new(track);
        RaceCar leader = CreateStationaryCar(track, line + 60f, "lead");
        RaceCar chaser = CreateStationaryCar(track, line + 40f, "chase");
        simulation.AddCar(leader);
        simulation.AddCar(chaser);

        simulation.Step(1f / 120f);

        Assert.Equal(0f, chaser.State.OvertakeAssist);
        Assert.Equal(0f, leader.State.OvertakeAssist);
    }

    [Theory]
    [InlineData(18f)]
    [InlineData(70f)]
    public void CrossingTheLineNeverInventsAnOvertakeGrant(float gapMeters)
    {
        TrackData track = Loop.Value;
        float line = track.StartingLineS;
        RaceSimulation simulation = new(track);
        RaceCar leader = CreateStationaryCar(track, line + gapMeters, "lead");
        RaceCar chaser = CreateStationaryCar(track, line - 0.5f, "chase");
        simulation.AddCar(leader);
        simulation.AddCar(chaser);
        simulation.Step(1f / 120f);
        float before = chaser.Progress.RaceDistanceMeters;
        PlaceAt(track, chaser, line + 0.25f, speed: 20f);
        simulation.Step(1f / 120f);
        Assert.True(before < 0f && chaser.Progress.RaceDistanceMeters > 0f);
        Assert.Equal(0f, chaser.State.OvertakeAssist);
        Assert.Equal(0f, leader.State.OvertakeAssist);
    }

    [Fact]
    public void OnlyTheExternalHostChangesDeviceActivation()
    {
        TrackData track = Loop.Value;
        float line = track.StartingLineS;
        RaceCar car = CreateStationaryCar(track, line - 0.5f, "external");
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        car.State.OvertakeAssist = 0.65f;
        simulation.Step(1f / 120f);
        PlaceAt(track, car, line + 0.25f, speed: 20f);
        simulation.Step(1f / 120f);
        Assert.Equal(0.65f, car.State.OvertakeAssist);
        Assert.Equal(0.65f, simulation.CaptureFrame()[0].OvertakeAssist);
        car.State.OvertakeAssist = 0f;
        simulation.Step(1f / 120f);
        Assert.Equal(0f, car.State.OvertakeAssist);
    }

    [Theory]
    [InlineData(5005f, 4010f, 1000f, 5f)]
    [InlineData(4010f, 5005f, 1000f, 995f)]
    [InlineData(3010f, 5015f, 1000f, 5f)]
    public void TheCarAheadIsMeasuredOnTheRoadNotInTheStandings(
        float ownRaceDistance,
        float otherRaceDistance,
        float trackLength,
        float expectedAhead
    )
    {
        Assert.Equal(
            expectedAhead,
            RaceSimulation.OnTrackDistanceAhead(
                otherRaceDistance,
                ownRaceDistance,
                trackLength
            ),
            precision: 3
        );
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
            OvertakeAssist = 1f
        };
        CarState cleanAirAssisted = new() { OvertakeAssist = 1f };

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

    [Fact]
    public void DragTrimMakesTheAssistedCarCoverMoreGround()
    {
        float plain = SoloDistance(assist: false);
        float assisted = SoloDistance(assist: true);
        Assert.True(
            assisted > plain + 1f,
            "trimming a tenth of the aero drag must be worth whole metres " +
            $"over twenty seconds; got {assisted:0.0} m vs {plain:0.0} m"
        );
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

    private static float SoloDistance(bool assist)
    {
        TrackData track = Loop.Value;
        RaceSimulation simulation = new(track);
        RaceCar car = CreateCar(track, 40f, "solo");
        simulation.AddCar(car);

        for (int frame = 0; frame < 20 * 60; frame++)
        {
            simulation.Step(1f / 60f);
            // Host-controlled activation isolates the device's physical effect.
            if (assist)
                car.State.OvertakeAssist = 1f;
        }
        return car.Progress.RaceDistanceMeters;
    }

    private static RaceCar CreateStationaryCar(
        TrackData track,
        float s,
        string id
    )
    {
        TrackSample sample = track.Sample(s);
        return TestControlFixtures.ExternalCar(
            id,
            new CarState
            {
                Position = sample.Center,
                Heading = sample.Heading,
                Speed = 0f,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
    }

    private static void PlaceAt(
        TrackData track,
        RaceCar car,
        float s,
        float speed
    )
    {
        TrackSample sample = track.Sample(s);
        car.State.Position = sample.Center;
        car.State.Heading = sample.Heading;
        car.State.SideslipAngleRadians = 0f;
        car.State.YawRateRadiansPerSecond = 0f;
        car.State.SteerAngleRadians = 0f;
        car.State.Speed = speed;
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
