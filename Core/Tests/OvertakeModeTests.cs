using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Track.RefLines;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class OvertakeModeTests
{
    private static readonly Lazy<TrackData> Loop = new(() =>
        new TrackBuilder(
                Vector2.Zero,
                startWidth: 12f,
                refLineSolver: CenterLineRefLineSolver.Instance
            )
            .AddStraight(400f)
            .AddTurn(180f, 40f)
            .AddStraight(400f)
            .AddTurn(180f, 40f)
            .CloseLoop()
            .Build(new TrackGridConfig())
    );

    [Fact]
    public void NobodyCarriesTheModeOffTheGrid()
    {
        TrackData track = Loop.Value;
        RaceSimulation simulation = new(track);
        RaceCar leader = CreateCar(track, 60f, "lead");
        RaceCar chaser = CreateCar(track, 40f, "chase");
        simulation.AddCar(leader);
        simulation.AddCar(chaser);

        simulation.Step(1f / 60f);

        Assert.Equal(0f, chaser.State.OvertakeAssist);
        Assert.Equal(0f, leader.State.OvertakeAssist);
    }

    [Theory]
    [InlineData(18f, 1f)]
    [InlineData(70f, 0f)]
    public void TheLineDecidesByTimeGapToTheCarAhead(
        float gapMeters,
        float expectedAssist
    )
    {
        // The cars start on the opening straight and run a whole lap, so the
        // decision happens at a genuine crossing. The line sits at the exit
        // of the final corner, taken at roughly 30 m/s, which puts the one
        // second threshold near 30 m. The metric reads a little low there -
        // the leader is already accelerating away while the chaser is still
        // slow - so 18 m sits safely inside it and 70 m safely outside, and
        // the observed gap is cross-checked so a drifting approach cannot
        // silently invert the case.
        TrackData track = Loop.Value;
        float length = track.LengthMeters;
        RaceSimulation simulation = new(track);
        RaceCar leader = CreateCar(track, 40f + gapMeters, "lead");
        RaceCar chaser = CreateCar(track, 40f, "chase");
        simulation.AddCar(leader);
        simulation.AddCar(chaser);

        bool crossed = false;
        float gapSecondsAtCrossing = float.NaN;
        int crossings = Crossings(chaser, length);
        for (int frame = 0; frame < 60 * 60 && !crossed; frame++)
        {
            simulation.Step(1f / 60f);
            if (Crossings(chaser, length) > crossings)
            {
                crossed = true;
                gapSecondsAtCrossing = RaceSimulation.OnTrackDistanceAhead(
                    leader.Progress.RaceDistanceMeters,
                    chaser.Progress.RaceDistanceMeters,
                    length
                ) / MathF.Max(chaser.State.Speed, 1f);
            }
        }

        Assert.True(crossed, "the chaser never crossed the line");
        Assert.True(
            expectedAssist > 0.5f
                ? gapSecondsAtCrossing < 0.9f
                : gapSecondsAtCrossing > 1.1f,
            $"approach drifted to an ambiguous {gapSecondsAtCrossing:0.00} s gap"
        );
        Assert.Equal(expectedAssist, chaser.State.OvertakeAssist);
        Assert.Equal(0f, leader.State.OvertakeAssist);
    }

    [Fact]
    public void TheGrantLatchesForTheLapAndExpiresAtTheNextCrossing()
    {
        TrackData track = Loop.Value;
        float length = track.LengthMeters;
        RaceSimulation simulation = new(track);
        RaceCar leader = CreateCar(track, 55f, "lead");
        RaceCar chaser = CreateCar(track, 40f, "chase");
        simulation.AddCar(leader);
        simulation.AddCar(chaser);

        // Lap one at equal pace, so the chaser earns the mode at its first
        // crossing. Then the leader gets full power while the chaser is
        // turned down, the gap opens well past one second, and the grant must
        // hold anyway until the next crossing takes it away.
        int startCrossings = Crossings(chaser, length);
        bool earned = false;
        bool sawWideGapWithModeStillOn = false;
        bool expired = false;
        for (int frame = 0; frame < 120 * 60 && !expired; frame++)
        {
            simulation.Step(1f / 60f);
            int lapsDone = Crossings(chaser, length) - startCrossings;
            if (lapsDone == 1 && !earned)
            {
                Assert.Equal(1f, chaser.State.OvertakeAssist);
                earned = true;
                leader.Strategy = new CarStrategy(
                    TireUsageMode.Attack,
                    PowerOutputMode.Attack
                );
                chaser.Strategy = new CarStrategy(
                    TireUsageMode.Protect,
                    PowerOutputMode.Save
                );
            }
            float gapSeconds = RaceSimulation.OnTrackDistanceAhead(
                leader.Progress.RaceDistanceMeters,
                chaser.Progress.RaceDistanceMeters,
                length
            ) / MathF.Max(chaser.State.Speed, 1f);
            if (lapsDone == 1 && gapSeconds > 1.1f)
            {
                Assert.Equal(1f, chaser.State.OvertakeAssist);
                sawWideGapWithModeStillOn = true;
            }
            if (lapsDone >= 2)
            {
                expired = true;
                Assert.Equal(0f, chaser.State.OvertakeAssist);
            }
        }

        Assert.True(earned, "the mode was never earned at the first crossing");
        Assert.True(
            sawWideGapWithModeStillOn,
            "the gap never opened past 1.1 s while the mode was held"
        );
        Assert.True(expired, "the chaser never reached its second crossing");
    }

    private static int Crossings(RaceCar car, float trackLength) =>
        (int)MathF.Floor(car.Progress.RaceDistanceMeters / trackLength);

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
        chaser.State.Position = chaserSample.RefPosition;
        chaser.State.Heading = chaserSample.RefHeading;
        leader.State.Position = leaderSample.RefPosition;
        leader.State.Heading = leaderSample.RefHeading;
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
            // A solo car never qualifies at the line, so hold the flag by
            // hand to isolate the physics of the mode itself.
            if (assist)
                car.State.OvertakeAssist = 1f;
        }
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
        return new RaceCar(
            id,
            config ?? new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            new ReferenceLineDriver(
                new VehicleSpeedPlanningConfig(),
                new DriverProfile(
                    id,
                    new DriverAbilities { Pace = 100f },
                    // One seed for the whole field: correlated form noise
                    // keeps a same-pace pair at a stable gap, which is what
                    // these scenarios rely on.
                    randomSeed: 7
                )
            ),
            new CarState
            {
                Position = sample.RefPosition,
                Heading = sample.RefHeading,
                Speed = 45f,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
    }
}
