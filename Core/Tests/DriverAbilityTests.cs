using System;
using TheStint.Core.Cars;
using TheStint.Core.Drivers;
using TheStint.Core.Racing;
using TheStint.Core.Track;
using Xunit;

namespace TheStint.Core.Tests;

public sealed class DriverAbilityTests
{
    [Fact]
    public void RatingsMustStayWithinManagerScale()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new DriverProfile(
                "invalid",
                new DriverAbilities { Pace = 101f }
            )
        );
    }

    [Fact]
    public void TireManagementMapsToSmallBoundedEnergyCorrection()
    {
        ReferenceLineDriver poor = CreateInitializedDriver(
            new DriverAbilities { TireManagement = 0f }
        );
        ReferenceLineDriver neutral = CreateInitializedDriver(
            new DriverAbilities { TireManagement = 80f }
        );
        ReferenceLineDriver excellent = CreateInitializedDriver(
            new DriverAbilities { TireManagement = 100f }
        );

        Assert.InRange(poor.TireEnergyEfficiency, 1.039f, 1.041f);
        Assert.InRange(neutral.TireEnergyEfficiency, 0.999f, 1.001f);
        Assert.InRange(excellent.TireEnergyEfficiency, 0.969f, 0.971f);
    }

    [Fact]
    public void LowerPaceReducesPlannedAndExecutedCapability()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar elite = CreateCar(
            track,
            "pace-elite",
            new DriverAbilities { Pace = 100f }
        );
        RaceCar developing = CreateCar(
            track,
            "pace-developing",
            new DriverAbilities { Pace = 45f }
        );
        RaceSimulation eliteSimulation = new(track);
        RaceSimulation developingSimulation = new(track);
        eliteSimulation.AddCar(elite);
        developingSimulation.AddCar(developing);

        eliteSimulation.Step(1f / 60f);
        developingSimulation.Step(1f / 60f);

        ReferenceLineDriver eliteDriver = (ReferenceLineDriver)elite.Driver;
        ReferenceLineDriver developingDriver = (ReferenceLineDriver)developing.Driver;
        Assert.True(
            developingDriver.LastTelemetry.PaceEfficiency <
            eliteDriver.LastTelemetry.PaceEfficiency
        );
        Assert.True(
            MinimumTargetSpeed(developingDriver.CurrentSpeedProfile!) <
            MinimumTargetSpeed(eliteDriver.CurrentSpeedProfile!)
        );
    }

    [Fact]
    public void SameSeedAndIdentityReproduceDriverForms()
    {
        DriverAbilities abilities = new()
        {
            Pace = 65f,
            Consistency = 35f,
            CarControl = 60f,
            TireManagement = 70f,
            Adaptability = 55f
        };
        TrackData firstTrack = TrackFactory.SimpleTestTrack();
        TrackData secondTrack = TrackFactory.SimpleTestTrack();
        RaceCar first = CreateCar(firstTrack, "same-car", abilities, profileId: "same-driver", seed: 42);
        RaceCar second = CreateCar(secondTrack, "same-car", abilities, profileId: "same-driver", seed: 42);
        RaceSimulation firstSimulation = new(firstTrack);
        RaceSimulation secondSimulation = new(secondTrack);
        firstSimulation.AddCar(first);
        secondSimulation.AddCar(second);

        for (int i = 0; i < 1200; i++)
        {
            firstSimulation.Step(1f / 60f);
            secondSimulation.Step(1f / 60f);
        }

        ReferenceLineDriverTelemetry a = ((ReferenceLineDriver)first.Driver).LastTelemetry;
        ReferenceLineDriverTelemetry b = ((ReferenceLineDriver)second.Driver).LastTelemetry;
        Assert.Equal(a.SessionForm, b.SessionForm);
        Assert.Equal(a.LapForm, b.LapForm);
        Assert.Equal(a.SegmentForm, b.SegmentForm);
        Assert.Equal(a.EffectivePace, b.EffectivePace);
        Assert.Equal(first.State.Position, second.State.Position);
        Assert.Equal(first.State.Speed, second.State.Speed);
    }

    [Fact]
    public void LowConsistencyCanRollAboveMeanAndHasWiderSpread()
    {
        (float lowMinimum, float lowMaximum) = ObservePaceRange(35f);
        (float highMinimum, float highMaximum) = ObservePaceRange(100f);

        Assert.True(lowMaximum > 0.65f, $"expected a high roll above mean, got {lowMaximum:0.000}");
        Assert.True(
            lowMaximum - lowMinimum > highMaximum - highMinimum + 0.03f,
            $"expected wider low-consistency spread, low={lowMinimum:0.000}-{lowMaximum:0.000}, " +
            $"high={highMinimum:0.000}-{highMaximum:0.000}"
        );
    }

    [Fact]
    public void TireEnergyEfficiencyChangesHeatAndWearButNotInstantaneousForces()
    {
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        CarConfig config = new();
        CarState efficient = NewState(tires);
        CarState wasteful = NewState(tires);
        DriverInput command = new(0.012f, 2f);

        CarPhysics.Step(
            efficient,
            config,
            tires,
            new CarPhysicsStepInput(command, CarStrategy.Default, 25f, 35f, 0.97f),
            1f / 60f
        );
        CarPhysics.Step(
            wasteful,
            config,
            tires,
            new CarPhysicsStepInput(command, CarStrategy.Default, 25f, 35f, 1.04f),
            1f / 60f
        );

        Assert.Equal(
            efficient.Telemetry.ActualLateralAccel,
            wasteful.Telemetry.ActualLateralAccel
        );
        Assert.Equal(
            efficient.Telemetry.ActualLongitudinalAccel,
            wasteful.Telemetry.ActualLongitudinalAccel
        );
        Assert.True(wasteful.FrontLeft.SurfaceTempC > efficient.FrontLeft.SurfaceTempC);
        Assert.True(wasteful.FrontLeft.Wear > efficient.FrontLeft.Wear);
    }

    [Fact]
    public void AdaptabilityControlsGripEstimateLagWithoutChangingActualGrip()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar slow = CreateCar(
            track,
            "slow-adaptation",
            new DriverAbilities { Adaptability = 0f }
        );
        RaceCar instant = CreateCar(
            track,
            "instant-adaptation",
            new DriverAbilities { Adaptability = 100f }
        );
        RaceSimulation slowSimulation = new(track);
        RaceSimulation instantSimulation = new(track);
        slowSimulation.AddCar(slow);
        instantSimulation.AddCar(instant);
        SetAllTireTemperatures(slow.State, 25f);
        SetAllTireTemperatures(instant.State, 25f);

        slowSimulation.Step(1f / 60f);
        instantSimulation.Step(1f / 60f);

        ReferenceLineDriverTelemetry slowTelemetry =
            ((ReferenceLineDriver)slow.Driver).LastTelemetry;
        ReferenceLineDriverTelemetry instantTelemetry =
            ((ReferenceLineDriver)instant.Driver).LastTelemetry;
        Assert.True(slowTelemetry.EstimatedGripScale > 1.02f);
        Assert.InRange(instantTelemetry.EstimatedGripScale, 0.999f, 1.001f);
        Assert.InRange(
            MathF.Abs(slowTelemetry.ActualGrip - instantTelemetry.ActualGrip),
            0f,
            1e-5f
        );
    }

    [Fact]
    public void CarControlChangesOnlyRecoveryCommandsDuringPhysicalInstability()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar developing = CreateCar(
            track,
            "developing-control",
            new DriverAbilities { CarControl = 25f }
        );
        RaceCar elite = CreateCar(
            track,
            "elite-control",
            new DriverAbilities { CarControl = 100f }
        );
        developing.State.SideslipAngleRadians = -0.08f;
        developing.State.YawRateRadiansPerSecond = 1.2f;
        elite.State.SideslipAngleRadians = -0.08f;
        elite.State.YawRateRadiansPerSecond = 1.2f;
        RaceSimulation developingSimulation = new(track);
        RaceSimulation eliteSimulation = new(track);
        developingSimulation.AddCar(developing);
        eliteSimulation.AddCar(elite);

        developingSimulation.Step(1f / 60f);
        eliteSimulation.Step(1f / 60f);

        ReferenceLineDriverTelemetry developingTelemetry =
            ((ReferenceLineDriver)developing.Driver).LastTelemetry;
        ReferenceLineDriverTelemetry eliteTelemetry =
            ((ReferenceLineDriver)elite.Driver).LastTelemetry;
        Assert.True(developingTelemetry.ControlSeverity > 0f);
        Assert.True(eliteTelemetry.ControlSeverity > 0f);
        Assert.True(eliteTelemetry.EffectiveControl > developingTelemetry.EffectiveControl);
        Assert.NotEqual(0f, developingTelemetry.ControlCurvatureCorrection);
        Assert.NotEqual(0f, eliteTelemetry.ControlCurvatureCorrection);
    }

    [Fact]
    public void OffsetGridLaunchKeepsRecoveryAccelerationWithRandomPaceError()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        DriverAbilities abilities = new()
        {
            Pace = 80f,
            Consistency = 90f,
            CarControl = 85f,
            TireManagement = 100f,
            Adaptability = 90f
        };
        const ulong seed = 9895077375579661904UL;
        ReferenceLineDriver driver = new(
            new DriverProfile("tire_whisperer", abilities, seed)
        );
        RaceCar car = CreateCar(
            track,
            $"tire_whisperer-{seed}",
            driver
        );
        car.State.Speed = 0f;
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        simulation.Step(1f);

        Assert.True(
            car.State.Speed > 2f,
            $"offset grid launch should accelerate, got {car.State.Speed:0.000} m/s"
        );
        Assert.True(driver.LastTelemetry.ReferenceAcceleration > 0f);
        Assert.True(driver.LastTelemetry.CorrectionDecayDistanceMeters > 0f);
    }

    private static (float Minimum, float Maximum) ObservePaceRange(float consistency)
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(
            track,
            $"range-{consistency}",
            new DriverAbilities
            {
                Pace = 60f,
                Consistency = consistency
            },
            seed: 981723
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        float minimum = 1f;
        float maximum = 0f;
        int previousSegment = -1;

        while (car.Progress.Lap < 2 && simulation.RaceTimeSeconds < 180f)
        {
            simulation.Step(1f / 30f);
            int segment = (int)(car.Progress.CurrentS / 40f);
            if (segment == previousSegment)
                continue;
            previousSegment = segment;
            float pace = ((ReferenceLineDriver)car.Driver).LastTelemetry.EffectivePace;
            minimum = MathF.Min(minimum, pace);
            maximum = MathF.Max(maximum, pace);
        }

        return (minimum, maximum);
    }

    private static ReferenceLineDriver CreateInitializedDriver(DriverAbilities abilities)
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver driver = new(
            new DriverProfile("mapping", abilities, randomSeed: 5)
        );
        RaceCar car = CreateCar(track, "mapping-car", driver);
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        return driver;
    }

    private static RaceCar CreateCar(
        TrackData track,
        string carId,
        DriverAbilities abilities,
        string profileId = "test-driver",
        ulong seed = 1
    )
    {
        return CreateCar(
            track,
            carId,
            new ReferenceLineDriver(new DriverProfile(profileId, abilities, seed))
        );
    }

    private static RaceCar CreateCar(
        TrackData track,
        string carId,
        ReferenceLineDriver driver
    )
    {
        TrackSample start = track.Sample(track.Grids[1].S);
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        return new RaceCar(
            carId,
            new CarConfig(),
            tires,
            driver,
            new CarState
            {
                Position = track.Grids[1].Position,
                Heading = start.RefHeading,
                Speed = 8f,
                BatterySoc = 0.9f
            }
        );
    }

    private static CarState NewState(TireConfig tires)
    {
        CarState state = new()
        {
            Speed = 25f,
            BatterySoc = 1f
        };
        state.InstallFreshTires(tires);
        return state;
    }

    private static void SetAllTireTemperatures(CarState state, float temperatureC)
    {
        foreach (WheelId wheel in Enum.GetValues<WheelId>())
        {
            TireState tire = state.GetTire(wheel);
            tire.SurfaceTempC = temperatureC;
            tire.CoreTempC = temperatureC;
        }
    }

    private static float MinimumTargetSpeed(VehicleSpeedProfile profile)
    {
        float minimum = float.PositiveInfinity;
        for (int i = 0; i < profile.Count; i++)
            minimum = MathF.Min(minimum, profile[i].TargetSpeed);
        return minimum;
    }
}
