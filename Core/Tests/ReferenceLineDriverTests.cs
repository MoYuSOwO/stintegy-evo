using System.Numerics;
using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;
using Xunit;

namespace TheStint.Core.Tests;

public sealed class ReferenceLineDriverTests
{
    [Fact]
    public void DriverCompletesALapOnSimpleTestTrack()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample start = track.Sample(track.Grids[1].S);
        RaceCar car = new(
            "lap-test",
            new CarConfig(),
            WarmTires(),
            new ReferenceLineDriver(),
            new CarState
            {
                Position = start.RefPosition,
                Heading = start.RefHeading,
                Speed = 8f,
                BatterySoc = 0.9f
            }
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        const float dt = 1f / 60f;
        int wallContactFrames = 0;
        string? firstWall = null;
        for (int i = 0; i < 60 * 150; i++)
        {
            simulation.Step(dt);
            if (car.LastBoundaryContact.HasValue)
            {
                wallContactFrames++;
                ReferenceLineDriver driver = (ReferenceLineDriver)car.Driver;
                firstWall ??=
                    $"firstWall t={simulation.RaceTimeSeconds:0.00}, s={car.Progress.CurrentS:0.0}, " +
                    $"d={car.Progress.CurrentD:0.0}, speed={car.State.Speed:0.0}, " +
                    $"error={driver.LastTelemetry.LateralErrorMeters:0.00}, " +
                    $"correction={driver.LastTelemetry.CurvatureCorrection:0.000}, " +
                    $"target={driver.LastTelemetry.TargetSpeed:0.0}";
            }
        }

        Assert.True(
            car.Progress.TotalDistance >= track.LengthMeters,
            $"expected a completed lap, got {car.Progress.TotalDistance:0.0}/{track.LengthMeters:0.0} m; " +
            $"s={car.Progress.CurrentS:0.0}, d={car.Progress.CurrentD:0.0}, " +
            $"speed={car.State.Speed:0.0}, region={car.Progress.Region}, " +
            $"soc={car.State.BatterySoc:0.000}, wall={car.LastBoundaryContact.HasValue}"
        );
        Assert.True(
            wallContactFrames == 0,
            $"expected a clean lap, got {wallContactFrames} wall-contact frames; " +
            $"s={car.Progress.CurrentS:0.0}, d={car.Progress.CurrentD:0.0}, " +
            $"speed={car.State.Speed:0.0}, error={((ReferenceLineDriver)car.Driver).LastTelemetry.LateralErrorMeters:0.00}, " +
            $"target={((ReferenceLineDriver)car.Driver).LastTelemetry.TargetSpeed:0.0}; {firstWall}"
        );
    }

    [Fact]
    public void LateralErrorKeepsGlobalLineAndBuildsCurvatureSpeedEnvelope()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver driver = new();
        RaceDriverFrameContext context = CreateContext(
            track,
            driver,
            s: 20f,
            speed: 16f,
            lateralError: 2f
        );

        driver.GetControl(in context, 1f / 60f);

        ReferenceLineDriverTelemetry telemetry = driver.LastTelemetry;
        Assert.True(MathF.Abs(telemetry.CurvatureCorrection) > 1e-3f);
        Assert.True(telemetry.CorrectionDecayDistanceMeters >= 15f);
        Assert.True(
            telemetry.CorrectionEnvelopeMaximumCurvature >=
            MathF.Abs(telemetry.DesiredCurvature) - 1e-4f
        );
        Assert.True(telemetry.TargetSpeed <= telemetry.GlobalProfileTargetSpeed + 1e-4f);
    }

    [Fact]
    public void AlignedCarUsesUnmodifiedGlobalSpeedProfile()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver driver = new();
        RaceDriverFrameContext context = CreateContext(track, driver, s: 20f, speed: 16f);

        driver.GetControl(in context, 1f / 60f);

        Assert.InRange(MathF.Abs(driver.LastTelemetry.CurvatureCorrection), 0f, 0.002f);
        Assert.Equal(0f, driver.LastTelemetry.CorrectionDecayDistanceMeters);
        Assert.InRange(
            MathF.Abs(
                driver.LastTelemetry.TargetSpeed -
                driver.LastTelemetry.GlobalProfileTargetSpeed
            ),
            0f,
            1e-5f
        );
    }

    [Fact]
    public void DriverBrakesForTightCurveAhead()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver driver = new();
        RaceDriverFrameContext context = CreateContext(track, driver, s: 55f, speed: 35f);

        DriverInput input = driver.GetControl(in context, 1f / 60f);

        Assert.True(input.DesiredAccel < 0f, "high speed before the hairpin should request braking");
    }

    [Fact]
    public void StationaryCarUsesFullAvailableDriveWhileReturningToLine()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver driver = new();
        RaceDriverFrameContext context = CreateContext(
            track,
            driver,
            s: 20f,
            speed: 0f,
            lateralError: 4f
        );

        DriverInput input = driver.GetControl(in context, 1f / 60f);

        float fullAvailableDrive = context.Car.CarConfig.GetBatteryForceAccelLimit(
            context.Car.Strategy.BatteryMode
        );
        Assert.InRange(fullAvailableDrive - input.DesiredAccel, 0f, 0.02f);
        Assert.Equal(0f, driver.LastTelemetry.LossCompensationAcceleration);
    }

    [Fact]
    public void AttackTireModeAllowsMorePaceThanProtect()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver protectDriver = new();
        ReferenceLineDriver attackDriver = new();
        RaceDriverFrameContext protect = CreateContext(
            track,
            protectDriver,
            s: 100f,
            speed: 14f,
            strategy: new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Normal)
        );
        RaceDriverFrameContext attack = CreateContext(
            track,
            attackDriver,
            s: 100f,
            speed: 14f,
            strategy: new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Normal)
        );

        DriverInput protectInput = protectDriver.GetControl(in protect, 1f / 60f);
        DriverInput attackInput = attackDriver.GetControl(in attack, 1f / 60f);

        Assert.True(attackInput.DesiredAccel > protectInput.DesiredAccel);
    }

    private static RaceDriverFrameContext CreateContext(
        TrackData track,
        IRaceDriver driver,
        float s,
        float speed,
        float lateralError = 0f,
        CarStrategy? strategy = null
    )
    {
        TrackSample sample = track.Sample(s);
        CarState state = new()
        {
            Position = sample.RefPosition + sample.Normal * lateralError,
            Heading = sample.RefHeading,
            Speed = speed,
            BatterySoc = 0.8f
        };
        RaceCar car = new("test", new CarConfig(), WarmTires(), driver, state)
        {
            Strategy = strategy ?? CarStrategy.Default
        };
        TrackPose pose = track.Project(state.Position);
        return new RaceDriverFrameContext(car, track, pose, new RaceEnvironment(), 0f);
    }

    private static TrackData BuildTrack()
    {
        return new TrackBuilder(Vector2.Zero, startWidth: 10f, startLeftBuffer: 3f, startRightBuffer: 3f)
            .AddStraight(80f)
            .AddTurn(180f, 24f)
            .AddStraight(80f)
            .AddTurn(180f, 24f)
            .CloseLoop()
            .Build(new TrackGridConfig());
    }

    private static TireConfig WarmTires()
    {
        return new TireConfig
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
    }
}
