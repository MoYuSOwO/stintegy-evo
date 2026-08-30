using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

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
                Energy = PowertrainState.Filled(0.9f)
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
            $"soc={car.State.Energy.Primary:0.000}, wall={car.LastBoundaryContact.HasValue}"
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
    public void LateralErrorBuildsStanleyMotionPredictionAndDynamicSpeedPlan()
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
        Assert.True(telemetry.PredictedPathLengthMeters >= 599f);
        Assert.True(
            telemetry.PredictedPathMaximumCurvature >=
            MathF.Abs(telemetry.DesiredCurvature) - 1e-4f
        );
        Assert.True(telemetry.JoinsReferenceLine);
        Assert.True(
            MathF.Abs(telemetry.PredictedTerminalLateralErrorMeters) <
            MathF.Abs(telemetry.LateralErrorMeters)
        );
        Assert.True(driver.CurrentSpeedLookahead.LengthMeters >= 599f);
        Assert.True(driver.CurrentPathPrediction.JoinsReferenceLine);
        Assert.True(
            driver.CurrentPathPrediction.DynamicPredictionLengthMeters <
            driver.CurrentPathPrediction.LengthMeters
        );
        Assert.InRange(
            driver.CurrentPathPrediction.ReferenceLineJoinCurvatureDelta,
            0f,
            0.002f
        );
    }

    [Fact]
    public void AlignedCarPredictionStartsAtCommandAndStaysNearReferenceLine()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver driver = new();
        RaceDriverFrameContext context = CreateContext(track, driver, s: 20f, speed: 16f);

        driver.GetControl(in context, 1f / 60f);

        Assert.True(driver.CurrentPathPrediction.Count > 2);
        Assert.InRange(
            MathF.Abs(
                driver.CurrentPathPrediction[0].CommandedCurvature -
                driver.LastTelemetry.DesiredCurvature
            ),
            0f,
            1e-5f
        );
        Assert.InRange(
            MathF.Abs(driver.CurrentPathPrediction.TerminalLateralErrorMeters),
            0f,
            0.5f
        );
    }

    [Fact]
    /// <summary>
    /// Arriving at a hairpin faster than it can be taken asks for the brakes,
    /// and arriving slower than that does not.
    ///
    /// Both halves, because only the pair says anything. A single speed picked
    /// as fast enough for one car stops being fast enough the moment the car
    /// is given more grip, and the test then passes or fails on how quick the
    /// car is rather than on whether it brakes for corners. The speeds here are
    /// read off what this car can actually carry through this corner, so the
    /// question stays the same question on any car.
    /// </summary>
    public void DriverBrakesForACurveItCannotCarryItsSpeedThrough()
    {
        TrackData track = BuildTrack();
        float apexCurvature = 0f;
        for (float s = 80f; s <= 160f; s += 1f)
        {
            apexCurvature = MathF.Max(
                apexCurvature,
                MathF.Abs(track.Sample(s).RefCurvature)
            );
        }

        VehicleSpeedPlanner planner = new();
        ReferenceLineDriver measuring = new();
        RaceDriverFrameContext measuringContext =
            CreateContext(track, measuring, s: 55f, speed: 35f);
        float throughTheCorner = planner.EstimateLateralSpeedLimit(
            measuringContext.Car,
            apexCurvature
        );

        // Far enough over that the twenty five metres of road left cannot
        // absorb the difference. A smaller margin proves nothing: a car with
        // four g of braking genuinely does not need to lift yet, and a test
        // that insists it should is testing the car's brakes, not its
        // willingness to use them.
        ReferenceLineDriver tooFast = new();
        DriverInput braking = tooFast.GetControl(
            CreateContext(track, tooFast, s: 55f, speed: throughTheCorner * 3f),
            1f / 60f
        );
        Assert.True(
            braking.DesiredAccel < 0f,
            "arriving above what the hairpin will take should request braking"
        );

        ReferenceLineDriver withinIt = new();
        DriverInput settled = withinIt.GetControl(
            CreateContext(track, withinIt, s: 55f, speed: throughTheCorner * 0.6f),
            1f / 60f
        );
        Assert.True(
            settled.DesiredAccel > braking.DesiredAccel,
            "arriving below it should not ask for as much braking"
        );
    }

    [Fact]
    public void DriverClampsSpeedFeedbackToEstimatedDriveLimit()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver driver = new() { SpeedGain = 100f };
        RaceDriverFrameContext context = CreateContext(
            track,
            driver,
            s: 20f,
            speed: 5f
        );

        DriverInput input = driver.GetControl(in context, 1f / 60f);

        Assert.InRange(
            driver.LastTelemetry.DriveAccelerationLimit - input.DesiredAccel,
            0f,
            0.02f
        );
    }

    [Fact]
    public void DriverCapabilityUsageScalesExecutionDriveLimit()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver fullDriver = new(
            new VehicleSpeedPlanningConfig { DriveAccelerationUsage = 1f }
        ) { SpeedGain = 100f };
        ReferenceLineDriver learningDriver = new(
            new VehicleSpeedPlanningConfig { DriveAccelerationUsage = 0.8f }
        ) { SpeedGain = 100f };
        RaceDriverFrameContext full = CreateContext(
            track,
            fullDriver,
            s: 20f,
            speed: 0f,
            lateralError: 4f
        );
        RaceDriverFrameContext learning = CreateContext(
            track,
            learningDriver,
            s: 20f,
            speed: 0f,
            lateralError: 4f
        );

        DriverInput fullInput = fullDriver.GetControl(in full, 1f / 60f);
        DriverInput learningInput = learningDriver.GetControl(in learning, 1f / 60f);

        Assert.InRange(
            fullDriver.LastTelemetry.DriveAccelerationLimit - fullInput.DesiredAccel,
            0f,
            0.02f
        );
        Assert.InRange(
            learningDriver.LastTelemetry.DriveAccelerationLimit - learningInput.DesiredAccel,
            0f,
            0.02f
        );
        Assert.True(learningInput.DesiredAccel < fullInput.DesiredAccel);
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
            strategy: new CarStrategy(TireUsageMode.Protect, PowerOutputMode.Normal)
        );
        RaceDriverFrameContext attack = CreateContext(
            track,
            attackDriver,
            s: 100f,
            speed: 14f,
            strategy: new CarStrategy(TireUsageMode.Attack, PowerOutputMode.Normal)
        );

        DriverInput protectInput = protectDriver.GetControl(in protect, 1f / 60f);
        DriverInput attackInput = attackDriver.GetControl(in attack, 1f / 60f);

        Assert.True(
            attackInput.DesiredAccel > protectInput.DesiredAccel
        );
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
            Energy = PowertrainState.Filled(0.8f)
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
