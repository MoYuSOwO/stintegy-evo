using System;
using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Real 3D-road inputs at the boundary between TrackData, RaceSimulation,
/// CarPhysics, and the external DriverInput host.  These tests intentionally
/// do not install a driver or a planning implementation.
/// </summary>
public sealed class WorldBoundaryRegressionTests
{
    private const float Dt = 1f / 60f;
    private const float TestGrade = 0.08f;

    [Fact]
    public void AFixedExternalCommandFeelsTheSameLongStraightRoadInAllThreeGrades()
    {
        DriverInput command = new(0f, 4f);
        SteppedRoadResult uphill = RunLongStraight(new(TestGrade, 0f), command);
        SteppedRoadResult level = RunLongStraight(TrackSurface.Flat, command);
        SteppedRoadResult downhill = RunLongStraight(new(-TestGrade, 0f), command);

        Assert.Equal(command, uphill.LastInput);
        Assert.Equal(command, level.LastInput);
        Assert.Equal(command, downhill.LastInput);

        Assert.True(
            uphill.SpeedMetersPerSecond < level.SpeedMetersPerSecond,
            $"uphill speed {uphill.SpeedMetersPerSecond:0.000} should be below " +
            $"level speed {level.SpeedMetersPerSecond:0.000}"
        );
        Assert.True(
            downhill.SpeedMetersPerSecond > level.SpeedMetersPerSecond,
            $"downhill speed {downhill.SpeedMetersPerSecond:0.000} should be above " +
            $"level speed {level.SpeedMetersPerSecond:0.000}"
        );
        Assert.True(
            uphill.DistanceMeters < level.DistanceMeters &&
            level.DistanceMeters < downhill.DistanceMeters,
            $"distance ordering was uphill {uphill.DistanceMeters:0.000}, " +
            $"level {level.DistanceMeters:0.000}, downhill {downhill.DistanceMeters:0.000}"
        );

        // The road attitude is consumed by the physical step, rather than
        // merely being visible in TrackData: the same pedal command produces
        // a matching causal ordering in the reported longitudinal response.
        Assert.True(
            uphill.ActualLongitudinalAccel < level.ActualLongitudinalAccel &&
            level.ActualLongitudinalAccel < downhill.ActualLongitudinalAccel,
            $"road input was not reflected in acceleration: uphill " +
            $"{uphill.ActualLongitudinalAccel:0.000}, level " +
            $"{level.ActualLongitudinalAccel:0.000}, downhill " +
            $"{downhill.ActualLongitudinalAccel:0.000}"
        );
    }

    [Fact]
    public void ASteppedCarObservesCrestAndCompressionThroughWheelLoad()
    {
        TrackData crestTrack = BuildCrestTrack(crest: true);
        TrackData levelTrack = BuildCrestTrack(crest: false, flat: true);
        TrackData compressionTrack = BuildCrestTrack(crest: false);
        TrackSample crestSample = crestTrack.Sample(400f);
        TrackSample levelSample = levelTrack.Sample(400f);
        TrackSample compressionSample = compressionTrack.Sample(400f);

        Assert.InRange(MathF.Abs(crestSample.Grade), 0f, 0.005f);
        Assert.InRange(MathF.Abs(compressionSample.Grade), 0f, 0.005f);
        Assert.Equal(0f, levelSample.VerticalRate, 6);
        Assert.True(crestSample.VerticalRate < -1e-4f);
        Assert.True(compressionSample.VerticalRate > 1e-4f);

        WheelLoadResult crest = RunOneStep(crestTrack);
        WheelLoadResult level = RunOneStep(levelTrack);
        WheelLoadResult compression = RunOneStep(compressionTrack);

        Assert.True(
            crest.FrontLeftLoadN < level.FrontLeftLoadN,
            $"crest load {crest.FrontLeftLoadN:0.0} should be below level " +
            $"load {level.FrontLeftLoadN:0.0}"
        );
        Assert.True(
            compression.FrontLeftLoadN > level.FrontLeftLoadN,
            $"compression load {compression.FrontLeftLoadN:0.0} should be above level " +
            $"load {level.FrontLeftLoadN:0.0}"
        );
        Assert.Equal(new DriverInput(0f, 0f), crest.LastInput);
        Assert.Equal(new DriverInput(0f, 0f), level.LastInput);
        Assert.Equal(new DriverInput(0f, 0f), compression.LastInput);
    }

    [Fact]
    public void RetainingMultipleFrameSnapshotsPreservesEachTyreEnergyAndWeatherValue()
    {
        TrackData track = BuildWorldTrack();
        RaceCar car = NewCar(track, "snapshot", 120f, default);
        RaceSimulation race = new(track);
        race.AddCar(car);

        car.State.FrontLeft.SurfaceTempC = 87f;
        car.State.Energy = PowertrainState.Filled(0.7f);
        race.Environment.AirTempC = 25f;
        race.Environment.TrackTempC = 35f;
        race.Environment.SurfaceGripScalar = 1f;
        RaceFrameSnapshot first = race.CaptureFrame();

        car.State.FrontLeft.SurfaceTempC = 121f;
        car.State.Energy = PowertrainState.Filled(0.2f);
        race.Environment.AirTempC = 42f;
        race.Environment.TrackTempC = 66f;
        race.Environment.SurfaceGripScalar = 0.9f;
        RaceFrameSnapshot second = race.CaptureFrame();

        // A third mutation proves neither retained frame is a live view of
        // the car or the environment after its own capture.
        car.State.FrontLeft.SurfaceTempC = 143f;
        car.State.Energy = PowertrainState.Filled(0.05f);
        race.Environment.AirTempC = -5f;
        race.Environment.TrackTempC = 12f;
        race.Environment.SurfaceGripScalar = 0.4f;

        Assert.Equal(87f, first[0].FrontLeft.SurfaceTempC);
        Assert.Equal(0.7f, first[0].Resources[0].Fraction);
        Assert.Equal(25f, first.Environment.AirTempC);
        Assert.Equal(35f, first.Environment.TrackTempC);
        Assert.Equal(1f, first.Environment.SurfaceGripScalar);

        Assert.Equal(121f, second[0].FrontLeft.SurfaceTempC);
        Assert.Equal(0.2f, second[0].Resources[0].Fraction);
        Assert.Equal(42f, second.Environment.AirTempC);
        Assert.Equal(66f, second.Environment.TrackTempC);
        Assert.Equal(0.9f, second.Environment.SurfaceGripScalar);
    }

    [Fact]
    public void ExternalInputsRemainAssociatedWithTheirCarsWhenInsertionOrderChanges()
    {
        Dictionary<string, ExternalStepResult> forward = RunExternalOrder(reverse: false);
        Dictionary<string, ExternalStepResult> reverse = RunExternalOrder(reverse: true);

        Assert.Equal(new DriverInput(0f, 5f), forward["alpha"].LastInput);
        Assert.Equal(new DriverInput(0f, -3f), forward["beta"].LastInput);
        Assert.Equal(forward["alpha"].LastInput, reverse["alpha"].LastInput);
        Assert.Equal(forward["beta"].LastInput, reverse["beta"].LastInput);

        Assert.Equal(forward["alpha"].SpeedMetersPerSecond, reverse["alpha"].SpeedMetersPerSecond, 5);
        Assert.Equal(forward["beta"].SpeedMetersPerSecond, reverse["beta"].SpeedMetersPerSecond, 5);
        Assert.Equal(forward["alpha"].DistanceMeters, reverse["alpha"].DistanceMeters, 5);
        Assert.Equal(forward["beta"].DistanceMeters, reverse["beta"].DistanceMeters, 5);
    }

    private static SteppedRoadResult RunLongStraight(
        TrackSurface surface,
        DriverInput command
    )
    {
        TrackData track = BuildWorldTrack(surface);
        RaceCar car = NewCar(track, "external", 120f, command, speed: 50f);
        RaceSimulation race = new(track);
        race.AddCar(car);
        for (int i = 0; i < 120; i++)
            race.Step(Dt);

        return new(
            car.State.Speed,
            car.Progress.TotalDistance,
            car.State.Telemetry.ActualLongitudinalAccel,
            car.LastInput
        );
    }

    private static WheelLoadResult RunOneStep(TrackData track)
    {
        RaceCar car = NewCar(track, "crest", 400f, default, speed: 60f);
        RaceSimulation race = new(track);
        race.AddCar(car);
        race.Step(Dt);
        RaceFrameSnapshot frame = race.CaptureFrame();
        return new(frame[0].FrontLeft.LoadN, car.LastInput);
    }


    private static TrackData BuildCrestTrack(bool crest, bool flat = false)
    {
        const float summit = 400f;
        const float height = 20f;
        float signedHeight = crest ? height : flat ? 0f : -height;
        return new TrackBuilder(Vector2.Zero, 16f, 5f, 5f)
            .AddStraight(800f)
            .AddTurn(180f, 80f)
            .AddStraight(800f)
            .AddTurn(180f, 80f)
            .CloseLoop()
            .WithSurface(TrackElevation.ProfileByDistance([
                (0f, 0f),
                (200f, 0f),
                (summit, signedHeight),
                (600f, 0f),
                (1000f, 0f)
            ]))
            .Build(new TrackGridConfig());
    }

    private static Dictionary<string, ExternalStepResult> RunExternalOrder(bool reverse)
    {
        TrackData track = BuildWorldTrack();
        RaceCar alpha = NewCar(track, "alpha", 120f, new(0f, 5f), speed: 35f);
        RaceCar beta = NewCar(track, "beta", 260f, new(0f, -3f), speed: 35f);
        RaceSimulation race = new(track);
        if (reverse)
        {
            race.AddCar(beta);
            race.AddCar(alpha);
        }
        else
        {
            race.AddCar(alpha);
            race.AddCar(beta);
        }

        race.Step(0.2f);
        Dictionary<string, ExternalStepResult> result = new(StringComparer.Ordinal)
        {
            [alpha.Id] = new(alpha.State.Speed, alpha.Progress.TotalDistance, alpha.LastInput),
            [beta.Id] = new(beta.State.Speed, beta.Progress.TotalDistance, beta.LastInput)
        };
        return result;
    }

    private static RaceCar NewCar(
        TrackData track,
        string id,
        float s,
        DriverInput input,
        float speed = 50f
    )
    {
        TrackSample at = track.Sample(s);
        CarState state = new()
        {
            Position = at.Center,
            Heading = at.Heading,
            Speed = speed,
            Energy = PowertrainState.Filled(0.8f)
        };
        return TestControlFixtures.ExternalCar(id, state, input);
    }

    private static TrackData BuildWorldTrack(TrackSurface surface = default) =>
        new TrackBuilder(Vector2.Zero, 16f, 5f, 5f)
            .AddStraight(800f)
            .AddTurn(180f, 80f)
            .AddStraight(800f)
            .AddTurn(180f, 80f)
            .CloseLoop()
            .WithSurface(_ => surface)
            .Build(new TrackGridConfig());

    private readonly record struct SteppedRoadResult(
        float SpeedMetersPerSecond,
        float DistanceMeters,
        float ActualLongitudinalAccel,
        DriverInput LastInput
    );

    private readonly record struct WheelLoadResult(float FrontLeftLoadN, DriverInput LastInput);

    private readonly record struct ExternalStepResult(
        float SpeedMetersPerSecond,
        float DistanceMeters,
        DriverInput LastInput
    );
}
