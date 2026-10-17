using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

public sealed class RuleLineControllerTests
{
    private readonly ITestOutputHelper _output;

    public RuleLineControllerTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void SilverstonePackLoadsTheBakedTumLineAndIgnoresBakedSpeed()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        GeometricRacingLine line = RuleLineAssets.LoadOrBake(RuleTrackPack.Silverstone, track, 0.95f);
        Assert.False(line.HasBakedSpeed, "TUM v* is a different car; speed is planned live");
        GeometricRacingLine.Sample start = line.At(track.Sample, track.WrapS, track.StartingLineS);
        Assert.InRange(start.OffsetMeters, -6f, 6f);
        Assert.True(
            MathF.Abs(start.OffsetMeters) > 0.4f,
            $"expected the TUM path, not centreline, d={start.OffsetMeters:0.00}"
        );
    }

    [Fact]
    public void PaceIsOnlyACornerShare()
    {
        DriverExecution full = DriverExecution.From(new DriverAbilities());
        DriverExecution back = DriverExecution.From(new DriverAbilities { Pace = 0f, CarControl = 0f });
        Assert.Equal(1f, full.PaceEfficiency, 2);
        Assert.Equal(1f, full.ControlAuthority, 2);
        Assert.Equal(0f, full.BrakeMarkerErrorMeters);
        Assert.Equal(0.95f, back.PaceEfficiency, 2);
        Assert.Equal(0f, back.ControlAuthority, 2);
    }

    [Fact]
    public void StandingStartDoesNotSteerOntoTheRacingLineAndDoesNotSpin()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        (RaceSimulation simulation, RaceCar car) = Seat(track, 0f, onLine: false);
        for (int i = 0; i < 60 * 12; i++)
            simulation.Step(1f / 60f);

        Assert.Equal(0, car.State.SpinEvents);
        Assert.False(car.State.Spinning);
        Assert.True(car.State.Speed > 8f, $"still crawling at {car.State.Speed:0.0} m/s");
        TrackSample road = track.Sample(car.Progress.CurrentS);
        Assert.True(
            MathF.Abs(car.Progress.CurrentD) < road.HalfWidth - 0.5f,
            $"left the road at d={car.Progress.CurrentD:0.00} half={road.HalfWidth:0.00}"
        );
    }

    [Fact]
    public void CompletesALapOfTheSimpleCircuitWithoutSpinning()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        (RaceSimulation simulation, RaceCar car) = Seat(track, 20f, onLine: true);
        int spinsSeen = 0;
        for (int i = 0; i < 60 * 180; i++)
        {
            simulation.Step(1f / 60f);
            if (car.State.SpinEvents > spinsSeen)
            {
                spinsSeen = car.State.SpinEvents;
                _output.WriteLine(
                    $"spin {spinsSeen} t={i / 60f:F2}s s={car.Progress.CurrentS:F0} " +
                    $"d={car.Progress.CurrentD:F2} v={car.State.Speed:F1} " +
                    $"slip={car.State.SideslipAngleRadians:F3} yaw={car.State.YawRateRadiansPerSecond:F2}"
                );
            }
        }

        Assert.Equal(0, car.State.SpinEvents);
        Assert.True(
            car.Progress.TotalDistance > track.LengthMeters,
            $"covered {car.Progress.TotalDistance:F0} m of a {track.LengthMeters:F0} m lap"
        );
    }

    [Fact]
    public void CompletesASilverstoneLapFromARollingStart()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        (RaceSimulation simulation, RaceCar car) = Seat(track, 40f, onLine: true);
        GeometricRacingLine racingLine = LineOf(car);
        float length = track.LengthMeters;
        float previousDistance = car.Progress.TotalDistance;
        float? firstCrossing = null;
        float flying = -1f;
        float peak = 0f;
        int spinsSeen = 0;
        float dt = 1f / 60f;
        for (int i = 0; i < 60 * 360; i++)
        {
            simulation.Step(dt);
            peak = MathF.Max(peak, car.State.Speed);
            if (car.State.SpinEvents > spinsSeen)
            {
                spinsSeen = car.State.SpinEvents;
                float tum = racingLine.SpeedAt(track.WrapS, car.Progress.CurrentS);
                var lineHere = racingLine.At(track.Sample, track.WrapS, car.Progress.CurrentS);
                _output.WriteLine(
                    $"spin {spinsSeen} t={i * dt:F2}s s={car.Progress.CurrentS:F0} " +
                    $"d={car.Progress.CurrentD:F2} n={lineHere.OffsetMeters:F2} " +
                    $"e={car.Progress.CurrentD - lineHere.OffsetMeters:F2} v={car.State.Speed:F1} " +
                    $"k={lineHere.PathCurvature:F4} tum={tum:F1} " +
                    $"slip={car.State.SideslipAngleRadians:F3} yaw={car.State.YawRateRadiansPerSecond:F2}"
                );
            }
            float distance = car.Progress.TotalDistance;
            if (distance > previousDistance)
            {
                int before = (int)MathF.Floor(previousDistance / length);
                int after = (int)MathF.Floor(distance / length);
                for (int line = before + 1; line <= after; line++)
                {
                    float share = (line * length - previousDistance) / (distance - previousDistance);
                    float at = (i - 1) * dt + share * dt;
                    if (firstCrossing is null)
                        firstCrossing = at;
                    else if (flying < 0f)
                        flying = at - firstCrossing.Value;
                }
            }

            previousDistance = distance;
            if (flying > 0f)
                break;
        }

        float outLap = firstCrossing ?? -1f;
        _output.WriteLine(
            $"out-lap {outLap:F2}s  flying {flying:F2}s  peak {peak:F1} m/s  spins {car.State.SpinEvents}"
        );
        Assert.Equal(0, car.State.SpinEvents);
        Assert.InRange(flying, 100f, 120f);
    }

    [Fact]
    public void SilverstoneFlyingLapSaysWhereTheTimeWent()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        (RaceSimulation simulation, RaceCar car) = Seat(track, 40f, onLine: true);
        GeometricRacingLine line = LineOf(car);
        float length = track.LengthMeters;
        float dt = 1f / 60f;
        float previousDistance = car.Progress.TotalDistance;
        bool flying = false;
        var planState = new CarState();
        LineSpeedProfile? profile = null;
        float grip = TiresOf(car).GetAccelerationUsage(CarStrategy.Default);

        const int sectors = 12;
        float sectorLength = length / sectors;
        float[] actual = new float[sectors];
        float[] planned = new float[sectors];
        float[] errorSum = new float[sectors];
        float[] ratioSum = new float[sectors];
        int[] samples = new int[sectors];
        float slow = 0f;
        float spareCorner = 0f;
        float saturatedOff = 0f;
        float onPlan = 0f;

        for (int i = 0; i < 60 * 400 && car.State.SpinEvents == 0; i++)
        {
            simulation.Step(dt);
            float distance = car.Progress.TotalDistance;
            int before = (int)MathF.Floor(previousDistance / length);
            int after = (int)MathF.Floor(distance / length);
            previousDistance = distance;
            if (!flying)
            {
                if (after > before)
                {
                    flying = true;
                    planState.CopyFrom(car.State);
                    profile = LineSpeedProfile.FromLine(
                        line,
                        track,
                        includeRoadCurvature: false,
                        windowMeters: RuleTrackPack.Silverstone.KappaWindowMeters,
                        usePathCurvature: true,
                        curvatureScale: RuleTrackPack.Silverstone.CurvatureScale
                    );
                    profile.Recompute(
                        planState,
                        ConfigOf(car),
                        TiresOf(car),
                        grip,
                        RuleTrackPack.Silverstone.GgUsage,
                        topSpeedMetersPerSecond: RuleTrackPack.Silverstone.TopSpeedMetersPerSecond,
                        brakeUsage: RuleTrackPack.Silverstone.BrakeUsage
                    );
                }

                continue;
            }

            if (after > before)
                break;

            float s = car.Progress.CurrentS;
            var sample = line.At(track.Sample, track.WrapS, s);
            float error = MathF.Abs(car.Progress.CurrentD - sample.OffsetMeters);
            float speed = MathF.Max(car.State.Speed, 0.5f);
            float target = MathF.Max(profile!.At(track.WrapS, s), 0.5f);
            CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
                planState,
                ConfigOf(car),
                TiresOf(car),
                CarStrategy.Default,
                speed,
                curvature: 0f,
                grip
            );
            float used = speed * speed * MathF.Abs(sample.PathCurvature);
            float ratio = used / MathF.Max(limits.LateralAccelerationLimit, 1f);
            float behind = MathF.Max(0f, target - speed) / target * dt;
            int sector = Math.Clamp((int)(s / sectorLength), 0, sectors - 1);
            actual[sector] += dt;
            planned[sector] += speed / target * dt;
            errorSum[sector] += error;
            ratioSum[sector] += ratio;
            samples[sector]++;

            if (error > 1.2f && ratio > 0.85f)
                saturatedOff += dt;
            else if (error > 1.2f)
                slow += behind;
            else if (behind > 0f)
                slow += behind;
            else if (MathF.Abs(sample.PathCurvature) > 0.004f && ratio < 0.45f)
                spareCorner += dt;
            else
                onPlan += dt;
        }

        Assert.NotNull(profile);
        Assert.Equal(0, car.State.SpinEvents);
        float drove = 0f;
        for (int sector = 0; sector < sectors; sector++)
            drove += actual[sector];
        Assert.InRange(drove, 110f, 125f);
        _output.WriteLine(
            $"plan {profile!.EstimateLapSeconds():F1}s  " +
            $"behind the table {slow:F1}s  " +
            $"corner with grip left {spareCorner:F1}s  " +
            $"full grip and off line {saturatedOff:F1}s  " +
            $"otherwise {onPlan:F1}s"
        );
        for (int sector = 0; sector < sectors; sector++)
        {
            if (samples[sector] == 0)
                continue;
            _output.WriteLine(
                $"s {sector * sectorLength:0000}-{(sector + 1) * sectorLength:0000}  " +
                $"drove {actual[sector]:00.0}s  table {planned[sector]:00.0}s  " +
                $"gap {actual[sector] - planned[sector]:+0.0;-0.0}s  " +
                $"off {errorSum[sector] / samples[sector]:0.00}m  " +
                $"grip {ratioSum[sector] / samples[sector]:0.00}"
            );
        }
    }

    [Fact]
    public void StaysOnTheRacingLineThroughASilverstoneLap()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        (RaceSimulation simulation, RaceCar car) = Seat(track, 40f, onLine: true);
        GeometricRacingLine line = LineOf(car);
        float worst = 0f;
        int samples = 0;
        float sum = 0f;
        int spinsSeen = 0;
        for (int i = 0; i < 60 * 180; i++)
        {
            simulation.Step(1f / 60f);
            if (car.State.SpinEvents > spinsSeen)
            {
                spinsSeen = car.State.SpinEvents;
                _output.WriteLine(
                    $"spin {spinsSeen} t={i / 60f:F2}s s={car.Progress.CurrentS:F0} " +
                    $"d={car.Progress.CurrentD:F2} v={car.State.Speed:F1} " +
                    $"slip={car.State.SideslipAngleRadians:F3}"
                );
            }
            if (i < 60 * 3)
                continue;
            GeometricRacingLine.Sample target = line.At(
                track.Sample,
                track.WrapS,
                car.Progress.CurrentS
            );
            float error = MathF.Abs(car.Progress.CurrentD - target.OffsetMeters);
            worst = MathF.Max(worst, error);
            sum += error;
            samples++;
        }

        Assert.Equal(0, car.State.SpinEvents);
        Assert.True(samples > 0, "no samples");
        float mean = sum / samples;
        _output.WriteLine($"mean |n| {mean:0.00} m, worst {worst:0.00} m");
        Assert.True(mean < 0.80f, $"mean |n| error {mean:0.00} m, worst {worst:0.00} m");
        Assert.True(worst < 3.5f, $"worst |n| error {worst:0.00} m");
    }

    [Fact]
    public void StaysOnTheRacingLineThroughTheSimpleCircuit()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        (RaceSimulation simulation, RaceCar car) = Seat(track, 20f, onLine: true);
        GeometricRacingLine line = LineOf(car);
        float worst = 0f;
        int samples = 0;
        float sum = 0f;
        for (int i = 0; i < 60 * 90; i++)
        {
            simulation.Step(1f / 60f);
            if (i < 60 * 3)
                continue;
            GeometricRacingLine.Sample target = line.At(
                track.Sample,
                track.WrapS,
                car.Progress.CurrentS
            );
            float error = MathF.Abs(car.Progress.CurrentD - target.OffsetMeters);
            worst = MathF.Max(worst, error);
            sum += error;
            samples++;
        }

        Assert.Equal(0, car.State.SpinEvents);
        Assert.True(samples > 0, "no samples");
        float mean = sum / samples;
        Assert.True(mean < 3.0f, $"mean |n| error {mean:0.00} m, worst {worst:0.00} m");
        Assert.True(worst < 9.5f, $"worst |n| error {worst:0.00} m");
    }

    private static (RaceSimulation Simulation, RaceCar Car) Seat(
        TrackData track,
        float startSpeed,
        bool onLine
    )
    {
        TrackSample start = track.Sample(track.StartingLineS);
        var carConfig = new CarConfig();
        var tires = new TireConfig
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        var controller = track.LengthMeters > 3_000f
            ? (RuleLineController)new SilverstoneRuleController(carConfig, tires)
            : new SimpleCircuitRuleController(carConfig, tires);
        GeometricRacingLine line = RuleLineAssets.LoadOrBake(
            RuleTrackPack.For(track.LengthMeters),
            track,
            0.95f
        );
        GeometricRacingLine.Sample pose = line.At(track.Sample, track.WrapS, track.StartingLineS);
        var car = new RaceCar(
            "rule",
            carConfig,
            tires,
            new Driver(new DriverProfile("rule", new DriverAbilities()), controller),
            new CarState
            {
                Position = onLine ? pose.Position : start.Center,
                Heading = onLine ? pose.Heading : start.Heading,
                Speed = startSpeed,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
        var simulation = new RaceSimulation(track);
        simulation.AddCar(car);
        return (simulation, car);
    }

    private static GeometricRacingLine LineOf(RaceCar car)
    {
        var controller = (RuleLineController)car.Driver!.Controller;
        GeometricRacingLine? line = controller.Line;
        Assert.NotNull(line);
        return line;
    }

    // The rule controller plans on the game's own chassis; the race now holds
    // the car as an ICar, so its numbers are read back off the chassis.
    private static CarConfig ConfigOf(RaceCar car) => ((SingleTrackCar)car.Car).Config;

    private static TireConfig TiresOf(RaceCar car) => ((StandardTire)((SingleTrackCar)car.Car).Tires.Front).Config;
}
