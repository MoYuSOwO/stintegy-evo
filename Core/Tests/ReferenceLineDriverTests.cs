using System.Numerics;
using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;
using Xunit;

namespace TheStint.Core.Tests;

public sealed class ReferenceLineDriverTests
{
    [Fact]
    public void LateralErrorRequestsCurvatureBackTowardReferenceLine()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver driver = new();
        RaceDriverFrameContext context = CreateContext(track, driver, s: 20f, speed: 16f, lateralError: 2f);

        DriverInput input = driver.GetControl(in context, 1f / 60f);

        Assert.True(input.DesiredCurvature > context.Pose.Sample.RefCurvature);
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
    public void AttackTireModeAllowsMorePaceThanProtect()
    {
        TrackData track = BuildTrack();
        ReferenceLineDriver driver = new();
        RaceDriverFrameContext protect = CreateContext(
            track,
            driver,
            s: 100f,
            speed: 14f,
            strategy: new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Normal)
        );
        RaceDriverFrameContext attack = CreateContext(
            track,
            driver,
            s: 100f,
            speed: 14f,
            strategy: new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Normal)
        );

        DriverInput protectInput = driver.GetControl(in protect, 1f / 60f);
        DriverInput attackInput = driver.GetControl(in attack, 1f / 60f);

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
