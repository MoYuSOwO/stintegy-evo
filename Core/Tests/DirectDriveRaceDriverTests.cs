using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Drivers.Learned;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Pins the direct physical interface: a policy that merely echoes the
/// coach block must lap like the analytic driver, because between a policy
/// and the car there is deliberately nothing else. If these pins break,
/// the interface or the observation is lying to every learner above it.
/// </summary>
public sealed class DirectDriveRaceDriverTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void ObservationLayoutIsConsistent()
    {
        Assert.Equal(
            DirectDriveObservation.GeometryPointCount,
            DirectDriveObservation.PreviewDistancesMeters.Length
        );
        Assert.True(
            DirectDriveObservation.CoachPlanSpeedCount <=
            DirectDriveObservation.PreviewDistancesMeters.Length
        );
        Assert.Equal(
            DirectDriveObservation.PreviousDynamicOffset +
            DirectDriveObservation.DynamicBlockSize,
            DirectDriveObservation.ObservationSize
        );
    }

    [Fact]
    public void CoachPassthroughLapsCloseToTheReferenceDriver()
    {
        float referenceDistance = RunSolo(
            new ReferenceLineDriver(),
            out bool referenceOnSurface
        );
        float directDistance = RunSolo(
            new DirectDriveRaceDriver(new CoachPassthroughPolicy()),
            out bool directOnSurface
        );

        Assert.True(referenceOnSurface);
        Assert.True(
            directOnSurface,
            "the passthrough car left the racing surface"
        );
        // The coach block is the reference-line plan, which samples peak
        // curvature per segment and is therefore deliberately conservative;
        // echoing it lands near ninety percent of the full driver. This pin
        // guards the interface, not performance: the learner's job is to
        // beat the coach, not to copy it.
        Assert.True(
            directDistance >= referenceDistance * 0.85f,
            $"passthrough covered {directDistance:0} m vs reference " +
            $"{referenceDistance:0} m"
        );
    }

    [Fact]
    public void ObservationsAndActionsStayFinite()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        DirectDriveRaceDriver driver = new(new CoachPassthroughPolicy());
        RaceCar car = CreateCar(track, 100f, 40f, driver);
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        for (int i = 0; i < 240; i++)
            simulation.Step(Dt);

        foreach (float value in driver.LastObservation)
            Assert.True(float.IsFinite(value));
        foreach (float value in driver.LastAction)
            Assert.InRange(value, -1f, 1f);
    }

    private static float RunSolo(
        IRaceDriver driver,
        out bool stayedOnSurface
    )
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, 100f, 40f, driver);
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        stayedOnSurface = true;
        for (int i = 0; i < 60 * 60; i++)
        {
            simulation.Step(Dt);
            if (car.LastBoundaryContact.HasValue)
                stayedOnSurface = false;
        }
        return car.Progress.TotalDistance;
    }

    private static RaceCar CreateCar(
        TrackData track,
        float s,
        float speed,
        IRaceDriver driver
    )
    {
        TrackSample sample = track.Sample(s);
        return new RaceCar(
            "direct-drive-test",
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            driver,
            new CarState
            {
                Position = sample.RefPosition,
                Heading = sample.RefHeading,
                Speed = speed,
                BatterySoc = 0.8f
            }
        );
    }
}
