using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Drivers.Learned;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Pins the direct physical interface. Between a policy and the car there
/// is deliberately nothing, so the only thing standing between a learner
/// and the road is the observation — and the way to show it is sound is to
/// drive with it. The pure-pursuit policy below reads nothing but the
/// geometry block and laps the track on it. If that breaks, the interface
/// or the observation is lying to every learner above it.
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
        Assert.Equal(
            DirectDriveObservation.PreviousDynamicOffset +
            DirectDriveObservation.DynamicBlockSize,
            DirectDriveObservation.ObservationSize
        );
    }

    [Fact]
    public void TheGeometryBlockIsEnoughToDriveOn()
    {
        // Not a performance pin. The claim is that everything a car needs to
        // stay on a road is in the observation, and the demonstration is a
        // policy that uses only the road part of it and gets round.
        float distance = RunSolo(
            new DirectDriveRaceDriver(new PurePursuitPolicy()),
            out bool stayedOnSurface
        );

        Assert.True(
            stayedOnSurface,
            $"the pure-pursuit car left the racing surface after {distance:0} m"
        );
        // Half a lap of the simple layout and then some, on a quarter
        // throttle. The bar is that it gets round a road, not that it is
        // quick: a policy this artless would be embarrassed to be quick.
        Assert.True(
            distance > 1000f,
            $"a minute of pure pursuit covered only {distance:0} m"
        );
    }

    /// <summary>
    /// Steers at a point down the road and holds a modest throttle, reading
    /// nothing but the geometry block. Deliberately artless: it is here to
    /// show the observation carries a road, not to drive well.
    /// </summary>
    private sealed class PurePursuitPolicy : IDrivingPolicy
    {
        private const int AimPoint = 5;          // thirty metres ahead

        public void Act(ReadOnlySpan<float> observation, Span<float> action)
        {
            int cursor = DirectDriveObservation.GeometryOffset +
                         AimPoint * DirectDriveObservation.GeometryFloatsPerPoint;
            float ahead = observation[cursor] *
                          DirectDriveObservation.DistanceScale;
            float across = observation[cursor + 1] *
                           DirectDriveObservation.LateralScale;

            float rangeSquared = ahead * ahead + across * across;
            float curvature = rangeSquared > 1f
                ? 2f * across / rangeSquared
                : 0f;

            // The interface takes curvature as a fraction of the car's own
            // steering limit, and a hundredth of a per-metre is a long way
            // round for these cars.
            action[0] = Math.Clamp(curvature / 0.05f, -1f, 1f);
            action[1] = 0.25f;
        }
    }

    [Fact]
    public void ObservationsAndActionsStayFinite()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        DirectDriveRaceDriver driver = new(new PurePursuitPolicy());
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
