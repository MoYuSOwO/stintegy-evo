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
            DirectDriveObservation.PreviousDynamicOffset +
            DirectDriveObservation.DynamicBlockSize,
            DirectDriveObservation.ObservationSize
        );
    }

    [Fact]
    public void ThePreviousFrameCoversEveryCarAndNothingElse()
    {
        // The previous-frame copy is one contiguous slice, so it is only the
        // ego and opponent blocks if those two are adjacent and last. They
        // used to be separated by the tyre, mode and aero blocks, and the
        // slice — sized for ego plus opponents — ran off the end of the
        // opponents with the final car and a bit missing from every
        // remembered frame. Nothing failed; the memory was just wrong.
        Assert.Equal(
            DirectDriveObservation.EgoOffset,
            DirectDriveObservation.DynamicBlockOffset
        );
        Assert.Equal(
            DirectDriveObservation.EgoOffset + DirectDriveObservation.EgoSize,
            DirectDriveObservation.OpponentOffset
        );
        Assert.Equal(
            DirectDriveObservation.OpponentOffset +
            DirectDriveObservation.OpponentCount *
            DirectDriveObservation.OpponentSize,
            DirectDriveObservation.DynamicBlockOffset +
            DirectDriveObservation.DynamicBlockSize
        );
    }

    [Fact]
    public void ThePreviewReachesSixSecondsAhead()
    {
        // A fixed metre array cannot serve both ends of the speed range. The
        // pin is that the last station is always the same number of seconds
        // away, which is the quantity braking is measured in.
        foreach (float speed in new[] { 15f, 40f, 70f })
        {
            float horizon =
                DirectDriveObservationBuilder.PreviewHorizonMeters(speed);
            float expected = MathF.Max(
                DirectDriveObservation.MinimumPreviewMeters,
                speed * DirectDriveObservation.PreviewHorizonSeconds
            );
            Assert.Equal(expected, horizon, 3);
        }

        // Fast enough that the floor is not what is being measured.
        Assert.True(
            DirectDriveObservationBuilder.PreviewHorizonMeters(70f) > 400f,
            "at seventy metres a second the car should see past four hundred"
        );
    }

    [Fact]
    public void TheThreePointsCarryTheBankExactly()
    {
        // The whole argument for giving points rather than scalars is that
        // the cross section is a quadratic, a quadratic has three
        // coefficients, and three heights at known offsets determine them
        // exactly. If that is true this test can read the banking back out
        // of the observation to the last few decimals; if it is not, the
        // road is being described to the policy in a form that has quietly
        // lost something.
        TrackData track = TrackFactory.ZandvoortStyleTestTrack();
        DirectDriveRaceDriver driver = new(new PurePursuitPolicy());
        // Approaching Hugenholtz, so the stations ahead are the banked ones.
        RaceCar car = CreateCar(track, 700f, 30f, driver);
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        // One step, so the observation on hand is the one built from the
        // position recorded here rather than from wherever the car had got
        // to by the time the policy was next asked.
        float s = track.Project(car.State.Position).S;
        float horizon = DirectDriveObservationBuilder.PreviewHorizonMeters(
            car.State.Speed
        );
        simulation.Step(Dt);
        float spacing = horizon / DirectDriveObservation.GeometryPointCount;

        int banked = 0;
        for (int i = 0; i < DirectDriveObservation.GeometryPointCount; i++)
        {
            TrackSample sample = track.Sample(s + spacing * (i + 1));
            int cursor = DirectDriveObservation.GeometryOffset +
                         i * DirectDriveObservation.GeometryFloatsPerPoint;
            // Slot 2 is the left edge's height above the centre, slot 8 the
            // right edge's. Both are stored on the cross-section scale.
            float leftRise = driver.LastObservation[cursor + 2] *
                             DirectDriveObservation.CrossHeightScale;
            float rightRise = driver.LastObservation[cursor + 8] *
                              DirectDriveObservation.CrossHeightScale;

            float halfWidth = sample.HalfWidth;
            float bank = (leftRise - rightRise) / (2f * halfWidth);
            float camber = (leftRise + rightRise) / (2f * halfWidth * halfWidth);

            Assert.Equal(sample.BankSlope, bank, 4);
            Assert.Equal(sample.BankCurvature, camber, 4);
            if (MathF.Abs(sample.BankSlope) > 0.2f)
                banked++;
        }

        Assert.True(
            banked > 0,
            "the run-up to Hugenholtz should have banked road in view"
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
        // Stations are spaced by speed, so a fixed index is a fixed time
        // ahead rather than a fixed distance — which is what pure pursuit
        // wants anyway, since its lookahead should grow with speed.
        private const int AimPoint = 4;          // one and a half seconds

        public void Act(ReadOnlySpan<float> observation, Span<float> action)
        {
            int cursor = DirectDriveObservation.GeometryOffset +
                         AimPoint * DirectDriveObservation.GeometryFloatsPerPoint;
            // Slots 0-2 are the left edge, 3-5 the centre line.
            float ahead = observation[cursor + 3] *
                          DirectDriveObservation.DistanceScale;
            float across = observation[cursor + 4] *
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
                Energy = PowertrainState.Filled(0.8f)
            }
        );
    }
}
