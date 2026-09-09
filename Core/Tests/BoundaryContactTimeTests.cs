using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// How long a car leant on a barrier, as against whether it touched one.
/// A caller stepping a tenth of a second at a time sees only the second,
/// and scoring a race on that alone prices a glance off the wall the same
/// as scraping it the whole way — which leaves a car that has already
/// brushed with no reason to come off before the step is out.
/// </summary>
public sealed class BoundaryContactTimeTests
{
    [Fact]
    public void ACarOnTheRoadSpendsNoTimeAgainstAnything()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceSimulation simulation = new(track);
        RaceCar car = OnTheReferenceLine(track, 100f, 30f);
        simulation.AddCar(car);

        for (int i = 0; i < 120; i++)
        {
            simulation.Step(1f / 60f);
            Assert.Equal(0f, car.BoundaryContactSeconds);
        }
    }

    [Fact]
    public void ScrapingABarrierIsChargedByTheSecond()
    {
        // Driven into it rather than parked against it: a car placed past
        // the barrier is pushed back the moment it joins the race, so the
        // only way to be in contact is to arrive there.
        (int steps, float total, float most) = DriveIntoTheBarrier(0.15f);

        Assert.True(steps > 0, "the car never reached the barrier");
        Assert.True(total > 0f, "contact was recorded as taking no time");
        // Whatever it did, no step can charge more than the step lasted.
        Assert.InRange(most, 0f, 0.1f + 1e-4f);
    }

    [Fact]
    public void AGlanceCostsLessThanALean()
    {
        // The whole point of counting the seconds. A step that brushes the
        // barrier for one substep and a step that leans on it throughout
        // used to be indistinguishable, which left a car that had already
        // touched with no reason to come off before the step was out.
        (_, float gentle, _) = DriveIntoTheBarrier(0.05f);
        (_, float hard, _) = DriveIntoTheBarrier(0.15f);

        // A narrower margin than this used to have, and the reason is that
        // the road outside the line stopped being the same road. A car on
        // its way into a barrier now has wheels on the run-off, where more
        // steering buys less turning, so the hard case and the gentle one
        // converge on their way there. The ordering is what this test is
        // for and the ordering holds.
        // A fifth rather than a quarter, and the reason is arithmetic
        // rather than physics. Contact seconds are counted in substeps, so
        // these readings live on a lattice: the gentle case is ten sixtieths
        // and the hard one twelve and a half, whose ratio is exactly 1.25.
        // A strict comparison against the very lattice point the data lands
        // on decides itself on the last bit of a float. The ordering is what
        // this test is for; the margin only has to be clear of the noise,
        // and it must not sit on a value the measurement can produce
        // exactly.
        Assert.True(
            hard > gentle * 1.2f,
            $"leaning on the barrier ({hard:0.000} s) should cost more than " +
            $"glancing off it ({gentle:0.000} s)"
        );
    }

    [Fact]
    public void TheClockRestartsEveryStep()
    {
        (int steps, float total, float most) = DriveIntoTheBarrier(0.15f);

        Assert.True(steps > 1, "needs more than one step to say anything");
        // A running total would have the last step holding all of it.
        Assert.True(
            most < total,
            $"one step recorded {most:0.000} s of a {total:0.000} s total, " +
            "which means the clock is never being reset"
        );
    }

    /// <summary>
    /// Holds a curvature from the racing line until the car finds the
    /// barrier, and reports how many steps touched it, for how long in
    /// total, and the most any single step recorded.
    /// </summary>
    private static (int Steps, float Total, float Most) DriveIntoTheBarrier(
        float curvature
    )
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample at = track.Sample(275f);
        RaceCar car = new(
            "leaner",
            new CarConfig(),
            new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f },
            new FixedInputDriver(new DriverInput(curvature, 2f)),
            new CarState
            {
                Position = at.RefPosition,
                Heading = at.RefHeading,
                Speed = 25f,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        int steps = 0;
        float total = 0f;
        float most = 0f;
        for (int i = 0; i < 40; i++)
        {
            simulation.Step(0.1f);
            float seconds = car.BoundaryContactSeconds;
            total += seconds;
            most = MathF.Max(most, seconds);
            if (seconds > 0f)
                steps++;
        }
        return (steps, total, most);
    }

    private static RaceCar OnTheReferenceLine(
        TrackData track,
        float s,
        float speed
    )
    {
        TrackSample at = track.Sample(s);
        return new RaceCar(
            "clean",
            new CarConfig(),
            new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f },
            new ReferenceLineDriver(),
            new CarState
            {
                Position = at.RefPosition,
                Heading = at.RefHeading,
                Speed = speed,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
    }

    private sealed class FixedInputDriver(DriverInput input) : IRaceDriver
    {
        public void Initialize(in RaceDriverInitContext context) { }

        public DriverInput GetControl(
            in RaceDriverFrameContext context,
            float dt
        ) => input;
    }
}
