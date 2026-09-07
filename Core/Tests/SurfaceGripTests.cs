using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Leaving the road costs grip, per wheel, continuously.
///
/// It used to cost a penalty term: the training environment noticed a
/// region flag and subtracted reward, and the evaluation added the seconds
/// back at a rate. That is a rule about a line, and rules about lines
/// invite arguments about lines - how far over, for how long, and whether
/// the gain was worth the fine. Grass is slippery. A car with two wheels on
/// it has less grip, is slower for having them there, and nobody has to
/// adjudicate anything.
/// </summary>
public sealed class SurfaceGripTests
{
    private readonly ITestOutputHelper _output;
    public SurfaceGripTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void TheRoadOnlyEverGetsWorseAsYouLeaveIt()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample sample = track.Sample(0f);
        float half = sample.HalfWidth;

        float previous = float.MaxValue;
        for (int i = 0; i <= 200; i++)
        {
            float offset = (half + 2f) * i / 200f;
            float grip = SurfaceGrip.StaticAt(sample, offset);
            Assert.True(
                grip <= previous + 1e-5f,
                $"grip rose on the way out, at {offset:0.00} m"
            );
            if (i > 0)
            {
                Assert.True(
                    previous - grip < 0.15f,
                    $"a cliff in the grip at {offset:0.00} m: " +
                    $"{previous:0.000} then {grip:0.000}"
                );
            }
            previous = grip;
        }

        Assert.Equal(SurfaceGrip.RacingSurface, SurfaceGrip.StaticAt(sample, 0f), 3);
        Assert.Equal(
            SurfaceGrip.Buffer,
            SurfaceGrip.StaticAt(sample, half + 2f),
            3
        );
        // Symmetric: which side of the centre line has never mattered.
        Assert.Equal(
            SurfaceGrip.StaticAt(sample, half * 0.9f),
            SurfaceGrip.StaticAt(sample, -half * 0.9f),
            5
        );
    }

    /// <summary>
    /// Two wheels over the line and two on it: the axle is worth the sum of
    /// what each wheel is standing on, so the car loses grip in proportion
    /// to how much of itself it put on the grass.
    /// </summary>
    [Fact]
    public void StraddlingTheLineCostsGripInProportion()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample sample = track.Sample(0f);
        float half = sample.HalfWidth;
        CarConfig car = new();
        float halfTrack = car.TrackWidthMeters * 0.5f;

        float previousInner = float.MaxValue;
        float previousOuter = float.MaxValue;
        _output.WriteLine("  车心偏移    外侧轮    内侧轮");
        for (int i = 0; i <= 8; i++)
        {
            // Straight ahead, so the wheels sit a half track width either
            // side of the centre of the car.
            float centre = half - halfTrack + 0.25f * i;
            float outer = SurfaceGrip.StaticAt(sample, centre + halfTrack);
            float inner = SurfaceGrip.StaticAt(sample, centre - halfTrack);
            _output.WriteLine(
                $"  {centre,8:0.00}  {outer,8:0.000}  {inner,8:0.000}"
            );

            Assert.True(
                outer <= inner + 1e-5f,
                "the wheel nearer the edge cannot have more grip than the " +
                "one further in"
            );
            Assert.True(outer <= previousOuter + 1e-5f);
            Assert.True(inner <= previousInner + 1e-5f);
            previousOuter = outer;
            previousInner = inner;
        }

        // And at the end of that walk the two are far apart, which is the
        // whole point: one axle, two different roads.
        float wideCentre = half + 0.25f;
        Assert.True(
            SurfaceGrip.StaticAt(sample, wideCentre - halfTrack) -
            SurfaceGrip.StaticAt(sample, wideCentre + halfTrack) > 0.2f,
            "a car straddling the line should have a measurably better " +
            "inside wheel than outside one"
        );
    }

    /// <summary>
    /// The simulation asks per wheel, and the answer follows the car's
    /// attitude rather than its centre alone.
    /// </summary>
    [Fact]
    public void TheSimulationSamplesEachWheelWhereItActuallyIs()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample sample = track.Sample(0f);
        CarConfig config = new();
        float half = sample.HalfWidth;

        RaceCar car = new(
            "straddle",
            config,
            new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f },
            new ReferenceLineDriver(),
            new CarState
            {
                Position = sample.Center + sample.Normal *
                           (half - config.TrackWidthMeters * 0.5f + 0.4f),
                Heading = sample.RefHeading,
                Speed = 40f,
                Energy = PowertrainState.Filled(0.8f)
            }
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        simulation.Step(1f / 120f);

        // The track's normal is the clockwise turn of its tangent, so
        // offset grows to the right of travel: displacing the car along it
        // puts the right-hand wheels over the line.
        float outerFront = car.State.FrontRight.SurfaceGrip;
        float innerFront = car.State.FrontLeft.SurfaceGrip;
        _output.WriteLine(
            $"外前轮 {outerFront:0.000}  内前轮 {innerFront:0.000}  " +
            $"外后轮 {car.State.RearRight.SurfaceGrip:0.000}  " +
            $"内后轮 {car.State.RearLeft.SurfaceGrip:0.000}"
        );
        Assert.True(
            outerFront < innerFront,
            $"the outside wheels should be on worse road: {outerFront:0.000} " +
            $"against {innerFront:0.000}"
        );
        Assert.True(innerFront > 0.95f, "the inside wheels are still on the road");
    }

    /// <summary>
    /// And it costs lap time, which is the whole reason for doing it this
    /// way. A car with two wheels over the line has less grip, so it holds
    /// less corner, so it goes slower - priced by the physics, in seconds,
    /// with nothing to argue about.
    /// </summary>
    [Fact]
    public void PuttingTwoWheelsOverTheLineCostsCornering()
    {
        CarConfig car = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };

        float onRoad = SteadyLateral(car, tires, WheelSurfaceGrip.Clean);
        // Two wheels on the grass: the outside pair at buffer grip, the
        // inside pair still on tarmac.
        float straddling = SteadyLateral(
            car,
            tires,
            new WheelSurfaceGrip(
                SurfaceGrip.RacingSurface,
                SurfaceGrip.Buffer,
                SurfaceGrip.RacingSurface,
                SurfaceGrip.Buffer
            )
        );

        _output.WriteLine(
            $"在路面上 {onRoad:0.00} m/s^2，两轮压线 {straddling:0.00} m/s^2 " +
            $"（{(1f - straddling / onRoad) * 100f:0.0}% 的过弯能力没了）"
        );
        Assert.True(
            straddling < onRoad * 0.85f,
            $"straddling the line should cost real cornering: {onRoad:0.00} " +
            $"became {straddling:0.00}"
        );
    }

    private static float SteadyLateral(
        CarConfig car,
        TireConfig tires,
        WheelSurfaceGrip surface
    )
    {
        CarState state = new()
        {
            Speed = 55f,
            Energy = PowertrainState.Filled(0.8f)
        };
        state.InstallFreshTires(tires);
        float best = 0f;
        for (int i = 1; i <= 24; i++)
        {
            float curvature = 0.0005f * i;
            CarState probe = state.Clone();
            for (int step = 0; step < 240; step++)
            {
                CarPhysics.Step(
                    probe,
                    car,
                    tires,
                    new CarPhysicsStepInput(
                        new DriverInput(curvature, 1.5f),
                        CarStrategy.Default,
                        25f,
                        35f
                    )
                    {
                        SurfaceGrip = surface
                    },
                    1f / 60f
                );
                if (!probe.Spinning)
                    probe.Speed = 55f;
            }
            if (!probe.Spinning)
            {
                best = MathF.Max(
                    best,
                    MathF.Abs(probe.Telemetry.ActualLateralAccel)
                );
            }
        }
        return best;
    }

    /// <summary>
    /// The edge reads the same on every circuit: road to the white line,
    /// then a kerb that gets worse the further onto it you go, then grass.
    ///
    /// Two things are being pinned. The order of the three surfaces — the
    /// kerb used to be the last strip of the racing surface, which meant a
    /// wheel crossing the line went from full grip to less than half of it
    /// inside one step. And the kerb's shape: it is a ramp across its whole
    /// width, not a plateau. A plateau says the second centimetre past the
    /// line costs what the fifty-ninth costs, and how far onto a kerb to
    /// put the wheel is exactly the thing a driver meters.
    /// </summary>
    [Fact]
    public void TheEdgeIsRoadThenKerbThenGrass()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample sample = track.Sample(120f);
        float half = sample.HalfWidth;

        // Racing surface right up to the line: a hand's width inside it is
        // still worth a hundred per cent.
        Assert.Equal(
            SurfaceGrip.RacingSurface, SurfaceGrip.StaticAt(sample, half - 0.1f), 3
        );
        // At the line itself the road has not been charged for yet.
        Assert.Equal(
            SurfaceGrip.RacingSurface, SurfaceGrip.StaticAt(sample, half), 3
        );
        // At the kerb's outer edge it is worth the quoted kerb value, and
        // that is where the strip ends rather than where it settles.
        Assert.Equal(
            SurfaceGrip.Kerb,
            SurfaceGrip.StaticAt(sample, half + SurfaceGrip.KerbWidthMeters),
            3
        );
        // Past the kerb, through the one remaining transition, grass.
        Assert.Equal(
            SurfaceGrip.Buffer,
            SurfaceGrip.StaticAt(
                sample,
                half + SurfaceGrip.KerbWidthMeters
                    + SurfaceGrip.TransitionMeters + 0.05f
            ),
            3
        );
    }

    /// <summary>
    /// Inside the kerb, deeper always costs more.
    ///
    /// This is what makes the strip a ramp rather than a threshold. It has
    /// to be strict, not merely non-increasing: a flat stretch anywhere in
    /// here is a range of depths the car cannot tell apart, and a driver
    /// choosing how much kerb to take would have no gradient to follow
    /// across it.
    /// </summary>
    [Fact]
    public void ThePriceOfAKerbRisesWithHowFarOntoItYouGo()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample sample = track.Sample(120f);
        float half = sample.HalfWidth;

        const int steps = 60;
        float previous = SurfaceGrip.StaticAt(sample, half);
        for (int i = 1; i <= steps; i++)
        {
            float depth = SurfaceGrip.KerbWidthMeters * i / steps;
            float grip = SurfaceGrip.StaticAt(sample, half + depth);
            Assert.True(
                grip < previous,
                $"a centimetre further onto the kerb ({depth:0.000} m) must " +
                $"cost something: {previous:0.00000} -> {grip:0.00000}"
            );
            previous = grip;
        }

        // And the ramp spans the whole quoted drop, so the kerb's outer
        // edge is not secretly most of the way to grass.
        Assert.Equal(
            SurfaceGrip.Kerb,
            SurfaceGrip.StaticAt(sample, half + SurfaceGrip.KerbWidthMeters),
            4
        );
    }

    /// <summary>
    /// Every circuit has at least a kerb's width of run-off, everywhere.
    /// The narrowest case the grammar allows is a street circuit: kerb,
    /// then wall. A white line with a barrier immediately behind it is not
    /// a narrow run-off, it is a case the edge grammar cannot describe,
    /// so the builder floors it instead of letting it happen.
    /// </summary>
    [Theory]
    [InlineData("monaco")]
    [InlineData("baku")]
    [InlineData("silverstone")]
    public void NoCircuitHasLessRunOffThanAKerb(string name)
    {
        TrackData track = name switch
        {
            "monaco" => TrackFactory.MonacoStyleTestTrack(),
            "baku" => TrackFactory.BakuStyleTestTrack(),
            _ => TrackFactory.SilverstoneStyleTestTrack(),
        };
        float narrowest = float.MaxValue;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            TrackSample sample = track.Sample(s);
            narrowest = MathF.Min(
                narrowest,
                MathF.Min(sample.LeftBufferWidth, sample.RightBufferWidth)
            );
        }
        Assert.True(
            narrowest >= SurfaceGrip.MinimumBufferMeters - 1e-4f,
            $"{name} has run-off {narrowest:0.000} m wide somewhere, " +
            $"narrower than the {SurfaceGrip.MinimumBufferMeters:0.0} m kerb"
        );
    }
}
