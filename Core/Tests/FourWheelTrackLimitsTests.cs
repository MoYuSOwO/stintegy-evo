using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The race steward's ruler: a car is off only when all four wheels are
/// beyond the white line. Training keeps its stricter centreline ruler; this
/// one is read for certification and never enters a reward.
/// </summary>
public sealed class FourWheelTrackLimitsTests
{
    [Fact]
    public void ThreeWheelsOverIsStillOnTheCircuit()
    {
        Assert.False(TrackLimits.AllFourWheelsBeyondTheLine(5f, 5.2f, 5.1f, 5.3f, 4.9f));
        Assert.False(TrackLimits.AllFourWheelsBeyondTheLine(5f, -5.2f, -5.1f, -4.9f, -5.3f));
    }

    [Fact]
    public void AllFourWheelsOverIsOffEitherSide()
    {
        Assert.True(TrackLimits.AllFourWheelsBeyondTheLine(5f, 5.2f, 5.1f, 5.3f, 5.05f));
        Assert.True(TrackLimits.AllFourWheelsBeyondTheLine(5f, -5.2f, -5.1f, -5.3f, -5.05f));
    }

    [Fact]
    public void TheLineItselfIsTrack()
    {
        Assert.False(TrackLimits.AllFourWheelsBeyondTheLine(5f, 5f, 5f, 5f, 5f));
    }

    /// <summary>
    /// On a real circuit, through the simulation. A car straight along the
    /// road with its left wheels over the line and its right wheels inside it
    /// has two wheels out and is on the circuit; move it far enough that the
    /// right wheels are over too and the ruler starts counting.
    /// </summary>
    [Fact]
    public void TheSimulationCountsOnlyTheSecondsAllFourAreOver()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        const float s = 275f;
        float halfWidth = track.Sample(s).HalfWidth;
        float halfTrack = new CarConfig().TrackWidthMeters * 0.5f;

        float twoOut = MeasureOffSeconds(track, s, halfWidth + halfTrack * 0.5f);
        float fourOut = MeasureOffSeconds(track, s, halfWidth + halfTrack + 0.4f);

        Assert.Equal(0f, twoOut);
        Assert.InRange(fourOut, 0.05f, 0.1f + 1e-4f);
    }

    private static float MeasureOffSeconds(TrackData track, float s, float offset)
    {
        TrackSample at = track.Sample(s);
        RaceCar car = new(
            "straddler",
            new CarConfig(),
            new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f },
            new HoldStill(),
            new CarState
            {
                Position = at.Center + at.Normal * offset,
                Heading = MathF.Atan2(at.Tangent.Y, at.Tangent.X),
                Speed = 5f,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        simulation.Step(0.1f);
        return car.FourWheelsOffSeconds;
    }

    private sealed class HoldStill : IRaceDriver
    {
        public void Initialize(in RaceDriverInitContext context) { }

        public DriverInput GetControl(in RaceDriverFrameContext context, float dt) =>
            new(0f, 0f);
    }
}
