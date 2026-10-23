using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// A circuit's asphalt: how much grip it gives and how hard it is on
/// rubber, read from <c>circuit.json</c>.
/// </summary>
public sealed class CircuitSurfaceTests
{
    [Fact]
    public void ACircuitThatSaysNothingIsTheAverageCircuit()
    {
        CircuitManifest manifest = CircuitManifest.Parse("""{ "id": "x", "lengthMeters": 1000 }""");
        Assert.Equal(1f, manifest.Surface.Grip);
        Assert.Equal(1f, manifest.Surface.Abrasion);

        CircuitManifest written = CircuitManifest.Parse(
            """{ "id": "x", "lengthMeters": 1000, "surface": { "grip": 0.96, "abrasion": 1.3 } }""");
        Near.Equal(0.96f, written.Surface.Grip);
        Near.Equal(1.3f, written.Surface.Abrasion);
    }

    [Fact]
    public void SlipperyAsphaltCostsTheRoadAndTheKerbButNotTheGrass()
    {
        TrackSample sample = TrackFactory.SimpleTestTrack().Sample(120f);
        float half = sample.HalfWidth;
        float grass = half + SurfaceGrip.KerbWidthMeters + SurfaceGrip.TransitionMeters + 1f;

        Assert.True(SurfaceGrip.StaticAt(sample, 0f, 0.9f) < SurfaceGrip.StaticAt(sample, 0f));
        Assert.True(
            SurfaceGrip.StaticAt(sample, half + SurfaceGrip.KerbWidthMeters, 0.9f) <
            SurfaceGrip.StaticAt(sample, half + SurfaceGrip.KerbWidthMeters));
        Near.Equal(SurfaceGrip.StaticAt(sample, grass), SurfaceGrip.StaticAt(sample, grass, 0.9f));
    }

    [Fact]
    public void ACarStandsOnTheCircuitsAsphalt()
    {
        float average = WheelGripOn(1f);
        float slippery = WheelGripOn(0.94f);
        Assert.True(slippery < average, $"{slippery} against {average}");
    }

    [Fact]
    public void AnAbrasiveCircuitWearsTheSameDrivingFaster()
    {
        float average = WearAfterCorner(1f);
        float abrasive = WearAfterCorner(1.5f);
        float smooth = WearAfterCorner(0.7f);
        Assert.True(abrasive > average && average > smooth, $"{smooth} < {average} < {abrasive}");
        // The asphalt multiplies the wear and nothing else, so the same
        // corner wears in proportion.
        Near.Equal(1.5f, abrasive / average, rel: 0.05f);
    }

    private static float WheelGripOn(float asphalt)
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        track.AsphaltGrip = asphalt;
        TrackSample sample = track.Sample(0f);
        RaceCar car = TestControlFixtures.ExternalCar("on-road", new CarState
        {
            Position = sample.Center,
            Heading = sample.Heading,
            Speed = 30f,
            Energy = PowertrainState.Filled(0.8f)
        });
        var simulation = new RaceSimulation(track);
        simulation.AddCar(car);
        simulation.Step(1f / 120f);
        return car.State.FrontLeft.SurfaceGrip;
    }

    private static float WearAfterCorner(float abrasion)
    {
        CarConfig car = new();
        TestTires tires = TestControlFixtures.WarmTires();
        CarState state = new()
        {
            Speed = 40f,
            Energy = PowertrainState.Filled(0.8f)
        };
        state.FrontLeft.SurfaceTempC = state.FrontRight.SurfaceTempC = 90f;
        state.RearLeft.SurfaceTempC = state.RearRight.SurfaceTempC = 90f;
        var input = new CarPhysicsStepInput(new DriverInput(0.01f, 0f), CarStrategy.Default, AirTempC: 25f)
        {
            SurfaceAbrasion = abrasion
        };
        for (int i = 0; i < 240; i++)
            CarPhysics.Step(state, car, tires, input, 1f / 120f);
        return state.FrontLeft.Wear + state.FrontRight.Wear + state.RearLeft.Wear + state.RearRight.Wear;
    }
}
