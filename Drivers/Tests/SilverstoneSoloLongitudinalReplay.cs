using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using StintegyEVO.GodotApp.Race;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

public sealed class SilverstoneSoloLongitudinalReplay(ITestOutputHelper output)
{
    [Fact]
    public void OneTimedLap()
    {
        string? path = Environment.GetEnvironmentVariable("STINTEGY_SOLO_LAP_CSV");
        if (string.IsNullOrEmpty(path)) return;

        const int hz = 60;
        const string compound = DevelopmentTyres.SuperId;
        var track = GameMods.Registry.Circuits.Get("silverstone").Track;
        var sim = new RaceSimulation(track);
        var model = StockCar.Build(compound);
        var driver = new LaneDriver(LaneTrackFile.ForCircuit("silverstone", track), model)
        {
            LearnsAsItDrives = false,
        };
        var (id, abilities) = SilverstoneGrid.Roster[2];
        var state = new CarState
        {
            Position = track.Grids.GetPosition(1),
            Heading = track.Sample(track.Grids.GetS(1)).Heading,
        };
        ((ICarTires)model).FitTires(state, compound);
        ((ICarTires)model).WarmTires(state, 90f, 90f);
        ((ICarStores)model).Fill(state, PowertrainState.Filled(0.8f));
        var car = new RaceCar(id, model, new Driver(new DriverProfile(id, abilities), driver), state);
        sim.AddCar(car);

        using var recorder = new RaceCsvTelemetryRecorder(path, firstTick: 1);
        var requests = new List<float>();
        double? start = null;
        int off = 0, spin = 0, wall = 0;
        for (int tick = 1; tick <= 240 * hz; tick++)
        {
            float prior = car.Progress.RaceDistanceMeters;
            sim.Step(1f / hz);
            float current = car.Progress.RaceDistanceMeters;
            double AtCrossing(float s) => (tick - 1d + (s - prior) / (current - prior)) / hz;
            if (start is null && prior <= 0f && current > 0f) start = AtCrossing(0f);
            if (start is null) continue;

            recorder.Write((float)tick / hz, car, track, sim.Environment, tick);
            requests.Add(car.LastInput.DesiredAccel);
            if (car.Progress.Region != TrackRegion.RacingSurface) off++;
            if (((ICarMotion)model).Motion(state).Spinning) spin++;
            if (car.LastBoundaryContact.HasValue) wall++;
            if (current < track.LengthMeters) continue;

            int pulses = Enumerable.Range(2, requests.Count - 4).Count(i =>
                requests[i - 2] <= -8f && requests[i] >= -1f && requests[i + 2] <= -8f);
            string summary = FormattableString.Invariant($"lap={AtCrossing(track.LengthMeters) - start.Value:F6}s frames={requests.Count} off={off} spin={spin} wall={wall} pulses={pulses} coast={driver.CoastSeconds:F3}s");
            output.WriteLine(summary);
            Assert.True(off + spin + wall == 0, summary);
            // The old controller produced repeated brake-release-brake events in free air.
            Assert.True(pulses == 0, summary);
            return;
        }
        Assert.Fail("The car did not complete the timed Silverstone lap.");
    }
}
