using System;
using System.Collections.Generic;
using System.Linq;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

public sealed class RungSweepProbe
{
    private readonly ITestOutputHelper _output;
    public RungSweepProbe(ITestOutputHelper output) => _output = output;

    /// <summary>The five tyre rungs on the shipped file, three hot laps each: lap, wear, heat.</summary>
    [Fact]
    public void FiveTyreRungsOnTheShippedFile()
    {
        if (!LaneProbes.Enabled) return;
        Sweep("shipped", new LaneDriverGains(), new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f });
    }

    private void Sweep(string label, LaneDriverGains gains, TireConfig tyresConfig)
    {
        _output.WriteLine(label);
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        foreach (TireUsageMode mode in Enum.GetValues<TireUsageMode>())
        {
            var crossings = new List<(float T, float Wear)>();
            int lastLap = int.MinValue;
            double surf = 0, n = 0;
            var r = LaneLapRunner.Run(track, file, gains, flyingLaps: 4, tireConfig: tyresConfig, strategy: new CarStrategy(mode, PowerOutputMode.Normal), onStep: (car, d, t) =>
            {
                var s = car.State;
                var tyres = new[] { s.FrontLeft, s.FrontRight, s.RearLeft, s.RearRight };
                if (car.Progress.Lap != lastLap) { lastLap = car.Progress.Lap; crossings.Add((t, tyres.Max(x => x.Wear))); }
                if (crossings.Count >= 3) { surf += tyres.Average(x => x.SurfaceTempC); n++; }
            });
            var a = crossings[2]; var b = crossings[^1]; int laps = crossings.Count - 3;
            _output.WriteLine($"{mode,-8} lap {(b.T - a.T) / laps:F2}  worst tyre wear {(b.Wear - a.Wear) / laps * 100f:F2}%/lap  surface {surf / n:F0} C  spins {r.Spins} off {r.OffTrackSeconds:F2}");
        }
    }
}

public sealed class LiftSweepProbe
{
    private readonly ITestOutputHelper _output;
    public LiftSweepProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void BatteryAndTimeByLift()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        foreach (float coast in new[] { 0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f, 1.25f, 1.5f })
        {
            var crossings = new List<(float T, float Soc, float Fuel)>();
            int lastLap = int.MinValue;
            var r = LaneLapRunner.Run(track, file, flyingLaps: 4, startingCharge: 0.6f, onStep: (car, d, t) =>
            {
                d.CoastOverride = coast;
                if (car.Progress.Lap != lastLap) { lastLap = car.Progress.Lap; crossings.Add((t, car.State.Energy.Primary, car.State.Energy.Secondary)); }
            });
            var a = crossings[2]; var b = crossings[^1]; int laps = crossings.Count - 3;
            float lap = (b.T - a.T) / laps, kwh = (a.Soc - b.Soc) / laps * 30f, fuel = (a.Fuel - b.Fuel) / laps * 120f;
            _output.WriteLine($"lift {coast:F2}s  lap {lap:F2}  battery {kwh:+0.00;-0.00} kWh/lap  fuel {fuel:F2} kg/lap  52 laps: battery {kwh * 52f:F1} kWh, fuel {fuel * 52f:F0} kg  spins {r.Spins} off {r.OffTrackSeconds:F2}");
        }
    }
}

public sealed class EnergyRungProbe
{
    private readonly ITestOutputHelper _output;
    public EnergyRungProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void FiveEnergyRungs()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        foreach (PowerOutputMode rung in Enum.GetValues<PowerOutputMode>())
        {
            var crossings = new List<(float T, float Soc)>();
            int lastLap = int.MinValue;
            var r = LaneLapRunner.Run(track, file, flyingLaps: 4, startingCharge: 0.6f, strategy: new CarStrategy(TireUsageMode.Normal, rung), onStep: (car, d, t) =>
            {
                if (car.Progress.Lap != lastLap) { lastLap = car.Progress.Lap; crossings.Add((t, car.State.Energy.Primary)); }
            });
            var a = crossings[2]; var b = crossings[^1]; int laps = crossings.Count - 3;
            float kwh = (a.Soc - b.Soc) / laps * 30f;
            _output.WriteLine($"{rung,-7} lap {(b.T - a.T) / laps:F2}  battery {kwh:+0.00;-0.00} kWh/lap  52 laps {kwh * 52f:+0.0;-0.0} kWh  spins {r.Spins} off {r.OffTrackSeconds:F2}");
        }
    }
}
