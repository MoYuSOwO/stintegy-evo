using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Probe, not a test: a flying lap of every circuit with notes, on the free
/// lane and all the way out on either side lane, with the hands as they are,
/// a flat 0.10 s lead, and the lead read off the rear's grip
/// (LaneDriverGains.LeadFromGrip).
/// </summary>
public sealed class ZzLeadLaps
{
    private readonly ITestOutputHelper _output;
    public ZzLeadLaps(ITestOutputHelper output) => _output = output;

    private static readonly string[] Circuits =
        ["silverstone", "monaco", "spa", "monza", "zandvoort", "baku", "interlagos", "singapore", "portimao", "shanghai", "sepang"];

    /// <summary>The turn-in delay the hands read ahead by under LeadFromTurnIn, beside the small-step turn lag.</summary>
    [Fact]
    public void TurnInTable()
    {
        if (!LaneProbes.Enabled) return;
        var lag = TurnLag.Measure(new StintegyEVO.Core.Cars.SingleTrackCar(StintegyEVO.Drivers.StockCar.Config,
            ((StintegyEVO.Core.Cars.ICarTires)StintegyEVO.Drivers.StockCar.Model).Compounds));
        foreach (float v in new[] { 15f, 25f, 40f, 55f, 70f, 85f })
            _output.WriteLine($"{v,3:F0} m/s: small step {lag.Seconds(v):F3} s; turn-in, braking 0 / 30% / 60% of all: {lag.Seconds(v, 0f):F3} / {lag.Seconds(v, 0.3f):F3} / {lag.Seconds(v, 0.6f):F3} s");
    }

    [Fact]
    public void Laps()
    {
        if (!LaneProbes.Enabled) return;
        string[] only = Environment.GetEnvironmentVariable("STINTEGY_LEAD_CIRCUITS")?.Split(',') ?? Circuits;
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "lead-laps.csv");
        var ways = new (string Name, LaneDriverGains Gains)[]
        {
            ("as they are", new LaneDriverGains()),
            ("flat 0.10 s", new LaneDriverGains { SteerLeadSeconds = 0.10f }),
            ("from the rear's grip", new LaneDriverGains { LeadFromGrip = true }),
            ("by the turn-in", new LaneDriverGains { LeadFromTurnIn = true }),
        };
        var jobs = (from c in only from side in new[] { 0f, 1f, -1f } from w in ways select (c, side, w)).ToArray();
        var rows = new string[jobs.Length];
        Parallel.For(0, jobs.Length, j =>
        {
            (string circuit, float side, var way) = jobs[j];
            TrackData track = CircuitLibrary.Load(circuit);
            LaneTrackFile file = LaneTrackFile.ForCircuit(circuit, track);
            LaneLapResult r = LaneLapRunner.Run(track, file, way.Gains, onStep: side == 0f ? null : (car, d, t) =>
            {
                d.TargetRange = 1f;
                d.TargetSide = side;
            });
            string lane = side == 0f ? "free" : side > 0f ? "left" : "right";
            rows[j] = string.Join(",", circuit, lane, way.Name,
                r.FlyingLapSeconds.ToString("F3", CultureInfo.InvariantCulture),
                r.MeanAbsErrorMeters.ToString("F3", CultureInfo.InvariantCulture),
                r.WorstErrorMeters.ToString("F3", CultureInfo.InvariantCulture),
                r.WorstErrorS.ToString("F0", CultureInfo.InvariantCulture),
                r.OffTrackSeconds.ToString("F2", CultureInfo.InvariantCulture),
                r.WallSeconds.ToString("F2", CultureInfo.InvariantCulture),
                r.Spins.ToString(CultureInfo.InvariantCulture));
        });
        foreach (string row in rows)
            _output.WriteLine(row);
        File.WriteAllLines(output, new[] { "circuit,lane,hands,lap_s,mean_error_m,worst_error_m,worst_at_s,off_s,wall_s,spins" }.Concat(rows));
    }
}
