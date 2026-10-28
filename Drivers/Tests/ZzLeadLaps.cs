using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Racing;
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

    /// <summary>
    /// Every touch of a wall and every moment off the road on the laps where
    /// the turn-in lead had them, step by step, beside the same lap with the
    /// hands as they are: where, how fast, which side, how far ahead the hands
    /// read, how far off the line, and the car as they are at the same place.
    /// </summary>
    [Fact]
    public void WallTrace()
    {
        if (!LaneProbes.Enabled) return;
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "walls.csv");
        var laps = new (string Circuit, float Side)[]
        {
            ("monaco", 0f), ("monaco", 1f), ("monaco", -1f), ("monza", 0f), ("baku", 0f), ("baku", 1f), ("singapore", -1f)
        };
        var ways = new (string Name, LaneDriverGains Gains)[]
        {
            ("as they are", new LaneDriverGains()),
            ("by the turn-in", new LaneDriverGains { LeadFromTurnIn = true }),
            ("no steer notes", new LaneDriverGains()),
        };
        var jobs = (from l in laps from w in ways select (l.Circuit, l.Side, w.Name, w.Gains)).ToArray();
        var rows = new List<string>[jobs.Length];
        Parallel.For(0, jobs.Length, j =>
        {
            (string circuit, float side, string name, LaneDriverGains gains) = jobs[j];
            TrackData track = CircuitLibrary.Load(circuit);
            LaneTrackFile file = LaneTrackFile.ForCircuit(circuit, track);
            LaneSet lanes = file.LanesFor(track);
            if (name.StartsWith("no steer notes"))
                WithoutSteerNotes(file, track);
            LaneId laneId = side == 0f ? LaneId.Free : side > 0f ? LaneId.Left : LaneId.Right;
            var mine = new List<string>();
            int lap0 = -1;
            LaneLapRunner.Run(track, file, gains, onStep: (car, d, t) =>
            {
                if (side != 0f)
                {
                    d.TargetRange = 1f;
                    d.TargetSide = side;
                }
                if (lap0 < 0) lap0 = car.Progress.Lap;
                if (car.Progress.Lap <= lap0) return;
                LaneDriverTelemetry tel = d.LastTelemetry;
                TrackSample at = track.Sample(car.Progress.CurrentS);
                float dd = car.Progress.CurrentD;
                float wallLeft = at.HalfWidth + at.LeftBufferWidth - dd, wallRight = at.HalfWidth + at.RightBufferWidth + dd;
                mine.Add(string.Join(",", circuit, side == 0f ? "free" : side > 0f ? "left" : "right", name,
                    string.Join(",", new float[]
                    {
                        t, car.Progress.CurrentS, dd, at.HalfWidth, wallLeft, wallRight, car.State.Speed, tel.CommandAccel,
                        d.LastLeadSeconds, tel.LateralErrorMeters, tel.FeedforwardCurvature, tel.WantedCurvature, tel.CurvatureCap,
                        tel.CommandCurvature, car.State.Telemetry.ActualCurvature, at.Curvature,
                        tel.HeadingErrorRadians, d.LastHereCurvature, lanes.Offset(laneId, lanes.Wrap((int)MathF.Round(lanes.StationOf(car.Progress.CurrentS)))), car.State.SideslipAngleRadians, d.LastMoveSpeed, d.Range,
                        car.BoundaryContactSeconds > 0f || car.Progress.HitWallThisFrame ? 1f : 0f,
                        car.Progress.Region != TrackRegion.RacingSurface ? 1f : 0f
                    }.Select(v => v.ToString("F5", System.Globalization.CultureInfo.InvariantCulture)))));
            });
            rows[j] = mine;
        });
        File.WriteAllLines(output, new[] { "circuit,lane,hands,t,s,d,half_width,wall_left,wall_right,speed,command_accel,lead,error,ff_k,wanted_k,cap_k,cmd_k,actual_k,road_k,heading_err,line_k,line_d,sideslip,move_speed,range,wall,off" }
            .Concat(rows.SelectMany(r => r)));
        _output.WriteLine($"{rows.Sum(r => r.Count)} steps to {output}");
    }

    /// <summary>The learned steering notes wiped, every lane: the hands on the line's own curvature alone.</summary>
    private static void WithoutSteerNotes(LaneTrackFile file, TrackData track)
    {
        file.LanesFor(track);
        for (int lane = 0; lane < LaneSet.LaneCount; lane++)
            Array.Clear(file.Steer((LaneId)lane));
    }

    /// <summary>
    /// The wheel on laps with the steering notes wiped: what the steering
    /// estimate made of the command, piece by piece, beside the slip angles
    /// the car then actually ran at each end and how much of each end's grip
    /// the pedals were using.
    /// </summary>
    [Fact]
    public void WheelTrace()
    {
        if (!LaneProbes.Enabled) return;
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "wheel.csv");
        string[] circuits = Environment.GetEnvironmentVariable("STINTEGY_LEAD_CIRCUITS")?.Split(',') ?? Circuits;
        var jobs = (from c in circuits from side in new[] { 0f, 1f, -1f } select (c, side)).ToArray();
        var rows = new List<string>[jobs.Length];
        StintegyEVO.Core.Cars.CarConfig config = StockCar.Config;
        Parallel.For(0, jobs.Length, j =>
        {
            (string circuit, float side) = jobs[j];
            TrackData track = CircuitLibrary.Load(circuit);
            LaneTrackFile file = LaneTrackFile.ForCircuit(circuit, track);
            WithoutSteerNotes(file, track);
            var mine = new List<string>();
            int lap0 = -1;
            LaneLapRunner.Run(track, file, new LaneDriverGains { OwnLock = Environment.GetEnvironmentVariable("STINTEGY_OWN_LOCK") == "1", SmoothHeading = Environment.GetEnvironmentVariable("STINTEGY_SMOOTH_HEADING") == "1" }, onStep: (car, d, t) =>
            {
                if (side != 0f)
                {
                    d.TargetRange = 1f;
                    d.TargetSide = side;
                }
                if (lap0 < 0) lap0 = car.Progress.Lap;
                if (car.Progress.Lap <= lap0) return;
                LaneDriverTelemetry tel = d.LastTelemetry;
                var state = car.State;
                var est = ((StintegyEVO.Core.Cars.ICarEnvelope)car.Car).EstimateSteering(state, StintegyEVO.Core.Cars.RoadAttitude.Flat);
                var grip = ((StintegyEVO.Core.Cars.ICarGripFeel)car.Car).GripUse(state);
                float k = tel.CommandCurvature;
                float wb = est.WheelBaseMeters, a = est.FrontArmMeters, b = est.RearArmMeters;
                // The full slip-angle part alone: the estimate as if fully loaded and already on the curve.
                float slip = (est with { ActualLateralAccel = 1e6f, ActualCurvature = k }).AngleFor(k) - MathF.Atan(wb * k);
                float load = Math.Clamp(MathF.Abs(est.ActualLateralAccel) / (StintegyEVO.Core.Cars.SteeringEstimate.FullSlipCompensationShare * (est.FrontGripAccel + est.RearGripAccel)), 0f, 1f);
                float v = MathF.Max(state.Speed, 1f);
                float af = state.SteerAngleRadians - state.SideslipAngleRadians - a * state.YawRateRadiansPerSecond / v;
                float ar = -state.SideslipAngleRadians + b * state.YawRateRadiansPerSecond / v;
                mine.Add(string.Join(",", circuit, side == 0f ? "free" : side > 0f ? "left" : "right",
                    string.Join(",", new float[]
                    {
                        t, car.Progress.CurrentS, state.Speed, tel.CommandAccel, d.LastHereCurvature, k, state.Telemetry.ActualCurvature, tel.LateralErrorMeters,
                        state.SteerAngleRadians, MathF.Atan(wb * k), slip, load, est.CurvatureFeedbackGain * wb * (k - est.ActualCurvature), est.AngleFor(k),
                        af, ar, grip.FrontLongitudinal, grip.RearLongitudinal, grip.FrontLateral, grip.RearLateral,
                        state.Telemetry.ActualLateralAccel, est.FrontGripAccel, est.RearGripAccel, grip.FrontGripAccel, grip.RearGripAccel, state.Telemetry.ActualLongitudinalAccel
                    }.Select(x => x.ToString("F6", CultureInfo.InvariantCulture)))));
            });
            rows[j] = mine;
        });
        File.WriteAllLines(output, new[] { "circuit,lane,t,s,speed,command_accel,line_k,cmd_k,actual_k,error,steer,kin,slip_full,load,fb,est_angle,af,ar,front_long,rear_long,front_lat,rear_lat,lat,front_grip,rear_grip,car_front_grip,car_rear_grip,long" }
            .Concat(rows.SelectMany(r => r)));
        _output.WriteLine($"{rows.Sum(r => r.Count)} steps to {output}");
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
            ("by the turn-in", new LaneDriverGains { LeadFromTurnIn = true }),
            ("no steer notes", new LaneDriverGains()),
            ("no steer notes + flat 0.10 s", new LaneDriverGains { SteerLeadSeconds = 0.10f }),
            ("no steer notes + turn-in", new LaneDriverGains { LeadFromTurnIn = true }),
            ("no steer notes + own lock", new LaneDriverGains { OwnLock = true }),
            ("no steer notes + smooth heading", new LaneDriverGains { SmoothHeading = true }),
            ("no steer notes + smooth heading + own lock", new LaneDriverGains { SmoothHeading = true, OwnLock = true }),
            ("no steer notes + smooth heading + turn-in", new LaneDriverGains { SmoothHeading = true, LeadFromTurnIn = true }),
            ("smooth heading", new LaneDriverGains { SmoothHeading = true }),
        };
        var jobs = (from c in only from side in new[] { 0f, 1f, -1f } from w in ways select (c, side, w)).ToArray();
        var rows = new string[jobs.Length];
        Parallel.For(0, jobs.Length, j =>
        {
            (string circuit, float side, var way) = jobs[j];
            TrackData track = CircuitLibrary.Load(circuit);
            LaneTrackFile file = LaneTrackFile.ForCircuit(circuit, track);
            if (way.Name.StartsWith("no steer notes"))
                WithoutSteerNotes(file, track);
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
