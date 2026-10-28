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
            LaneLapRunner.Run(track, file, carConfig: Environment.GetEnvironmentVariable("STINTEGY_CAR") is { } carSpec ? CarWith(carSpec) : null, shares: Environment.GetEnvironmentVariable("STINTEGY_REAR_TURN_IN") is { } rearTurnIn ? new LaneShares { RearTurnIn = float.Parse(rearTurnIn, CultureInfo.InvariantCulture) }
                : Environment.GetEnvironmentVariable("STINTEGY_BRAKES_ADD") is { } brakesAdd ? new LaneShares { RearBrakeAdds = float.Parse(brakesAdd, CultureInfo.InvariantCulture) } : null, gains: new LaneDriverGains { OwnLock = Environment.GetEnvironmentVariable("STINTEGY_OWN_LOCK") == "1", LockForRotation = Environment.GetEnvironmentVariable("STINTEGY_LOCK_ROTATION") == "1", LockWhereTheCarIs = Environment.GetEnvironmentVariable("STINTEGY_LOCK_WHERE") == "1", FadeByAsk = Environment.GetEnvironmentVariable("STINTEGY_FADE_ASK") != "0" }, onStep: (car, d, t) =>
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
                TrackSample under = track.Sample(car.Progress.CurrentS);
                // The road as the race reads it under the car (RaceSimulation.RoadAttitudeAt), so the estimate is the one the driver was handed.
                var road = under.Grade == 0f && under.BankSlope == 0f && under.BankCurvature == 0f && under.VerticalRate == 0f
                    ? StintegyEVO.Core.Cars.RoadAttitude.Flat
                    : new StintegyEVO.Core.Cars.RoadAttitude(under.Grade, under.BankSlopeAt(car.Progress.CurrentD), under.VerticalRate);
                var est = ((StintegyEVO.Core.Cars.ICarEnvelope)car.Car).EstimateSteering(state, road);
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
                        state.Telemetry.ActualLateralAccel, est.FrontGripAccel, est.RearGripAccel, grip.FrontGripAccel, grip.RearGripAccel, state.Telemetry.ActualLongitudinalAccel, est.LateralDemand(k)
                    }.Select(x => x.ToString("F6", CultureInfo.InvariantCulture)))));
            });
            rows[j] = mine;
        });
        File.WriteAllLines(output, new[] { "circuit,lane,t,s,speed,command_accel,line_k,cmd_k,actual_k,error,steer,kin,slip_full,load,fb,est_angle,af,ar,front_long,rear_long,front_lat,rear_lat,lat,front_grip,rear_grip,car_front_grip,car_rear_grip,long,demand" }
            .Concat(rows.SelectMany(r => r)));
        _output.WriteLine($"{rows.Sum(r => r.Count)} steps to {output}");
    }

    /// <summary>The learned corner factors, and the braking ones too if asked, put back to one: the plan on the envelope alone.</summary>
    private static void FlatFactors(LaneTrackFile file, TrackData track, bool brakes)
    {
        file.LanesFor(track);
        for (int lane = 0; lane < LaneSet.LaneCount; lane++)
            for (int level = 0; level < LaneTrackFile.GripLevels.Length; level++)
            {
                Array.Fill(file.Factors((LaneId)lane, level), 1f);
                if (brakes)
                    Array.Fill(file.BrakeFactors((LaneId)lane, level), 1f);
            }
    }

    /// <summary>
    /// The stock car with its setup changed, "Name=value;Name=value": a
    /// CarConfig property, or the aero's FrontDownforceShare. A copy for a
    /// probe; the car itself is not changed.
    /// </summary>
    internal static StintegyEVO.Core.Cars.CarConfig CarWith(string spec)
    {
        const System.Reflection.BindingFlags flags = System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance;
        object Copy(object o) => typeof(object).GetMethod("MemberwiseClone", flags)!.Invoke(o, null)!;
        var config = (StintegyEVO.Core.Cars.CarConfig)Copy(StockCar.Config);
        foreach (string part in spec.Split(';', StringSplitOptions.RemoveEmptyEntries))
        {
            string[] kv = part.Split('=');
            float value = float.Parse(kv[1], CultureInfo.InvariantCulture);
            if (kv[0] == "FrontDownforceShare")
            {
                object aero = Copy(config.Aero);
                aero.GetType().GetProperty("FrontDownforceShare")!.SetValue(aero, (float?)value);
                typeof(StintegyEVO.Core.Cars.CarConfig).GetProperty("Aero")!.SetValue(config, aero);
            }
            else
            {
                System.Reflection.PropertyInfo property = typeof(StintegyEVO.Core.Cars.CarConfig).GetProperty(kv[0])!;
                property.SetValue(config, property.PropertyType == typeof(float?) ? (float?)value : value);
            }
        }
        return config;
    }

    [Fact]
    public void Laps()
    {
        if (!LaneProbes.Enabled) return;
        string[] only = Environment.GetEnvironmentVariable("STINTEGY_LEAD_CIRCUITS")?.Split(',') ?? Circuits;
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "lead-laps.csv");
        var old = new LaneDriverGains { LeadFromTurnIn = false, SmoothHeading = false, UseNotes = true, FadeByAsk = false };
        var ways = new (string Name, LaneDriverGains Gains)[]
        {
            ("old hands", old),
            ("old hands without notes", old with { UseNotes = false }),
            ("as they are with notes", new LaneDriverGains { UseNotes = true }),
            ("as they are", new LaneDriverGains()),
            ("as they are but small-step lag", new LaneDriverGains { LeadFromTurnIn = false }),
            ("as they are but chord heading", new LaneDriverGains { SmoothHeading = false }),
            ("lock for rotation", new LaneDriverGains { LockForRotation = true }),
            ("lock where the car is", new LaneDriverGains { LockWhereTheCarIs = true }),
            ("lock for both", new LaneDriverGains { LockForRotation = true, LockWhereTheCarIs = true }),
            ("as they are but faded by all the grip", new LaneDriverGains { FadeByAsk = false }),
            ("car FrontLateralLoadTransferShare=0.60", new LaneDriverGains()),
            ("car FrontLateralLoadTransferShare=0.65", new LaneDriverGains()),
            ("car FrontLateralLoadTransferShare=0.70", new LaneDriverGains()),
            ("car FrontDownforceShare=0.44", new LaneDriverGains()),
            ("car FixedFrontBrakeShare=0.60", new LaneDriverGains()),
        };
        var jobs = (from c in only from side in new[] { 0f, 1f, -1f } from w in ways select (c, side, w)).ToArray();
        var rows = new string[jobs.Length];
        Parallel.For(0, jobs.Length, j =>
        {
            (string circuit, float side, var way) = jobs[j];
            TrackData track = CircuitLibrary.Load(circuit);
            LaneTrackFile file = LaneTrackFile.ForCircuit(circuit, track);
            if (way.Name.StartsWith("flat"))
                FlatFactors(file, track, brakes: way.Name == "flat factors");
            LaneShares? shares = way.Name.StartsWith("rear turn-in ")
                ? new LaneShares { RearTurnIn = float.Parse(way.Name["rear turn-in ".Length..], CultureInfo.InvariantCulture) }
                : way.Name.StartsWith("brakes add ")
                ? new LaneShares { RearBrakeAdds = float.Parse(way.Name["brakes add ".Length..], CultureInfo.InvariantCulture) }
                : null;
            StintegyEVO.Core.Cars.CarConfig? config = way.Name.StartsWith("car ") ? CarWith(way.Name["car ".Length..]) : null;
            LaneLapResult r = LaneLapRunner.Run(track, file, way.Gains, shares, carConfig: config, onStep: side == 0f ? null : (car, d, t) =>
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
