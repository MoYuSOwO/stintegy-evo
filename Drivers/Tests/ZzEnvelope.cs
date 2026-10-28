using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using StintegyEVO.Core.Cars;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Probe, not a test: the car's envelope measured on its own physics, the
/// way TUM's GGGVDiagrams does it (arXiv 2504.10225), beside what the car's
/// published estimate says. Speed is held, a braking demand is held, and
/// the wheel is turned slowly until the car will take no more corner; the
/// most it held steadily is the measured limit.
/// </summary>
public sealed class ZzEnvelope
{
    private readonly ITestOutputHelper _output;
    public ZzEnvelope(ITestOutputHelper output) => _output = output;

    private static string F(float v) => v.ToString("F3", CultureInfo.InvariantCulture);

    private sealed record Point(float Speed, float Demand, float Lateral, float Longitudinal, float EstimateLateral, float EstimateLateralUsage, float EstimateBrake, float PlanDecelRaw, float PlanDecelShares);

    [Fact]
    public void Measure()
    {
        if (!LaneProbes.Enabled) return;
        string output = Environment.GetEnvironmentVariable("STINTEGY_ENVELOPE_OUT") ?? Path.Combine(LaneProbes.Dir, "envelope.csv");
        float[] speeds = [20f, 30f, 40f, 50f, 60f, 70f, 80f, 90f];
        float[] demands = [0f, 5f, 10f, 15f, 20f, 25f, 30f, 40f, 60f];
        var jobs = speeds.SelectMany(v => demands.Select(d => (v, d))).ToArray();
        var points = new Point[jobs.Length];
        Parallel.For(0, jobs.Length, i => points[i] = Ramp(jobs[i].v, jobs[i].d));

        var rows = new List<string> { "speed_mps,brake_demand,measured_lat,measured_long,estimate_lat,estimate_lat_at_rung,estimate_brake,plan_decel_raw,plan_decel_shares" };
        rows.AddRange(points.Select(p => string.Join(",", new[] { p.Speed, p.Demand, p.Lateral, p.Longitudinal, p.EstimateLateral, p.EstimateLateralUsage, p.EstimateBrake, p.PlanDecelRaw, p.PlanDecelShares }.Select(F))));
        File.WriteAllLines(output, rows);
        foreach (Point p in points)
            _output.WriteLine($"v {p.Speed,3:F0}  brake asked {p.Demand,3:F0}  measured lat {p.Lateral,5:F1} long {p.Longitudinal,6:F1}  | plan allows decel {p.PlanDecelRaw,5:F1} (with shares {p.PlanDecelShares,5:F1})");
    }

    private static Point Ramp(float speed, float demand)
    {
        var model = TestCar.OnEveryCompound(TestCar.Config);
        var state = new CarState { Speed = speed };
        model.FitTires(state, DevelopmentTyres.SuperId);
        state.Chassis().Energy = PowertrainState.Filled(0.8f);
        state.Chassis().WarmTires(90f, 90f);
        CarStrategy strategy = CarStrategy.Default;
        CarState start = state.Clone();

        const float dt = 1f / 60f;
        float wheelBase = TestCar.Config.WheelBaseMeters;
        // Far enough to be past any limit: three times the angle a 6 g corner would want.
        float steerMax = 3f * wheelBase * 60f / (speed * speed) + 0.03f;
        float rampSeconds = 8f;
        int steps = (int)(rampSeconds / dt);
        var lat = new Queue<float>();
        var lon = new Queue<float>();
        const int Window = 15; // a quarter of a second: what the car holds, not what it touches
        float best = 0f, bestLong = 0f;
        for (int i = 0; i < steps; i++)
        {
            float steer = steerMax * (i + 1) / steps;
            model.Step(state, new CarStepInput(new DriverInput(steer, -demand), strategy, 25f), dt);
            if (state.Chassis().Spinning || state.Chassis().SpinEvents > 0)
                break;
            float geometric = MathF.Atan(TestCar.Config.RearAxleOffsetMeters * state.Chassis().Telemetry.ActualCurvature);
            if (MathF.Abs(state.SideslipAngleRadians - geometric) > 0.15f)
                break;
            lat.Enqueue(MathF.Abs(state.Chassis().Telemetry.ActualLateralAccel));
            lon.Enqueue(-state.Chassis().Telemetry.ActualLongitudinalAccel);
            if (lat.Count > Window) { lat.Dequeue(); lon.Dequeue(); }
            if (lat.Count == Window && lat.Average() > best)
            {
                best = lat.Average();
                bestLong = lon.Average();
            }
            // Held at speed: the virtual force that stands in for the braking.
            state.Speed = speed;
        }

        var envelope = (ICarEnvelope)model;
        float Settled(float usage)
        {
            float a = envelope.EstimatePerformanceLimits(start, strategy, speed, 0f, usage, -bestLong).LateralAccelerationLimit;
            for (int k = 0; k < 6; k++)
                a = envelope.EstimatePerformanceLimits(start, strategy, speed, a / (speed * speed), usage, -bestLong).LateralAccelerationLimit;
            return a;
        }
        float brake = envelope.EstimatePerformanceLimits(start, strategy, speed, 0f, 1f).MaximumBrakeDeceleration;
        // What the driver's own plan allows at the corner the car held: the
        // deceleration it would plan on an arc of that curvature at that speed.
        LaneEnvelope plan = LaneEnvelope.From(start, envelope, (ICarGripFeel)model, strategy, turnAware: true);
        float Allowed(LaneShares shares)
        {
            if (best <= 0f) return float.NaN;
            const float ds = 0.5f;
            float k = best / (speed * speed);
            Span<float> v = stackalloc float[2];
            LaneSpeedPlanner.PlanWindow(plan, [k, k], [ds, ds], [1f, 1f], [1f, 1f], ReadOnlySpan<LaneRoad>.Empty, shares, speed, v);
            return v[0] < speed - 1e-3f ? float.NaN : (v[0] * v[0] - speed * speed) / (2f * ds);
        }
        return new Point(speed, demand, best, bestLong, Settled(1f), Settled(envelope.GripUsage(strategy)), brake,
            Allowed(new LaneShares { Lateral = 1f, Brake = 1f }), Allowed(new LaneShares()));
    }
}
