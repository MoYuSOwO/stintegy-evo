using System.Globalization;
using System.Numerics;
using System.Text;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Bake;

/// <summary>
/// Writes what the offline 3D racing-line bake needs and nothing else: each
/// circuit's two road edges in 3D, and this car's grip as a function of speed
/// and apparent gravity, read off the same published envelope the driver
/// plans with. The bake itself runs in Python, outside the program; its
/// results come back as lane files.
/// </summary>
internal static class Program
{
    private const float StepMeters = 2f;
    private const float Gravity = 9.81f;

    public static int Main(string[] args)
    {
        string outDir = args.Length > 0
            ? Path.GetFullPath(args[0])
            : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "raw"));
        Directory.CreateDirectory(outDir);
        string[] ids = args.Length > 1 ? args[1..] : ["silverstone"];
        foreach (string id in ids)
            WriteBounds(id, Build(id), outDir);
        WriteGg(StockCar.Config, new TireConfig(), Path.Combine(outDir, "stintegy_gg.csv"));
        Console.WriteLine(outDir);
        return 0;
    }

    // Every circuit, shipped or private (private/circuits), is a folder now.
    private static TrackData Build(string id) => CircuitLibrary.Load(id);

    /// <summary>
    /// Both edges, in the bake's convention: its n is positive to the left of
    /// travel, counter-clockwise of the tangent, and our track's normal is
    /// clockwise of it, so the bake's left edge is our negative offset.
    /// Height is the grade integrated round the lap, with whatever it fails
    /// to close by spread evenly, plus the cross section at each edge.
    /// </summary>
    private static void WriteBounds(string id, TrackData track, string outDir)
    {
        int count = Math.Max(64, (int)MathF.Round(track.LengthMeters / StepMeters));
        float ds = track.LengthMeters / count;
        var z = new float[count];
        for (int i = 1; i < count; i++)
            z[i] = z[i - 1] + 0.5f * (track.Sample((i - 1) * ds).Grade + track.Sample(i * ds).Grade) * ds;
        float misclose = z[count - 1] + 0.5f * (track.Sample((count - 1) * ds).Grade + track.Sample(0f).Grade) * ds;
        for (int i = 0; i < count; i++)
            z[i] -= misclose * i / count;

        var csv = new StringBuilder();
        csv.AppendLine("right_bound_x,right_bound_y,right_bound_z,left_bound_x,left_bound_y,left_bound_z");
        // The bake wants a closed loop: the first point again at the end.
        for (int k = 0; k <= count; k++)
        {
            int i = k % count;
            TrackSample road = track.Sample(i * ds);
            float half = road.HalfWidth;
            float Height(float d) => z[i] + road.BankSlope * d + road.BankCurvature * d * d;
            Vector2 bakeRight = road.Center + road.Normal * half;   // our +d
            Vector2 bakeLeft = road.Center - road.Normal * half;    // our -d
            csv.AppendLine(string.Join(",",
                F(bakeRight.X), F(bakeRight.Y), F(Height(half)),
                F(bakeLeft.X), F(bakeLeft.Y), F(Height(-half))));
        }

        string path = Path.Combine(outDir, $"{id}_bounds_3d.csv");
        File.WriteAllText(path, csv.ToString());
        Console.WriteLine($"{id} {track.LengthMeters:0.0} m -> {path} ({count} pts, misclose {misclose:0.00} m)");
    }

    /// <summary>
    /// Grip against speed and apparent gravity. The envelope is read on new
    /// tyres at their best temperature; its grip splits into a part that
    /// goes with the load gravity puts on the car and a part the wings put
    /// on it, and only the first scales with apparent gravity.
    /// </summary>
    private static void WriteGg(CarConfig car, TireConfig tires, string path)
    {
        var state = new CarState();
        float temp = 0.5f * (tires.IdealSurfaceTempLowC + tires.IdealSurfaceTempHighC);
        foreach (TireState tyre in new[] { state.Chassis().FrontLeft, state.Chassis().FrontRight, state.Chassis().RearLeft, state.Chassis().RearRight })
        {
            tyre.SurfaceTempC = temp;
            tyre.CoreTempC = temp;
            tyre.Wear = 0f;
            tyre.SurfaceGrip = 1f;
        }
        float grip = CarPhysics.GripUsage(CarStrategy.Default);

        CarPerformanceLimits Coast(float v) => CarPhysics.EstimatePerformanceLimits(
            state, car, tires, CarStrategy.Default, v, curvature: 0f, grip);
        CarPerformanceLimits Braking(float v) => CarPhysics.EstimatePerformanceLimits(
            state, car, tires, CarStrategy.Default, v, curvature: 0f, grip,
            assumedLongitudinalAcceleration: -Coast(v).MaximumBrakeDeceleration);

        // Static share of each grip figure from the two ends of the range.
        (float A, float B) Split(Func<float, float> figure)
        {
            float lo = figure(10f), hi = figure(90f);
            float b = (hi - lo) / (90f * 90f - 10f * 10f);
            return (lo - b * 100f, b);
        }
        (float latA, _) = Split(v => Coast(v).LateralAccelerationLimit);
        (float brakeA, _) = Split(v => Braking(v).MaximumBrakeDeceleration);

        // The bake's longitudinal acceleration is the car's net one: it has no
        // drag of its own. So the air and the rolling tyres come off what the
        // motor gives and add to what the brakes do. Left out, the bake drove
        // a car with no drag, still gaining 6 m/s² at 95 m/s where this one
        // has stopped gaining, and every line ran to the table's last speed.
        //
        // The table runs to the first speed the car cannot gain at, so the
        // bake finds the car's top speed rather than the table's end.
        float top = 10f;
        while (top < 150f && Coast(top).MaximumDriveAcceleration - Coast(top).LossAcceleration > 0f)
            top += 5f;

        var csv = new StringBuilder();
        csv.AppendLine("v_mps,g_mps2,ax_max,ax_min,ay_max");
        for (float v = 10f; v <= top + 1e-3f; v += 5f)
        {
            CarPerformanceLimits coast = Coast(v);
            float brake = Braking(v).MaximumBrakeDeceleration;
            for (float g = 0.5f * Gravity; g <= 3.5f * Gravity + 1e-3f; g += 0.25f * Gravity)
            {
                float extra = g / Gravity - 1f;
                float ay = MathF.Max(0.5f, coast.LateralAccelerationLimit + latA * extra);
                float ax = MathF.Max(0.5f, MathF.Min(car.MaxBrakeAccel, brake + brakeA * extra)) + coast.LossAcceleration;
                float drive = coast.MaximumDriveAcceleration * MathF.Min(1f, g / Gravity) - coast.LossAcceleration;
                csv.AppendLine(string.Join(",", F(v), F(g), F(drive), F(-ax), F(ay)));
            }
        }
        File.WriteAllText(path, csv.ToString());
        Console.WriteLine($"gg -> {path} (static lateral {latA:0.0}, static brake {brakeA:0.0} m/s² per g, to {top:0} m/s)");
    }


    private static string F(float value) => value.ToString("0.######", CultureInfo.InvariantCulture);
}
