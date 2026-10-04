using System.Globalization;
using System.Text;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Bake;

internal static class Program
{
    public static int Main(string[] args)
    {
        string outDir = args.Length > 0
            ? args[0]
            : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "raw"));
        Directory.CreateDirectory(outDir);
        WriteCenterline("silverstone", TrackFactory.SilverstoneStyleTestTrack(), 8f, outDir);
        WriteCenterline("simple", TrackFactory.SimpleTestTrack(), 5f, outDir);
        Console.WriteLine(outDir);
        return 0;
    }

    private static void WriteCenterline(string id, TrackData track, float stepMeters, string outDir)
    {
        int count = Math.Max(32, (int)MathF.Round(track.LengthMeters / stepMeters));
        float ds = track.LengthMeters / count;
        var csv = new StringBuilder();
        csv.AppendLine("x_m,y_m,w_tr_right_m,w_tr_left_m,banking_rad");
        for (int i = 0; i < count; i++)
        {
            TrackSample road = track.Sample(i * ds);
            csv.AppendLine(string.Join(
                ",",
                F(road.Center.X),
                F(road.Center.Y),
                F(road.HalfWidth),
                F(road.HalfWidth),
                F(MathF.Atan(road.BankSlope))
            ));
        }

        string path = Path.Combine(outDir, $"{id}_centerline_banking.csv");
        File.WriteAllText(path, csv.ToString());
        Console.WriteLine($"{id} {track.LengthMeters:0.0} m -> {path} ({count} pts)");
    }

    private static string F(float value) =>
        value.ToString("0.######", CultureInfo.InvariantCulture);
}
