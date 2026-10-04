using System.Reflection;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers;

public static class RuleLineAssets
{
    public static GeometricRacingLine LoadOrBake(RuleTrackPack pack, TrackData track, float vehicleHalfWidthMeters) =>
        LoadOrBake(pack, track.Project, track.Sample, track.WrapS, track.LengthMeters, vehicleHalfWidthMeters);

    public static GeometricRacingLine LoadOrBake(
        RuleTrackPack pack,
        DriverTrackView track,
        float vehicleHalfWidthMeters
    ) =>
        LoadOrBake(pack, track.Project, track.Sample, track.WrapS, track.LengthMeters, vehicleHalfWidthMeters);

    public static GeometricRacingLine LoadOrBake(
        RuleTrackPack pack,
        Func<System.Numerics.Vector2, TrackPose> project,
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        float lengthMeters,
        float vehicleHalfWidthMeters
    )
    {
        string? path = FindCsv(pack.Id);
        if (path is not null)
            return GeometricRacingLine.FromBakedCsv(path, project, sample, wrap, lengthMeters);
        return GeometricRacingLine.Bake(sample, lengthMeters, wrap, vehicleHalfWidthMeters);
    }

    public static string? FindCsv(string id)
    {
        string file = $"{id}.line.csv";
        foreach (string candidate in CandidatePaths(file))
        {
            if (File.Exists(candidate))
                return candidate;
        }

        return ExtractEmbedded(file);
    }

    private static IEnumerable<string> CandidatePaths(string file)
    {
        string baseDir = AppContext.BaseDirectory;
        yield return Path.Combine(baseDir, "Tracks", file);
        yield return Path.Combine(baseDir, file);
        string? dir = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);
        if (dir is not null)
            yield return Path.Combine(dir, "Tracks", file);
        yield return Path.GetFullPath(Path.Combine(baseDir, "..", "..", "..", "..", "Tracks", file));
    }

    private static string? ExtractEmbedded(string file)
    {
        Assembly assembly = typeof(RuleLineAssets).Assembly;
        string name = $"StintegyEVO.Drivers.Tracks.{file}";
        using Stream? stream = assembly.GetManifestResourceStream(name);
        if (stream is null)
            return null;
        string temp = Path.Combine(Path.GetTempPath(), "stintegy-rule-lines", file);
        Directory.CreateDirectory(Path.GetDirectoryName(temp)!);
        using FileStream output = File.Create(temp);
        stream.CopyTo(output);
        return temp;
    }
}
