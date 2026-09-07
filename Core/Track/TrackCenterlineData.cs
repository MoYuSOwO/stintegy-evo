using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Reflection;

namespace StintegyEVO.Core.Track;

internal readonly record struct TrackCenterlinePoint(
    Vector2 Center,
    float RightWidth,
    float LeftWidth
)
{
    public float Width => RightWidth + LeftWidth;
}

internal static class TrackCenterlineData
{
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        SilverstonePoints = new(() => LoadEmbeddedCsv("Silverstone.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        MonacoPoints = new(() => LoadEmbeddedCsv(
            "Monaco.csv",
            startingPointIndex: 118
        ));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        ShanghaiPoints = new(() => LoadEmbeddedCsv("Shanghai.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        SepangPoints = new(() => LoadEmbeddedCsv("Sepang.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        ZandvoortPoints = new(() => LoadEmbeddedCsv("Zandvoort.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        BakuPoints = new(() => LoadEmbeddedCsv("Baku.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        SpaPoints = new(() => LoadEmbeddedCsv("Spa.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        MonzaPoints = new(() => LoadEmbeddedCsv("Monza.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        InterlagosPoints = new(() => LoadEmbeddedCsv("Interlagos.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        SingaporePoints = new(() => LoadEmbeddedCsv("Singapore.csv"));
    private static readonly Lazy<IReadOnlyList<TrackCenterlinePoint>>
        PortimaoPoints = new(() => LoadEmbeddedCsv("Portimao.csv"));

    public static IReadOnlyList<TrackCenterlinePoint> Silverstone =>
        SilverstonePoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Monaco =>
        MonacoPoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Shanghai =>
        ShanghaiPoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Sepang =>
        SepangPoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Zandvoort =>
        ZandvoortPoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Baku =>
        BakuPoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Spa =>
        SpaPoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Monza =>
        MonzaPoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Interlagos =>
        InterlagosPoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Singapore =>
        SingaporePoints.Value;
    public static IReadOnlyList<TrackCenterlinePoint> Portimao =>
        PortimaoPoints.Value;

    private static IReadOnlyList<TrackCenterlinePoint> LoadEmbeddedCsv(
        string fileName,
        int startingPointIndex = 0
    )
    {
        Assembly assembly = typeof(TrackCenterlineData).Assembly;
        string resourceName = $"StintegyEVO.Core.Track.Data.{fileName}";
        using Stream stream = assembly.GetManifestResourceStream(resourceName)
            ?? throw new InvalidOperationException(
                $"Embedded track centerline '{resourceName}' was not found."
            );
        using StreamReader reader = new(stream);

        List<TrackCenterlinePoint> points = [];
        while (reader.ReadLine() is { } line)
        {
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith('#'))
                continue;

            string[] fields = line.Split(',');
            if (fields.Length < 4)
                throw new InvalidDataException(
                    $"Track centerline row has {fields.Length} fields; four are required."
                );

            float x = Parse(fields[0]);
            float y = Parse(fields[1]);
            float rightWidth = Parse(fields[2]);
            float leftWidth = Parse(fields[3]);
            points.Add(
                new TrackCenterlinePoint(
                    new Vector2(x, y),
                    rightWidth,
                    leftWidth
                )
            );
        }

        if (points.Count < 3)
            throw new InvalidDataException(
                $"Track centerline '{resourceName}' contains too few points."
            );
        if (startingPointIndex < 0 || startingPointIndex >= points.Count)
            throw new InvalidDataException(
                $"Track centerline '{resourceName}' has no point {startingPointIndex}."
            );
        if (startingPointIndex == 0)
            return points;

        // Point 118 is the closest source point to Monaco's modern
        // start/finish line on Boulevard Albert 1er. The source GeoJSON begins
        // near Massenet/Casino instead, so rotate without changing geometry.
        TrackCenterlinePoint[] rotated = new TrackCenterlinePoint[points.Count];
        for (int i = 0; i < points.Count; i++)
            rotated[i] = points[(startingPointIndex + i) % points.Count];
        return rotated;
    }

    private static float Parse(string value) =>
        float.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);
}
