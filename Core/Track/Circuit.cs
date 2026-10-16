using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace StintegyEVO.Core.Track;

/// <summary>Where the grid stands, as a circuit folder writes it down.</summary>
public sealed record CircuitGrid
{
    /// <summary>How many boxes this circuit paints. A fact, not a capacity.</summary>
    public int Count { get; init; }

    /// <summary>Whether pole is on the left of the centreline.</summary>
    public bool FirstLeft { get; init; } = true;

    /// <summary>Half the lateral stagger: pole this far one side of the centreline, second this far the other.</summary>
    public float OffsetMeters { get; init; }
}

/// <summary>Where a circuit's data came from and on what terms.</summary>
public sealed record CircuitSource
{
    public string Centreline { get; init; } = "";
    public string License { get; init; } = "";
    public string LicenseFile { get; init; } = "";
    public string Notes { get; init; } = "";
}

/// <summary>
/// A circuit's <c>circuit.json</c>: everything about it that is not the
/// road itself. The road is <see cref="Track"/>, one row a metre.
/// </summary>
public sealed record CircuitManifest
{
    public const string FileName = "circuit.json";
    public const int CurrentFormat = 1;

    public int Format { get; init; } = CurrentFormat;
    public string Id { get; init; } = "";
    public string Name { get; init; } = "";

    /// <summary>The road, relative to the folder.</summary>
    public string Track { get; init; } = "track.csv";

    /// <summary>The lap, in metres: the number of rows in the track file.</summary>
    public int LengthMeters { get; init; }

    /// <summary>How far along the track file the start/finish line crosses.</summary>
    public int StartLineMeters { get; init; }

    public CircuitGrid Grid { get; init; } = new();
    public CircuitSource Source { get; init; } = new();

    internal static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        WriteIndented = true,
        DefaultIgnoreCondition = JsonIgnoreCondition.Never
    };

    public static CircuitManifest Parse(string json) =>
        JsonSerializer.Deserialize<CircuitManifest>(json, Json)
        ?? throw new InvalidDataException("circuit.json is empty.");

    public string ToJson() => JsonSerializer.Serialize(this, Json).Replace("\r\n", "\n");
}

/// <summary>
/// The road of a circuit as it is stored, and the one way it becomes a
/// <see cref="TrackData"/>.
///
/// The file is the road, finished: closed, a row a metre, the centreline in
/// the middle of the road, already smoothed and already the length it is.
/// Nothing here scales, smooths or resamples it. What is read off it is
/// only what follows from it: the heading and curvature from where the
/// points are, the gradient and the vertical bend from how high they are.
/// Anything else is done by the tool that writes the file.
/// </summary>
public static class CircuitTrackFile
{
    public static readonly string[] Columns =
    [
        "x_m", "y_m", "z_m", "width_m", "runoff_left_m", "runoff_right_m", "bank_slope", "crown"
    ];

    /// <summary>
    /// How far a row may sit from a metre after its neighbour. The writer
    /// spreads a lap that is not a whole number of metres over every row,
    /// so a row is a little more or a little less than a metre; much more
    /// than that is a file that is not one row a metre.
    /// </summary>
    public const float SpacingTolerance = 0.05f;

    public readonly record struct Row(
        Vector2 Center,
        float Z,
        float Width,
        float RunoffLeft,
        float RunoffRight,
        float BankSlope,
        float Crown
    );

    public static TrackData Parse(CircuitManifest manifest, string csv)
    {
        ArgumentNullException.ThrowIfNull(manifest);
        IReadOnlyList<Row> rows = ReadRows(csv, manifest.Id);
        if (manifest.LengthMeters != rows.Count)
            throw new InvalidDataException(
                $"Circuit '{manifest.Id}' says it is {manifest.LengthMeters} m and its track file has {rows.Count} rows."
            );
        CheckSpacing(rows, manifest.Id);
        return Build(rows, manifest);
    }

    public static IReadOnlyList<Row> ReadRows(string csv, string id = "")
    {
        ArgumentNullException.ThrowIfNull(csv);
        var rows = new List<Row>();
        int[]? index = null;
        using var reader = new StringReader(csv);
        while (reader.ReadLine() is { } line)
        {
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith('#'))
                continue;
            string[] fields = line.Split(',');
            if (index is null)
            {
                index = new int[Columns.Length];
                for (int c = 0; c < Columns.Length; c++)
                {
                    index[c] = Array.IndexOf(fields, Columns[c]);
                    if (index[c] < 0)
                        throw new InvalidDataException($"Track file of '{id}' has no '{Columns[c]}' column.");
                }
                continue;
            }
            float F(int c) => float.Parse(fields[index[c]], NumberStyles.Float, CultureInfo.InvariantCulture);
            rows.Add(new Row(new Vector2(F(0), F(1)), F(2), F(3), F(4), F(5), F(6), F(7)));
        }
        if (rows.Count < 3)
            throw new InvalidDataException($"Track file of '{id}' has {rows.Count} rows; a closed road needs at least three.");
        return rows;
    }

    private static void CheckSpacing(IReadOnlyList<Row> rows, string id)
    {
        for (int i = 0; i < rows.Count; i++)
        {
            float step = Vector2.Distance(rows[i].Center, rows[(i + 1) % rows.Count].Center);
            if (MathF.Abs(step - TrackData.StepLength) > SpacingTolerance)
                throw new InvalidDataException(
                    $"Track file of '{id}': row {i} is {step:F3} m from the next, not a metre."
                );
        }
    }

    private static TrackData Build(IReadOnlyList<Row> rows, CircuitManifest manifest)
    {
        int count = rows.Count;
        var surfaces = new TrackSurface[count];
        for (int i = 0; i < count; i++)
        {
            float ahead = rows[(i + 1) % count].Z;
            float behind = rows[(i - 1 + count) % count].Z;
            surfaces[i] = new TrackSurface(
                Grade: (ahead - behind) / (2f * TrackData.StepLength),
                BankSlope: rows[i].BankSlope,
                BankCurvature: rows[i].Crown
            );
        }

        TrackBuilder builder = TrackBuilder.FromRows(rows);
        builder.WithSurface(context =>
            surfaces[(int)MathF.Round(context.DistanceMeters / TrackData.StepLength) % count]);
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = manifest.StartLineMeters,
            GridCount = manifest.Grid.Count,
            IsFirstGridLeft = manifest.Grid.FirstLeft,
            GridOffset = manifest.Grid.OffsetMeters
        });
    }

    /// <summary>Writes rows in the form <see cref="ReadRows"/> reads.</summary>
    public static string Write(IReadOnlyList<Row> rows, string header = "")
    {
        var text = new System.Text.StringBuilder();
        if (header.Length > 0)
            text.Append("# ").Append(header).Append('\n');
        text.Append(string.Join(',', Columns)).Append('\n');
        foreach (Row row in rows)
        {
            text.Append(row.Center.X.ToString("F5", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.Center.Y.ToString("F5", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.Z.ToString("F4", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.Width.ToString("F4", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.RunoffLeft.ToString("F3", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.RunoffRight.ToString("F3", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.BankSlope.ToString("F6", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.Crown.ToString("F7", CultureInfo.InvariantCulture)).Append('\n');
        }
        return text.ToString();
    }
}

/// <summary>
/// The circuits on disk: one folder per circuit under each root, found by
/// id. The roots default to the checkout's <c>Circuits/</c> and
/// <c>private/circuits/</c>; a host whose files are not plain files (a
/// packed Godot build) sets <see cref="Roots"/> and <see cref="ReadText"/>.
/// </summary>
public static class CircuitLibrary
{
    public const string FolderName = "Circuits";
    public const string PrivateFolder = "private/circuits";

    /// <summary>Where to look, in order. The first root holding the circuit wins.</summary>
    public static List<string> Roots { get; } = DefaultRoots();

    /// <summary>Reads one file, or null when there is none.</summary>
    public static Func<string, string?> ReadText { get; set; } =
        path => File.Exists(path) ? File.ReadAllText(path) : null;

    public static CircuitManifest Manifest(string id) => Find(id).Manifest;

    public static TrackData Load(string id)
    {
        (string folder, CircuitManifest manifest) = Find(id);
        string csv = ReadText($"{folder}/{manifest.Track}")
            ?? throw new FileNotFoundException($"Circuit '{id}' has no {manifest.Track} in {folder}.");
        return CircuitTrackFile.Parse(manifest, csv);
    }

    private static (string Folder, CircuitManifest Manifest) Find(string id)
    {
        foreach (string root in Roots)
        {
            string folder = $"{root.TrimEnd('/', '\\')}/{id}";
            if (ReadText($"{folder}/{CircuitManifest.FileName}") is { } json)
                return (folder, CircuitManifest.Parse(json));
        }
        throw new FileNotFoundException(
            $"No circuit '{id}' under {string.Join(", ", Roots)}."
        );
    }

    private static List<string> DefaultRoots()
    {
        var roots = new List<string>();
        if (Environment.GetEnvironmentVariable("STINTEGY_CIRCUITS") is { Length: > 0 } configured)
            roots.AddRange(configured.Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries));
        foreach (string start in new[] { AppContext.BaseDirectory, Environment.CurrentDirectory })
        {
            for (DirectoryInfo? dir = new(start); dir is not null; dir = dir.Parent)
            {
                string circuits = Path.Combine(dir.FullName, FolderName);
                if (!Directory.Exists(circuits))
                    continue;
                if (!roots.Contains(circuits))
                {
                    roots.Add(circuits);
                    roots.Add(Path.Combine(dir.FullName, PrivateFolder));
                }
                break;
            }
        }
        return roots;
    }
}
