using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;
using StintegyEVO.Core.Content;

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
/// What the circuit's asphalt is like, as multipliers on an average
/// Grand Prix surface: how much grip it gives and how hard it is on
/// rubber. One and one is the average circuit. These are the road's own
/// and do not change through a race; rubber laid down and washed off is
/// the race's dynamic layer, <c>RaceEnvironment.SurfaceGripScalar</c>.
/// </summary>
public sealed record CircuitSurface
{
    /// <summary>The asphalt's grip, on the racing surface and the kerbs; the grass does not change.</summary>
    public float Grip { get; init; } = 1f;

    /// <summary>How fast the asphalt takes rubber off: a multiplier on tyre wear.</summary>
    public float Abrasion { get; init; } = 1f;

    /// <summary>Where the two numbers come from, in words.</summary>
    public string Basis { get; init; } = "";
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
    public CircuitSurface Surface { get; init; } = new();
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
        TrackData track = builder.Build(new TrackGridConfig
        {
            StartingLineIdx = manifest.StartLineMeters,
            GridCount = manifest.Grid.Count,
            IsFirstGridLeft = manifest.Grid.FirstLeft,
            GridOffset = manifest.Grid.OffsetMeters
        });
        track.AsphaltGrip = manifest.Surface.Grip;
        track.AsphaltAbrasion = manifest.Surface.Abrasion;
        return track;
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
/// The circuits on disk: namespaced packs like cars and tyres, each a folder
/// <c>&lt;root&gt;/&lt;namespace&gt;/circuits/&lt;name&gt;/</c> holding its
/// <c>circuit.json</c>, found by id (<c>official:silverstone</c>; a bare
/// name is the game's own). The roots default to the data folders cars are
/// found in; a host whose files are not plain files (a packed Godot build)
/// sets <see cref="Roots"/> and <see cref="ReadText"/>.
/// </summary>
public static class CircuitLibrary
{
    public const string CircuitsFolder = "circuits";

    /// <summary>Where to look, in order. The first root holding the circuit wins.</summary>
    public static List<string> Roots { get; } = new(DataFolders.Roots);

    /// <summary>Reads one file, or null when there is none.</summary>
    public static Func<string, string?> ReadText { get; set; } =
        path => File.Exists(path) ? File.ReadAllText(path) : null;

    public static CircuitManifest Manifest(string id) => Find(ResourceId.Parse(id)).Manifest;

    /// <summary>The folder the circuit was found in, for whatever else it carries beside the road: its scenery.</summary>
    public static string Folder(string id) => Find(ResourceId.Parse(id)).Folder;

    public static TrackData Load(string id)
    {
        ResourceId circuit = ResourceId.Parse(id);
        (string folder, CircuitManifest manifest) = Find(circuit);
        string csv = ReadText($"{folder}/{manifest.Track}")
            ?? throw new FileNotFoundException($"Circuit '{circuit}' has no {manifest.Track} in {folder}.");
        return CircuitTrackFile.Parse(manifest, csv);
    }

    /// <summary>Where <paramref name="id"/> is found among other roots than the library's own.</summary>
    internal static string FolderIn(IEnumerable<string> roots, string id) => Find(ResourceId.Parse(id), roots).Folder;

    private static (string Folder, CircuitManifest Manifest) Find(ResourceId id) => Find(id, Roots);

    private static (string Folder, CircuitManifest Manifest) Find(ResourceId id, IEnumerable<string> roots)
    {
        foreach (string root in roots)
        {
            string folder = $"{root.TrimEnd('/', '\\')}/{id.Namespace}/{CircuitsFolder}/{id.Name}";
            if (ReadText($"{folder}/{CircuitManifest.FileName}") is not { } json)
                continue;
            CircuitManifest manifest = CircuitManifest.Parse(json);
            if (manifest.Format > CircuitManifest.CurrentFormat)
                throw new InvalidDataException($"Circuit '{id}' is format {manifest.Format}; this game reads up to {CircuitManifest.CurrentFormat}.");
            if (manifest.Id != id.Name)
                throw new InvalidDataException($"Circuit '{id}' calls itself '{manifest.Id}'; a pack's id is its folder's name.");
            return (folder, manifest);
        }
        throw new FileNotFoundException(
            $"No circuit '{id}' under {string.Join(", ", roots)}."
        );
    }
}
