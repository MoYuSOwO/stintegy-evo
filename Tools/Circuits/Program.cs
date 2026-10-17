using System.Numerics;
using StintegyEVO.Circuits;
using StintegyEVO.Core.Track;

// Writes Circuits/<id>/ for every circuit the project ships: circuit.json and
// track.csv, the road finished and one row a metre. Everything that turns
// source data into a road (smoothing, scaling to the published length,
// centring, run-off, height and banking) happens here and nowhere else.
//
//   dotnet run --project Tools/Circuits -- export [root]
//   dotnet run --project Tools/Circuits -- check  [root]
//   dotnet run --project Tools/Circuits -- fetch-elevation [id ...]   (needs the network)

string repo = FindRepository();
Sources.Folder = Path.Combine(repo, "Tools", "Circuits", "Sources");
string command = args.Length > 0 ? args[0] : "export";
string outRoot = args.Length > 1 && command != "fetch-elevation" ? args[1] : Path.Combine(repo, CircuitLibrary.FolderName);

switch (command)
{
    case "export":
        foreach (CircuitSpec spec in CircuitSpecs.All)
            Export(spec, outRoot, repo);
        break;
    case "check":
    {
        // The folders are what the tool writes now, to the byte.
        string fresh = Path.Combine(Path.GetTempPath(), "stintegy-circuits-check");
        if (Directory.Exists(fresh))
            Directory.Delete(fresh, recursive: true);
        foreach (CircuitSpec spec in CircuitSpecs.All)
            Export(spec, fresh, repo);
        int failed = 0;
        foreach (string file in Directory.GetFiles(fresh, "*", SearchOption.AllDirectories))
        {
            string shipped = Path.Combine(outRoot, Path.GetRelativePath(fresh, file));
            if (!File.Exists(shipped) || !File.ReadAllBytes(shipped).SequenceEqual(File.ReadAllBytes(file)))
            {
                Console.WriteLine($"out of date: {Path.GetRelativePath(fresh, file)}");
                failed++;
            }
        }
        Console.WriteLine(failed == 0 ? "all circuits up to date" : $"{failed} files out of date; run export");
        Environment.Exit(failed == 0 ? 0 : 1);
        break;
    }
    case "fetch-elevation":
        await Elevation.Fetch(args.Length > 1 ? args[1..] : Elevation.Codes.Keys);
        break;
    default:
        Console.Error.WriteLine($"unknown command '{command}'; use export or check");
        Environment.Exit(2);
        break;
}

static void Export(CircuitSpec spec, string outRoot, string repo)
{
    TrackData track = spec.Build();
    int count = (int)MathF.Round(track.LengthMeters / TrackData.StepLength);
    var rows = new CircuitTrackFile.Row[count];
    var grades = new float[count];
    for (int i = 0; i < count; i++)
    {
        TrackSample sample = track.Sample(i * TrackData.StepLength);
        grades[i] = sample.Grade;
        rows[i] = new CircuitTrackFile.Row(
            sample.Center, 0f, sample.Width, sample.LeftBufferWidth, sample.RightBufferWidth,
            sample.BankSlope, sample.BankCurvature);
    }

    // Height from survey where the circuit has one; otherwise from the
    // layout's own gradient, closed so the lap comes back to where it left.
    float[] z = Elevation.ForTrack(spec.Id, rows.Select(r => r.Center).ToArray(), out string survey) ?? Heights(grades);
    for (int i = 0; i < count; i++)
        rows[i] = rows[i] with { Z = z[i] };

    (float shortest, float longest) = Spacing(rows);
    string respaced = "";
    if (MathF.Max(1f - shortest, longest - 1f) > CircuitTrackFile.SpacingTolerance * 0.5f)
    {
        // A seam where the layout closed short or long: spread the lap over
        // every row instead, so each is as near a metre as the rest.
        rows = Respace(rows);
        respaced = $"  respaced from {shortest:F4}..{longest:F4}";
        (shortest, longest) = Spacing(rows);
    }

    string folder = Path.Combine(outRoot, spec.Id);
    Directory.CreateDirectory(folder);
    TrackGridConfig grid = track.GridConfig;
    var manifest = new CircuitManifest
    {
        Id = spec.Id,
        Name = spec.Name,
        LengthMeters = count,
        StartLineMeters = grid.StartingLineIdx,
        Grid = new CircuitGrid
        {
            Count = grid.GridCount,
            FirstLeft = grid.IsFirstGridLeft,
            OffsetMeters = grid.GridOffset
        },
        Source = survey.Length > 0
            ? spec.Source with { Notes = spec.Source.Notes.Replace(" Elevation is authored in the style of the venue, not surveyed.", "") + " " + Elevation.Notes(spec.Id) }
            : spec.Source
    };
    string trackCsv = CircuitTrackFile.Write(rows, $"{spec.Id}: {count} m, one row a metre; written by Tools/Circuits");
    // Godot reads a .csv as a translation table unless told to leave it be.
    const string keepImport = "[remap]\n\nimporter=\"keep\"\n";
    string pitLine = "";
    if (spec.Pit is { } pit)
    {
        // Drawn against the road exactly as the game will load it, and
        // read back the way the game reads it.
        TrackData written = CircuitTrackFile.Parse(manifest, trackCsv);
        (CircuitPitLane lane, PitLaneFile.Row[] laneRows) = Pits.Build(pit, written, rows.Select(r => r.Z).ToArray());
        manifest = manifest with { PitLane = lane };
        string laneCsv = PitLaneFile.Write(laneRows, $"{spec.Id} pit lane: {lane.LengthMeters} m, one row a metre; written by Tools/Circuits");
        PitLaneFile.Parse(lane, laneCsv, written, spec.Id);
        File.WriteAllText(Path.Combine(folder, lane.Track), laneCsv);
        File.WriteAllText(Path.Combine(folder, lane.Track + ".import"), keepImport);
        pitLine = $"  pit lane {lane.LengthMeters} m, {lane.Boxes.Count} boxes";
    }
    File.WriteAllText(Path.Combine(folder, CircuitManifest.FileName), manifest.ToJson() + "\n");
    File.WriteAllText(Path.Combine(folder, manifest.Track), trackCsv);
    File.WriteAllText(Path.Combine(folder, manifest.Track + ".import"), keepImport);
    if (spec.LicenseFrom is { } license)
        File.Copy(Path.Combine(repo, license), Path.Combine(folder, spec.Source.LicenseFile), overwrite: true);

    Console.WriteLine($"{spec.Id,-16} {count,6} m  rows {shortest:F4}..{longest:F4} m apart  climb {z.Max() - z.Min():F1} m{respaced}{pitLine}");
    if (survey.Length > 0)
        Console.WriteLine($"{"",-16} {survey}");
}

static (float Shortest, float Longest) Spacing(CircuitTrackFile.Row[] rows)
{
    float shortest = float.MaxValue, longest = 0f;
    for (int i = 0; i < rows.Length; i++)
    {
        float step = Vector2.Distance(rows[i].Center, rows[(i + 1) % rows.Length].Center);
        shortest = MathF.Min(shortest, step);
        longest = MathF.Max(longest, step);
    }
    return (shortest, longest);
}

// The same closed road, the same number of rows, at equal steps along it.
static CircuitTrackFile.Row[] Respace(CircuitTrackFile.Row[] rows)
{
    int n = rows.Length;
    var along = new double[n + 1];
    for (int i = 0; i < n; i++)
        along[i + 1] = along[i] + Vector2.Distance(rows[i].Center, rows[(i + 1) % n].Center);
    double step = along[n] / n;
    var result = new CircuitTrackFile.Row[n];
    int k = 0;
    for (int j = 0; j < n; j++)
    {
        double s = j * step;
        while (k < n - 1 && along[k + 1] <= s)
            k++;
        float t = (float)((s - along[k]) / Math.Max(along[k + 1] - along[k], 1e-9));
        CircuitTrackFile.Row a = rows[k], b = rows[(k + 1) % n];
        result[j] = new CircuitTrackFile.Row(
            Vector2.Lerp(a.Center, b.Center, t),
            a.Z + (b.Z - a.Z) * t,
            a.Width + (b.Width - a.Width) * t,
            a.RunoffLeft + (b.RunoffLeft - a.RunoffLeft) * t,
            a.RunoffRight + (b.RunoffRight - a.RunoffRight) * t,
            a.BankSlope + (b.BankSlope - a.BankSlope) * t,
            a.Crown + (b.Crown - a.Crown) * t);
    }
    return result;
}

static float[] Heights(float[] grades)
{
    int n = grades.Length;
    var z = new float[n];
    double h = 0;
    for (int i = 1; i < n; i++)
    {
        h += 0.5 * (grades[i - 1] + grades[i]) * TrackData.StepLength;
        z[i] = (float)h;
    }
    double closing = h + 0.5 * (grades[n - 1] + grades[0]) * TrackData.StepLength;
    for (int i = 0; i < n; i++)
        z[i] -= (float)(closing * i / n);
    return z;
}


static string FindRepository()
{
    for (DirectoryInfo? dir = new(Environment.CurrentDirectory); dir is not null; dir = dir.Parent)
        if (File.Exists(Path.Combine(dir.FullName, "StintegyEVO.sln")))
            return dir.FullName;
    throw new DirectoryNotFoundException("Run from inside the repository.");
}

sealed record CircuitSpec(string Id, string Name, Func<TrackData> Build, CircuitSource Source, string? LicenseFrom = null, PitSpec? Pit = null);

static class CircuitSpecs
{
    private const string Tum = "TUMFTM/racetrack-database @ 2b44832e938e707103a0975ff5677a6be80362c4";
    private const string TumLicense = "Tools/Circuits/Sources/LICENSE.LGPL-3.0";
    private const string Bacinger = "bacinger/f1-circuits";
    private const string BacingerLicense = "Tools/Circuits/Sources/LICENSE.bacinger-MIT";
    private const string Authored = "Elevation is authored in the style of the venue, not surveyed.";

    private static CircuitSource FromTum(string notes) =>
        new() { Centreline = Tum, License = "LGPL-3.0", LicenseFile = "LICENSE.LGPL-3.0", Notes = notes };

    private static CircuitSource FromBacinger(string file, string notes) =>
        new() { Centreline = $"{Bacinger} ({file})", License = "MIT", LicenseFile = "LICENSE.bacinger-MIT", Notes = notes };

    private static CircuitSource Project(string notes) =>
        new() { Centreline = "Built from straights and arcs by Tools/Circuits", License = "Project content, see CONTENT_LICENSE.md", Notes = notes };

    public static readonly CircuitSpec[] All =
    [
        new("silverstone", "Silverstone (Arena Grand Prix)", Layouts.SilverstoneStyleTestTrack,
            FromTum("TUM smoothed OpenStreetMap centreline with surveyed widths, scaled to the FIA 5,891 m. " + Authored), TumLicense,
            // The Wing: inside the Hamilton straight, from the exit of Club to before Abbey.
            new PitSpec(EntryRow: -82, ExitRow: 368, Side: -1, SpeedLimitKph: 80f)),
        new("shanghai", "Shanghai International Circuit", Layouts.ShanghaiStyleTestTrack,
            FromTum("TUM centreline and satellite widths, scaled from 5,445.25 m to the FIA 5,451 m. " + Authored), TumLicense,
            // Inside the start straight, from the exit of the last hairpin to before the first corner.
            new PitSpec(EntryRow: -240, ExitRow: 400, Side: -1, SpeedLimitKph: 80f)),
        new("sepang", "Sepang International Circuit", Layouts.SepangStyleTestTrack,
            FromTum("TUM centreline scaled to 5,543 m; widths normalised to the operator's 16-22 m. " + Authored), TumLicense,
            // Inside the start straight, from the exit of the last hairpin to before the first corner.
            new PitSpec(EntryRow: -300, ExitRow: 450, Side: -1, SpeedLimitKph: 80f)),
        new("monaco", "Circuit de Monaco", Layouts.MonacoStyleTestTrack,
            FromBacinger("mc-1929 @ b8c7e17c", "Projected to local metres, rotated to start on Boulevard Albert 1er, 8-15.5 m wide by section from OpenStreetMap (ODbL), walled at the kerb, scaled to the FIA 3,337 m. " + Authored), BacingerLicense,
            // Inside Boulevard Albert 1er, from Anthony Noghes to before Sainte Devote; sixty, as at Monaco.
            new PitSpec(EntryRow: -245, ExitRow: 195, Side: -1, SpeedLimitKph: 60f)),
        new("zandvoort", "Circuit Zandvoort", Layouts.ZandvoortStyleTestTrack,
            FromBacinger("nl-1948", "Projected to local metres, about 10 m wide from OpenStreetMap (ODbL), scaled to 4,259 m; Hugenholtz and Arie Luyendijk banked at 18 degrees. " + Authored), BacingerLicense,
            // Outside the start straight, from Arie Luyendijk to before Tarzan (Hugenholtz is
            // inside it); sixty, as at Zandvoort.
            new PitSpec(EntryRow: -270, ExitRow: 300, Side: 1, SpeedLimitKph: 60f)),
        new("baku", "Baku City Circuit", Layouts.BakuStyleTestTrack,
            FromBacinger("az-2016", "Projected, rotated to start on a straight, scaled to 6,003 m; widths by section from OpenStreetMap (ODbL), 7.6 m at the castle, walled at the kerb. " + Authored), BacingerLicense,
            // Beside the long run down Neftchilar Avenue, either side of the line, to before the first corner.
            new PitSpec(EntryRow: -350, ExitRow: 400, Side: 1, SpeedLimitKph: 80f)),
        new("spa", "Circuit de Spa-Francorchamps", Layouts.SpaStyleTestTrack,
            FromBacinger("be-1925", "Projected, 13 m wide, scaled to 7,004 m, the line on the climb from the Bus Stop to La Source. " + Authored), BacingerLicense,
            // Inside the climb from the Bus Stop to La Source, either side of the line.
            new PitSpec(EntryRow: 1300, ExitRow: 1805, Side: -1, SpeedLimitKph: 80f)),
        new("monza", "Autodromo Nazionale Monza", Layouts.MonzaStyleTestTrack,
            FromBacinger("it-1922", "Projected, 12 m wide, scaled to 5,793 m. " + Authored), BacingerLicense,
            // Inside the main straight, from the tail of the Parabolica to before the Rettifilo.
            new PitSpec(EntryRow: -560, ExitRow: 300, Side: -1, SpeedLimitKph: 80f)),
        new("interlagos", "Autodromo Jose Carlos Pace", Layouts.InterlagosStyleTestTrack,
            FromBacinger("br-1940", "Projected, 12 m wide, scaled to 4,309 m. " + Authored), BacingerLicense,
            // On the left, from the climb out of Junção to before the Senna S.
            new PitSpec(EntryRow: -450, ExitRow: 240, Side: 1, SpeedLimitKph: 80f)),
        new("singapore", "Marina Bay Street Circuit", Layouts.SingaporeStyleTestTrack,
            FromBacinger("sg-2008", "Projected, rotated to start on a straight, 9-16 m wide by section from OpenStreetMap (ODbL), walled at the kerb, scaled to 4,928 m. " + Authored +
                " (SRTM reads the tower blocks along it; the published figure is 5 m, which the authored profile keeps.)"), BacingerLicense,
            // Beside the start straight, from the last corner to past the first kink; sixty, as at Marina Bay.
            new PitSpec(EntryRow: 136 - 240, ExitRow: 136 + 380, Side: -1, SpeedLimitKph: 60f)),
        new("portimao", "Autodromo Internacional do Algarve", Layouts.PortimaoStyleTestTrack,
            FromBacinger("pt-2008", "Projected, 14-18 m wide from OpenStreetMap (ODbL), scaled to 4,653 m. " + Authored), BacingerLicense,
            // Inside the start straight, from the last corner to before the first.
            new PitSpec(EntryRow: -450, ExitRow: 300, Side: -1, SpeedLimitKph: 80f)),
        new("simple", "Simple test layout", () => Layouts.SimpleTestTrack(),
            Project("Hairpin, esses, a 30 m climb and two banked corners, turning right.")),
        new("simple-left", "Simple test layout, mirrored", () => Layouts.SimpleTestTrack(isLeft: true),
            Project("The simple layout turning left.")),
        new("banked-speedway", "Banked short track", Layouts.BankedSpeedwayTestTrack,
            Project("Ninety-metre turns banked from about 18 degrees at the apron to 31 at the wall.")),
        new("banked-sweeper", "Banked sweeper", Layouts.BankedSweeperTestTrack,
            Project("Corners from 60 to 400 m radius, banked up to 18 degrees with the radius.")),
        new("flat-sweeper", "Flat sweeper", Layouts.FlatSweeperTestTrack,
            Project("The banked sweeper's layout with only drainage crossfall.")),
        new("daytona", "Superspeedway (Daytona-style)", Layouts.DaytonaStyleTestTrack,
            Project("Two 1,050 m straights and two 305 m turns, 31 degrees in the turns and 3 on the straights.")),
    ];
}
