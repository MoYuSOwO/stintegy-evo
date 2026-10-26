using System;
using System.IO;
using System.Linq;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class CircuitFolderTests
{
    /// <summary>
    /// Every circuit pack the game ships is a circuit it can load: the
    /// manifest's length is the track file's, the start line is on the lap,
    /// and the rows are a metre apart.
    /// </summary>
    [Fact]
    public void EveryShippedCircuitLoads()
    {
        ResourceId[] ids = DataFolders.Available(CircuitLibrary.CircuitsFolder, CircuitManifest.FileName)
            .Where(id => id.Namespace == ResourceId.Official)
            .ToArray();
        Assert.NotEmpty(ids);
        foreach (ResourceId id in ids)
        {
            CircuitManifest manifest = CircuitLibrary.Manifest(id.ToString());
            TrackData track = CircuitLibrary.Load(id.ToString());
            Assert.Equal(id.Name, manifest.Id);
            Assert.Equal(manifest.LengthMeters, (int)track.LengthMeters);
            Assert.InRange(manifest.StartLineMeters, 0, manifest.LengthMeters - 1);
            Assert.True(manifest.Grid.Count >= 0, $"{id}: a negative grid");
        }
    }

    /// <summary>A bare name is the game's own circuit, the same one its full id names.</summary>
    [Fact]
    public void ABareNameIsTheOfficialCircuit()
    {
        Assert.Equal(
            CircuitLibrary.Folder("official:silverstone"),
            CircuitLibrary.Folder("silverstone"));
        Assert.Equal(
            CircuitLibrary.Load("official:silverstone").LengthMeters,
            CircuitLibrary.Load("silverstone").LengthMeters);
    }

    /// <summary>
    /// Someone else's circuit is theirs, under their namespace, and never
    /// stands in for the game's own of the same name; a pack whose manifest
    /// calls it something other than its folder is refused, and a name that
    /// cannot be a folder on every system is not an id.
    /// </summary>
    [Fact]
    public void ACircuitIsFoundUnderItsOwnNamespace()
    {
        string root = Path.Combine(Path.GetTempPath(), "stintegy-circuits-" + Guid.NewGuid().ToString("N"));
        string official = CircuitLibrary.Folder("silverstone");
        try
        {
            string theirs = Path.Combine(root, "someone", CircuitLibrary.CircuitsFolder, "silverstone");
            Directory.CreateDirectory(theirs);
            CircuitManifest manifest = CircuitLibrary.Manifest("silverstone");
            File.WriteAllText(Path.Combine(theirs, CircuitManifest.FileName), manifest.ToJson());
            File.Copy(Path.Combine(official, manifest.Track), Path.Combine(theirs, manifest.Track));
            string wrong = Path.Combine(root, "someone", CircuitLibrary.CircuitsFolder, "elsewhere");
            Directory.CreateDirectory(wrong);
            File.WriteAllText(Path.Combine(wrong, CircuitManifest.FileName), manifest.ToJson());

            string[] roots = [root, .. CircuitLibrary.Roots];
            Assert.Equal(official, CircuitLibrary.FolderIn(roots, "silverstone"));
            Assert.Equal($"{root}/someone/{CircuitLibrary.CircuitsFolder}/silverstone", CircuitLibrary.FolderIn(roots, "someone:silverstone"));
            Assert.Throws<InvalidDataException>(() => CircuitLibrary.FolderIn(roots, "someone:elsewhere"));
            Assert.Throws<FileNotFoundException>(() => CircuitLibrary.FolderIn(roots, "nobody:silverstone"));
            Assert.Throws<ArgumentException>(() => CircuitLibrary.FolderIn(roots, "banked-speedway"));
        }
        finally
        {
            Directory.Delete(root, recursive: true);
        }
    }

    /// <summary>
    /// The line is where the race starts, on the run to the first slow
    /// corner, never somewhere else on the lap. A source centreline starts
    /// wherever its author began drawing it: Spa's began on the run to
    /// Blanchimont, so its grid stood in the bends and the first slow corner
    /// was the Bus Stop, 1.2 km on, instead of La Source.
    /// </summary>
    [Theory]
    [InlineData("baku")]
    [InlineData("interlagos")]
    [InlineData("monaco")]
    [InlineData("monza")]
    [InlineData("portimao")]
    [InlineData("sepang")]
    [InlineData("shanghai")]
    [InlineData("silverstone")]
    [InlineData("singapore")]
    [InlineData("spa")]
    [InlineData("zandvoort")]
    public void TheLineIsOnTheRunToTheFirstSlowCorner(string id)
    {
        const float SlowRadius = 45f;
        const float FurthestRun = 900f;
        TrackData track = CircuitLibrary.Load(id);
        float line = track.StartingLineS;
        float run = 0f;
        while (run < track.LengthMeters && MathF.Abs(track.Sample(track.WrapS(line + run)).Curvature) < 1f / SlowRadius)
            run += 1f;
        Assert.True(run < FurthestRun, $"{id}: the first corner tighter than {SlowRadius} m is {run:F0} m after the line");
    }

    /// <summary>A file that is not one row a metre is refused, not stretched to fit.</summary>
    [Fact]
    public void ATrackFileThatIsNotARowAMetreIsRefused()
    {
        var rows = Enumerable.Range(0, 400)
            .Select(i =>
            {
                // A circle of 400 rows 1.1 m apart.
                float angle = 2f * MathF.PI * i / 400f;
                float radius = 400f * 1.1f / (2f * MathF.PI);
                return new CircuitTrackFile.Row(
                    new System.Numerics.Vector2(radius * MathF.Cos(angle), radius * MathF.Sin(angle)),
                    0f, 12f, 5f, 5f, 0f, 0f);
            })
            .ToArray();
        var manifest = new CircuitManifest { Id = "stretched", LengthMeters = rows.Length };
        Assert.Throws<InvalidDataException>(() => CircuitTrackFile.Parse(manifest, CircuitTrackFile.Write(rows)));
    }
}
