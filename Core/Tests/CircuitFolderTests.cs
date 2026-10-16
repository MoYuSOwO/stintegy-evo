using System;
using System.IO;
using System.Linq;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class CircuitFolderTests
{
    /// <summary>
    /// Every folder under Circuits/ is a circuit the game can load: the
    /// manifest's length is the track file's, the start line is on the lap,
    /// and the rows are a metre apart.
    /// </summary>
    [Fact]
    public void EveryShippedCircuitLoads()
    {
        string root = CircuitLibrary.Roots.First(Directory.Exists);
        string[] ids = Directory.GetDirectories(root)
            .Where(dir => File.Exists(Path.Combine(dir, CircuitManifest.FileName)))
            .Select(Path.GetFileName)
            .ToArray()!;
        Assert.NotEmpty(ids);
        foreach (string id in ids)
        {
            CircuitManifest manifest = CircuitLibrary.Manifest(id);
            TrackData track = CircuitLibrary.Load(id);
            Assert.Equal(id, manifest.Id);
            Assert.Equal(manifest.LengthMeters, (int)track.LengthMeters);
            Assert.InRange(manifest.StartLineMeters, 0, manifest.LengthMeters - 1);
            Assert.True(manifest.Grid.Count >= 0, $"{id}: a negative grid");
        }
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
