using System;
using System.IO;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Drivers.Learned;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The catalogue is the only thing that knows which driver drives which
/// circuit, so what it does when asked for one that is not there matters
/// as much as what it does when it is.
/// </summary>
public sealed class DriverCatalogTests
{
    private const string TwoCircuits = """
        [
          { "car": "default", "track": "silverstone",
            "weights": "silverstone-expert.nn", "formatVersion": 2,
            "certifiedOn": "2026-09-09", "cleanLapSeconds": 101.4 },
          { "car": "default", "track": "monaco",
            "weights": "monaco-expert.nn", "formatVersion": 2 }
        ]
        """;

    private static string RepositoryRoot()
    {
        DirectoryInfo? at = new(Directory.GetCurrentDirectory());
        while (at != null &&
               !File.Exists(Path.Combine(at.FullName, "StintegyEVO.sln")))
        {
            at = at.Parent;
        }
        Assert.NotNull(at);
        return at!.FullName;
    }

    [Fact]
    public void ACircuitWithNoDriverSaysSoRatherThanSubstitutingOne()
    {
        DriverCatalog catalog = DriverCatalog.Parse(TwoCircuits);

        InvalidOperationException error =
            Assert.Throws<InvalidOperationException>(
                () => catalog.Load(
                    DriverCatalog.DefaultCar,
                    "spa",
                    _ => Array.Empty<byte>()
                )
            );

        // The message has to name the circuit and say what is available,
        // because the alternative -- quietly handing back some other
        // driver -- would put a player against something nobody certified
        // while the game said nothing at all.
        Assert.Contains("spa", error.Message);
        Assert.Contains("silverstone", error.Message);
        Assert.Contains("monaco", error.Message);
    }

    [Fact]
    public void ThePlayableCircuitsAreExactlyTheOnesWithDrivers()
    {
        DriverCatalog catalog = DriverCatalog.Parse(TwoCircuits);
        Assert.Equal(
            new[] { "monaco", "silverstone" },
            catalog.PlayableTracks(DriverCatalog.DefaultCar)
        );
        Assert.Empty(catalog.PlayableTracks("some-other-car"));
    }

    [Fact]
    public void GraduationFiguresSurviveTheRoundTrip()
    {
        DriverCatalog catalog = DriverCatalog.Parse(TwoCircuits);
        Assert.True(
            catalog.TryFind(
                DriverCatalog.DefaultCar,
                "silverstone",
                out DriverCatalogEntry entry
            )
        );
        Assert.Equal("2026-09-09", entry.CertifiedOn);
        Assert.Equal(101.4f, entry.CleanLapSeconds, 3);
        // A circuit whose driver has not been certified yet says nothing
        // rather than zero-as-a-time.
        Assert.True(
            catalog.TryFind(DriverCatalog.DefaultCar, "monaco", out entry)
        );
        Assert.Null(entry.CertifiedOn);
    }

    [Fact]
    public void TwoDriversForTheSameCarAndCircuitIsAnError()
    {
        const string duplicated = """
            [
              { "car": "default", "track": "silverstone",
                "weights": "a.nn", "formatVersion": 2 },
              { "car": "default", "track": "silverstone",
                "weights": "b.nn", "formatVersion": 2 }
            ]
            """;
        InvalidOperationException error =
            Assert.Throws<InvalidOperationException>(
                () => DriverCatalog.Parse(duplicated)
            );
        Assert.Contains("twice", error.Message);
    }

    [Fact]
    public void AListedFileThatIsNotThereIsSaidPlainly()
    {
        DriverCatalog catalog = DriverCatalog.Parse(TwoCircuits);
        InvalidOperationException error =
            Assert.Throws<InvalidOperationException>(
                () => catalog.Load(
                    DriverCatalog.DefaultCar,
                    "monaco",
                    _ => Array.Empty<byte>()
                )
            );
        Assert.Contains("monaco-expert.nn", error.Message);
    }

    /// <summary>
    /// The catalogue that ships parses, and describes what is actually on
    /// disk.
    /// </summary>
    [Fact]
    public void TheShippedCatalogueParsesAndItsFilesExist()
    {
        string root = RepositoryRoot();
        string directory = Path.Combine(root, "Assets", "Drivers");
        DriverCatalog catalog = DriverCatalog.Parse(
            File.ReadAllText(Path.Combine(directory, "manifest.json"))
        );

        string[] tracks =
            catalog.PlayableTracks(DriverCatalog.DefaultCar).ToArray();
        Assert.NotEmpty(tracks);
        foreach (string track in tracks)
        {
            Assert.True(
                catalog.TryFind(
                    DriverCatalog.DefaultCar,
                    track,
                    out DriverCatalogEntry entry
                )
            );
            Assert.True(
                File.Exists(Path.Combine(directory, entry.Weights)),
                $"the catalogue lists {entry.Weights} for {track} and it is " +
                "not in Assets/Drivers"
            );
        }
    }
}
