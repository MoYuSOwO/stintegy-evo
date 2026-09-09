using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Drivers.Learned;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The catalogue is a scan result, and the things worth pinning about it
/// are what it does when two packs disagree and what it does when nobody
/// supplies the circuit somebody asked for.
/// </summary>
public sealed class DriverCatalogTests
{
    private static DriverPackEntry Entry(
        string pack,
        ContentPackTier tier,
        string track,
        string? certifiedOn = null,
        int bytes = 4
    ) => new(
        pack,
        tier,
        DriverCatalog.DefaultCar,
        track,
        FormatVersion: 2,
        certifiedOn,
        CleanLapSeconds: 101.4f,
        ReadWeights: () => new byte[bytes]
    );

    [Fact]
    public void ACircuitNobodySuppliesSaysSoRatherThanSubstitutingOne()
    {
        DriverCatalog catalog = DriverCatalog.Aggregate(new[]
        {
            Entry("base", ContentPackTier.Builtin, "silverstone"),
            Entry("base", ContentPackTier.Builtin, "monaco"),
        });

        InvalidOperationException error =
            Assert.Throws<InvalidOperationException>(
                () => catalog.Load(DriverCatalog.DefaultCar, "spa")
            );

        // Naming what is available matters as much as naming what is not:
        // quietly handing back some other driver would put a player against
        // something nobody certified while the game said nothing at all.
        Assert.Contains("spa", error.Message);
        Assert.Contains("silverstone", error.Message);
        Assert.Contains("monaco", error.Message);
    }

    [Fact]
    public void ThePlayableCircuitsAreExactlyTheOnesWithDrivers()
    {
        DriverCatalog catalog = DriverCatalog.Aggregate(new[]
        {
            Entry("base", ContentPackTier.Builtin, "silverstone"),
            Entry("mod", ContentPackTier.User, "monaco"),
        });
        Assert.Equal(
            new[] { "monaco", "silverstone" },
            catalog.PlayableTracks(DriverCatalog.DefaultCar)
        );
        Assert.Empty(catalog.PlayableTracks("some-other-car"));
    }

    /// <summary>
    /// Installing something is asking for it, so a player's pack beats a
    /// built-in one — and it does so whichever order the scan happened to
    /// walk them in, which is the part worth testing.
    /// </summary>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void APlayersPackBeatsTheBuiltInOneEitherWayRound(bool userFirst)
    {
        DriverPackEntry builtin =
            Entry("base", ContentPackTier.Builtin, "silverstone", "2026-01-01");
        DriverPackEntry user =
            Entry("mod", ContentPackTier.User, "silverstone", "2020-01-01");

        DriverCatalog catalog = DriverCatalog.Aggregate(
            userFirst
                ? new[] { user, builtin }
                : new[] { builtin, user }
        );

        Assert.True(
            catalog.TryFind(
                DriverCatalog.DefaultCar, "silverstone", out DriverPackEntry won
            )
        );
        Assert.Equal("mod", won.PackId);
    }

    [Fact]
    public void WithinATierTheNewerCertificateWins()
    {
        DriverCatalog catalog = DriverCatalog.Aggregate(new[]
        {
            Entry("old", ContentPackTier.User, "silverstone", "2026-01-01"),
            Entry("new", ContentPackTier.User, "silverstone", "2026-09-09"),
        });
        Assert.True(
            catalog.TryFind(
                DriverCatalog.DefaultCar, "silverstone", out DriverPackEntry won
            )
        );
        Assert.Equal("new", won.PackId);
    }

    /// <summary>
    /// A certified driver beats an uncertified one, and two uncertified
    /// ones are raised rather than guessed between: picking one silently is
    /// the same substitution the missing-driver rule exists to prevent.
    /// </summary>
    [Fact]
    public void ATieTheRulesCannotBreakIsRaisedRatherThanGuessed()
    {
        DriverCatalog certifiedWins = DriverCatalog.Aggregate(new[]
        {
            Entry("blank", ContentPackTier.User, "silverstone"),
            Entry("graded", ContentPackTier.User, "silverstone", "2026-09-09"),
        });
        Assert.True(
            certifiedWins.TryFind(
                DriverCatalog.DefaultCar, "silverstone", out DriverPackEntry won
            )
        );
        Assert.Equal("graded", won.PackId);

        InvalidOperationException error =
            Assert.Throws<InvalidOperationException>(
                () => DriverCatalog.Aggregate(new[]
                {
                    Entry("one", ContentPackTier.User, "silverstone"),
                    Entry("two", ContentPackTier.User, "silverstone"),
                })
            );
        Assert.Contains("one", error.Message);
        Assert.Contains("two", error.Message);
    }

    [Fact]
    public void ADeclaredDriverWithNoWeightsIsSaidPlainly()
    {
        DriverCatalog catalog = DriverCatalog.Aggregate(new[]
        {
            Entry("hollow", ContentPackTier.User, "monaco", bytes: 0),
        });
        InvalidOperationException error =
            Assert.Throws<InvalidOperationException>(
                () => catalog.Load(DriverCatalog.DefaultCar, "monaco")
            );
        Assert.Contains("hollow", error.Message);
    }

    /// <summary>
    /// A driver with no sidecar still loads. It claims no contract and no
    /// certificate, so it loses every conflict it takes part in — but
    /// refusing to load a mod over a missing metadata file would be worse
    /// than carrying the unknown honestly.
    /// </summary>
    [Fact]
    public void ADriverWithoutASidecarLoadsAndVouchesForNothing()
    {
        DriverPackEntry[] entries = DriverPackScanner.Entries(
            new ContentPackManifest("mod", DriverCatalog.DefaultCar),
            ContentPackTier.User,
            new[] { "monaco" },
            _ => null,
            _ => new byte[4]
        ).ToArray();

        Assert.Single(entries);
        Assert.Equal(0, entries[0].FormatVersion);
        Assert.Null(entries[0].CertifiedOn);
    }

    /// <summary>
    /// Pack zero declares itself exactly the way a mod would, and what is
    /// on disk is what it says.
    /// </summary>
    [Fact]
    public void PackZeroGoesThroughTheSameDoorAsAMod()
    {
        DirectoryInfo? at = new(Directory.GetCurrentDirectory());
        while (at != null &&
               !File.Exists(Path.Combine(at.FullName, "StintegyEVO.sln")))
        {
            at = at.Parent;
        }
        Assert.NotNull(at);

        string pack = Path.Combine(at!.FullName, "Assets", "Packs", "000-base");
        ContentPackManifest manifest = DriverPackScanner.ReadManifest(
            File.ReadAllText(Path.Combine(pack, "pack.json")),
            pack
        );
        Assert.Equal(DriverCatalog.DefaultCar, manifest.Car);

        string drivers = Path.Combine(pack, "drivers");
        List<string> tracks = Directory
            .EnumerateFiles(drivers, "*.nn")
            .Select(Path.GetFileNameWithoutExtension)
            .Select(name => name!)
            .ToList();
        Assert.NotEmpty(tracks);

        DriverCatalog catalog = DriverCatalog.Aggregate(
            DriverPackScanner.Entries(
                manifest,
                ContentPackTier.Builtin,
                tracks,
                track =>
                {
                    string sidecar = Path.Combine(drivers, track + ".json");
                    return File.Exists(sidecar)
                        ? File.ReadAllText(sidecar)
                        : null;
                },
                track => File.ReadAllBytes(Path.Combine(drivers, track + ".nn"))
            )
        );

        Assert.Equal(
            tracks.OrderBy(t => t, StringComparer.Ordinal).ToArray(),
            catalog.PlayableTracks(DriverCatalog.DefaultCar)
        );
    }
}
