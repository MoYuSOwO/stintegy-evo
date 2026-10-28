using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Numerics;
using System.Text.Json;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// One registry for every mod: found in the game's folders and the
/// player's, put in order by what each depends on, its code loaded from
/// its bytes, its listed data read by the game's readers, its entrypoint
/// run, then frozen. Each test makes its own small mods and throws them away.
/// </summary>
public sealed class ModRegistryTests : IDisposable
{
    private readonly string _root = Path.Combine(Path.GetTempPath(), "stintegy-mods-" + Guid.NewGuid().ToString("N"));

    public ModRegistryTests() => Directory.CreateDirectory(_root);

    public void Dispose() => Directory.Delete(_root, recursive: true);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AModOfDataAloneRegistersWhatItListsPackedOrNot(bool packed)
    {
        string game = Folder("game");
        var files = new Dictionary<string, byte[]>
        {
            ["data/circuits/ring/circuit.json"] = Text(Ring("ring", 200).ToJson()),
            ["data/circuits/ring/track.csv"] = Text(RingTrack(200)),
            ["data/circuits/unlisted/circuit.json"] = Text(Ring("unlisted", 200).ToJson()),
            ["data/circuits/unlisted/track.csv"] = Text(RingTrack(200)),
            ["data/tyres/slicks.json"] = Text(OfficialTyres().Replace("\"formula_a\"", "\"slicks\"")),
            ["props/props.json"] = Text("""{ "props": { "cone": { "model": "models/cone.glb", "scale": 0.5 } } }"""),
            ["models/cone.glb"] = [1, 2, 3, 4],
        };
        WriteMod(game, Manifest("trackside", extra: new()
        {
            ["circuits"] = new[] { "data/circuits/ring" },
            ["tyres"] = new[] { "data/tyres/slicks.json" },
            ["props"] = new[] { "props/props.json" },
        }), files, packed);

        ModRegistry registry = ModLoader.Load([new ModRoot(game, ModOrigin.Game)]);

        Assert.True(registry.IsFrozen);
        ModCircuit ring = registry.Circuits.Get("trackside:ring");
        Assert.Equal(ring.Manifest.LengthMeters, ring.Track.LengthMeters, 0.5);
        // Only what mod.json lists is there.
        Assert.False(registry.Circuits.Contains(new ResourceId("trackside", "unlisted")));

        LoadedTires slicks = registry.Tyres.Get("trackside:slicks");
        Assert.NotEmpty(slicks.Compounds);
        Assert.All(slicks.Compounds, c => Assert.StartsWith("trackside:slicks/", c.Id));

        ModProp cone = registry.Props.Get("trackside:cone");
        using Stream model = registry.Open($"trackside:{cone.Model}");
        Assert.Equal(4, model.Length);
        Assert.Empty(registry.Drivers.Ids);
    }

    /// <summary>A circuit's road is read when it is first used, so a bad one is found then and not at the start.</summary>
    [Fact]
    public void ACircuitsRoadIsReadWhenFirstUsed()
    {
        string game = Folder("game");
        WriteMod(game, Manifest("trackside", extra: new() { ["circuits"] = new[] { "ring" } }), new()
        {
            ["ring/circuit.json"] = Text(Ring("ring", 300).ToJson()),
            ["ring/track.csv"] = Text(RingTrack(200)),
        });

        ModRegistry registry = ModLoader.Load([new ModRoot(game, ModOrigin.Game)]);

        ModCircuit ring = registry.Circuits.Get("trackside:ring");
        Assert.Throws<InvalidDataException>(() => ring.Track);
    }

    [Fact]
    public void AModWithCodeRegistersThroughItsEntrypointAndIsKept()
    {
        string game = Folder("game");
        WriteCodeMod(game, "alpha", "Alpha.AlphaMod", packed: false);

        ModRegistry registry = ModLoader.Load([new ModRoot(game, ModOrigin.Game)]);

        Assert.True(registry.Drivers.Contains(new ResourceId("alpha", "steady")));
        Assert.Equal("alpha", registry.Drivers.Owner(new ResourceId("alpha", "steady")));
        Assert.IsAssignableFrom<IMod>(registry.Mod("alpha").Entrypoint);
        Assert.Contains(registry.Notices, n => n.StartsWith("alpha:"));
    }

    /// <summary>A packed mod whose code uses another's: it is loaded after it, and its code finds the other's.</summary>
    [Fact]
    public void AModsCodeUsesTheCodeOfAModItDependsOn()
    {
        string game = Folder("game");
        WriteCodeMod(game, "alpha", "Alpha.AlphaMod", packed: true);
        WriteCodeMod(game, "beta", "Beta.BetaMod", packed: true, depends: new() { ["alpha"] = ">=1.0.0" },
            files: new() { ["data/driver_name.txt"] = Text("follower\n") });

        ModRegistry registry = ModLoader.Load([new ModRoot(game, ModOrigin.Game)]);

        Assert.Equal(["alpha", "beta"], registry.Mods.Select(m => m.Id));
        Assert.True(registry.Drivers.Contains(new ResourceId("beta", "follower")));
        Assert.Contains(registry.Notices, n => n.StartsWith("beta:") && n.Contains("steady"));
    }

    [Fact]
    public void AModsCodeMayNotUseAModItDoesNotDependOn()
    {
        string game = Folder("game");
        WriteCodeMod(game, "alpha", "Alpha.AlphaMod", packed: false);
        WriteCodeMod(game, "beta", "Beta.BetaMod", packed: false,
            files: new() { ["data/driver_name.txt"] = Text("follower") });

        ModLoadException refused = Assert.Throws<ModLoadException>(() => ModLoader.Load([new ModRoot(game, ModOrigin.Game)]));
        Assert.Contains(refused.Problems, p => p.StartsWith("beta:") && p.Contains("alpha"));
    }

    [Fact]
    public void AMissingDependencyIsRefusedByName()
    {
        string game = Folder("game");
        WriteMod(game, Manifest("beta", depends: new() { ["alpha"] = ">=1.0.0" }), new());

        ModLoadException refused = Assert.Throws<ModLoadException>(() => ModLoader.Load([new ModRoot(game, ModOrigin.Game)]));
        Assert.Contains(refused.Problems, p => p.StartsWith("beta:") && p.Contains("alpha") && p.Contains("not installed"));
    }

    /// <summary>At least the version asked for, within the same major version.</summary>
    [Theory]
    [InlineData("1.4.0", ">=1.2.0", true)]
    [InlineData("1.1.9", ">=1.2.0", false)]
    [InlineData("2.0.0", ">=1.2.0", false)]
    public void ADependencyMustBeAtLeastTheVersionAskedWithTheSameMajor(string installed, string wanted, bool loads)
    {
        string game = Folder("game");
        WriteMod(game, Manifest("alpha", version: installed), new());
        WriteMod(game, Manifest("beta", depends: new() { ["alpha"] = wanted }), new());

        if (loads)
        {
            Assert.Equal(["alpha", "beta"], ModLoader.Load([new ModRoot(game, ModOrigin.Game)]).Mods.Select(m => m.Id));
            return;
        }
        ModLoadException refused = Assert.Throws<ModLoadException>(() => ModLoader.Load([new ModRoot(game, ModOrigin.Game)]));
        Assert.Contains(refused.Problems, p => p.StartsWith("beta:") && p.Contains(installed));
    }

    [Fact]
    public void DependenciesThatGoRoundInACircleAreRefused()
    {
        string game = Folder("game");
        WriteMod(game, Manifest("alpha", depends: new() { ["beta"] = ">=1.0.0" }), new());
        WriteMod(game, Manifest("beta", depends: new() { ["alpha"] = ">=1.0.0" }), new());
        WriteMod(game, Manifest("gamma", depends: new() { ["alpha"] = ">=1.0.0" }), new());

        ModLoadException refused = Assert.Throws<ModLoadException>(() => ModLoader.Load([new ModRoot(game, ModOrigin.Game)]));
        string circle = Assert.Single(refused.Problems);
        Assert.Contains("alpha", circle);
        Assert.Contains("beta", circle);
    }

    [Fact]
    public void AModRegistersOnlyInItsOwnNamespace()
    {
        string game = Folder("game");
        WriteCodeMod(game, "squat", "Alpha.Squatter", packed: false, assembly: "alpha");

        ModLoadException refused = Assert.Throws<ModLoadException>(() => ModLoader.Load([new ModRoot(game, ModOrigin.Game)]));
        Assert.Contains(refused.Problems, p => p.StartsWith("squat:") && p.Contains("namespace"));
    }

    [Fact]
    public void AnIdIsRegisteredOnce()
    {
        string game = Folder("game");
        WriteMod(game, Manifest("trackside", extra: new() { ["circuits"] = new[] { "a", "b" } }), new()
        {
            ["a/circuit.json"] = Text(Ring("ring", 200).ToJson()),
            ["a/track.csv"] = Text(RingTrack(200)),
            ["b/circuit.json"] = Text(Ring("ring", 200).ToJson()),
            ["b/track.csv"] = Text(RingTrack(200)),
        });

        ModLoadException refused = Assert.Throws<ModLoadException>(() => ModLoader.Load([new ModRoot(game, ModOrigin.Game)]));
        Assert.Contains(refused.Problems, p => p.Contains("trackside:ring") && p.Contains("already registered"));
    }

    /// <summary>
    /// Scenery is the one thing a mod may add to another mod's circuit; for
    /// a circuit nobody installed it is left out, and the game still starts.
    /// </summary>
    [Fact]
    public void AModAddsSceneryToAnotherModsCircuit()
    {
        string game = Folder("game");
        string user = Folder("user");
        WriteMod(game, Manifest("trackside", extra: new() { ["circuits"] = new[] { "ring" } }), new()
        {
            ["ring/circuit.json"] = Text(Ring("ring", 200).ToJson()),
            ["ring/track.csv"] = Text(RingTrack(200)),
        });
        WriteMod(user, Manifest("greener", extra: new()
        {
            ["scenery"] = new[]
            {
                new Dictionary<string, string> { ["circuit"] = "trackside:ring", ["file"] = "scenery/trees.json" },
                new Dictionary<string, string> { ["circuit"] = "elsewhere:ring", ["file"] = "scenery/trees.json" },
            },
        }), new() { ["scenery/trees.json"] = Text("{}") });

        ModRegistry registry = ModLoader.Load([new ModRoot(game, ModOrigin.Game), new ModRoot(user, ModOrigin.User)]);

        ModSceneryOverlay trees = Assert.Single(registry.SceneryFor(new ResourceId("trackside", "ring")));
        Assert.Equal("greener", trees.Mod);
        Assert.Empty(registry.SceneryFor(new ResourceId("elsewhere", "ring")));
        Assert.Contains(registry.Notices, n => n.StartsWith("greener:") && n.Contains("elsewhere:ring"));
        // Adding scenery is not owning the circuit.
        Assert.Equal("trackside", registry.Circuits.Owner(new ResourceId("trackside", "ring")));
    }

    [Fact]
    public void TheGamesOwnModWinsOverAPlayersWithTheSameId()
    {
        string game = Folder("game");
        string user = Folder("user");
        WriteMod(user, Manifest("trackside", name: "Player's"), new());
        WriteMod(game, Manifest("trackside", name: "Game's"), new());

        ModRegistry registry = ModLoader.Load([new ModRoot(user, ModOrigin.User), new ModRoot(game, ModOrigin.Game)]);

        LoadedMod loaded = Assert.Single(registry.Mods);
        Assert.Equal("Game's", loaded.Manifest.Name.In());
        Assert.Equal(ModOrigin.Game, loaded.Origin);
        Assert.Contains(registry.Notices, n => n.StartsWith("trackside:") && n.Contains(user));
    }

    [Fact]
    public void AModsFilesAreReadTheSamePackedOrNot()
    {
        var files = new Dictionary<string, byte[]> { ["data/a.txt"] = Text("one"), ["models/b.bin"] = [9, 8] };
        string folder = WriteMod(Folder("loose"), Manifest("same"), files, packed: false);
        string zip = WriteMod(Folder("packed"), Manifest("same"), files, packed: true);

        IModResources loose = ModResources.At(folder);
        IModResources packed = ModResources.At(zip);
        Assert.Equal(loose.Files, packed.Files);
        foreach (string path in loose.Files)
            Assert.Equal(loose.ReadAllBytes(path), packed.ReadAllBytes(path));
        Assert.Equal("one", packed.ReadAllText("./data\\a.txt"));
        Assert.False(packed.Exists("data/none.txt"));
        Assert.Throws<FileNotFoundException>(() => loose.Open("data/none.txt"));
        Assert.Throws<ArgumentException>(() => packed.Open("../outside.txt"));
        Assert.Throws<ArgumentException>(() => loose.Open("../outside.txt"));
    }

    [Fact]
    public void AModThatIsNotWhatItsNameSaysIsRefused()
    {
        string game = Folder("game");
        WriteMod(game, Manifest("other"), new(), name: "trackside");

        ModLoadException refused = Assert.Throws<ModLoadException>(() => ModLoader.Load([new ModRoot(game, ModOrigin.Game)]));
        Assert.Contains(refused.Problems, p => p.StartsWith("trackside:") && p.Contains("other"));
    }

    // --- Throwaway mods ---

    private string Folder(string name) => Directory.CreateDirectory(Path.Combine(_root, name)).FullName;

    private static byte[] Text(string text) => System.Text.Encoding.UTF8.GetBytes(text);

    private static Dictionary<string, object> Manifest(
        string id, string version = "1.0.0", string? name = null,
        Dictionary<string, string>? depends = null, Dictionary<string, object>? extra = null)
    {
        var manifest = new Dictionary<string, object>
        {
            ["format"] = 1,
            ["id"] = id,
            ["name"] = name ?? id,
            ["version"] = version,
            ["depends"] = depends ?? new Dictionary<string, string>(),
        };
        foreach ((string key, object value) in extra ?? [])
            manifest[key] = value;
        return manifest;
    }

    /// <summary>A mod written out as a folder, or packed; returns where it is.</summary>
    private static string WriteMod(string root, Dictionary<string, object> manifest, Dictionary<string, byte[]> files,
        bool packed = false, string? name = null)
    {
        string folder = Path.Combine(root, name ?? (string)manifest["id"]);
        Directory.CreateDirectory(folder);
        File.WriteAllText(Path.Combine(folder, ModFormat.ManifestFile), JsonSerializer.Serialize(manifest));
        foreach ((string path, byte[] bytes) in files)
        {
            string file = Path.Combine(folder, path);
            Directory.CreateDirectory(Path.GetDirectoryName(file)!);
            File.WriteAllBytes(file, bytes);
        }
        if (!packed)
            return folder;
        string zip = folder + ModFormat.Extension;
        ZipFile.CreateFromDirectory(folder, zip);
        Directory.Delete(folder, recursive: true);
        return zip;
    }

    /// <summary>A mod carrying one of the fixture assemblies (Core/Tests/ModFixtures) as its code.</summary>
    private static void WriteCodeMod(string root, string id, string entrypoint, bool packed,
        Dictionary<string, string>? depends = null, Dictionary<string, byte[]>? files = null, string? assembly = null)
    {
        string built = Path.Combine(FixturesFolder(), assembly ?? id);
        string dll = $"{assembly ?? id}_mod.dll";
        var all = new Dictionary<string, byte[]>(files ?? [])
        {
            [$"code/{dll}"] = File.ReadAllBytes(Path.Combine(built, dll)),
        };
        string pdb = Path.ChangeExtension(dll, ".pdb");
        if (File.Exists(Path.Combine(built, pdb)))
            all[$"code/{pdb}"] = File.ReadAllBytes(Path.Combine(built, pdb));
        WriteMod(root, Manifest(id, depends: depends, extra: new()
        {
            ["assembly"] = $"code/{dll}",
            ["entrypoint"] = entrypoint,
        }), all, packed);
    }

    private static string FixturesFolder()
    {
        for (DirectoryInfo? dir = new(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            string fixtures = Path.Combine(dir.FullName, "ModFixtures");
            if (dir.Name == "bin" && Directory.Exists(fixtures))
                return fixtures;
        }
        throw new DirectoryNotFoundException("The mod fixtures are built beside the tests, under bin/ModFixtures.");
    }

    private static string OfficialTyres() =>
        File.ReadAllText(Path.Combine(DataFolders.Find(new ResourceId("official", "formula_a"), TireLibrary.TyresFolder,
            TirePackManifest.FileName)!, TirePackManifest.FileName));

    // A round road of whole metres: made here for the test, registered nowhere else.
    private static CircuitManifest Ring(string id, int metres) => new()
    {
        Id = id,
        Name = id,
        LengthMeters = metres,
        StartLineMeters = 10,
        Grid = new CircuitGrid { Count = 2 },
    };

    private static string RingTrack(int metres)
    {
        float radius = metres / (2f * MathF.PI);
        var rows = Enumerable.Range(0, metres).Select(i =>
        {
            float angle = 2f * MathF.PI * i / metres;
            return new CircuitTrackFile.Row(new Vector2(radius * MathF.Cos(angle), radius * MathF.Sin(angle)), 0f, 12f, 5f, 5f, 0f, 0f);
        }).ToList();
        return CircuitTrackFile.Write(rows);
    }
}
