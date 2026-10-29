using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Saves;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The whole way in, as the game takes it: the official mod and a player's
/// mod with a driver, loaded into one registry; the driver seated by the
/// registry in the official car at a real circuit; and the race run until
/// the car has been all the way round.
/// </summary>
public sealed class OfficialModRaceTests : IDisposable
{
    private readonly string _root = Path.Combine(Path.GetTempPath(), "stintegy-official-race-" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_root))
            Directory.Delete(_root, recursive: true);
    }

    [Theory]
    [InlineData("official:silverstone")]
    [InlineData("official:monaco")]
    [InlineData("official:spa")]
    public void ARegisteredDriverTakesTheOfficialCarRoundARealCircuit(string id)
    {
        ModRegistry registry = ModLoader.Load([.. GameMods.Roots, new ModRoot(PlayersMods(), ModOrigin.User)]);
        ModCar car = registry.Cars.Get("official:formula_a_mixed");
        ResourceId circuit = ResourceId.Parse(id);
        ResourceId follower = new("alpha", "follower");
        Assert.True(registry.Check(follower, car, circuit).CanRace);

        TrackData track = registry.Circuits.Get(circuit).Track;
        var person = new DriverProfile("tester", new DriverAbilities());
        Driver driver = registry.Seat(follower, person, car, circuit, new ThrowawaySave());
        TrackSample start = track.Sample(track.StartingLineS);
        var entry = new RaceCar("car-1", car.Car, driver, new CarState { Position = start.Center, Heading = start.Heading });
        var simulation = new RaceSimulation(track, seed: 1);
        simulation.AddCar(entry);
        ((ICarStores)car.Car).Fill(entry.State, PowertrainState.Filled(0.8f));

        for (int i = 0; i < 60 * 600 && entry.Progress.TotalDistance <= track.LengthMeters; i++)
            simulation.Step(1f / 60f);

        Assert.True(entry.Progress.TotalDistance > track.LengthMeters,
            $"{id}: covered {entry.Progress.TotalDistance:F0} m of a {track.LengthMeters:F0} m lap");
    }

    // The registry tests' alpha fixture, as a player's mod in a folder of its own.
    private string PlayersMods()
    {
        string mods = Directory.CreateDirectory(Path.Combine(_root, "mods")).FullName;
        string alpha = Directory.CreateDirectory(Path.Combine(mods, "alpha", "code")).FullName;
        string built = Fixture("alpha");
        File.Copy(Path.Combine(built, "alpha_mod.dll"), Path.Combine(alpha, "alpha_mod.dll"));
        File.WriteAllText(Path.Combine(mods, "alpha", ModFormat.ManifestFile), """
            { "format": 1, "id": "alpha", "name": "alpha", "version": "1.0.0",
              "assembly": "code/alpha_mod.dll", "entrypoint": "Alpha.AlphaMod" }
            """);
        return mods;
    }

    private static string Fixture(string name)
    {
        for (DirectoryInfo? dir = new(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            string fixtures = Path.Combine(dir.FullName, "ModFixtures", name);
            if (dir.Name == "bin" && Directory.Exists(fixtures))
                return fixtures;
        }
        throw new DirectoryNotFoundException("The mod fixtures are built beside the tests, under bin/ModFixtures.");
    }
}
