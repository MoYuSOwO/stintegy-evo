using System;
using System.Collections.Generic;
using System.IO;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// A driver is a pack like a car is: found by its namespaced id, loaded
/// through one definition, checked against the car before the race, and
/// seated with what was learned about that car there, kept in the car's
/// pack, and with the person's own experience, kept where the save says.
/// </summary>
public sealed class DriverPackTests
{
    private static readonly ResourceId Lanes = new("someone", "lanes");
    private static readonly ResourceId Silverstone = new("official", "silverstone");

    [Fact]
    public void ADriverPackLoadsThroughItsOneDefinition()
    {
        string folder = TempFolder();
        try
        {
            WriteManifest(folder, new DriverManifest
            {
                Id = "lanes",
                Name = "Lanes",
                Assembly = typeof(DriverPackTests).Assembly.Location,
                Definition = nameof(PackDriver)
            });
            LoadedDriver driver = DriverLibrary.LoadFrom(folder, Lanes);
            Assert.Equal(Lanes, driver.Id);
            Assert.IsType<PackDriver>(driver.Definition);

            // A pack is its folder's name, written in a format the game reads.
            Assert.Throws<InvalidDataException>(() => DriverLibrary.LoadFrom(folder, new ResourceId("someone", "other")));
            WriteManifest(folder, new DriverManifest { Format = DriverManifest.CurrentFormat + 1, Id = "lanes", Assembly = "x.dll" });
            Assert.Throws<InvalidDataException>(() => DriverLibrary.LoadFrom(folder, Lanes));
            WriteManifest(folder, new DriverManifest { Id = "lanes", Assembly = "missing.dll" });
            Assert.Throws<FileNotFoundException>(() => DriverLibrary.LoadFrom(folder, Lanes));
        }
        finally
        {
            Directory.Delete(folder, recursive: true);
        }
    }

    /// <summary>
    /// Before the race: a car without what the driver needs is refused; a
    /// car the driver has no notes for, or notes learned on another car, is
    /// driven, and the pit wall is told; notes learned on the car as it is
    /// are nothing to tell.
    /// </summary>
    [Fact]
    public void TheEntryIsCheckedBeforeTheRace()
    {
        LoadedCar car = OfficialCarIn(TempFolder());
        try
        {
            LoadedDriver fussy = Driver(new CarNeeds([typeof(NoSuchFeature)], []));
            DriverEntryCheck refused = DriverLibrary.Check(fussy, car, Silverstone);
            Assert.False(refused.CanRace);

            LoadedDriver driver = Driver(CarNeeds.None);
            DriverEntryCheck unlearned = DriverLibrary.Check(driver, car, Silverstone);
            Assert.True(unlearned.CanRace);
            Assert.Single(unlearned.Notices);

            DriverNotes notes = DriverNotes.For(Lanes, car, Silverstone);
            notes.StampLearned(Lanes, car, new DateTime(2026, 10, 27));
            DriverEntryCheck learned = DriverLibrary.Check(driver, car, Silverstone);
            Assert.True(learned.CanRace);
            Assert.Empty(learned.Notices);
            Assert.True(DriverNotes.For(Lanes, car, Silverstone).IsCurrent);

            LoadedCar changed = car with { Fingerprint = "000000000000" };
            DriverEntryCheck stale = DriverLibrary.Check(driver, changed, Silverstone);
            Assert.True(stale.CanRace);
            Assert.Single(stale.Notices);
            Assert.False(DriverNotes.For(Lanes, changed, Silverstone).IsCurrent);
        }
        finally
        {
            Directory.Delete(car.Folder, recursive: true);
        }
    }

    /// <summary>
    /// Notes are kept in the car's pack under the driver and the circuit,
    /// every id split into its namespace and its name, so no colon reaches a
    /// path; and a driver's files stay inside its notes.
    /// </summary>
    [Fact]
    public void NotesAreKeptInTheCarsPackByNamespace()
    {
        LoadedCar car = OfficialCarIn(TempFolder());
        try
        {
            DriverNotes notes = DriverNotes.For(Lanes, car, Silverstone);
            Assert.Equal(
                Path.Combine(car.Folder, "drivers", "someone", "lanes", "official", "silverstone"),
                notes.Folder);
            Assert.DoesNotContain(':', Path.GetRelativePath(car.Folder, notes.Folder));
            Assert.Null(notes.Stamp);

            notes.WriteText("lanes.csv", "s,offset\n");
            Assert.Equal("s,offset\n", notes.ReadText("lanes.csv"));
            Assert.Null(notes.ReadText("nothing.csv"));
            Assert.Throws<ArgumentException>(() => notes.ReadText("../../car.json"));
            Assert.Throws<ArgumentException>(() => notes.WriteText("/tmp/elsewhere", ""));

            // Another driver, or another circuit, is another folder.
            Assert.NotEqual(notes.Folder, DriverNotes.For(new ResourceId("official", "lanes"), car, Silverstone).Folder);
            Assert.NotEqual(notes.Folder, DriverNotes.For(Lanes, car, new ResourceId("someone", "silverstone")).Folder);
        }
        finally
        {
            Directory.Delete(car.Folder, recursive: true);
        }
    }

    /// <summary>
    /// Seated, the driver is given the person, the car, the circuit, its
    /// notes there and the person's own experience of it; a car without
    /// what it needs is refused.
    /// </summary>
    [Fact]
    public void ADriverIsSeatedWithItsNotesAndThePersonsExperience()
    {
        LoadedCar car = OfficialCarIn(TempFolder());
        string saves = TempFolder();
        try
        {
            var definition = new PackDriver();
            var driver = new LoadedDriver(Lanes, new DriverManifest { Id = "lanes" }, car.Folder, definition);
            var person = new DriverProfile("ana", new DriverAbilities());
            var store = new DriverProgressFolder(saves);

            Driver seated = DriverLibrary.Seat(driver, person, car, "silverstone", store);
            Assert.Same(person, seated.Profile);
            DriverSeat seat = definition.LastSeat!;
            Assert.Equal(Silverstone, seat.Circuit);
            Assert.Equal(DriverNotes.For(Lanes, car, Silverstone).Folder, seat.Notes.Folder);
            Assert.True(seat.Track.LengthMeters > 0f);

            seat.Progress!.Write("{\"laps\":3}");
            Assert.Equal("{\"laps\":3}", store.Load(new DriverProgressKey("ana", Lanes, car.Id, Silverstone)));

            var fussy = new LoadedDriver(Lanes, new DriverManifest { Id = "lanes" }, car.Folder,
                new PackDriver(new CarNeeds([typeof(NoSuchFeature)], [])));
            Assert.Throws<CarUnfitForDriverException>(() => DriverLibrary.Seat(fussy, person, car, "silverstone"));
        }
        finally
        {
            Directory.Delete(car.Folder, recursive: true);
            Directory.Delete(saves, recursive: true);
        }
    }

    /// <summary>
    /// A person's experience is theirs, with that driver pack, in that car,
    /// at that circuit: each kept apart, every id split into folders, and a
    /// person named only as a folder can be named everywhere.
    /// </summary>
    [Fact]
    public void ExperienceIsKeptPerPersonDriverCarAndCircuit()
    {
        string saves = TempFolder();
        try
        {
            var store = new DriverProgressFolder(saves);
            ResourceId car = new("official", "formula_a_mixed");
            var key = new DriverProgressKey("ana", Lanes, car, Silverstone);
            store.Save(key, "a");
            Assert.Equal("a", store.Load(key));
            Assert.Null(store.Load(new DriverProgressKey("ben", Lanes, car, Silverstone)));
            Assert.Null(store.Load(new DriverProgressKey("ana", new ResourceId("official", "lanes"), car, Silverstone)));
            Assert.Null(store.Load(new DriverProgressKey("ana", Lanes, new ResourceId("someone", "formula_a_mixed"), Silverstone)));
            Assert.Null(store.Load(new DriverProgressKey("ana", Lanes, car, new ResourceId("official", "spa"))));

            foreach (string file in Directory.GetFiles(saves, "*", SearchOption.AllDirectories))
                Assert.DoesNotContain(':', Path.GetRelativePath(saves, file));
            Assert.Throws<ArgumentException>(() => new DriverProgressKey("Ana", Lanes, car, Silverstone));
            Assert.Throws<ArgumentException>(() => new DriverProgressKey("../x", Lanes, car, Silverstone));
        }
        finally
        {
            Directory.Delete(saves, recursive: true);
        }
    }

    private static LoadedDriver Driver(CarNeeds needs) =>
        new(Lanes, new DriverManifest { Id = "lanes" }, "", new PackDriver(needs));

    // The game's own car, as if its pack were here: notes written in a test
    // never reach the real pack.
    private static LoadedCar OfficialCarIn(string folder) =>
        CarLibrary.Load("official:formula_a_mixed") with { Folder = folder };

    private static string TempFolder()
    {
        string folder = Path.Combine(Path.GetTempPath(), "stintegy-drivers-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(folder);
        return folder;
    }

    private static void WriteManifest(string folder, DriverManifest manifest) =>
        File.WriteAllText(Path.Combine(folder, DriverManifest.FileName), manifest.ToJson());

    private interface NoSuchFeature;

    /// <summary>A driver pack's definition that remembers how it was last seated.</summary>
    public sealed class PackDriver(CarNeeds needs) : IDriverDefinition
    {
        public PackDriver() : this(CarNeeds.None)
        {
        }

        public CarNeeds Needs { get; } = needs;

        public DriverSeat? LastSeat { get; private set; }

        public IDriverController Seat(DriverSeat seat)
        {
            LastSeat = seat;
            return new Coasting();
        }

        private sealed class Coasting : IDriverController
        {
            public DriverInput GetControl(in DriverContext context, float dt) => default;
        }
    }
}
