using System;
using System.Collections.Generic;
using System.IO;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Saves;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// A driver is a mod's registered definition: checked against the car
/// before the race, and seated with what was learned about that car there,
/// kept in the car's folder, and with a slot in the save.
/// </summary>
public sealed class DriverSeatTests
{
    private static readonly ResourceId Lanes = new("someone", "lanes");
    private static readonly ResourceId Silverstone = new("official", "silverstone");

    /// <summary>
    /// Before the race: a car without what the driver needs is refused; a
    /// car the driver has no notes for, or notes learned on another car, is
    /// driven, and the pit wall is told; notes learned on the car as it is
    /// are nothing to tell.
    /// </summary>
    [Fact]
    public void TheEntryIsCheckedBeforeTheRace()
    {
        (ModCar car, string folder) = OfficialCarInTempFolder();
        try
        {
            var fussy = new PackDriver(new CarNeeds([typeof(NoSuchFeature)], []));
            DriverEntryCheck refused = ModDrivers.Check(Mods, Lanes, fussy, car, Silverstone);
            Assert.False(refused.CanRace);
            Assert.False(ModDrivers.Check(Mods, Lanes, new PackDriver(), car, new ResourceId("nobody", "nowhere")).CanRace);

            var driver = new PackDriver();
            DriverEntryCheck unlearned = ModDrivers.Check(Mods, Lanes, driver, car, Silverstone);
            Assert.True(unlearned.CanRace);
            Assert.Single(unlearned.Notices);

            DriverNotes notes = DriverNotes.For(Lanes, car, Silverstone);
            notes.StampLearned(Lanes, car, new DateTime(2026, 10, 27));
            DriverEntryCheck learned = ModDrivers.Check(Mods, Lanes, driver, car, Silverstone);
            Assert.True(learned.CanRace);
            Assert.Empty(learned.Notices);
            Assert.True(DriverNotes.For(Lanes, car, Silverstone).IsCurrent);

            ModCar changed = Copy(car, folder, "000000000000");
            DriverEntryCheck stale = ModDrivers.Check(Mods, Lanes, driver, changed, Silverstone);
            Assert.True(stale.CanRace);
            Assert.Single(stale.Notices);
            Assert.False(DriverNotes.For(Lanes, changed, Silverstone).IsCurrent);
        }
        finally
        {
            Directory.Delete(folder, recursive: true);
        }
    }

    /// <summary>
    /// Notes are kept in the car's folder under the driver and the circuit,
    /// every id split into its namespace and its name, so no colon reaches a
    /// path; and a driver's files stay inside its notes.
    /// </summary>
    [Fact]
    public void NotesAreKeptInTheCarsPackByNamespace()
    {
        (ModCar car, string folder) = OfficialCarInTempFolder();
        try
        {
            DriverNotes notes = DriverNotes.For(Lanes, car, Silverstone);
            Assert.Equal(
                Path.Combine(folder, "drivers", "someone", "lanes", "official", "silverstone"),
                notes.Folder);
            Assert.DoesNotContain(':', Path.GetRelativePath(folder, notes.Folder));
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
            Directory.Delete(folder, recursive: true);
        }
    }

    /// <summary>
    /// Seated, the driver is given the person, the car, the circuit, its
    /// notes there and a slot in the save for what it learns of this person
    /// there; a car without what it needs is refused.
    /// </summary>
    [Fact]
    public void ADriverIsSeatedWithItsNotesAndASaveSlot()
    {
        (ModCar car, string folder) = OfficialCarInTempFolder();
        try
        {
            var definition = new PackDriver();
            var person = new DriverProfile("ana", new DriverAbilities());
            var saves = new ThrowawaySave();

            Driver seated = ModDrivers.Seat(Mods, Lanes, definition, person, car, Silverstone, saves);
            Assert.Same(person, seated.Profile);
            DriverSeat seat = definition.LastSeat!;
            Assert.Equal(Silverstone, seat.Circuit);
            Assert.Equal(DriverNotes.For(Lanes, car, Silverstone).Folder, seat.Notes.Folder);
            Assert.True(seat.Track.LengthMeters > 0f);

            seat.Save!.Write("laps.json", "{\"laps\":3}");
            Assert.Equal("{\"laps\":3}", saves.Slot(new SaveKey(Lanes, "ana", car.Id, Silverstone)).Read("laps.json"));

            ModDrivers.Seat(Mods, Lanes, definition, person, car, Silverstone);
            Assert.Null(definition.LastSeat!.Save);

            var fussy = new PackDriver(new CarNeeds([typeof(NoSuchFeature)], []));
            Assert.Throws<CarUnfitForDriverException>(() => ModDrivers.Seat(Mods, Lanes, fussy, person, car, Silverstone));
        }
        finally
        {
            Directory.Delete(folder, recursive: true);
        }
    }

    private static ModRegistry Mods => GameMods.Registry;

    // The game's own car, as if its mod were a folder here: notes written in
    // a test never reach the real mod.
    private static (ModCar Car, string Folder) OfficialCarInTempFolder()
    {
        string folder = TempFolder();
        return (Copy(Mods.Cars.Get("official:formula_a_mixed"), folder), folder);
    }

    private static ModCar Copy(ModCar car, string folder, string? fingerprint = null) =>
        new(new CarDeclaration(car.Id, car.Manifest, "", ModResources.At(folder)), car.Mod, car.Definition,
            car.Compounds, car.Car, fingerprint ?? car.Fingerprint);

    private static string TempFolder()
    {
        string folder = Path.Combine(Path.GetTempPath(), "stintegy-drivers-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(folder);
        return folder;
    }

    private interface NoSuchFeature;

    /// <summary>A driver's definition that remembers how it was last seated.</summary>
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
