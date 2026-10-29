using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Saves;

namespace StintegyEVO.Core.Mods;

/// <summary>Putting a person in a registered car, with a registered driver, at a registered circuit.</summary>
public static class ModDrivers
{
    /// <summary>
    /// Whether <paramref name="driver"/> can drive <paramref name="car"/> at
    /// <paramref name="circuit"/>, and what the pit wall should know if it
    /// can: refused when the car lacks what the driver needs; told when
    /// nothing has been learned about this car there, or what has was
    /// learned on another car, since the driver still drives, but worse.
    /// </summary>
    public static DriverEntryCheck Check(this ModRegistry registry, ResourceId driver, ModCar car, ResourceId circuit) =>
        Check(registry, driver, registry.Drivers.Get(driver), car, circuit);

    internal static DriverEntryCheck Check(ModRegistry registry, ResourceId driver, IDriverDefinition definition, ModCar car, ResourceId circuit)
    {
        var refusals = new List<string>();
        var notices = new List<string>();
        IReadOnlyList<Type> missing = definition.Needs.MissingFrom(car.Car);
        if (missing.Count > 0)
            refusals.Add($"{driver} cannot drive {car.Id}: it has no {string.Join(", ", missing.Select(t => t.Name))}.");
        if (!registry.Circuits.Contains(circuit))
            refusals.Add($"{driver} cannot drive at {circuit}: no such circuit is installed.");
        DriverNotes notes = DriverNotes.For(driver, car, circuit);
        if (notes.Stamp is not { } stamp)
            notices.Add($"{driver} has no notes for {car.Id} at {circuit}: it will drive without, and slower.");
        else if (!notes.IsCurrent)
            notices.Add($"{driver}'s notes for {car.Id} at {circuit} were learned on another car ({stamp.CarFingerprint}, now {car.Fingerprint}): it will drive on them, and slower.");
        return new DriverEntryCheck(refusals, notices);
    }

    /// <summary>
    /// <paramref name="person"/> in <paramref name="car"/> at
    /// <paramref name="circuit"/>, driven by <paramref name="driver"/>, with
    /// what has been learned there and, when the game keeps one, a slot in
    /// <paramref name="saves"/> for what it learns of this person there. Refused, as <see cref="CarUnfitForDriverException"/>,
    /// when the car lacks what the driver needs.
    /// </summary>
    public static Driver Seat(
        this ModRegistry registry,
        ResourceId driver,
        DriverProfile person,
        ModCar car,
        ResourceId circuit,
        ISaveStore? saves = null) =>
        Seat(registry, driver, registry.Drivers.Get(driver), person, car, circuit, saves);

    internal static Driver Seat(
        ModRegistry registry,
        ResourceId driver,
        IDriverDefinition definition,
        DriverProfile person,
        ModCar car,
        ResourceId circuit,
        ISaveStore? saves = null)
    {
        IReadOnlyList<Type> missing = definition.Needs.MissingFrom(car.Car);
        if (missing.Count > 0)
            throw new CarUnfitForDriverException(person.Id, car.Car, missing);
        var seat = new DriverSeat(
            person,
            driver,
            car,
            circuit,
            registry.Circuits.Get(circuit).Track,
            DriverNotes.For(driver, car, circuit),
            saves?.Slot(new SaveKey(driver, person.Id, car.Id, circuit)));
        IDriverController controller = definition.Seat(seat)
            ?? throw new InvalidDataException($"Driver '{driver}' seated nobody.");
        return new Driver(person, controller);
    }
}
