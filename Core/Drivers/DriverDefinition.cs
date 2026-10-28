using System;
using System.Collections.Generic;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// What a mod's code registers for a driver: a way to seat it in a car. The
/// game knows a driver only through this, and finds it by id in the
/// registry as it finds a car.
/// </summary>
public interface IDriverDefinition
{
    /// <summary>
    /// What the driver needs of a car, known before it is put in one: a car
    /// without a required feature is refused before the race, not found out
    /// during it.
    /// </summary>
    CarNeeds Needs { get; }

    /// <summary>The driver's controller for one race, seated as <paramref name="seat"/> says.</summary>
    IDriverController Seat(DriverSeat seat);
}

/// <summary>
/// Everything a driver is seated with: who is driving, in what, where, what
/// has been learned about that car there, and where this person's own
/// experience of it is kept.
/// </summary>
/// <param name="Person">The person driving: their name and ratings.</param>
/// <param name="Driver">The driver doing the driving, by its id.</param>
/// <param name="Car">The car, as the registry built it.</param>
/// <param name="Circuit">The circuit's id.</param>
/// <param name="Track">The circuit's road.</param>
/// <param name="Notes">What has been learned about this car at this circuit, by this driver.</param>
/// <param name="Progress">This person's own experience of the car there, if the game keeps it.</param>
public sealed record DriverSeat(
    DriverProfile Person,
    ResourceId Driver,
    ModCar Car,
    ResourceId Circuit,
    TrackData Track,
    DriverNotes Notes,
    DriverProgress? Progress);

/// <summary>
/// What stands between a driver and a car at a circuit, asked before the
/// race: what refuses the seat, and what only the pit wall should be told.
/// </summary>
/// <param name="Refusals">Why the driver cannot drive the car at all; none when it can.</param>
/// <param name="Notices">What the driver can drive through, but worse: no notes for this car there, or notes from another car.</param>
public sealed record DriverEntryCheck(IReadOnlyList<string> Refusals, IReadOnlyList<string> Notices)
{
    public bool CanRace => Refusals.Count == 0;
}
