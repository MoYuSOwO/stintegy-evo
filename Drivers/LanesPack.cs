using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Mods;
using StintegyEVO.Drivers.Lanes;

namespace StintegyEVO.Drivers;

/// <summary>The lane driver as a mod: <c>lanes:lanes</c>.</summary>
public static class LanesPack
{
    public static readonly ResourceId Id = new("lanes", "lanes");
}

/// <summary>The lane driver's mod: it registers the driver and nothing else.</summary>
public sealed class LanesMod : IMod
{
    public void Register(ModContext context) =>
        context.Registry.Drivers.Register(LanesPack.Id.Name, new LaneDriverDefinition());
}

/// <summary>
/// What the lane driver's mod offers the game: the driver, seated in a car
/// with its notes for that car at the circuit and the person's own
/// experience there.
/// </summary>
public sealed class LaneDriverDefinition : IDriverDefinition
{
    public CarNeeds Needs => LaneDriver.CarNeeds;

    public IDriverController Seat(DriverSeat seat) =>
        new LaneDriver(LaneTrackFile.FromNotes(seat.Notes, seat.Track), seat.Car.Car)
        {
            Save = seat.Save
        };
}
