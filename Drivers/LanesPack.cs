using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Drivers.Lanes;

namespace StintegyEVO.Drivers;

/// <summary>The lane driver as a pack: <c>official:lanes</c>.</summary>
public static class LanesPack
{
    public static readonly ResourceId Id = new(ResourceId.Official, "lanes");
}

/// <summary>
/// What the lane driver's pack offers the game: the driver, seated in a car
/// with its notes for that car at the circuit and the person's own
/// experience there.
/// </summary>
public sealed class LaneDriverDefinition : IDriverDefinition
{
    public CarNeeds Needs => LaneDriver.CarNeeds;

    public IDriverController Seat(DriverSeat seat) =>
        new LaneDriver(LaneTrackFile.FromNotes(seat.Notes, seat.Track), seat.Car.Car)
        {
            Progress = seat.Progress
        };
}
