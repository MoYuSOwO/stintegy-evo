using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// Puts a lane-driven car on the road: at a place along the lap, aside of
/// the racing line by a given amount, rolling at a given speed, pointed
/// along the racing line there. One lap or a pack of cars is placed the same way.
/// </summary>
public static class LaneGrid
{
    public static (RaceCar Car, LaneDriver Driver) Seat(
        RaceSimulation simulation,
        LaneTrackFile file,
        LaneSet lanes,
        string id,
        ICar model,
        string compound,
        float s,
        float offLine,
        float speed,
        LaneDriverGains? gains = null,
        LaneShares? shares = null,
        DriverAbilities? abilities = null,
        CarStrategy? strategy = null,
        float startingCharge = 0.8f,
        float tireTempC = 90f,
        bool learnsAsItDrives = false
    )
    {
        int station = (int)MathF.Round(lanes.StationOf(s));
        Vector2 here = lanes.Position(station, lanes.Offset(LaneId.Free, station) + offLine);
        Vector2 next = lanes.Position(station + 1, lanes.Offset(LaneId.Free, station + 1) + offLine);
        float heading = MathF.Atan2(next.Y - here.Y, next.X - here.X);

        var driver = new LaneDriver(file, model)
        {
            Gains = gains ?? new LaneDriverGains(),
            Shares = shares ?? new LaneShares(),
            // A lap of the plan and the notes, not of a driver's nerves.
            LearnsAsItDrives = learnsAsItDrives
        };
        var profile = new DriverProfile(id, abilities ?? new DriverAbilities());
        var car = new RaceCar(
            id,
            model,
            new Driver(profile, driver),
            new CarState
            {
                Position = here,
                Heading = heading,
                Speed = speed
            }
        )
        {
            Strategy = strategy ?? CarStrategy.Default
        };
        ((ICarTires)model).FitTires(car.State, compound);
        ((ICarStores)model).Fill(car.State, PowertrainState.Filled(startingCharge));
        ((ICarTires)model).WarmTires(car.State, tireTempC, tireTempC);
        simulation.AddCar(car);
        return (car, driver);
    }
}
