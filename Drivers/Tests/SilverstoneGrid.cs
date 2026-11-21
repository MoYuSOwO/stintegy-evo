using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Drivers.Lanes;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Shared Silverstone grid roster for driver telemetry and replay tests.
/// Test-only; nothing outside Drivers/Tests compiles it.
/// </summary>
internal static class SilverstoneGrid
{
    public static readonly (string Id, DriverAbilities Abilities)[] Roster =
    [
        ("P1-Rookie", new DriverAbilities { Pace = 62, Consistency = 65, Focus = 70, Precision = 68, Reactions = 65, Technical = 70, Awareness = 65, Overtaking = 60, Defending = 60, Boldness = 40 }),
        ("P2-Midfield", new DriverAbilities { Pace = 72, Consistency = 75, Focus = 78, Precision = 75, Reactions = 74, Technical = 78, Awareness = 75, Overtaking = 70, Defending = 72, Boldness = 48 }),
        ("P3-Regular", new DriverAbilities { Pace = 80, Consistency = 82, Focus = 84, Precision = 82, Reactions = 80, Technical = 84, Awareness = 82, Overtaking = 78, Defending = 80, Boldness = 55 }),
        ("P4-Charger", new DriverAbilities { Pace = 87, Consistency = 86, Focus = 88, Precision = 87, Reactions = 86, Technical = 88, Awareness = 86, Overtaking = 88, Defending = 82, Boldness = 68 }),
        ("P5-Contender", new DriverAbilities { Pace = 94, Consistency = 94, Focus = 95, Precision = 94, Reactions = 93, Technical = 95, Awareness = 93, Overtaking = 92, Defending = 90, Boldness = 62 }),
        ("P6-Champion", new DriverAbilities { Pace = 99, Consistency = 98, Focus = 99, Precision = 98, Reactions = 98, Technical = 98, Awareness = 97, Overtaking = 96, Defending = 95, Boldness = 65 }),
    ];

    public static int IndexOf(string id) => Array.FindIndex(Roster, r => r.Id == id);

    /// <summary>
    /// Seats the roster on the circuit's grid, from P1 back, at rest. Past the
    /// six named drivers the ratings cycle with a suffix on the id, so a full
    /// twenty-car field still spans the whole range of drivers.
    /// </summary>
    public static List<RaceCar> Seat(RaceSimulation simulation, LaneTrackFile file,
        StintegyEVO.Core.Track.TrackData track, string compound, int? carCount = null)
    {
        var cars = new List<RaceCar>();
        int total = carCount is { } wanted
            ? Math.Max(1, wanted)
            : Math.Min(Roster.Length, Math.Max(1, track.StartingGridCount));
        for (int i = 0; i < total; i++)
        {
            int gridPos = i + 1;
            (string id, DriverAbilities abilities) = Roster[i % Roster.Length];
            if (i >= Roster.Length)
                id = $"{id}-{i / Roster.Length + 1}";
            float s = track.Grids.GetS(gridPos);
            System.Numerics.Vector2 pos = track.Grids.GetPosition(gridPos);
            float heading = track.Sample(s).Heading;
            ICar model = StockCar.Build(compound);
            var driver = new LaneDriver(file, model) { LearnsAsItDrives = false };
            var car = new RaceCar(id, model,
                new Driver(new DriverProfile(id, abilities), driver),
                new CarState { Position = pos, Heading = heading, Speed = 0f });
            (model as ICarTires)?.FitTires(car.State, compound);
            (model as ICarStores)?.Fill(car.State, PowertrainState.Filled(0.8f));
            (model as ICarTires)?.WarmTires(car.State, 90f, 90f);
            simulation.AddCar(car);
            cars.Add(car);
        }
        return cars;
    }
}
