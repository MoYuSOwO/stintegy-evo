using Godot;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers;
using StintegyEVO.Drivers.Lanes;
using StintegyEVO.GodotApp.Interop;
using StintegyEVO.GodotApp.LowPoly;

namespace StintegyEVO.Presentation.Demo;

/// <summary>
/// Local viewer: the three-lane driver on a circuit with its lanes file,
/// from a standing start on the line, in the low-poly 3D view. The circuit
/// is STINTEGY_CIRCUIT (a circuit's id), Silverstone by default.
/// </summary>
public partial class LaneDriverDemo : Node
{
    private LaneDriver.JointTable? _jointTable;

    public override void _ExitTree() => _jointTable?.Dispose();

    public override void _Ready()
    {
        string requested = System.Environment.GetEnvironmentVariable("STINTEGY_CIRCUIT") is { Length: > 0 } named ? named : "silverstone";
        ResourceId circuit = ResourceId.Parse(requested.Contains(':') ? requested : $"official:{requested}");
        string id = circuit.ToString();
        TrackData track = GodotMods.Registry.Circuits.Get(id).Track;
        // STINTEGY_LANES_FOLDER: a lanes folder not yet shipped, to watch it
        // drive. STINTEGY_LANES3D: a bake's output folder, to watch the baked
        // line as baked.
        LaneTrackFile file;
        if (System.Environment.GetEnvironmentVariable("STINTEGY_LANES_FOLDER") is { Length: > 0 } folder)
            file = LaneTrackFile.LoadCircuit(folder, track);
        else if (System.Environment.GetEnvironmentVariable("STINTEGY_LANES3D") is { Length: > 0 } baked)
            file = LaneTrackFile.FromBakedLanes(System.IO.Path.Combine(baked, $"{circuit.Name}.lanes3d.csv"), track);
        else
            file = LaneTrackFile.ForCircuit(id, track);
        if (!file.IsCurrent)
            GD.PushWarning($"{id} lanes were baked for another car ({file.Stamp?.Learned ?? "unstamped"})");
        LaneSet lanes = file.LanesFor(track);

        var simulation = new RaceSimulation(track);
        _jointTable = System.Environment.GetEnvironmentVariable("STINTEGY_JOINT_TABLE") is { Length: > 0 } path
            ? LaneDriver.LoadJointTable(path, id, System.Environment.GetEnvironmentVariable("STINTEGY_JOINT_GEOMETRY"))
            : null;
        int count = _jointTable is null ? 1 : 2;
        bool duel = _jointTable is not null &&
            System.Environment.GetEnvironmentVariable("STINTEGY_JOINT_DUEL") == "attack-protect";
        bool jointLookahead = System.Environment.GetEnvironmentVariable("STINTEGY_JOINT_LOOKAHEAD") == "1";
        for (int i = 0; i < count; i++)
        {
            int station = (int)System.MathF.Round(lanes.StationOf(track.StartingLineS)) + (duel ? 6 : 15) * i;
            var here = lanes.Position(station, lanes.Offset(LaneId.Free, station));
            var next = lanes.Position(station + 1, lanes.Offset(LaneId.Free, station + 1));
            ICar model = StockCar.Build(DevelopmentTyres.SuperId);
            var driver = new LaneDriver(file, model) { JointProtect = !duel || i == 1, JointLookahead = jointLookahead };
            if (_jointTable is not null)
                driver.UseJointTable(_jointTable);
            string carId = $"lanes-{i + 1}";
            var car = new RaceCar(carId, model,
                new Driver(new DriverProfile(carId, new DriverAbilities()), driver),
                new CarState
                {
                    Position = here,
                    Heading = System.MathF.Atan2(next.Y - here.Y, next.X - here.X),
                    Speed = duel ? 45f : count == 2 ? 45f - 4f * i : 0f
                });
            (car.Car as ICarStores)?.Fill(car.State, PowertrainState.Filled(0.8f));
            simulation.AddCar(car);
        }
        var view = new RaceView3D { CircuitId = id };
        view.BindSimulation(simulation);
        AddChild(view);
    }
}
