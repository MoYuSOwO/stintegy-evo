using Godot;
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
    public override void _Ready()
    {
        string id = System.Environment.GetEnvironmentVariable("STINTEGY_CIRCUIT") is { Length: > 0 } named ? named : "silverstone";
        GodotCircuits.Use();
        TrackData track = CircuitLibrary.Load(id);
        // STINTEGY_LANES_FOLDER: a lanes folder not yet shipped, to watch it
        // drive. STINTEGY_LANES3D: a bake's output folder, to watch the baked
        // line as baked.
        LaneTrackFile file;
        if (System.Environment.GetEnvironmentVariable("STINTEGY_LANES_FOLDER") is { Length: > 0 } folder)
            file = LaneTrackFile.LoadCircuit(folder, track);
        else if (System.Environment.GetEnvironmentVariable("STINTEGY_LANES3D") is { Length: > 0 } baked)
            file = LaneTrackFile.FromBakedLanes(System.IO.Path.Combine(baked, $"{id}.lanes3d.csv"), track);
        else
            file = LaneTrackFile.ForCircuit(id, track);
        if (!file.IsCurrent)
            GD.PushWarning($"{id} lanes were baked for another car ({file.Stamp?.Learned ?? "unstamped"})");
        LaneSet lanes = file.LanesFor(track);

        int station = (int)System.MathF.Round(lanes.StationOf(track.StartingLineS));
        System.Numerics.Vector2 here = lanes.Position(station, lanes.Offset(LaneId.Free, station));
        System.Numerics.Vector2 next = lanes.Position(station + 1, lanes.Offset(LaneId.Free, station + 1));

        ICar model = new SingleTrackCar(StockCar.Config, DevelopmentTyres.Super);
        var driver = new LaneDriver(file, model);
        var car = new RaceCar(
            "lanes",
            model,
            new Driver(new DriverProfile("three-lane", new DriverAbilities()), driver),
            new CarState
            {
                Position = here,
                Heading = System.MathF.Atan2(next.Y - here.Y, next.X - here.X)
            });
        (car.Car as ICarStores)?.Fill(car.State, PowertrainState.Filled(0.8f));

        var simulation = new RaceSimulation(track);
        simulation.AddCar(car);
        var view = new RaceView3D { CircuitId = id };
        view.BindSimulation(simulation);
        AddChild(view);
    }
}
