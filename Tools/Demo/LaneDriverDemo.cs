using Godot;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using StintegyEVO.GodotApp.LowPoly;

namespace StintegyEVO.Presentation.Demo;

/// <summary>
/// Local viewer: the three-lane driver on Silverstone with its tuning file,
/// from a standing start on the line, in the low-poly 3D view.
/// </summary>
public partial class LaneDriverDemo : Node
{
    public override void _Ready()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        if (!file.IsCurrent)
            GD.PushWarning($"silverstone was learned on another car ({file.LearnedFor?.Date ?? "unstamped"}); it drives on old notes until relearned");
        LaneSet lanes = file.LanesFor(track);

        int station = (int)System.MathF.Round(lanes.StationOf(track.StartingLineS));
        System.Numerics.Vector2 here = lanes.Position(station, lanes.Offset(LaneId.Free, station));
        System.Numerics.Vector2 next = lanes.Position(station + 1, lanes.Offset(LaneId.Free, station + 1));

        var config = new CarConfig();
        var tires = new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f };
        var driver = new LaneDriver(file, config, tires);
        var car = new RaceCar(
            "lanes",
            config,
            tires,
            new Driver(new DriverProfile("three-lane", new DriverAbilities()), driver),
            new CarState
            {
                Position = here,
                Heading = System.MathF.Atan2(next.Y - here.Y, next.X - here.X),
                Energy = PowertrainState.Filled(0.8f)
            });

        var simulation = new RaceSimulation(track);
        simulation.AddCar(car);
        var view = new RaceView3D();
        view.BindSimulation(simulation);
        AddChild(view);
    }
}
