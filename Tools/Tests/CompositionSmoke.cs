using System;
using System.Threading.Tasks;
using Godot;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.GodotApp.LowPoly;

namespace StintegyEVO.Presentation.Tests;

/// <summary>Opt-in integration fixture, excluded from every ordinary product build.</summary>
public partial class CompositionSmoke : Node
{
    public override async void _Ready()
    {
        try
        {
            await Verify();
            GD.Print("COMPOSITION PASS");
            GetTree().Quit(0);
        }
        catch (Exception error)
        {
            GD.PushError(error.ToString());
            GetTree().Quit(1);
        }
    }

    private async Task Verify()
    {
        TrackData track = new TrackBuilder(System.Numerics.Vector2.Zero, 14f)
            .AddStraight(400f).AddTurn(180f, 50f).AddStraight(400f).AddTurn(180f, 50f)
            .CloseLoop().Build(new TrackGridConfig());

        // An injected world with no controller and a zero command still has
        // physical momentum. Only the standalone preview may stop its clock.
        RaceSimulation passive = new(track);
        RaceCar coasting = Car(track, "passive", 50f, 12f);
        passive.AddCar(coasting);
        passive.Step(0.25f);
        var view = new RaceView3D();
        view.BindSimulation(passive);
        AddChild(view);
        await WaitReady(view);
        Check(!view.IsPreview && ReferenceEquals(passive, view.Simulation), "Injected world was replaced or treated as preview.");
        float before = passive.RaceTimeSeconds;
        var position = coasting.State.Position;
        await Delay(0.4);
        view.TogglePause();
        await Delay(0.15); // Let the at-most-six-substep worker finish.
        Check(view.RaceSeconds > before, "A passive injected world must advance.");
        Check(coasting.State.Position != position, "A zero command must not freeze momentum.");
        float paused = view.RaceSeconds;
        await Delay(0.15);
        Check(view.RaceSeconds == paused, "Paused world continued stepping.");
        CheckThrows<InvalidOperationException>(() => view.BindSimulation(passive));
        CheckThrows<ArgumentOutOfRangeException>(() => view.SetExternalInput(-1, 0, 1));
        CheckThrows<ArgumentOutOfRangeException>(() => view.SetExternalInput(0, float.NaN, 1));
        view.SetExternalInput(0, 0, 3);
        await Delay(0.1);
        Check(coasting.ExternalInput == new DriverInput(0, 3), "Queued command was not applied at a safe boundary.");
        Check(view.RaceSeconds == paused, "Submitting input bypassed pause.");
        view.TogglePause();
        await Delay(0.3);
        RemoveChild(view); // _ExitTree joins worker: the host now owns passive again.
        view.Free();
        float detachedTime = passive.RaceTimeSeconds;
        await Delay(0.1);
        Check(passive.RaceTimeSeconds == detachedTime, "Worker still advances a detached world.");
        passive.Step(1f / 60f);
        Check(passive.RaceTimeSeconds > detachedTime, "Detached world could not be reused.");

        // Test-only controller implementation proves the actual 3D composition
        // path, not just the Core interface, accepts externally supplied logic.
        var controller = new FixedController();
        var profile = new DriverProfile("external-person", new DriverAbilities { Pace = 63f }, 42);
        RaceSimulation controlled = new(track);
        RaceCar driven = Car(track, "controlled", 50f, 0f, new Driver(profile, controller));
        controlled.AddCar(driven);
        var controlledView = new RaceView3D();
        controlledView.BindSimulation(controlled);
        AddChild(controlledView);
        await WaitReady(controlledView);
        await Delay(0.4);
        controlledView.TogglePause();
        await Delay(0.15);
        Check(controller.Calls > 0 && ReferenceEquals(controller.Profile, profile), "External controller did not receive its domain profile.");
        Check(driven.LastInput == new DriverInput(0, 2) && driven.State.Speed > 0, "External controller did not drive physical state.");
        RemoveChild(controlledView);
        controlledView.Free();
    }

    private async Task WaitReady(RaceView3D view)
    {
        ulong deadline = Time.GetTicksMsec() + 30000;
        while (!view.IsInitialized && Time.GetTicksMsec() < deadline)
            await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
        Check(view.IsInitialized, "3D view failed to initialize.");
    }

    private async Task Delay(double seconds) =>
        await ToSignal(GetTree().CreateTimer(seconds), SceneTreeTimer.SignalName.Timeout);

    private static RaceCar Car(TrackData track, string id, float s, float speed, Driver? driver = null)
    {
        TrackSample at = track.Sample(s);
        return new RaceCar(id, new(), new(), driver,
            new CarState { Position = at.Center, Heading = at.Heading, Speed = speed });
    }

    private static void Check(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }

    private static void CheckThrows<T>(Action action) where T : Exception
    {
        try { action(); }
        catch (T) { return; }
        throw new InvalidOperationException($"Expected {typeof(T).Name}.");
    }

    private sealed class FixedController : IDriverController
    {
        public int Calls { get; private set; }
        public DriverProfile? Profile { get; private set; }
        public DriverInput GetControl(in DriverContext context, float dt)
        {
            Calls++;
            Profile = context.Profile;
            return new(0, 2);
        }
    }
}
