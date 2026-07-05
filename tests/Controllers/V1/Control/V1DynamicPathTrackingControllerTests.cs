using GdUnit4;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Car.Controllers.V1.Control;
using StintegyEVO.Core.Track;
using StintegyEVO.Nodes.Race;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Controllers.V1.Control;

[TestSuite]
public sealed class V1DynamicPathTrackingControllerTests
{
    [TestCase]
    [RequireGodotRuntime]
    public void DefaultControllerInitializesAndProducesFiniteCommands()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleTestTrack();
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        V1DynamicPathTrackingController controller = new();
        controller.Init(logic, track);
        TrackPoint start = track[track.GridConfig.FirstGridIdx];
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            Position = start.Center,
            Rotation = start.Tangent.Angle()
        };

        controller.ThinkTick(1.0f / 60.0f, sensor, logic, track);

        AssertThat(float.IsFinite(controller.Input)).IsTrue();
        AssertThat(float.IsFinite(controller.Steer)).IsTrue();
        AssertThat(controller.Input).IsBetween(-1.0f, 1.0f);
        AssertThat(controller.Steer).IsBetween(-1.0f, 1.0f);
    }
}
