using Godot;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers;

public struct CarSensor
{
    public float Mass;
    public Vector2 LinearVelocity;
    public float AngularVelocity;
    public Vector2 Position;
    public float Rotation;
    public Vector2 LocalAccel;
    public IntermediateParams Params;
}

public interface IController
{
    public float Input { get; }
    public float SteeringAngle { get; }

    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }

    public abstract void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track);
    public abstract void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track);
}

public partial class DummyController : IController
{
    public static readonly DummyController Instance = new();

    private DummyController() {}

    public float Input => 0.5f;
    public float SteeringAngle => 0.3f;

    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }
}