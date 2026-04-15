using Godot;
using PloyRacing.Core.Track;

namespace PloyRacing.Core.Car.Controllers;

public struct CarSensor
{
    public Vector2 LinearVelocity;
    public float AngularVelocity;
    public Vector2 Position;
    public float Rotation;
}

public interface IController
{
    public float Throttle { get; }
    public float Brake { get; }
    public float SteeringAngle { get; }

    public abstract void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track);
    public abstract void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track);
}

public partial class DummyController : IController
{
    public static readonly DummyController Instance = new();

    private DummyController() {}

    public float Throttle => 0.0f;
    public float Brake => 0.0f;
    public float SteeringAngle => 0.0f;

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }
}