using Godot;
using PloyRacing.Core.Track;

namespace PloyRacing.Nodes.Car.Controllers;

public abstract partial class BaseController : Node2D
{

    public float Throttle { get; protected set; }
    public float Brake { get; protected set; }
    public float SteeringAngle { get; protected set; }

    public abstract void Think(float dt, RigidBody2D myCar, TrackData track);
}

public partial class DummyController : BaseController
{
    public static readonly DummyController Instance = new();

    private DummyController() {}

    public override void Think(float dt, RigidBody2D myCar, TrackData track)
    {
        Throttle = 0;
        Brake = 0;
        SteeringAngle = 0;
    }
}