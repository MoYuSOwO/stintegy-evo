using System;
using Godot;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Track;
using StintegyEVO.Nodes.Race;

namespace StintegyEVO.Tests.Simulation;

internal sealed class VehicleSimulationHarness
{
    private readonly CarLogic _logic;
    private readonly IController _controller;
    private readonly float _mass;
    private readonly float _inertia;

    private Vector2 _position;
    private Vector2 _velocity;
    private Vector2 _acceleration;
    private float _rotation;
    private float _angularVelocity;
    private float _time;

    public VehicleSimulationHarness(float input, float steer)
        : this(new FixedInputController(input, steer))
    {
    }

    public VehicleSimulationHarness(IController controller)
    {
        var config = new CarConfig();
        var track = TrackFactory.SimpleTestTrack();
        _logic = new CarLogic(config, track, DummyEnvironment.Instance);
        _controller = controller;
        _mass = config.Chassis.DryMass;
        _inertia = config.Chassis.DryI;

        var grid = track.Grids[config.CarGridOrPitIdx];
        _position = grid.Position;
        _rotation = track[grid.Index].Tangent.Angle();
    }

    public VehicleSimulationResult Run(float durationSeconds, float dt = 1f / 60f, Action<VehicleSimulationSample>? onSample = null)
    {
        int steps = Mathf.CeilToInt(durationSeconds / dt);
        VehicleSimulationSample last = default;
        var summary = new VehicleSimulationResult();

        for (int i = 0; i < steps; i++)
        {
            last = Step(dt);
            onSample?.Invoke(last);
            summary.Add(last);
        }

        summary.Last = last;
        return summary;
    }

    private VehicleSimulationSample Step(float dt)
    {
        Vector2 localVelocity = _velocity.Rotated(-_rotation);
        Vector2 localAcceleration = _acceleration.Rotated(-_rotation);

        var sensor = new CarSensor
        {
            Mass = _mass,
            LinearVelocity = _velocity,
            AngularVelocity = _angularVelocity,
            Position = _position,
            Rotation = _rotation,
            LocalAccel = localAcceleration
        };

        float input = Mathf.Clamp(_controller.Input, -1f, 1f);
        float steer = Mathf.Clamp(_controller.Steer, -1f, 1f);
        PhysicsOutput output = _logic.Tick(dt, input, steer, localVelocity, localAcceleration, _angularVelocity, _mass);

        Vector2 localForce =
            output.DragForce +
            output.FrontLeft.Force +
            output.FrontRight.Force +
            output.RearLeft.Force +
            output.RearRight.Force;

        float torque =
            Cross(output.FrontLeft.Pos, output.FrontLeft.Force) +
            Cross(output.FrontRight.Pos, output.FrontRight.Force) +
            Cross(output.RearLeft.Pos, output.RearLeft.Force) +
            Cross(output.RearRight.Pos, output.RearRight.Force);

        _acceleration = localForce.Rotated(_rotation) / _mass;
        _velocity += _acceleration * dt;
        _position += _velocity * dt;
        _angularVelocity += torque / _inertia * dt;
        _rotation += _angularVelocity * dt;
        _time += dt;

        _controller.ThinkTick(dt, sensor, _logic, _logic.Track);
        Vector2 postStepLocalVelocity = _velocity.Rotated(-_rotation);

        return new VehicleSimulationSample(
            _time,
            input,
            steer,
            _position,
            _rotation,
            _velocity.Length(),
            postStepLocalVelocity.X,
            postStepLocalVelocity.Y,
            _angularVelocity,
            output,
            [
                _logic.TireFrontLeft.Wear,
                _logic.TireFrontRight.Wear,
                _logic.TireRearLeft.Wear,
                _logic.TireRearRight.Wear
            ],
            [
                _logic.TireFrontLeft.WheelAngularVel,
                _logic.TireFrontRight.WheelAngularVel,
                _logic.TireRearLeft.WheelAngularVel,
                _logic.TireRearRight.WheelAngularVel
            ]
        );
    }

    private static float Cross(Vector2 a, Vector2 b)
    {
        return a.X * b.Y - a.Y * b.X;
    }
}
