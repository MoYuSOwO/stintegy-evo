using Godot;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Tests.Simulation;

internal sealed class FixedInputController(float input, float steer) : IController
{
    public float Input { get; } = Mathf.Clamp(input, -1f, 1f);
    public float Steer { get; } = Mathf.Clamp(steer, -1f, 1f);

    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }

    public void Init(CarLogic carLogic, TrackData track)
    {
    }

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }
}

internal sealed class TimedInputController(params TimedInputController.Phase[] phases) : IController
{
    private float _time;

    public float Input { get; private set; } = phases.Length > 0 ? phases[0].Input : 0f;
    public float Steer { get; private set; } = phases.Length > 0 ? phases[0].Steer : 0f;

    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }

    public void Init(CarLogic carLogic, TrackData track)
    {
    }

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
        _time += dt;
        float elapsed = 0f;
        foreach (Phase phase in phases)
        {
            elapsed += phase.Duration;
            if (_time <= elapsed)
            {
                Input = Mathf.Clamp(phase.Input, -1f, 1f);
                Steer = Mathf.Clamp(phase.Steer, -1f, 1f);
                return;
            }
        }

        if (phases.Length == 0) return;
        Phase last = phases[^1];
        Input = Mathf.Clamp(last.Input, -1f, 1f);
        Steer = Mathf.Clamp(last.Steer, -1f, 1f);
    }

    public readonly record struct Phase(float Duration, float Input, float Steer);
}
