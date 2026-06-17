using Godot;
using StintegyEVO.Core.Car;

namespace StintegyEVO.Tests.Simulation;

internal sealed class VehicleSimulationResult
{
    public int SampleCount { get; private set; }
    public VehicleSimulationSample Last { get; set; }
    public float MaxSpeed { get; private set; }
    public float MaxAbsAngularVelocity { get; private set; }
    public float MaxAbsSlipRatio { get; private set; }
    public float MaxAbsSlipAngle { get; private set; }
    public float MaxAbsWheelAngularVelocity { get; private set; }
    public float MinWheelAngularVelocity { get; private set; } = float.PositiveInfinity;
    public float MinWheelAngularVelocityWhileForward { get; private set; } = float.PositiveInfinity;
    public float MinWheelAngularVelocityWithoutPositiveInput { get; private set; } = float.PositiveInfinity;
    public float MinForwardSpeed { get; private set; } = float.PositiveInfinity;
    public float MaxAbsSideSpeed { get; private set; }
    public float MaxWear { get; private set; }
    public int SlidingSamples { get; private set; }
    public bool HasInvalidNumber { get; private set; }

    public void Add(VehicleSimulationSample sample)
    {
        SampleCount++;
        MaxSpeed = Mathf.Max(MaxSpeed, sample.Speed);
        MinForwardSpeed = Mathf.Min(MinForwardSpeed, sample.ForwardSpeed);
        MaxAbsSideSpeed = Mathf.Max(MaxAbsSideSpeed, Mathf.Abs(sample.SideSpeed));
        MaxAbsAngularVelocity = Mathf.Max(MaxAbsAngularVelocity, Mathf.Abs(sample.AngularVelocity));

        foreach (var tire in sample.Output.Params.Tires)
        {
            MaxAbsSlipRatio = Mathf.Max(MaxAbsSlipRatio, Mathf.Abs(tire.SlipRatio));
            MaxAbsSlipAngle = Mathf.Max(MaxAbsSlipAngle, Mathf.Abs(tire.SlipAngle));
            if (tire.IsSliding) SlidingSamples++;
        }

        foreach (float wear in sample.Wear)
        {
            MaxWear = Mathf.Max(MaxWear, wear);
        }

        foreach (float wheelAngularVelocity in sample.WheelAngularVelocity)
        {
            MaxAbsWheelAngularVelocity = Mathf.Max(MaxAbsWheelAngularVelocity, Mathf.Abs(wheelAngularVelocity));
            MinWheelAngularVelocity = Mathf.Min(MinWheelAngularVelocity, wheelAngularVelocity);
            if (sample.ForwardSpeed > 0.5f)
            {
                MinWheelAngularVelocityWhileForward = Mathf.Min(MinWheelAngularVelocityWhileForward, wheelAngularVelocity);
            }
            if (sample.Input <= 0f)
            {
                MinWheelAngularVelocityWithoutPositiveInput = Mathf.Min(MinWheelAngularVelocityWithoutPositiveInput, wheelAngularVelocity);
            }
        }

        HasInvalidNumber |=
            !IsFinite(sample.Speed) ||
            !IsFinite(sample.AngularVelocity) ||
            !IsFinite(sample.Position.X) ||
            !IsFinite(sample.Position.Y) ||
            !IsFinite(sample.Rotation) ||
            !IsFinite(MaxAbsSlipRatio) ||
            !IsFinite(MaxAbsSlipAngle) ||
            !IsFinite(MaxWear) ||
            !IsFinite(MaxAbsWheelAngularVelocity);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}

internal readonly record struct VehicleSimulationSample(
    float Time,
    float Input,
    float Steer,
    Vector2 Position,
    float Rotation,
    float Speed,
    float ForwardSpeed,
    float SideSpeed,
    float AngularVelocity,
    PhysicsOutput Output,
    float[] Wear,
    float[] WheelAngularVelocity
);
