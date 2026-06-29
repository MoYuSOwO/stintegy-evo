using Godot;
using GdUnit4;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Simulation;

[TestSuite]
public sealed class VehicleSimulationTests
{
    [TestCase]
    [RequireGodotRuntime]
    public void StraightAccelerationStaysBounded()
    {
        var result = new VehicleSimulationHarness(input: 0.5f, steer: 0f).Run(durationSeconds: 4.0f);

        AssertThat(result.HasInvalidNumber).IsFalse();
        AssertThat(result.Last.Speed).IsBetween(7.5f, 11.5f);
        AssertThat(result.Last.AngularVelocity).IsBetween(-0.02f, 0.02f);
        AssertThat(result.LastRearSlipRatio()).IsLess(0.08f);
        AssertThat(result.MaxAbsWheelAngularVelocity).IsLess(40f);
        AssertThat(result.MaxWear).IsLess(0.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void MildConstantRadiusBuildsYawWithoutWheelspin()
    {
        var result = new VehicleSimulationHarness(input: 0.25f, steer: 0.15f).Run(durationSeconds: 4.0f);

        AssertThat(result.HasInvalidNumber).IsFalse();
        AssertThat(result.Last.Speed).IsBetween(3.5f, 6.0f);
        AssertThat(result.Last.AngularVelocity).IsBetween(0.08f, 0.28f);
        AssertThat(result.LastRearSlipRatio()).IsLess(0.08f);
        AssertThat(result.MaxAbsSlipAngle).IsLess(0.08f);
        AssertThat(result.MaxWear).IsLess(0.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void TighterConstantRadiusStaysFinite()
    {
        var result = new VehicleSimulationHarness(input: 0.35f, steer: 0.35f).Run(durationSeconds: 4.0f);

        AssertThat(result.HasInvalidNumber).IsFalse();
        AssertThat(result.Last.Speed).IsBetween(4.5f, 8.0f);
        AssertThat(result.Last.AngularVelocity).IsBetween(0.25f, 0.75f);
        AssertThat(result.LastRearSlipRatio()).IsLess(0.08f);
        AssertThat(result.MaxAbsSlipAngle).IsLess(0.12f);
        AssertThat(result.MaxWear).IsLess(0.0015f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void HardBrakeLocksWheelWithoutReverseSpin()
    {
        var result = new VehicleSimulationHarness(
            new TimedInputController(
                new TimedInputController.Phase(Duration: 10.0f, Input: 1.0f, Steer: 0.0f),
                new TimedInputController.Phase(Duration: 5.0f, Input: -1.0f, Steer: 0.0f)
            )
        ).Run(durationSeconds: 15.0f);

        AssertThat(result.HasInvalidNumber).IsFalse();
        AssertThat(result.MaxSpeed).IsGreater(25f);
        AssertThat(result.Last.Speed).IsLess(10.0f);
        AssertThat(result.MinWheelAngularVelocityWithoutPositiveInput).IsGreater(-0.2f);
        AssertThat(result.MaxAbsWheelAngularVelocity).IsLess(170f);
        AssertThat(result.MaxWear).IsLess(0.15f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void LowSpeedBrakeStillStopsCar()
    {
        var result = new VehicleSimulationHarness(
            new TimedInputController(
                new TimedInputController.Phase(Duration: 0.7f, Input: 0.3f, Steer: 0.0f),
                new TimedInputController.Phase(Duration: 2.0f, Input: -1.0f, Steer: 0.0f)
            )
        ).Run(durationSeconds: 2.7f);

        AssertThat(result.HasInvalidNumber).IsFalse();
        AssertThat(result.MaxSpeed).IsGreater(0.5f);
        AssertThat(result.Last.Speed).IsLess(0.2f);
        AssertThat(result.MinWheelAngularVelocityWithoutPositiveInput).IsGreater(-0.2f);
        AssertThat(result.MaxWear).IsLess(0.01f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void NeutralAtRestDoesNotCreateMotion()
    {
        var result = new VehicleSimulationHarness(input: 0.0f, steer: 0.0f).Run(durationSeconds: 2.0f);

        AssertThat(result.HasInvalidNumber).IsFalse();
        AssertThat(result.MaxSpeed).IsLess(0.02f);
        AssertThat(result.MaxAbsWheelAngularVelocity).IsLess(0.02f);
        AssertThat(result.MaxWear).IsLess(0.00001f);
    }
}

internal static class VehicleSimulationAssertions
{
    public static float LastRearSlipRatio(this VehicleSimulationResult result)
    {
        var tires = result.Last.Output.Params;
        return (Mathf.Abs(tires.RearLeft.SlipRatio) + Mathf.Abs(tires.RearRight.SlipRatio)) * 0.5f;
    }
}
