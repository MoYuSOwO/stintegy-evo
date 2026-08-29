using System;

namespace StintegyEVO.Core.Drivers.Learned;

/// <summary>
/// A driving brain behind the direct physical interface: it reads one
/// observation vector and writes one action vector, both laid out by
/// <see cref="DirectDriveObservation"/>. Actions are raw policy outputs in
/// [-1, 1]; the driver maps them onto the car's physical actuator ranges.
/// There is nothing else between a policy and the car.
/// </summary>
public interface IDrivingPolicy
{
    void Act(ReadOnlySpan<float> observation, Span<float> action);
}

/// <summary>
/// Drives exactly what the coach suggests: the policy that reproduces the
/// analytic baseline through the direct interface. It is the pipeline's
/// ground truth — if this policy cannot lap close to the reference driver,
/// the interface or the observation is broken, not the learner.
/// </summary>
public sealed class CoachPassthroughPolicy : IDrivingPolicy
{
    public void Act(ReadOnlySpan<float> observation, Span<float> action)
    {
        action[0] = observation[DirectDriveObservation.CoachCurvatureIndex];
        action[1] = observation[DirectDriveObservation.CoachAccelerationIndex];
    }
}
