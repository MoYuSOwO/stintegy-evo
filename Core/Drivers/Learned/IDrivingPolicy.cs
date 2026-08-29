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
