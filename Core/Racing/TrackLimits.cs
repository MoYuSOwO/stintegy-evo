using System;

namespace StintegyEVO.Core.Racing;

/// <summary>
/// The race ruler for track limits: a car is off only when all four wheels
/// are beyond the white line, and the line itself is track.
/// </summary>
public static class TrackLimits
{
    /// <summary>
    /// Whether every wheel is past the line, on either side of the road.
    /// Offsets are across the road from its centre, in metres; the sign
    /// says which side and does not matter here. Three wheels out is still
    /// a car on the circuit, which is the whole point of this ruler.
    /// </summary>
    public static bool AllFourWheelsBeyondTheLine(
        float halfWidth,
        float frontLeft,
        float frontRight,
        float rearLeft,
        float rearRight
    )
    {
        float line = MathF.Max(0f, halfWidth);
        return MathF.Abs(frontLeft) > line &&
               MathF.Abs(frontRight) > line &&
               MathF.Abs(rearLeft) > line &&
               MathF.Abs(rearRight) > line;
    }
}
