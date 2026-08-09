using System;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Refines one already-committed spatial handover against the same Stanley
/// prediction and speed planning used for execution. This runs only when a
/// target changes; the selected length remains fixed afterwards.
/// </summary>
internal static class ReferenceLineHandoverSpeedSelector
{
    private const float AllowedTargetSpeedLossMetersPerSecond = 0.15f;

    public static void Refine(
        TrackConstrainedLateralOffset profile,
        float baselineTargetSpeedMetersPerSecond,
        Func<float> evaluateCurrentTargetSpeed
    )
    {
        ArgumentNullException.ThrowIfNull(profile);
        ArgumentNullException.ThrowIfNull(evaluateCurrentTargetSpeed);
        float baselineSpeed = MathF.Max(
            0f,
            baselineTargetSpeedMetersPerSecond
        );
        if (baselineSpeed <= AllowedTargetSpeedLossMetersPerSecond)
            return;

        if (RetainsSpeed(evaluateCurrentTargetSpeed(), baselineSpeed))
        {
            return;
        }

        // The cheap geometry pass has already found the shortest physically
        // comfortable line. If the real controller and speed planner still
        // price it, use the whole tactical window. The normal frame plan will
        // evaluate that final line anyway, so probing intermediate lengths
        // here would add latency without changing the safety decision.
        profile.SetCommittedHandoverLength(
            profile.MaximumCommittedHandoverLengthMeters
        );
    }

    private static bool RetainsSpeed(float targetSpeed, float baselineSpeed)
    {
        return targetSpeed + AllowedTargetSpeedLossMetersPerSecond >=
               baselineSpeed;
    }
}
