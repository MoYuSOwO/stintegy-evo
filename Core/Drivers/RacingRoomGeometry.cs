using System;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.Core.Drivers;

internal readonly record struct RacingRoomCandidate(
    string LeftCarId,
    string RightCarId,
    string EntryTrailingCarId,
    float LongitudinalOverlapMeters,
    float SeparatorFraction
);

/// <summary>
/// Pure geometry rules for deciding when two cars have earned persistent
/// side-by-side racing room. This deliberately reads only the frozen physical
/// snapshot; planned paths and traffic decisions cannot feed back into it.
/// </summary>
internal static class RacingRoomGeometry
{
    internal const float EntitlementOverlapMeters = 0.2f;
    internal const float ReleaseClearanceMeters = 0.5f;

    private const float SameDirectionDotThreshold = 0.5f;
    private const float TrackEdgeMarginMeters = 0.3f;
    private const float InterCarClearanceMeters = 0.2f;
    private const float Epsilon = 1e-5f;

    public static bool TryCreateCandidate(
        in RaceCarSnapshot first,
        in RaceCarSnapshot second,
        out RacingRoomCandidate candidate
    )
    {
        candidate = default;
        if (string.Equals(first.Id, second.Id, StringComparison.Ordinal) ||
            first.Region != TrackRegion.RacingSurface ||
            second.Region != TrackRegion.RacingSurface ||
            HeadingDot(in first, in second) <= SameDirectionDotThreshold ||
            !FitsOnRacingSurface(in first) ||
            !FitsOnRacingSurface(in second) ||
            !HasTwoCarTrackCapacity(in first, in second))
        {
            return false;
        }

        RaceCarSnapshot leader;
        RaceCarSnapshot trailing;
        float firstLead = SignedLongitudinalDelta(in first, in second);
        if (firstLead > Epsilon ||
            (MathF.Abs(firstLead) <= Epsilon &&
             string.CompareOrdinal(first.Id, second.Id) < 0))
        {
            leader = first;
            trailing = second;
        }
        else
        {
            leader = second;
            trailing = first;
        }

        float centerSeparation = MathF.Abs(firstLead);
        float overlap = 0.5f * (
            MathF.Max(0f, trailing.WheelBaseMeters) +
            MathF.Max(0f, leader.WheelBaseMeters)
        ) - centerSeparation;
        if (overlap + Epsilon < EntitlementOverlapMeters)
            return false;

        CarBodyGeometry firstBody = CarBodyGeometry.FromPose(
            first.Position,
            first.HeadingRadians,
            first.LengthMeters,
            first.WidthMeters
        );
        CarBodyGeometry secondBody = CarBodyGeometry.FromPose(
            second.Position,
            second.HeadingRadians,
            second.LengthMeters,
            second.WidthMeters
        );
        if (firstBody.Overlaps(in secondBody))
            return false;

        RaceCarSnapshot left;
        RaceCarSnapshot right;
        if (first.TrackD > second.TrackD)
        {
            left = first;
            right = second;
        }
        else
        {
            left = second;
            right = first;
        }

        float leftInnerEdgeD = left.TrackD - left.WidthMeters * 0.5f;
        float rightInnerEdgeD = right.TrackD + right.WidthMeters * 0.5f;
        float separatorD = 0.5f * (leftInnerEdgeD + rightInnerEdgeD);
        float halfWidth = MathF.Max(
            Epsilon,
            MathF.Min(first.TrackWidthMeters, second.TrackWidthMeters) * 0.5f
        );
        float minimumSeparatorD = -halfWidth + TrackEdgeMarginMeters +
                                  right.WidthMeters +
                                  InterCarClearanceMeters * 0.5f;
        float maximumSeparatorD = halfWidth - TrackEdgeMarginMeters -
                                  left.WidthMeters -
                                  InterCarClearanceMeters * 0.5f;
        separatorD = Math.Clamp(
            separatorD,
            minimumSeparatorD,
            maximumSeparatorD
        );

        candidate = new RacingRoomCandidate(
            left.Id,
            right.Id,
            trailing.Id,
            overlap,
            Math.Clamp(separatorD / halfWidth, -1f, 1f)
        );
        return true;
    }

    public static bool HasFullBodyClearance(
        in RaceCarSnapshot first,
        in RaceCarSnapshot second
    )
    {
        float centerSeparation = MathF.Abs(
            SignedLongitudinalDelta(in first, in second)
        );
        float required = 0.5f * (
            MathF.Max(0f, first.LengthMeters) +
            MathF.Max(0f, second.LengthMeters)
        ) + ReleaseClearanceMeters;
        return centerSeparation + Epsilon >= required;
    }

    /// <summary>
    /// Signed local longitudinal separation, positive when first is ahead.
    /// Race distance supplies continuity at the lap seam; reducing whole-lap
    /// differences keeps lapped cars physically beside each other rather than
    /// pretending they are a circuit apart.
    /// </summary>
    internal static float SignedLongitudinalDelta(
        in RaceCarSnapshot first,
        in RaceCarSnapshot second
    )
    {
        float delta = first.RaceDistanceMeters - second.RaceDistanceMeters;
        float trackLength = MathF.Max(
            first.TrackLengthMeters,
            second.TrackLengthMeters
        );
        if (trackLength > Epsilon)
            delta -= MathF.Round(delta / trackLength) * trackLength;
        return delta;
    }

    private static bool FitsOnRacingSurface(in RaceCarSnapshot car)
    {
        float centerLimit = car.TrackWidthMeters * 0.5f -
                            car.WidthMeters * 0.5f;
        return centerLimit >= 0f &&
               MathF.Abs(car.TrackD) <= centerLimit + Epsilon;
    }

    private static bool HasTwoCarTrackCapacity(
        in RaceCarSnapshot first,
        in RaceCarSnapshot second
    )
    {
        float available = MathF.Min(
            first.TrackWidthMeters,
            second.TrackWidthMeters
        );
        float required = first.WidthMeters + second.WidthMeters +
                         2f * TrackEdgeMarginMeters +
                         InterCarClearanceMeters;
        return available + Epsilon >= required;
    }

    private static float HeadingDot(
        in RaceCarSnapshot first,
        in RaceCarSnapshot second
    )
    {
        return MathF.Cos(first.HeadingRadians - second.HeadingRadians);
    }
}
