using System;
using System.Numerics;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Converts state-only opponent predictions into longitudinal constraints on
/// an already selected ego path. No opponent driver or opponent plan is read.
/// </summary>
internal sealed class TrafficSpeedConstraintPlanner
{
    private const float SameDirectionDotThreshold = 0.5f;
    private const float MovingSpeedThresholdMetersPerSecond = 0.5f;
    private const float OpponentAccelerationHoldSeconds = 0.75f;
    private const float OpponentLateralVelocityHoldSeconds = 0.75f;
    private const float OpponentHeadingConvergenceSeconds = 0.75f;
    private const float MaximumOpponentDriveAcceleration = 5f;
    private const float MaximumOpponentBrakeDeceleration = 14f;
    private const float LongitudinalUncertaintyGrowthMetersPerSecond = 0.08f;
    private const float LateralUncertaintyGrowthMetersPerSecond = 0.04f;
    private const float FollowingReleaseMarginMeters = 1.5f;
    private const float ConstraintEpsilon = 1e-4f;
    private const int ConflictSearchStride = 4;
    private const int PathProjectionSearchRadius = 16;
    private const float BroadphaseCurvePaddingMeters = 2f;

    private string? _heldOpponentId;
    private TrafficSpeedConstraintKind _heldKind;
    private float _heldUntilSeconds;
    private float _heldRemainingDistanceMeters;
    private float _heldTargetSpeedMetersPerSecond;
    private float _heldConflictTimeSeconds;
    private Vector2 _heldEgoPosition;

    public TrafficSpeedConstraint LastConstraint { get; private set; }

    public void BeginPlan(bool enabled, in RaceFrameSnapshot frame)
    {
        LastConstraint = default;
        if (enabled && frame.Count > 1)
            return;

        ClearHold();
    }

    public bool ApplyConstraints(
        VehicleSpeedPlanningConfig config,
        TrackData track,
        VehiclePathPrediction path,
        in RaceFrameSnapshot frame,
        int egoSnapshotIndex,
        float[] segmentLengths,
        float[] speeds,
        float[] speedLimits,
        float[] arrivalTimes
    )
    {
        if (!config.EnableTrafficAvoidance ||
            egoSnapshotIndex < 0 ||
            egoSnapshotIndex >= frame.Count ||
            frame.Count <= 1)
        {
            return false;
        }

        RaceCarSnapshot ego = frame[egoSnapshotIndex];
        FillArrivalTimes(
            path.Count,
            segmentLengths,
            speeds,
            arrivalTimes
        );

        bool changed = ApplyHeldConstraint(
            config,
            track,
            path,
            in frame,
            in ego,
            speedLimits
        );

        ReadOnlySpan<RaceCarSnapshot> cars = frame.Cars;
        for (int opponentIndex = 0; opponentIndex < cars.Length; opponentIndex++)
        {
            if (opponentIndex == egoSnapshotIndex)
                continue;

            RaceCarSnapshot opponent = cars[opponentIndex];
            if (string.Equals(opponent.Id, ego.Id, StringComparison.Ordinal))
                continue;

            float currentDirectionDot = HeadingDot(
                ego.HeadingRadians,
                opponent.HeadingRadians
            );
            if (!ShouldYield(
                    track,
                    in ego,
                    in opponent,
                    currentDirectionDot
                ))
            {
                continue;
            }

            float estimatedAlongDistance = track.WrapS(
                opponent.TrackS - ego.TrackS
            );
            PathProjection currentProjection = ProjectOntoPath(
                path,
                opponent.Position,
                estimatedAlongDistance
            );
            changed |= ApplyCloseFollowingConstraint(
                config,
                in ego,
                in opponent,
                in currentProjection,
                currentDirectionDot,
                speedLimits
            );

            if (TryFindFirstConflict(
                    config,
                    track,
                    path,
                    in ego,
                    in opponent,
                    speeds,
                    arrivalTimes,
                    out PredictedConflict conflict
                ))
            {
                changed |= ApplyPredictedConflict(
                    config,
                    path,
                    in ego,
                    in opponent,
                    in currentProjection,
                    in conflict,
                    speedLimits,
                    frame.RaceTimeSeconds
                );
            }
        }

        return changed;
    }

    private bool ApplyPredictedConflict(
        VehicleSpeedPlanningConfig config,
        VehiclePathPrediction path,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        in PathProjection currentProjection,
        in PredictedConflict conflict,
        float[] speedLimits,
        float raceTimeSeconds
    )
    {
        float desiredGap = DesiredGap(
            config,
            conflict.EgoSpeedMetersPerSecond
        );
        float constraintDistance = MathF.Max(
            0f,
            conflict.PathDistanceMeters - desiredGap
        );
        int constraintIndex = IndexAtOrBefore(path, constraintDistance);
        bool changed = ApplySpeedConstraint(
            conflict.Kind,
            constraintIndex,
            conflict.TargetSpeedMetersPerSecond,
            speedLimits,
            path.Count
        );

        float currentClearance = CurrentClearance(
            in ego,
            in opponent,
            in currentProjection
        );
        TrafficSpeedConstraint constraint = new(
            conflict.Kind,
            opponent.Id,
            path[constraintIndex].DistanceMeters,
            conflict.TargetSpeedMetersPerSecond,
            conflict.TimeSeconds,
            currentClearance
        );
        if (RecordConstraint(in constraint))
        {
            _heldOpponentId = opponent.Id;
            _heldKind = conflict.Kind;
            _heldUntilSeconds = raceTimeSeconds + config.TrafficConstraintHoldSeconds;
            _heldRemainingDistanceMeters = constraint.PathDistanceMeters;
            _heldTargetSpeedMetersPerSecond = conflict.TargetSpeedMetersPerSecond;
            _heldConflictTimeSeconds = conflict.TimeSeconds;
            _heldEgoPosition = ego.Position;
        }
        return changed;
    }

    private bool ApplyCloseFollowingConstraint(
        VehicleSpeedPlanningConfig config,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        in PathProjection projection,
        float directionDot,
        float[] speedLimits
    )
    {
        if (!projection.IsValid ||
            directionDot <= SameDirectionDotThreshold ||
            projection.AlongDistanceMeters <= 0f)
        {
            return false;
        }

        float lateralLimit = (ego.WidthMeters + opponent.WidthMeters) * 0.5f +
                             config.TrafficLateralSafetyMarginMeters;
        if (MathF.Abs(projection.LateralDistanceMeters) > lateralLimit)
            return false;

        float clearance = CurrentClearance(
            in ego,
            in opponent,
            in projection
        );
        float desiredGap = DesiredGap(
            config,
            ego.SpeedMetersPerSecond
        );
        if (clearance > desiredGap + FollowingReleaseMarginMeters)
            return false;

        float opponentAlongSpeed = MathF.Max(
            0f,
            Vector2.Dot(opponent.Velocity, projection.Tangent)
        );
        TrafficSpeedConstraintKind kind =
            opponentAlongSpeed > MovingSpeedThresholdMetersPerSecond
                ? TrafficSpeedConstraintKind.Follow
                : TrafficSpeedConstraintKind.Stop;
        float constraintDistance = MathF.Max(
            0f,
            projection.AlongDistanceMeters -
            (ego.LengthMeters + opponent.LengthMeters) * 0.5f -
            desiredGap
        );
        int constraintIndex = IndexAtOrBeforeDistance(
            projection.Path,
            constraintDistance
        );
        bool changed = ApplySpeedConstraint(
            kind,
            constraintIndex,
            opponentAlongSpeed,
            speedLimits,
            projection.Path.Count
        );
        TrafficSpeedConstraint constraint = new(
            kind,
            opponent.Id,
            projection.Path[constraintIndex].DistanceMeters,
            opponentAlongSpeed,
            0f,
            clearance
        );
        RecordConstraint(in constraint);
        return changed;
    }

    private bool ApplyHeldConstraint(
        VehicleSpeedPlanningConfig config,
        TrackData track,
        VehiclePathPrediction path,
        in RaceFrameSnapshot frame,
        in RaceCarSnapshot ego,
        float[] speedLimits
    )
    {
        if (_heldOpponentId is null ||
            frame.RaceTimeSeconds > _heldUntilSeconds ||
            !frame.TryGetCar(_heldOpponentId, out RaceCarSnapshot opponent))
        {
            ClearHold();
            return false;
        }

        float travelled = Vector2.Distance(ego.Position, _heldEgoPosition);
        _heldRemainingDistanceMeters = MathF.Max(
            0f,
            _heldRemainingDistanceMeters - travelled
        );
        _heldEgoPosition = ego.Position;
        int index = IndexAtOrBefore(path, _heldRemainingDistanceMeters);
        bool changed = ApplySpeedConstraint(
            _heldKind,
            index,
            _heldTargetSpeedMetersPerSecond,
            speedLimits,
            path.Count
        );
        float estimatedAlongDistance = track.WrapS(
            opponent.TrackS - ego.TrackS
        );
        PathProjection projection = ProjectOntoPath(
            path,
            opponent.Position,
            estimatedAlongDistance
        );
        TrafficSpeedConstraint constraint = new(
            _heldKind,
            opponent.Id,
            path[index].DistanceMeters,
            _heldTargetSpeedMetersPerSecond,
            _heldConflictTimeSeconds,
            CurrentClearance(in ego, in opponent, in projection)
        );
        RecordConstraint(in constraint);
        return changed;
    }

    private static bool TryFindFirstConflict(
        VehicleSpeedPlanningConfig config,
        TrackData track,
        VehiclePathPrediction path,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        float[] speeds,
        float[] arrivalTimes,
        out PredictedConflict conflict
    )
    {
        int lastIndex = 0;
        while (lastIndex + 1 < path.Count &&
               float.IsFinite(arrivalTimes[lastIndex + 1]) &&
               arrivalTimes[lastIndex + 1] <=
               config.TrafficPredictionHorizonSeconds)
        {
            lastIndex++;
        }
        if (lastIndex == 0)
        {
            conflict = default;
            return false;
        }

        int rangeStart = 0;
        PredictedTrafficPose opponentAtRangeStart = PredictOpponent(
            track,
            in opponent,
            arrivalTimes[0]
        );
        while (rangeStart < lastIndex)
        {
            int rangeEnd = Math.Min(
                rangeStart + ConflictSearchStride,
                lastIndex
            );
            PredictedTrafficPose opponentAtRangeEnd = PredictOpponent(
                track,
                in opponent,
                arrivalTimes[rangeEnd]
            );
            Vector2 relativeStart = opponentAtRangeStart.Position -
                                    path[rangeStart].Position;
            Vector2 relativeEnd = opponentAtRangeEnd.Position -
                                  path[rangeEnd].Position;
            float broadphaseRadius = BroadphaseRadius(
                config,
                in ego,
                in opponent,
                arrivalTimes[rangeEnd]
            );
            if (SquaredDistanceFromOriginToSegment(
                    relativeStart,
                    relativeEnd
                ) <= broadphaseRadius * broadphaseRadius)
            {
                for (int i = rangeStart + 1; i <= rangeEnd; i++)
                {
                    if (TryBuildConflictAtIndex(
                            config,
                            track,
                            path,
                            in ego,
                            in opponent,
                            speeds,
                            arrivalTimes,
                            i,
                            out conflict
                        ))
                    {
                        return true;
                    }
                }
            }

            rangeStart = rangeEnd;
            opponentAtRangeStart = opponentAtRangeEnd;
        }

        conflict = default;
        return false;
    }

    private static bool TryBuildConflictAtIndex(
        VehicleSpeedPlanningConfig config,
        TrackData track,
        VehiclePathPrediction path,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        float[] speeds,
        float[] arrivalTimes,
        int index,
        out PredictedConflict conflict
    )
    {
        float time = arrivalTimes[index];
        VehiclePathPredictionPoint egoPoint = path[index];
        PredictedTrafficPose opponentPose = PredictOpponent(
            track,
            in opponent,
            time
        );
        float longitudinalGrowth =
            LongitudinalUncertaintyGrowthMetersPerSecond * time;
        float lateralGrowth =
            LateralUncertaintyGrowthMetersPerSecond * time;
        CarBodyGeometry egoBody = CarBodyGeometry.FromPose(
            egoPoint.Position,
            egoPoint.VelocityHeading,
            ego.LengthMeters +
            2f * config.TrafficLongitudinalSafetyMarginMeters,
            ego.WidthMeters +
            2f * config.TrafficLateralSafetyMarginMeters
        );
        CarBodyGeometry opponentBody = CarBodyGeometry.FromPose(
            opponentPose.Position,
            opponentPose.HeadingRadians,
            opponent.LengthMeters + 2f * longitudinalGrowth,
            opponent.WidthMeters + 2f * lateralGrowth
        );
        if (!egoBody.Overlaps(in opponentBody))
        {
            conflict = default;
            return false;
        }

        Vector2 egoForward = new(
            MathF.Cos(egoPoint.VelocityHeading),
            MathF.Sin(egoPoint.VelocityHeading)
        );
        float directionDot = Vector2.Dot(
            egoForward,
            opponentBody.Forward
        );
        float opponentAlongSpeed = MathF.Max(
            0f,
            Vector2.Dot(opponentPose.Velocity, egoForward)
        );
        TrafficSpeedConstraintKind kind =
            directionDot > SameDirectionDotThreshold &&
            opponentAlongSpeed > MovingSpeedThresholdMetersPerSecond
                ? TrafficSpeedConstraintKind.Follow
                : TrafficSpeedConstraintKind.Stop;
        conflict = new PredictedConflict(
            kind,
            egoPoint.DistanceMeters,
            time,
            speeds[index],
            kind == TrafficSpeedConstraintKind.Follow
                ? opponentAlongSpeed
                : 0f
        );
        return true;
    }

    private static float BroadphaseRadius(
        VehicleSpeedPlanningConfig config,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        float timeSeconds
    )
    {
        float egoLength = ego.LengthMeters +
                          2f * config.TrafficLongitudinalSafetyMarginMeters;
        float egoWidth = ego.WidthMeters +
                         2f * config.TrafficLateralSafetyMarginMeters;
        float opponentLength = opponent.LengthMeters +
                               2f * LongitudinalUncertaintyGrowthMetersPerSecond *
                               timeSeconds;
        float opponentWidth = opponent.WidthMeters +
                              2f * LateralUncertaintyGrowthMetersPerSecond *
                              timeSeconds;
        return 0.5f * MathF.Sqrt(
                   egoLength * egoLength + egoWidth * egoWidth
               ) +
               0.5f * MathF.Sqrt(
                   opponentLength * opponentLength +
                   opponentWidth * opponentWidth
               ) +
               BroadphaseCurvePaddingMeters;
    }

    private static float SquaredDistanceFromOriginToSegment(
        Vector2 start,
        Vector2 end
    )
    {
        Vector2 segment = end - start;
        float lengthSquared = segment.LengthSquared();
        if (lengthSquared <= 1e-8f)
            return start.LengthSquared();
        float t = Math.Clamp(
            -Vector2.Dot(start, segment) / lengthSquared,
            0f,
            1f
        );
        return (start + segment * t).LengthSquared();
    }

    private static PredictedTrafficPose PredictOpponent(
        TrackData track,
        in RaceCarSnapshot opponent,
        float timeSeconds
    )
    {
        if (timeSeconds <= 0f)
        {
            return new PredictedTrafficPose(
                opponent.Position,
                opponent.HeadingRadians,
                opponent.Velocity
            );
        }

        TrackSample currentSample = track.Sample(opponent.TrackS);
        Vector2 velocity = opponent.Velocity;
        float signedLongitudinalSpeed = Vector2.Dot(
            velocity,
            currentSample.Tangent
        );
        float lateralSpeed = Vector2.Dot(velocity, currentSample.Normal);
        float direction = signedLongitudinalSpeed < -MovingSpeedThresholdMetersPerSecond
            ? -1f
            : 1f;
        float longitudinalSpeed = MathF.Abs(signedLongitudinalSpeed);
        float acceleration = float.IsFinite(
            opponent.LongitudinalAccelMetersPerSecondSquared
        )
            ? Math.Clamp(
                opponent.LongitudinalAccelMetersPerSecondSquared * direction,
                -MaximumOpponentBrakeDeceleration,
                MaximumOpponentDriveAcceleration
            )
            : 0f;
        IntegrateHeldAcceleration(
            longitudinalSpeed,
            acceleration,
            timeSeconds,
            out float longitudinalDistance,
            out float predictedLongitudinalSpeed
        );

        float predictedS = opponent.TrackS + direction * longitudinalDistance;
        TrackSample predictedSample = track.Sample(predictedS);
        float lateralTime = MathF.Min(
            timeSeconds,
            OpponentLateralVelocityHoldSeconds
        );
        float predictedD = opponent.TrackD + lateralSpeed * lateralTime;
        Vector2 position = predictedSample.Center +
                           predictedSample.Normal * predictedD;

        float predictedTrackHeading = MathF.Atan2(
            direction * predictedSample.Tangent.Y,
            direction * predictedSample.Tangent.X
        );
        float currentTrackHeading = MathF.Atan2(
            direction * currentSample.Tangent.Y,
            direction * currentSample.Tangent.X
        );
        float heading;
        if (opponent.SpeedMetersPerSecond < MovingSpeedThresholdMetersPerSecond)
        {
            heading = opponent.HeadingRadians;
        }
        else
        {
            float headingOffset = MathHelper.NormalizeAngle(
                opponent.HeadingRadians - currentTrackHeading
            );
            float headingWeight = Math.Clamp(
                1f - timeSeconds / OpponentHeadingConvergenceSeconds,
                0f,
                1f
            );
            heading = MathHelper.NormalizeAngle(
                predictedTrackHeading + headingOffset * headingWeight
            );
        }

        float retainedLateralSpeed = timeSeconds <
                                     OpponentLateralVelocityHoldSeconds
            ? lateralSpeed
            : 0f;
        Vector2 predictedVelocity =
            direction * predictedSample.Tangent * predictedLongitudinalSpeed +
            predictedSample.Normal * retainedLateralSpeed;
        return new PredictedTrafficPose(
            position,
            heading,
            predictedVelocity
        );
    }

    private static void IntegrateHeldAcceleration(
        float initialSpeed,
        float acceleration,
        float totalTime,
        out float distance,
        out float finalSpeed
    )
    {
        float accelerationTime = MathF.Min(
            totalTime,
            OpponentAccelerationHoldSeconds
        );
        if (acceleration < 0f && initialSpeed > 0f)
        {
            accelerationTime = MathF.Min(
                accelerationTime,
                -initialSpeed / acceleration
            );
        }

        distance = initialSpeed * accelerationTime +
                   0.5f * acceleration * accelerationTime * accelerationTime;
        finalSpeed = MathF.Max(
            0f,
            initialSpeed + acceleration * accelerationTime
        );
        if (accelerationTime >= OpponentAccelerationHoldSeconds)
        {
            distance += finalSpeed *
                        (totalTime - OpponentAccelerationHoldSeconds);
        }
        else if (acceleration >= 0f)
        {
            distance += finalSpeed * (totalTime - accelerationTime);
        }
        distance = MathF.Max(0f, distance);
    }

    private static void FillArrivalTimes(
        int count,
        float[] segmentLengths,
        float[] speeds,
        float[] arrivalTimes
    )
    {
        arrivalTimes[0] = 0f;
        for (int i = 1; i < count; i++)
        {
            float speedSum = speeds[i - 1] + speeds[i];
            if (!float.IsFinite(arrivalTimes[i - 1]) || speedSum <= 0.05f)
            {
                arrivalTimes[i] = float.PositiveInfinity;
                continue;
            }

            arrivalTimes[i] = arrivalTimes[i - 1] +
                              2f * segmentLengths[i - 1] / speedSum;
        }
    }

    private static bool ApplySpeedConstraint(
        TrafficSpeedConstraintKind kind,
        int constraintIndex,
        float targetSpeed,
        float[] speedLimits,
        int count
    )
    {
        targetSpeed = MathF.Max(0f, targetSpeed);
        bool changed = false;
        if (kind == TrafficSpeedConstraintKind.Follow)
        {
            for (int i = constraintIndex; i < count; i++)
            {
                if (targetSpeed >= speedLimits[i] - ConstraintEpsilon)
                    continue;
                speedLimits[i] = targetSpeed;
                changed = true;
            }
            return changed;
        }

        if (targetSpeed < speedLimits[constraintIndex] - ConstraintEpsilon)
        {
            speedLimits[constraintIndex] = targetSpeed;
            changed = true;
        }
        return changed;
    }

    private bool RecordConstraint(in TrafficSpeedConstraint constraint)
    {
        TrafficSpeedConstraint current = LastConstraint;
        if (current.Active && !IsHigherPriority(
                in constraint,
                in current
            ))
            return false;

        LastConstraint = constraint;
        return true;
    }

    private static bool IsHigherPriority(
        in TrafficSpeedConstraint candidate,
        in TrafficSpeedConstraint current
    )
    {
        float distanceDelta = candidate.PathDistanceMeters -
                              current.PathDistanceMeters;
        if (distanceDelta < -ConstraintEpsilon)
            return true;
        if (distanceDelta > ConstraintEpsilon)
            return false;
        if (candidate.Kind != current.Kind)
            return candidate.Kind == TrafficSpeedConstraintKind.Stop;
        float speedDelta = candidate.TargetSpeedMetersPerSecond -
                           current.TargetSpeedMetersPerSecond;
        if (speedDelta < -ConstraintEpsilon)
            return true;
        if (speedDelta > ConstraintEpsilon)
            return false;
        return string.CompareOrdinal(
            candidate.OpponentId,
            current.OpponentId
        ) < 0;
    }

    private static bool ShouldYield(
        TrackData track,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        float directionDot
    )
    {
        if (directionDot <= SameDirectionDotThreshold)
        {
            return string.CompareOrdinal(ego.Id, opponent.Id) > 0;
        }

        float forwardDelta = track.WrapS(opponent.TrackS - ego.TrackS);
        if (forwardDelta > 0.25f &&
            forwardDelta < track.LengthMeters * 0.5f)
        {
            return true;
        }
        if (forwardDelta > track.LengthMeters * 0.5f)
            return false;
        return string.CompareOrdinal(ego.Id, opponent.Id) > 0;
    }

    private static float HeadingDot(float first, float second)
    {
        return MathF.Cos(MathHelper.NormalizeAngle(first - second));
    }

    private static float DesiredGap(
        VehicleSpeedPlanningConfig config,
        float egoSpeedMetersPerSecond
    )
    {
        return config.TrafficMinimumGapMeters +
               config.TrafficTimeHeadwaySeconds *
               MathF.Max(0f, egoSpeedMetersPerSecond);
    }

    private static float CurrentClearance(
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        in PathProjection projection
    )
    {
        if (!projection.IsValid)
            return float.PositiveInfinity;
        return projection.AlongDistanceMeters -
               (ego.LengthMeters + opponent.LengthMeters) * 0.5f;
    }

    private static PathProjection ProjectOntoPath(
        VehiclePathPrediction path,
        Vector2 position,
        float estimatedAlongDistanceMeters
    )
    {
        float bestDistanceSquared = float.PositiveInfinity;
        float bestAlongDistance = 0f;
        float bestLateralDistance = 0f;
        Vector2 bestTangent = Vector2.UnitX;
        bool found = false;

        float normalizedDistance = path.LengthMeters <= 1e-5f
            ? 0f
            : Math.Clamp(
                estimatedAlongDistanceMeters / path.LengthMeters,
                0f,
                1f
            );
        int centerIndex = (int)MathF.Round(
            normalizedDistance * (path.Count - 1)
        );
        int firstIndex = Math.Max(
            0,
            centerIndex - PathProjectionSearchRadius
        );
        int lastIndex = Math.Min(
            path.Count - 2,
            centerIndex + PathProjectionSearchRadius
        );
        for (int i = firstIndex; i <= lastIndex; i++)
        {
            VehiclePathPredictionPoint start = path[i];
            VehiclePathPredictionPoint end = path[i + 1];
            Vector2 segment = end.Position - start.Position;
            float lengthSquared = segment.LengthSquared();
            if (lengthSquared <= 1e-8f)
                continue;

            float t = Math.Clamp(
                Vector2.Dot(position - start.Position, segment) /
                lengthSquared,
                0f,
                1f
            );
            Vector2 projected = start.Position + segment * t;
            Vector2 delta = position - projected;
            float distanceSquared = delta.LengthSquared();
            if (distanceSquared >= bestDistanceSquared)
                continue;

            float length = MathF.Sqrt(lengthSquared);
            Vector2 tangent = segment / length;
            Vector2 left = new(-tangent.Y, tangent.X);
            bestDistanceSquared = distanceSquared;
            bestAlongDistance = start.DistanceMeters +
                                t * (end.DistanceMeters - start.DistanceMeters);
            bestLateralDistance = Vector2.Dot(delta, left);
            bestTangent = tangent;
            found = true;
        }

        return new PathProjection(
            path,
            found,
            bestAlongDistance,
            bestLateralDistance,
            bestTangent
        );
    }

    private static int IndexAtOrBefore(
        VehiclePathPrediction path,
        float distanceMeters
    )
    {
        return IndexAtOrBeforeDistance(path, distanceMeters);
    }

    private static int IndexAtOrBeforeDistance(
        VehiclePathPrediction path,
        float distanceMeters
    )
    {
        int low = 0;
        int high = path.Count - 1;
        while (low < high)
        {
            int middle = (low + high + 1) / 2;
            if (path[middle].DistanceMeters <= distanceMeters)
                low = middle;
            else
                high = middle - 1;
        }
        return low;
    }

    private void ClearHold()
    {
        _heldOpponentId = null;
        _heldKind = TrafficSpeedConstraintKind.None;
        _heldUntilSeconds = 0f;
        _heldRemainingDistanceMeters = 0f;
        _heldTargetSpeedMetersPerSecond = 0f;
        _heldConflictTimeSeconds = 0f;
        _heldEgoPosition = default;
    }

    private readonly record struct PathProjection(
        VehiclePathPrediction Path,
        bool IsValid,
        float AlongDistanceMeters,
        float LateralDistanceMeters,
        Vector2 Tangent
    );

    private readonly record struct PredictedTrafficPose(
        Vector2 Position,
        float HeadingRadians,
        Vector2 Velocity
    );

    private readonly record struct PredictedConflict(
        TrafficSpeedConstraintKind Kind,
        float PathDistanceMeters,
        float TimeSeconds,
        float EgoSpeedMetersPerSecond,
        float TargetSpeedMetersPerSecond
    );
}
