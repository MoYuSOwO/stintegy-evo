using System;
using System.Numerics;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Converts frozen opponent motion predictions into longitudinal constraints
/// on an already selected ego path, with state extrapolation as a fallback.
/// </summary>
internal static class TrafficConflictEvaluator
{
    private const float SameDirectionDotThreshold = 0.5f;
    private const float MovingSpeedThresholdMetersPerSecond = 0.5f;
    private const float OpponentAccelerationHoldSeconds = 0.75f;
    private const float OpponentLateralVelocityHoldSeconds = 0.75f;
    private const float OpponentHeadingConvergenceSeconds = 0.75f;
    private const float ReferenceOffsetDerivativeProbeMeters = 1f;
    private const float MaximumOpponentDriveAcceleration = 5f;
    private const float MaximumOpponentBrakeDeceleration = 14f;
    private const float LongitudinalUncertaintyGrowthMetersPerSecond = 0.08f;
    private const float LateralUncertaintyGrowthMetersPerSecond = 0.04f;
    private const float FollowingReleaseMarginMeters = 1.5f;
    private const float ConstraintEpsilon = 1e-4f;
    private const int ConflictSearchStride = 4;
    private const int PathProjectionSearchRadius = 16;
    private const float BroadphaseCurvePaddingMeters = 2f;
    private const float ClearanceSearchStepSeconds = 0.1f;
    private const int ClearanceRefinementIterations = 5;

    public static bool ApplyConstraints(
        VehicleSpeedPlanningConfig config,
        TrackData track,
        VehiclePathPrediction path,
        in RaceFrameSnapshot frame,
        int egoSnapshotIndex,
        float[] segmentLengths,
        float[] speeds,
        float[] speedLimits,
        float[] arrivalTimes,
        ref TrafficConstraintMemory memory,
        ref TrafficSpeedConstraint lastConstraint
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

        if (memory.OpponentId is not null &&
            frame.FindTrafficMotionPlan(memory.OpponentId) is not null)
        {
            // The held constraint came from an older state-only estimate. A
            // freshly published motion plan supersedes that estimate and is
            // evaluated below from the same frozen frame.
            memory.Clear();
        }
        bool changed = ApplyHeldConstraint(
            config,
            track,
            path,
            in frame,
            in ego,
            speedLimits,
            ref memory,
            ref lastConstraint
        );

        ReadOnlySpan<RaceCarSnapshot> cars = frame.Cars;
        for (int opponentIndex = 0; opponentIndex < cars.Length; opponentIndex++)
        {
            if (opponentIndex == egoSnapshotIndex)
                continue;

            RaceCarSnapshot opponent = cars[opponentIndex];
            if (string.Equals(opponent.Id, ego.Id, StringComparison.Ordinal))
                continue;

            TrafficMotionPlan? opponentPlan =
                frame.GetTrafficMotionPlan(opponentIndex);

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
            if (TryFindFirstConflict(
                    config,
                    track,
                    path,
                    in ego,
                    in opponent,
                    speeds,
                    arrivalTimes,
                    opponentPlan,
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
                    track,
                    opponentPlan,
                    segmentLengths,
                    speeds,
                    speedLimits,
                    frame.RaceTimeSeconds,
                    conflict.UsesPublishedMotion,
                    ref memory,
                    ref lastConstraint
                );
            }
            else
            {
                changed |= ApplyCloseFollowingConstraint(
                    config,
                    in ego,
                    in opponent,
                    in currentProjection,
                    currentDirectionDot,
                    speedLimits,
                    ref lastConstraint
                );
            }
        }

        return changed;
    }

    private static bool ApplyPredictedConflict(
        VehicleSpeedPlanningConfig config,
        VehiclePathPrediction path,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        in PathProjection currentProjection,
        in PredictedConflict conflict,
        TrackData track,
        TrafficMotionPlan? opponentPlan,
        float[] segmentLengths,
        float[] speeds,
        float[] speedLimits,
        float raceTimeSeconds,
        bool usesPublishedMotion,
        ref TrafficConstraintMemory memory,
        ref TrafficSpeedConstraint lastConstraint
    )
    {
        int conflictIndex = IndexAtOrBefore(
            path,
            conflict.PathDistanceMeters
        );
        int constraintIndex = conflictIndex;
        float targetSpeed = conflict.TargetSpeedMetersPerSecond;
        TrafficSpeedConstraintKind appliedKind = conflict.Kind;
        bool usesArrivalConstraint = false;
        bool changed;
        if (TryFindClearTime(
                config,
                track,
                path[conflictIndex],
                in ego,
                in opponent,
                conflict.TimeSeconds,
                opponentPlan,
                out float clearTimeSeconds
            ))
        {
            float earliestArrivalTime = clearTimeSeconds +
                                        config.TrafficTimeHeadwaySeconds;
            usesArrivalConstraint = true;
            changed = ApplyArrivalConstraint(
                config,
                conflictIndex,
                earliestArrivalTime,
                segmentLengths,
                speeds,
                speedLimits,
                out targetSpeed
            );
        }
        else
        {
            float desiredGap = MathF.Max(
                DesiredGap(config, conflict.EgoSpeedMetersPerSecond),
                SafeStoppingGap(
                    config,
                    in ego,
                    in opponent,
                    conflict.EgoSpeedMetersPerSecond,
                    conflict.TargetSpeedMetersPerSecond
                )
            );
            float constraintDistance = MathF.Max(
                0f,
                conflict.PathDistanceMeters - desiredGap
            );
            constraintIndex = IndexAtOrBefore(path, constraintDistance);
            targetSpeed = 0f;
            appliedKind = TrafficSpeedConstraintKind.Stop;
            changed = ApplySpeedConstraint(
                appliedKind,
                constraintIndex,
                targetSpeed,
                speedLimits,
                path.Count
            );
        }

        float currentClearance = CurrentClearance(
            in ego,
            in opponent,
            in currentProjection
        );
        TrafficSpeedConstraint constraint = new(
            appliedKind,
            opponent.Id,
            path[constraintIndex].DistanceMeters,
            targetSpeed,
            conflict.TimeSeconds,
            currentClearance
        );
        if (RecordConstraint(in constraint, ref lastConstraint))
        {
            if (usesPublishedMotion || usesArrivalConstraint)
            {
                memory.Clear();
            }
            else
            {
                memory.OpponentId = opponent.Id;
                memory.Kind = constraint.Kind;
                memory.HeldUntilSeconds = raceTimeSeconds +
                                          config.TrafficConstraintHoldSeconds;
                memory.RemainingDistanceMeters = constraint.PathDistanceMeters;
                memory.TargetSpeedMetersPerSecond =
                    targetSpeed;
                memory.ConflictTimeSeconds = conflict.TimeSeconds;
                memory.EgoPosition = ego.Position;
            }
        }
        return changed;
    }

    private static bool ApplyCloseFollowingConstraint(
        VehicleSpeedPlanningConfig config,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        in PathProjection projection,
        float directionDot,
        float[] speedLimits,
        ref TrafficSpeedConstraint lastConstraint
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
        Vector2 pathLeft = new(
            -projection.Tangent.Y,
            projection.Tangent.X
        );
        float relativeLateralSpeed = Vector2.Dot(
            opponent.Velocity - ego.Velocity,
            pathLeft
        );
        float predictedLateralDistance =
            projection.LateralDistanceMeters +
            relativeLateralSpeed *
            MathF.Max(
                config.TrafficLateralMergePredictionSeconds,
                0f
            );
        float minimumLateralDistance = MathF.Min(
            projection.LateralDistanceMeters,
            predictedLateralDistance
        );
        float maximumLateralDistance = MathF.Max(
            projection.LateralDistanceMeters,
            predictedLateralDistance
        );
        bool mayOverlapLaterally =
            minimumLateralDistance <= lateralLimit &&
            maximumLateralDistance >= -lateralLimit;
        if (!mayOverlapLaterally)
            return false;

        float egoAlongSpeed = MathF.Max(
            0f,
            Vector2.Dot(ego.Velocity, projection.Tangent)
        );
        float opponentAlongSpeed = MathF.Max(
            0f,
            Vector2.Dot(opponent.Velocity, projection.Tangent)
        );
        float clearance = CurrentClearance(
            in ego,
            in opponent,
            in projection
        );
        float desiredGap = MathF.Max(
            DesiredGap(config, egoAlongSpeed),
            SafeStoppingGap(
                config,
                in ego,
                in opponent,
                egoAlongSpeed,
                opponentAlongSpeed
            )
        );
        if (clearance > desiredGap + FollowingReleaseMarginMeters)
            return false;

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
        RecordConstraint(in constraint, ref lastConstraint);
        return changed;
    }

    private static bool ApplyHeldConstraint(
        VehicleSpeedPlanningConfig config,
        TrackData track,
        VehiclePathPrediction path,
        in RaceFrameSnapshot frame,
        in RaceCarSnapshot ego,
        float[] speedLimits,
        ref TrafficConstraintMemory memory,
        ref TrafficSpeedConstraint lastConstraint
    )
    {
        if (memory.OpponentId is null ||
            frame.RaceTimeSeconds > memory.HeldUntilSeconds ||
            !frame.TryGetCar(memory.OpponentId, out RaceCarSnapshot opponent))
        {
            memory.Clear();
            return false;
        }

        float travelled = Vector2.Distance(ego.Position, memory.EgoPosition);
        memory.RemainingDistanceMeters = MathF.Max(
            0f,
            memory.RemainingDistanceMeters - travelled
        );
        memory.EgoPosition = ego.Position;
        int index = IndexAtOrBefore(path, memory.RemainingDistanceMeters);
        bool changed = ApplySpeedConstraint(
            memory.Kind,
            index,
            memory.TargetSpeedMetersPerSecond,
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
            memory.Kind,
            opponent.Id,
            path[index].DistanceMeters,
            memory.TargetSpeedMetersPerSecond,
            memory.ConflictTimeSeconds,
            CurrentClearance(in ego, in opponent, in projection)
        );
        RecordConstraint(in constraint, ref lastConstraint);
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
        TrafficMotionPlan? opponentPlan,
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
            arrivalTimes[0],
            opponentPlan
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
                arrivalTimes[rangeEnd],
                opponentPlan
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
                            opponentPlan,
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
        TrafficMotionPlan? opponentPlan,
        int index,
        out PredictedConflict conflict
    )
    {
        float time = arrivalTimes[index];
        VehiclePathPredictionPoint egoPoint = path[index];
        PredictedTrafficPose opponentPose = PredictOpponent(
            track,
            in opponent,
            time,
            opponentPlan
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
                : 0f,
            opponentPose.UsesPublishedMotion
        );
        return true;
    }

    private static bool TryFindClearTime(
        VehicleSpeedPlanningConfig config,
        TrackData track,
        in VehiclePathPredictionPoint gate,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        float conflictTimeSeconds,
        TrafficMotionPlan? opponentPlan,
        out float clearTimeSeconds
    )
    {
        if (opponentPlan is null &&
            opponent.SpeedMetersPerSecond <
            MovingSpeedThresholdMetersPerSecond &&
            MathF.Abs(opponent.LongitudinalAccelMetersPerSecondSquared) < 0.1f)
        {
            clearTimeSeconds = 0f;
            return false;
        }

        float lastSearchTime = config.TrafficPredictionHorizonSeconds;
        if (opponentPlan is not null)
        {
            lastSearchTime = MathF.Min(
                lastSearchTime,
                opponentPlan.EndTimeSeconds
            );
        }
        if (conflictTimeSeconds >= lastSearchTime)
        {
            clearTimeSeconds = 0f;
            return false;
        }

        float occupiedTime = conflictTimeSeconds;
        while (occupiedTime < lastSearchTime)
        {
            float candidateTime = MathF.Min(
                occupiedTime + ClearanceSearchStepSeconds,
                lastSearchTime
            );
            if (!BodiesOverlapAtGate(
                    config,
                    track,
                    in gate,
                    in ego,
                    in opponent,
                    candidateTime,
                    opponentPlan
                ))
            {
                float lower = occupiedTime;
                float upper = candidateTime;
                for (int i = 0; i < ClearanceRefinementIterations; i++)
                {
                    float middle = (lower + upper) * 0.5f;
                    if (BodiesOverlapAtGate(
                            config,
                            track,
                            in gate,
                            in ego,
                            in opponent,
                            middle,
                            opponentPlan
                        ))
                    {
                        lower = middle;
                    }
                    else
                    {
                        upper = middle;
                    }
                }
                clearTimeSeconds = upper;
                return true;
            }

            if (candidateTime <= occupiedTime)
                break;
            occupiedTime = candidateTime;
        }

        clearTimeSeconds = 0f;
        return false;
    }

    private static bool BodiesOverlapAtGate(
        VehicleSpeedPlanningConfig config,
        TrackData track,
        in VehiclePathPredictionPoint gate,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        float timeSeconds,
        TrafficMotionPlan? opponentPlan
    )
    {
        PredictedTrafficPose opponentPose = PredictOpponent(
            track,
            in opponent,
            timeSeconds,
            opponentPlan
        );
        CarBodyGeometry egoBody = CarBodyGeometry.FromPose(
            gate.Position,
            gate.VelocityHeading,
            ego.LengthMeters +
            2f * config.TrafficLongitudinalSafetyMarginMeters,
            ego.WidthMeters +
            2f * config.TrafficLateralSafetyMarginMeters
        );
        CarBodyGeometry opponentBody = CarBodyGeometry.FromPose(
            opponentPose.Position,
            opponentPose.HeadingRadians,
            opponent.LengthMeters +
            2f * LongitudinalUncertaintyGrowthMetersPerSecond * timeSeconds,
            opponent.WidthMeters +
            2f * LateralUncertaintyGrowthMetersPerSecond * timeSeconds
        );
        return egoBody.Overlaps(in opponentBody);
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
        float timeSeconds,
        TrafficMotionPlan? opponentPlan
    )
    {
        if (opponentPlan is not null &&
            TryPredictFromPublishedPlan(
                in opponent,
                opponentPlan,
                timeSeconds,
                out PredictedTrafficPose plannedPose
            ))
        {
            return plannedPose;
        }

        if (timeSeconds <= 0f)
        {
            return new PredictedTrafficPose(
                opponent.Position,
                opponent.HeadingRadians,
                opponent.Velocity,
                false
            );
        }

        TrackSample currentSample = track.Sample(opponent.TrackS);
        Vector2 velocity = opponent.Velocity;
        float signedLongitudinalSpeed = Vector2.Dot(
            velocity,
            currentSample.Tangent
        );
        float centerlineLateralSpeed = Vector2.Dot(
            velocity,
            currentSample.Normal
        );
        float referenceOffsetDerivative = (
            track.Sample(
                opponent.TrackS + ReferenceOffsetDerivativeProbeMeters
            ).RefOffset -
            track.Sample(
                opponent.TrackS - ReferenceOffsetDerivativeProbeMeters
            ).RefOffset
        ) / (2f * ReferenceOffsetDerivativeProbeMeters);
        float relativeLateralSpeed = centerlineLateralSpeed -
                                     referenceOffsetDerivative *
                                     signedLongitudinalSpeed;
        float currentOffsetFromReference = opponent.TrackD -
                                           currentSample.RefOffset;
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
        bool convergingToReference =
            currentOffsetFromReference * relativeLateralSpeed < 0f;
        float lateralTime = convergingToReference
            ? timeSeconds
            : MathF.Min(
                timeSeconds,
                OpponentLateralVelocityHoldSeconds
            );
        float predictedOffsetFromReference = currentOffsetFromReference +
                                             relativeLateralSpeed *
                                             lateralTime;
        if (convergingToReference &&
            currentOffsetFromReference * predictedOffsetFromReference < 0f)
        {
            predictedOffsetFromReference = 0f;
        }
        float predictedD = predictedSample.RefOffset +
                           predictedOffsetFromReference;
        Vector2 position = predictedSample.Center +
                           predictedSample.Normal * predictedD;

        float reverseHeadingOffset = direction < 0f ? MathF.PI : 0f;
        float predictedReferenceHeading = MathHelper.NormalizeAngle(
            predictedSample.RefHeading + reverseHeadingOffset
        );
        float currentReferenceHeading = MathHelper.NormalizeAngle(
            currentSample.RefHeading + reverseHeadingOffset
        );
        float heading;
        if (opponent.SpeedMetersPerSecond < MovingSpeedThresholdMetersPerSecond)
        {
            heading = opponent.HeadingRadians;
        }
        else
        {
            float headingOffset = MathHelper.NormalizeAngle(
                opponent.HeadingRadians - currentReferenceHeading
            );
            float headingWeight = Math.Clamp(
                1f - timeSeconds / OpponentHeadingConvergenceSeconds,
                0f,
                1f
            );
            heading = MathHelper.NormalizeAngle(
                predictedReferenceHeading + headingOffset * headingWeight
            );
        }

        bool reachedReference = convergingToReference &&
                                predictedOffsetFromReference == 0f;
        float retainedLateralSpeed = !reachedReference &&
                                     (convergingToReference ||
                                      timeSeconds <
                                      OpponentLateralVelocityHoldSeconds)
            ? relativeLateralSpeed
            : 0f;
        Vector2 predictedReferenceForward = new(
            MathF.Cos(predictedReferenceHeading),
            MathF.Sin(predictedReferenceHeading)
        );
        Vector2 predictedVelocity =
            predictedReferenceForward * predictedLongitudinalSpeed +
            predictedSample.Normal * retainedLateralSpeed;
        return new PredictedTrafficPose(
            position,
            heading,
            predictedVelocity,
            false
        );
    }

    private static bool TryPredictFromPublishedPlan(
        in RaceCarSnapshot opponent,
        TrafficMotionPlan plan,
        float timeSeconds,
        out PredictedTrafficPose pose
    )
    {
        if (!plan.TrySample(
                MathF.Max(0f, timeSeconds),
                out TrafficMotionPlanPoint planned
            ) ||
            !plan.TrySample(0f, out TrafficMotionPlanPoint plannedStart))
        {
            pose = default;
            return false;
        }

        float correctionWeight = Math.Clamp(
            1f - timeSeconds / OpponentHeadingConvergenceSeconds,
            0f,
            1f
        );
        Vector2 position = planned.Position +
                           (opponent.Position - plannedStart.Position) *
                           correctionWeight;
        float heading = MathHelper.NormalizeAngle(
            planned.HeadingRadians +
            MathHelper.NormalizeAngle(
                opponent.HeadingRadians - plannedStart.HeadingRadians
            ) * correctionWeight
        );
        Vector2 plannedVelocity = new(
            MathF.Cos(planned.HeadingRadians) * planned.SpeedMetersPerSecond,
            MathF.Sin(planned.HeadingRadians) * planned.SpeedMetersPerSecond
        );
        Vector2 velocity = Vector2.Lerp(
            plannedVelocity,
            opponent.Velocity,
            correctionWeight
        );
        pose = new PredictedTrafficPose(
            position,
            heading,
            velocity,
            true
        );
        return true;
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

    private static bool ApplyArrivalConstraint(
        VehicleSpeedPlanningConfig config,
        int gateIndex,
        float earliestArrivalTimeSeconds,
        float[] segmentLengths,
        float[] speeds,
        float[] speedLimits,
        out float targetSpeedMetersPerSecond
    )
    {
        targetSpeedMetersPerSecond = speeds[Math.Max(0, gateIndex)];
        if (gateIndex <= 0 ||
            !float.IsFinite(earliestArrivalTimeSeconds) ||
            earliestArrivalTimeSeconds <= 0f)
        {
            return false;
        }

        float baselineArrival = PrefixArrivalTime(
            gateIndex,
            segmentLengths,
            speeds,
            speedScale: 1f
        );
        if (baselineArrival >= earliestArrivalTimeSeconds - ConstraintEpsilon)
            return false;

        float slowerScale = 0f;
        float fasterScale = 1f;
        for (int iteration = 0;
             iteration < config.TrafficArrivalSolveIterations;
             iteration++)
        {
            float candidateScale = (slowerScale + fasterScale) * 0.5f;
            float candidateArrival = PrefixArrivalTime(
                gateIndex,
                segmentLengths,
                speeds,
                candidateScale
            );
            if (candidateArrival >= earliestArrivalTimeSeconds)
                slowerScale = candidateScale;
            else
                fasterScale = candidateScale;
        }

        bool changed = false;
        for (int i = 1; i <= gateIndex; i++)
        {
            float scaledSpeed = MathF.Max(0f, speeds[i] * slowerScale);
            if (scaledSpeed >= speedLimits[i] - ConstraintEpsilon)
                continue;
            speedLimits[i] = scaledSpeed;
            changed = true;
        }
        targetSpeedMetersPerSecond = MathF.Max(
            0f,
            speeds[gateIndex] * slowerScale
        );
        return changed;
    }

    private static float PrefixArrivalTime(
        int gateIndex,
        float[] segmentLengths,
        float[] speeds,
        float speedScale
    )
    {
        float time = 0f;
        float previousSpeed = MathF.Max(0f, speeds[0]);
        for (int i = 1; i <= gateIndex; i++)
        {
            float currentSpeed = MathF.Max(0f, speeds[i] * speedScale);
            float speedSum = previousSpeed + currentSpeed;
            if (speedSum <= 0.05f)
                return float.PositiveInfinity;
            time += 2f * segmentLengths[i - 1] / speedSum;
            previousSpeed = currentSpeed;
        }
        return time;
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

    private static bool RecordConstraint(
        in TrafficSpeedConstraint constraint,
        ref TrafficSpeedConstraint lastConstraint
    )
    {
        TrafficSpeedConstraint current = lastConstraint;
        if (current.Active && !IsHigherPriority(
                in constraint,
                in current
            ))
            return false;

        lastConstraint = constraint;
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

    private static float SafeStoppingGap(
        VehicleSpeedPlanningConfig config,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot opponent,
        float egoSpeedMetersPerSecond,
        float opponentSpeedMetersPerSecond
    )
    {
        float egoBrakeDeceleration = MathF.Max(
            ego.MaximumBrakeDecelerationMetersPerSecondSquared *
            config.BrakeDecelerationUsage,
            0.1f
        );
        float opponentBrakeDeceleration = MathF.Max(
            opponent.MaximumBrakeDecelerationMetersPerSecondSquared,
            0.1f
        );
        float egoStoppingDistance =
            egoSpeedMetersPerSecond * egoSpeedMetersPerSecond /
            (2f * egoBrakeDeceleration);
        float opponentStoppingDistance =
            opponentSpeedMetersPerSecond * opponentSpeedMetersPerSecond /
            (2f * opponentBrakeDeceleration);
        return MathF.Max(
            config.TrafficMinimumGapMeters,
            config.TrafficMinimumGapMeters +
            egoStoppingDistance - opponentStoppingDistance
        );
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
        Vector2 Velocity,
        bool UsesPublishedMotion
    );

    private readonly record struct PredictedConflict(
        TrafficSpeedConstraintKind Kind,
        float PathDistanceMeters,
        float TimeSeconds,
        float EgoSpeedMetersPerSecond,
        float TargetSpeedMetersPerSecond,
        bool UsesPublishedMotion
    );
}
