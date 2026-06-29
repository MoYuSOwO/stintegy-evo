using System;
using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public sealed class FbgaVelocityProfileSolver : IVelocityProfileSolver
{
    private const float MinDistanceStep = 0.001f;
    private const float MinSpeedSquared = 0.000001f;
    private const float FeasibilityTolerance = 0.0001f;
    private const int RootIterations = 24;

    public VelocityProfile Solve(VelocityProfileRequest request)
    {
        if (request.Path.Count == 0)
            return VelocityProfile.Empty;

        int count = request.Path.Count;
        float maxSpeed = Math.Max(0f, request.MaxSpeed);
        IAccelerationEnvelope envelope = request.AccelerationEnvelope ??
                                         new EllipticAccelerationEnvelope(
                                             request.MaxLongitudinalAccel,
                                             request.MaxLongitudinalDecel,
                                             request.MaxLateralAccel,
                                             request.SafetyFactor
                                         );
        float[] saturatedSpeeds = CalculateSaturatedSpeeds(request, envelope, maxSpeed);
        float[] speeds = new float[count];
        float[] accelerations = new float[Math.Max(count - 1, 1)];
        bool[] hasForwardAcceleration = new bool[Math.Max(count - 1, 1)];

        ForwardPass(request, envelope, saturatedSpeeds, speeds, accelerations, hasForwardAcceleration);

        if (request.EnforceEndSpeed)
            speeds[count - 1] = Math.Min(speeds[count - 1], Math.Max(0f, request.EndSpeed));

        BackwardPass(request, envelope, saturatedSpeeds, speeds, accelerations, hasForwardAcceleration);
        ClampToSaturatedSpeeds(speeds, saturatedSpeeds);
        ApplyInitialSpeedReachabilityPass(request, envelope, saturatedSpeeds, speeds, accelerations, hasForwardAcceleration);

        VelocityProfilePoint[] points = new VelocityProfilePoint[count];
        for (int i = 0; i < count; i++)
        {
            float acceleration = i < count - 1
                ? accelerations[i]
                : 0f;
            if (!float.IsFinite(acceleration))
                acceleration = 0f;

            points[i] = new VelocityProfilePoint(request.Path[i].Distance, speeds[i], acceleration);
        }

        return new VelocityProfile(points);
    }

    private static float[] CalculateSaturatedSpeeds(
        VelocityProfileRequest request,
        IAccelerationEnvelope envelope,
        float maxSpeed
    )
    {
        int count = request.Path.Count;
        float[] saturated = new float[count];
        for (int i = 0; i < count; i++)
        {
            float curvature = request.Path[i].Curvature;
            float speed = SolveSaturatedSpeed(curvature, maxSpeed, envelope);
            saturated[i] = speed;
        }

        return saturated;
    }

    private static float SolveSaturatedSpeed(float curvature, float maxSpeed, IAccelerationEnvelope envelope)
    {
        if (maxSpeed <= 0f)
            return 0f;

        if (!float.IsFinite(curvature) || Math.Abs(curvature) < 1e-6f)
            return maxSpeed;

        float SignedLimitError(float speed)
        {
            envelope.GetLateralBounds(speed, out float minAy, out float maxAy);
            float ay = curvature * speed * speed;
            return curvature >= 0f ? ay - maxAy : ay - minAy;
        }

        if (SignedLimitError(maxSpeed) <= 0f)
            return maxSpeed;

        return SolveBoundaryFromFeasibleLow(SignedLimitError, 0f, maxSpeed) ?? maxSpeed;
    }

    private static void ForwardPass(
        VelocityProfileRequest request,
        IAccelerationEnvelope envelope,
        float[] saturatedSpeeds,
        float[] speeds,
        float[] accelerations,
        bool[] hasForwardAcceleration
    )
    {
        speeds[0] = Math.Min(Math.Max(0f, request.StartSpeed), saturatedSpeeds[0]);
        for (int i = 0; i < request.Path.Count - 1; i++)
        {
            float v0 = speeds[i];
            float ds = SegmentLength(request, i, i + 1);
            float k0 = request.Path[i].Curvature;
            float k1 = request.Path[i + 1].Curvature;
            float ay0 = k0 * v0 * v0;
            float clippedAy0 = ClipLateralAccel(envelope, ay0, v0);
            envelope.GetLongitudinalBounds(clippedAy0, v0, out float axMin, out float axMax);

            float SignedDistanceAtEnd(float acceleration)
            {
                float v1Squared = v0 * v0 + 2f * ds * acceleration;
                if (v1Squared < MinSpeedSquared)
                    return float.PositiveInfinity;

                float v1 = Mathf.Sqrt(v1Squared);
                float ay1 = k1 * v1Squared;
                return SignedDistance(envelope, acceleration, ay1, v1);
            }

            if (SignedDistanceAtEnd(axMax) <= FeasibilityTolerance)
            {
                accelerations[i] = axMax;
                hasForwardAcceleration[i] = true;
                speeds[i + 1] = Math.Min(IntegrateSpeed(v0, axMax, ds), saturatedSpeeds[i + 1]);
                continue;
            }

            float? solvedAcceleration = SolveBoundaryFromFeasibleLow(SignedDistanceAtEnd, axMin, axMax);
            if (solvedAcceleration.HasValue)
            {
                accelerations[i] = solvedAcceleration.Value;
                hasForwardAcceleration[i] = true;
                speeds[i + 1] = Math.Min(IntegrateSpeed(v0, solvedAcceleration.Value, ds), saturatedSpeeds[i + 1]);
            }
            else
            {
                accelerations[i] = axMin;
                hasForwardAcceleration[i] = true;
                speeds[i + 1] = Math.Min(IntegrateSpeed(v0, axMin, ds), saturatedSpeeds[i + 1]);
            }
        }
    }

    private static void BackwardPass(
        VelocityProfileRequest request,
        IAccelerationEnvelope envelope,
        float[] saturatedSpeeds,
        float[] speeds,
        float[] accelerations,
        bool[] hasForwardAcceleration
    )
    {
        for (int i = request.Path.Count - 1; i >= 1; i--)
        {
            int segment = i - 1;
            float v0 = speeds[segment];
            float v1 = speeds[i];
            float ds = SegmentLength(request, segment, i);
            float acceleration = accelerations[segment];
            if (IsForwardSegmentStillValid(v0, v1, acceleration, ds, hasForwardAcceleration[segment]))
                continue;

            float k0 = request.Path[segment].Curvature;
            float k1 = request.Path[i].Curvature;
            float ay1 = k1 * v1 * v1;
            float clippedAy1 = ClipLateralAccel(envelope, ay1, v1);
            envelope.GetLongitudinalBounds(clippedAy1, v1, out float axMin, out float axMax);

            float averageAcceleration = SegmentAcceleration(request, v0, v1, segment);
            float reachMax = ReverseIntegrateSpeed(v1, axMin, ds);
            float reachMin = ReverseIntegrateSpeed(v1, axMax, ds);
            bool validStartSpeed = v0 <= reachMax + FeasibilityTolerance &&
                                   v0 >= reachMin - FeasibilityTolerance;
            bool validAverageAcceleration = averageAcceleration <= axMax + FeasibilityTolerance &&
                                            averageAcceleration >= axMin - FeasibilityTolerance;

            float SignedDistanceAtStart(float candidateAcceleration)
            {
                float v0Squared = v1 * v1 - 2f * ds * candidateAcceleration;
                if (v0Squared < MinSpeedSquared)
                    return float.PositiveInfinity;

                float candidateV0 = Mathf.Sqrt(v0Squared);
                float candidateAy0 = k0 * v0Squared;
                return SignedDistance(envelope, candidateAcceleration, candidateAy0, candidateV0);
            }

            if (validStartSpeed &&
                validAverageAcceleration &&
                SignedDistanceAtStart(averageAcceleration) <= FeasibilityTolerance)
            {
                accelerations[segment] = averageAcceleration;
                hasForwardAcceleration[segment] = true;
                continue;
            }

            float chosenAcceleration;
            if (SignedDistanceAtStart(axMin) <= FeasibilityTolerance)
            {
                chosenAcceleration = axMin;
            }
            else
            {
                chosenAcceleration = SolveFirstFeasibleBoundary(SignedDistanceAtStart, axMin, axMax) ??
                                     Math.Min(Math.Max(averageAcceleration, axMin), axMax);
            }

            float updatedStartSpeed = Math.Min(ReverseIntegrateSpeed(v1, chosenAcceleration, ds), saturatedSpeeds[segment]);
            speeds[segment] = updatedStartSpeed;
            accelerations[segment] = SegmentAcceleration(request, updatedStartSpeed, v1, segment);
            hasForwardAcceleration[segment] = true;
        }
    }

    private static void ClampToSaturatedSpeeds(float[] speeds, float[] saturatedSpeeds)
    {
        int count = Math.Min(speeds.Length, saturatedSpeeds.Length);
        for (int i = 0; i < count; i++)
            speeds[i] = Math.Min(Math.Max(0f, speeds[i]), saturatedSpeeds[i]);
    }

    private static void ApplyInitialSpeedReachabilityPass(
        VelocityProfileRequest request,
        IAccelerationEnvelope envelope,
        float[] saturatedSpeeds,
        float[] speeds,
        float[] accelerations,
        bool[] hasForwardAcceleration
    )
    {
        float[] speedCaps = new float[speeds.Length];
        for (int i = 0; i < speeds.Length; i++)
            speedCaps[i] = Math.Min(speeds[i], saturatedSpeeds[i]);

        if (request.EnforceEndSpeed && speedCaps.Length > 0)
            speedCaps[^1] = Math.Min(speedCaps[^1], Math.Max(0f, request.EndSpeed));

        ForwardPass(request, envelope, speedCaps, speeds, accelerations, hasForwardAcceleration);
        ClampToSaturatedSpeeds(speeds, speedCaps);
    }

    private static bool IsForwardSegmentStillValid(
        float v0,
        float v1,
        float acceleration,
        float distance,
        bool hasAcceleration
    )
    {
        if (!hasAcceleration)
            return false;

        float predicted = IntegrateSpeed(v0, acceleration, distance);
        return Math.Abs(predicted - v1) <= 0.001f;
    }

    private static float ClipLateralAccel(IAccelerationEnvelope envelope, float lateralAccel, float speed)
    {
        envelope.GetLateralBounds(speed, out float minAy, out float maxAy);
        return Mathf.Clamp(lateralAccel, minAy, maxAy);
    }

    private static float SignedDistance(
        IAccelerationEnvelope envelope,
        float longitudinalAccel,
        float lateralAccel,
        float speed
    )
    {
        envelope.GetLateralBounds(speed, out float minAy, out float maxAy);
        float clippedAy = Mathf.Clamp(lateralAccel, minAy, maxAy);
        envelope.GetLongitudinalBounds(clippedAy, speed, out float minAx, out float maxAx);
        float x = Phi(longitudinalAccel, minAx, maxAx);
        float y = Phi(lateralAccel, minAy, maxAy);
        return Math.Max(Math.Max(x - 1f, -1f - x), Math.Max(y - 1f, -1f - y));
    }

    private static float Phi(float value, float min, float max)
    {
        if (max - min <= 1e-5f)
            return value <= max + FeasibilityTolerance && value >= min - FeasibilityTolerance ? 0f : 2f;

        return 2f * (value - min) / (max - min) - 1f;
    }

    private static float? SolveBoundaryFromFeasibleLow(Func<float, float> signedDistance, float low, float high)
    {
        float lowValue = signedDistance(low);
        float highValue = signedDistance(high);
        if (lowValue > FeasibilityTolerance)
            return null;
        if (highValue <= FeasibilityTolerance)
            return high;

        float feasible = low;
        float infeasible = high;
        for (int i = 0; i < RootIterations; i++)
        {
            float mid = (feasible + infeasible) * 0.5f;
            if (signedDistance(mid) <= FeasibilityTolerance)
                feasible = mid;
            else
                infeasible = mid;
        }

        return feasible;
    }

    private static float? SolveFirstFeasibleBoundary(Func<float, float> signedDistance, float low, float high)
    {
        float lowValue = signedDistance(low);
        if (lowValue <= FeasibilityTolerance)
            return low;

        float highValue = signedDistance(high);
        if (highValue > FeasibilityTolerance)
            return null;

        float infeasible = low;
        float feasible = high;
        for (int i = 0; i < RootIterations; i++)
        {
            float mid = (infeasible + feasible) * 0.5f;
            if (signedDistance(mid) <= FeasibilityTolerance)
                feasible = mid;
            else
                infeasible = mid;
        }

        return feasible;
    }

    private static float IntegrateSpeed(float startSpeed, float acceleration, float distance)
    {
        return Mathf.Sqrt(Math.Max(0f, startSpeed * startSpeed + 2f * acceleration * distance));
    }

    private static float ReverseIntegrateSpeed(float endSpeed, float acceleration, float distance)
    {
        return Mathf.Sqrt(Math.Max(0f, endSpeed * endSpeed - 2f * acceleration * distance));
    }

    private static float SegmentAcceleration(VelocityProfileRequest request, float startSpeed, float endSpeed, int segmentIndex)
    {
        float ds = SegmentLength(request, segmentIndex, segmentIndex + 1);
        return (endSpeed * endSpeed - startSpeed * startSpeed) / (2f * ds);
    }

    private static float SegmentLength(VelocityProfileRequest request, int fromIndex, int toIndex)
    {
        float distanceDelta = request.Path[toIndex].Distance - request.Path[fromIndex].Distance;
        if (float.IsFinite(distanceDelta) && distanceDelta > MinDistanceStep)
            return distanceDelta;

        float spatialDistance = request.Path[fromIndex].Position.DistanceTo(request.Path[toIndex].Position);
        return Math.Max(spatialDistance, MinDistanceStep);
    }
}
