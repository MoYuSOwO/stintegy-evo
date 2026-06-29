using System;
using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public sealed class ForwardBackwardVelocityProfileSolver : IVelocityProfileSolver
{
    private const float MinDistanceStep = 0.001f;
    private const float CurvatureEpsilon = 1e-5f;

    public VelocityProfile Solve(VelocityProfileRequest request)
    {
        if (request.Path.Count == 0)
            return VelocityProfile.Empty;

        int count = request.Path.Count;
        float maxSpeed = Math.Max(0f, request.MaxSpeed);
        float maxAccel = Math.Max(0f, request.MaxLongitudinalAccel);
        float maxDecel = Math.Max(0f, request.MaxLongitudinalDecel);
        float maxLateralAccel = Math.Max(0f, request.MaxLateralAccel);
        float safetyFactor = Mathf.Clamp(request.SafetyFactor, 0.05f, 1f);
        float[] speeds = new float[count];

        for (int i = 0; i < count; i++)
            speeds[i] = CurvatureLimitedSpeed(request.Path[i].Curvature, maxSpeed, maxLateralAccel, safetyFactor);

        speeds[0] = Math.Min(speeds[0], Math.Max(0f, request.StartSpeed));

        for (int i = 1; i < count; i++)
        {
            float ds = SegmentLength(request, i - 1, i);
            float reachableSpeed = Mathf.Sqrt(Math.Max(0f, speeds[i - 1] * speeds[i - 1] + 2f * maxAccel * ds));
            speeds[i] = Math.Min(speeds[i], reachableSpeed);
        }

        if (request.EnforceEndSpeed)
            speeds[count - 1] = Math.Min(speeds[count - 1], Math.Max(0f, request.EndSpeed));

        for (int i = count - 2; i >= 0; i--)
        {
            float ds = SegmentLength(request, i, i + 1);
            float brakeReachableSpeed = Mathf.Sqrt(Math.Max(0f, speeds[i + 1] * speeds[i + 1] + 2f * maxDecel * ds));
            speeds[i] = Math.Min(speeds[i], brakeReachableSpeed);
        }

        VelocityProfilePoint[] points = new VelocityProfilePoint[count];
        for (int i = 0; i < count; i++)
        {
            float acceleration = 0f;
            if (i < count - 1)
            {
                float ds = SegmentLength(request, i, i + 1);
                acceleration = (speeds[i + 1] * speeds[i + 1] - speeds[i] * speeds[i]) / (2f * ds);
            }

            if (!float.IsFinite(acceleration))
                acceleration = 0f;

            points[i] = new VelocityProfilePoint(request.Path[i].Distance, speeds[i], acceleration);
        }

        return new VelocityProfile(points);
    }

    private static float CurvatureLimitedSpeed(
        float curvature,
        float maxSpeed,
        float maxLateralAccel,
        float safetyFactor
    )
    {
        if (maxSpeed <= 0f)
            return 0f;

        float absCurvature = Math.Abs(curvature);
        if (!float.IsFinite(absCurvature) || absCurvature < CurvatureEpsilon || maxLateralAccel <= 0f)
            return maxSpeed;

        float limitedSpeed = Mathf.Sqrt(maxLateralAccel / absCurvature) * safetyFactor;
        if (!float.IsFinite(limitedSpeed))
            return maxSpeed;

        return Mathf.Clamp(limitedSpeed, 0f, maxSpeed);
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
