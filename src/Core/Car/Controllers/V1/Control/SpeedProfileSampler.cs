using System;
using Godot;
using StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;

namespace StintegyEVO.Core.Car.Controllers.V1.Control;

public static class SpeedProfileSampler
{
    public readonly record struct Projection(
        SpeedProfilePoint Point,
        int Index,
        float DistanceSquared
    );

    public static int FindNearestIndex(SpeedProfile profile, Vector2 position)
    {
        ArgumentNullException.ThrowIfNull(profile);
        if (profile.Count == 0)
            return -1;

        int bestIndex = 0;
        float bestDistanceSquared = float.PositiveInfinity;
        for (int i = 0; i < profile.Count; i++)
        {
            float distanceSquared = position.DistanceSquaredTo(profile[i].Position);
            if (distanceSquared >= bestDistanceSquared)
                continue;

            bestDistanceSquared = distanceSquared;
            bestIndex = i;
        }

        return bestIndex;
    }

    public static bool TryCalculateTrackProgress(SpeedProfile profile, int trackIndex, out float progress)
    {
        ArgumentNullException.ThrowIfNull(profile);
        progress = 0.0f;
        if (!profile.HasTrackProgress)
            return false;

        int wrappedTrackIndex = WrapIndex(trackIndex, profile.TrackLength);
        int wrappedAnchorIndex = WrapIndex(profile.AnchorTrackIndex, profile.TrackLength);
        progress = wrappedTrackIndex >= wrappedAnchorIndex
            ? wrappedTrackIndex - wrappedAnchorIndex
            : profile.TrackLength - wrappedAnchorIndex + wrappedTrackIndex;
        return true;
    }

    public static int FindFirstIndexAtOrAfterTrackProgress(SpeedProfile profile, float trackProgress)
    {
        ArgumentNullException.ThrowIfNull(profile);
        if (profile.Count == 0)
            return -1;

        for (int i = 0; i < profile.Count; i++)
        {
            if (profile[i].TrackProgress >= trackProgress)
                return i;
        }

        return profile.Count - 1;
    }

    public static SpeedProfilePoint SampleAtTrackProgress(SpeedProfile profile, float trackProgress)
    {
        ArgumentNullException.ThrowIfNull(profile);
        if (profile.Count == 0)
            return default;
        if (profile.Count == 1 || !profile.HasTrackProgress)
            return profile[0];

        int index = FindFirstIndexAtOrAfterTrackProgress(profile, trackProgress);
        if (index <= 0)
            return profile[0];
        if (index >= profile.Count)
            return profile[^1];

        SpeedProfilePoint a = profile[index - 1];
        SpeedProfilePoint b = profile[index];
        float span = b.TrackProgress - a.TrackProgress;
        float t = span > 1e-4f ? Mathf.Clamp((trackProgress - a.TrackProgress) / span, 0.0f, 1.0f) : 0.0f;
        return Lerp(a, b, t);
    }

    public static Projection ProjectPosition(SpeedProfile profile, Vector2 position)
    {
        ArgumentNullException.ThrowIfNull(profile);
        if (profile.Count == 0)
            return new Projection(default, -1, float.PositiveInfinity);
        if (profile.Count == 1)
            return new Projection(profile[0], 0, position.DistanceSquaredTo(profile[0].Position));

        int bestSegment = 0;
        float bestT = 0.0f;
        float bestDistanceSquared = float.PositiveInfinity;
        for (int i = 0; i < profile.Count - 1; i++)
        {
            Vector2 a = profile[i].Position;
            Vector2 b = profile[i + 1].Position;
            Vector2 ab = b - a;
            float segmentLengthSquared = ab.LengthSquared();
            float t = segmentLengthSquared > 1e-6f
                ? Mathf.Clamp((position - a).Dot(ab) / segmentLengthSquared, 0.0f, 1.0f)
                : 0.0f;
            Vector2 projected = a.Lerp(b, t);
            float distanceSquared = position.DistanceSquaredTo(projected);
            if (distanceSquared >= bestDistanceSquared)
                continue;

            bestDistanceSquared = distanceSquared;
            bestSegment = i;
            bestT = t;
        }

        SpeedProfilePoint point = Lerp(profile[bestSegment], profile[bestSegment + 1], bestT);
        int index = FindFirstIndexAtOrAfterDistance(profile, point.Distance);
        return new Projection(point, index, bestDistanceSquared);
    }

    public static SpeedProfilePoint SampleAtDistance(SpeedProfile profile, float distance)
    {
        ArgumentNullException.ThrowIfNull(profile);
        if (profile.Count == 0)
            return default;
        if (profile.Count == 1 || distance <= profile[0].Distance)
            return profile[0];

        for (int i = 1; i < profile.Count; i++)
        {
            if (profile[i].Distance < distance)
                continue;

            SpeedProfilePoint a = profile[i - 1];
            SpeedProfilePoint b = profile[i];
            float span = b.Distance - a.Distance;
            float t = span > 1e-4f ? Mathf.Clamp((distance - a.Distance) / span, 0.0f, 1.0f) : 0.0f;
            return Lerp(a, b, t);
        }

        return profile[^1];
    }

    public static SpeedProfilePoint FindMinimumSpeedPointInDistanceRange(
        SpeedProfile profile,
        float startDistance,
        float endDistance
    )
    {
        ArgumentNullException.ThrowIfNull(profile);
        if (profile.Count == 0)
            return default;

        float start = MathF.Min(startDistance, endDistance);
        float end = MathF.Max(startDistance, endDistance);
        SpeedProfilePoint best = SampleAtDistance(profile, start);
        SpeedProfilePoint endPoint = SampleAtDistance(profile, end);
        if (endPoint.Speed < best.Speed)
            best = endPoint;

        for (int i = 0; i < profile.Count; i++)
        {
            SpeedProfilePoint point = profile[i];
            if (point.Distance < start || point.Distance > end)
                continue;
            if (point.Speed < best.Speed)
                best = point;
        }

        return best;
    }

    public static int FindFirstIndexAtOrAfterDistance(SpeedProfile profile, float distance)
    {
        ArgumentNullException.ThrowIfNull(profile);
        if (profile.Count == 0)
            return -1;

        for (int i = 0; i < profile.Count; i++)
        {
            if (profile[i].Distance >= distance)
                return i;
        }

        return profile.Count - 1;
    }

    private static SpeedProfilePoint Lerp(SpeedProfilePoint a, SpeedProfilePoint b, float t)
    {
        return new SpeedProfilePoint(
            SampleIndex: t < 0.5f ? a.SampleIndex : b.SampleIndex,
            Position: a.Position.Lerp(b.Position, t),
            Heading: LerpAngle(a.Heading, b.Heading, t),
            Curvature: Mathf.Lerp(a.Curvature, b.Curvature, t),
            Distance: Mathf.Lerp(a.Distance, b.Distance, t),
            Speed: Mathf.Lerp(a.Speed, b.Speed, t),
            AccelerationToNext: Mathf.Lerp(a.AccelerationToNext, b.AccelerationToNext, t),
            TimeFromStart: Mathf.Lerp(a.TimeFromStart, b.TimeFromStart, t),
            MaxSpeed: Mathf.Lerp(a.MaxSpeed, b.MaxSpeed, t),
            MaxAcceleration: Mathf.Lerp(a.MaxAcceleration, b.MaxAcceleration, t),
            MaxDeceleration: Mathf.Lerp(a.MaxDeceleration, b.MaxDeceleration, t),
            LateralAcceleration: Mathf.Lerp(a.LateralAcceleration, b.LateralAcceleration, t),
            TrackProgress: Mathf.Lerp(a.TrackProgress, b.TrackProgress, t)
        );
    }

    private static float LerpAngle(float a, float b, float t)
    {
        return a + WrapAngle(b - a) * Mathf.Clamp(t, 0.0f, 1.0f);
    }

    public static float WrapAngle(float angle)
    {
        while (angle <= -Mathf.Pi)
            angle += Mathf.Tau;
        while (angle > Mathf.Pi)
            angle -= Mathf.Tau;
        return angle;
    }

    private static int WrapIndex(int index, int length)
    {
        int wrapped = index % length;
        return wrapped < 0 ? wrapped + length : wrapped;
    }
}
