using System;
using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Track.RefLines;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Track;

public class TrackBuilder
{
    private Func<TrackSurfaceContext, TrackSurface>? _surfaceAt;

    private static readonly IRefLineSolver DefaultRefLineSolver =
        new MinimumCurvatureRefLineSolver();

    private readonly struct BuilderNode(Vector2 center, float width, float leftBuffer, float rightBuffer)
    {
        public readonly Vector2 Center = center;
        public readonly float Width = width;
        public readonly float LeftBuffer = leftBuffer;
        public readonly float RightBuffer = rightBuffer;
    }

    private readonly record struct SmoothedCenterlinePoint(
        Vector2 Center,
        float Width
    );

    private readonly List<BuilderNode> nodes = [];

    private readonly Vector2 _startPos;
    private readonly float _startWidth;
    private readonly float _startLeftBuffer;
    private readonly float _startRightBuffer;
    private readonly float _startAngle;
    private readonly IRefLineSolver _refLineSolver;

    private Vector2 currentPos;
    private float currentWidth;
    private float currentLeftBuffer;
    private float currentRightBuffer;
    private float currentAngle;

    public TrackBuilder(
        Vector2 startPos,
        float startWidth,
        float startLeftBuffer = 0,
        float startRightBuffer = 0,
        float startAngleDeg = 0,
        IRefLineSolver? refLineSolver = null
    )
    {
        _startPos = startPos;
        _startWidth = startWidth;
        _startLeftBuffer = startLeftBuffer;
        _startRightBuffer = startRightBuffer;
        _startAngle = MathHelper.DegToRad(startAngleDeg);
        _refLineSolver = refLineSolver ?? DefaultRefLineSolver;

        currentPos = _startPos;
        currentWidth = _startWidth;
        currentLeftBuffer = _startLeftBuffer;
        currentRightBuffer = _startRightBuffer;
        currentAngle = _startAngle;

        nodes.Add(
            new(
                startPos,
                startWidth,
                startLeftBuffer,
                startRightBuffer
            )
        );
    }

    internal static TrackBuilder FromClosedCenterline(
        IReadOnlyList<TrackCenterlinePoint> sourcePoints,
        float targetLengthMeters,
        float leftBuffer = 0f,
        float rightBuffer = 0f,
        float controlSpacingMeters = 8f
    )
    {
        if (sourcePoints.Count < 3)
            throw new ArgumentException(
                "A closed centerline requires at least three points.",
                nameof(sourcePoints)
            );
        if (!float.IsFinite(targetLengthMeters) || targetLengthMeters <= 0f)
            throw new ArgumentOutOfRangeException(nameof(targetLengthMeters));
        if (!float.IsFinite(controlSpacingMeters) || controlSpacingMeters < 2f)
            throw new ArgumentOutOfRangeException(nameof(controlSpacingMeters));

        float sourceLength = ClosedSourceLength(sourcePoints);
        if (sourceLength <= 1e-3f)
            throw new ArgumentException(
                "The centerline must have a non-zero closed length.",
                nameof(sourcePoints)
            );

        int controlCount = Math.Max(
            8,
            (int)MathF.Round(sourceLength / controlSpacingMeters)
        );
        TrackCenterlinePoint[] sourceControls = ResampleSourcePoints(
            sourcePoints,
            controlCount,
            sourceLength
        );
        SmoothedCenterlinePoint[] centeredControls =
            CenterSourceControls(sourceControls);
        int subdivisionsPerControl = Math.Max(
            4,
            (int)MathF.Ceiling(controlSpacingMeters / 0.5f)
        );
        SmoothedCenterlinePoint[] highResolution = EvaluatePeriodicCubicBSpline(
            centeredControls,
            subdivisionsPerControl
        );
        highResolution = RotateToClosestStart(
            highResolution,
            centeredControls[0].Center
        );

        int sampleCount = Math.Max(
            3,
            (int)MathF.Round(targetLengthMeters / TrackData.StepLength)
        );
        float sampledLength = sampleCount * TrackData.StepLength;
        float splineLength = ClosedSmoothedLength(highResolution);
        float geometryScale = sampledLength / splineLength;
        Vector2 sourceOrigin = highResolution[0].Center;
        for (int i = 0; i < highResolution.Length; i++)
        {
            highResolution[i] = new SmoothedCenterlinePoint(
                (highResolution[i].Center - sourceOrigin) * geometryScale,
                highResolution[i].Width * geometryScale
            );
        }
        SmoothedCenterlinePoint[] samples = ResampleSmoothedPoints(
            highResolution,
            sampleCount,
            sampledLength
        );
        samples = EnsureSymmetricCorridorFeasible(samples, sampledLength);
        (float[] leftBuffers, float[] rightBuffers) = CalculateAdaptiveBuffers(
            samples,
            leftBuffer,
            rightBuffer
        );

        TrackBuilder builder = new(
            samples[0].Center,
            samples[0].Width,
            leftBuffers[0],
            rightBuffers[0]
        );
        for (int sampleIndex = 1; sampleIndex < sampleCount; sampleIndex++)
        {
            SmoothedCenterlinePoint sample = samples[sampleIndex];
            builder.nodes.Add(
                new BuilderNode(
                    sample.Center,
                    sample.Width,
                    leftBuffers[sampleIndex],
                    rightBuffers[sampleIndex]
                )
            );
        }

        return builder;
    }

    private static float ClosedSourceLength(
        IReadOnlyList<TrackCenterlinePoint> points
    )
    {
        float length = 0f;
        for (int i = 0; i < points.Count; i++)
        {
            length += Vector2.Distance(
                points[i].Center,
                points[(i + 1) % points.Count].Center
            );
        }
        return length;
    }

    private static TrackCenterlinePoint[] ResampleSourcePoints(
        IReadOnlyList<TrackCenterlinePoint> source,
        int sampleCount,
        float sourceLength
    )
    {
        TrackCenterlinePoint[] result = new TrackCenterlinePoint[sampleCount];
        int segmentIndex = 0;
        float segmentStartDistance = 0f;
        float segmentLength = Vector2.Distance(
            source[0].Center,
            source[1].Center
        );
        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
        {
            float targetDistance = sampleIndex * sourceLength / sampleCount;
            while (segmentStartDistance + segmentLength < targetDistance)
            {
                segmentStartDistance += segmentLength;
                segmentIndex = (segmentIndex + 1) % source.Count;
                segmentLength = Vector2.Distance(
                    source[segmentIndex].Center,
                    source[(segmentIndex + 1) % source.Count].Center
                );
            }

            TrackCenterlinePoint start = source[segmentIndex];
            TrackCenterlinePoint end =
                source[(segmentIndex + 1) % source.Count];
            float t = (targetDistance - segmentStartDistance) /
                      Math.Max(segmentLength, 1e-5f);
            result[sampleIndex] = new TrackCenterlinePoint(
                Vector2.Lerp(start.Center, end.Center, t),
                Lerp(start.RightWidth, end.RightWidth, t),
                Lerp(start.LeftWidth, end.LeftWidth, t)
            );
        }
        return result;
    }

    private static SmoothedCenterlinePoint[] EnsureSymmetricCorridorFeasible(
        SmoothedCenterlinePoint[] samples,
        float targetLength
    )
    {
        const int adjustmentIterations = 3;
        const int transitionMeters = 30;
        const float innerEdgeCurvatureSafetyFactor = 0.80f;
        int count = samples.Length;

        for (int iteration = 0; iteration < adjustmentIterations; iteration++)
        {
            Vector2[] normals = new Vector2[count];
            float[] requiredOffsets = new float[count];
            for (int i = 0; i < count; i++)
            {
                Vector2 tangent =
                    samples[(i + 1) % count].Center -
                    samples[(i - 1 + count) % count].Center;
                tangent = tangent.LengthSquared() <= 1e-8f
                    ? Vector2.UnitX
                    : Vector2.Normalize(tangent);
                normals[i] = new Vector2(tangent.Y, -tangent.X);

                Vector2 nextTangent =
                    samples[(i + 2) % count].Center - samples[i].Center;
                nextTangent = nextTangent.LengthSquared() <= 1e-8f
                    ? tangent
                    : Vector2.Normalize(nextTangent);
                float headingDelta = MathHelper.NormalizeAngle(
                    nextTangent.Angle() - tangent.Angle()
                );
                if (MathF.Abs(headingDelta) <= 1e-5f)
                    continue;

                float actualRadius = TrackData.StepLength /
                                     MathF.Abs(headingDelta);
                float requiredRadius = samples[i].Width * 0.5f /
                                       innerEdgeCurvatureSafetyFactor;
                float deficit = Math.Max(0f, requiredRadius - actualRadius);
                requiredOffsets[i] = MathF.CopySign(deficit, headingDelta);
            }

            float[] smoothOffsets = (float[])requiredOffsets.Clone();
            for (int distance = 1; distance <= transitionMeters; distance++)
            {
                float weight = 0.5f * (
                    1f + MathF.Cos(
                        MathF.PI * distance / (transitionMeters + 1f)
                    )
                );
                for (int i = 0; i < count; i++)
                {
                    int before = (i - distance + count) % count;
                    int after = (i + distance) % count;
                    float beforeCandidate = requiredOffsets[before] * weight;
                    float afterCandidate = requiredOffsets[after] * weight;
                    if (MathF.Abs(beforeCandidate) > MathF.Abs(smoothOffsets[i]))
                        smoothOffsets[i] = beforeCandidate;
                    if (MathF.Abs(afterCandidate) > MathF.Abs(smoothOffsets[i]))
                        smoothOffsets[i] = afterCandidate;
                }
            }

            SmoothedCenterlinePoint[] adjusted =
                new SmoothedCenterlinePoint[count];
            for (int i = 0; i < count; i++)
            {
                adjusted[i] = new SmoothedCenterlinePoint(
                    samples[i].Center + normals[i] * smoothOffsets[i],
                    samples[i].Width
                );
            }

            float adjustedLength = ClosedSmoothedLength(adjusted);
            float scale = targetLength / adjustedLength;
            Vector2 origin = adjusted[0].Center;
            for (int i = 0; i < count; i++)
            {
                adjusted[i] = new SmoothedCenterlinePoint(
                    (adjusted[i].Center - origin) * scale,
                    adjusted[i].Width * scale
                );
            }
            samples = ResampleSmoothedPoints(adjusted, count, targetLength);
        }

        return samples;
    }

    private static (float[] Left, float[] Right) CalculateAdaptiveBuffers(
        IReadOnlyList<SmoothedCenterlinePoint> samples,
        float desiredLeftBuffer,
        float desiredRightBuffer
    )
    {
        const int minimumNonlocalSeparationMeters = 50;
        const int bufferTransitionMeters = 30;
        const float wallClearanceMarginMeters = 1f;
        const float curvatureOffsetSafetyFactor = 0.80f;
        int count = samples.Count;
        float[] left = new float[count];
        float[] right = new float[count];
        Vector2[] tangents = new Vector2[count];
        Vector2[] normals = new Vector2[count];

        for (int i = 0; i < count; i++)
        {
            left[i] = Math.Max(0f, desiredLeftBuffer);
            right[i] = Math.Max(0f, desiredRightBuffer);
            Vector2 tangent =
                samples[(i + 1) % count].Center -
                samples[(i - 1 + count) % count].Center;
            tangents[i] = tangent.LengthSquared() <= 1e-8f
                ? Vector2.UnitX
                : Vector2.Normalize(tangent);
            normals[i] = new Vector2(tangents[i].Y, -tangents[i].X);
        }

        for (int i = 0; i < count; i++)
        {
            float headingDelta = MathHelper.NormalizeAngle(
                tangents[(i + 1) % count].Angle() - tangents[i].Angle()
            );
            float arcLength = Vector2.Distance(
                samples[i].Center,
                samples[(i + 1) % count].Center
            );
            if (MathF.Abs(headingDelta) <= 1e-5f || arcLength <= 1e-4f)
                continue;

            float curvature = headingDelta / arcLength;
            float safeWallOffset = curvatureOffsetSafetyFactor /
                                   MathF.Abs(curvature);
            float availableBuffer = Math.Max(
                0f,
                safeWallOffset - samples[i].Width * 0.5f
            );
            if (curvature < 0f)
                left[i] = Math.Min(left[i], availableBuffer);
            else
                right[i] = Math.Min(right[i], availableBuffer);
        }

        float maximumWallRadius = 0f;
        for (int i = 0; i < count; i++)
        {
            maximumWallRadius = Math.Max(
                maximumWallRadius,
                samples[i].Width * 0.5f + Math.Max(left[i], right[i])
            );
        }
        float cellSize = Math.Max(10f, maximumWallRadius * 2f + 1f);
        Dictionary<(int X, int Y), List<int>> buckets = [];
        for (int i = 0; i < count; i++)
        {
            (int X, int Y) key = SpatialKey(samples[i].Center, cellSize);
            if (!buckets.TryGetValue(key, out List<int>? bucket))
            {
                bucket = [];
                buckets[key] = bucket;
            }
            bucket.Add(i);
        }

        for (int i = 0; i < count; i++)
        {
            (int X, int Y) key = SpatialKey(samples[i].Center, cellSize);
            for (int cellX = key.X - 1; cellX <= key.X + 1; cellX++)
            for (int cellY = key.Y - 1; cellY <= key.Y + 1; cellY++)
            {
                if (!buckets.TryGetValue((cellX, cellY), out List<int>? bucket))
                    continue;
                foreach (int j in bucket)
                {
                    if (j <= i)
                        continue;
                    int progressGap = Math.Abs(j - i);
                    progressGap = Math.Min(progressGap, count - progressGap);
                    if (progressGap < minimumNonlocalSeparationMeters)
                        continue;

                    Vector2 delta = samples[j].Center - samples[i].Center;
                    float distance = delta.Length();
                    float edgeGap = distance -
                                    samples[i].Width * 0.5f -
                                    samples[j].Width * 0.5f;
                    float allowedBufferSum = Math.Max(
                        0f,
                        edgeGap - wallClearanceMarginMeters
                    );

                    bool iPositiveNormal = Vector2.Dot(delta, normals[i]) >= 0f;
                    bool jPositiveNormal = Vector2.Dot(-delta, normals[j]) >= 0f;
                    float iBuffer = iPositiveNormal ? left[i] : right[i];
                    float jBuffer = jPositiveNormal ? left[j] : right[j];
                    float currentBufferSum = iBuffer + jBuffer;
                    if (currentBufferSum <= allowedBufferSum ||
                        currentBufferSum <= 1e-5f)
                        continue;

                    float scale = allowedBufferSum / currentBufferSum;
                    if (iPositiveNormal)
                        left[i] *= scale;
                    else
                        right[i] *= scale;
                    if (jPositiveNormal)
                        left[j] *= scale;
                    else
                        right[j] *= scale;
                }
            }
        }

        TaperBufferReductions(
            left,
            Math.Max(0f, desiredLeftBuffer),
            bufferTransitionMeters
        );
        TaperBufferReductions(
            right,
            Math.Max(0f, desiredRightBuffer),
            bufferTransitionMeters
        );
        return (left, right);

        static (int X, int Y) SpatialKey(Vector2 point, float cellSize) =>
            (
                (int)MathF.Floor(point.X / cellSize),
                (int)MathF.Floor(point.Y / cellSize)
            );
    }

    private static void TaperBufferReductions(
        float[] values,
        float desiredValue,
        int transitionMeters
    )
    {
        float[] constrained = (float[])values.Clone();
        for (int source = 0; source < values.Length; source++)
        {
            float reduction = desiredValue - values[source];
            if (reduction <= 1e-5f)
                continue;
            for (int distance = 1; distance <= transitionMeters; distance++)
            {
                float weight = 0.5f * (
                    1f + MathF.Cos(
                        MathF.PI * distance / (transitionMeters + 1f)
                    )
                );
                float limit = desiredValue - reduction * weight;
                int before = (source - distance + values.Length) % values.Length;
                int after = (source + distance) % values.Length;
                constrained[before] = Math.Min(constrained[before], limit);
                constrained[after] = Math.Min(constrained[after], limit);
            }
        }
        Array.Copy(constrained, values, values.Length);
    }

    private static SmoothedCenterlinePoint[] CenterSourceControls(
        IReadOnlyList<TrackCenterlinePoint> controls
    )
    {
        SmoothedCenterlinePoint[] centered =
            new SmoothedCenterlinePoint[controls.Count];
        for (int i = 0; i < controls.Count; i++)
        {
            Vector2 tangent =
                controls[(i + 1) % controls.Count].Center -
                controls[(i - 1 + controls.Count) % controls.Count].Center;
            tangent = tangent.LengthSquared() <= 1e-8f
                ? Vector2.UnitX
                : Vector2.Normalize(tangent);
            Vector2 rightNormal = new(tangent.Y, -tangent.X);
            TrackCenterlinePoint control = controls[i];
            Vector2 center = control.Center + rightNormal *
                ((control.RightWidth - control.LeftWidth) * 0.5f);
            centered[i] = new SmoothedCenterlinePoint(center, control.Width);
        }
        return centered;
    }

    private static SmoothedCenterlinePoint[] EvaluatePeriodicCubicBSpline(
        IReadOnlyList<SmoothedCenterlinePoint> controls,
        int subdivisionsPerControl
    )
    {
        SmoothedCenterlinePoint[] result =
            new SmoothedCenterlinePoint[controls.Count * subdivisionsPerControl];
        int outputIndex = 0;
        for (int i = 0; i < controls.Count; i++)
        {
            SmoothedCenterlinePoint p0 =
                controls[(i - 1 + controls.Count) % controls.Count];
            SmoothedCenterlinePoint p1 = controls[i];
            SmoothedCenterlinePoint p2 = controls[(i + 1) % controls.Count];
            SmoothedCenterlinePoint p3 = controls[(i + 2) % controls.Count];
            for (int sample = 0; sample < subdivisionsPerControl; sample++)
            {
                float t = sample / (float)subdivisionsPerControl;
                float t2 = t * t;
                float t3 = t2 * t;
                float b0 = (-t3 + 3f * t2 - 3f * t + 1f) / 6f;
                float b1 = (3f * t3 - 6f * t2 + 4f) / 6f;
                float b2 = (-3f * t3 + 3f * t2 + 3f * t + 1f) / 6f;
                float b3 = t3 / 6f;
                result[outputIndex++] = new SmoothedCenterlinePoint(
                    p0.Center * b0 + p1.Center * b1 +
                    p2.Center * b2 + p3.Center * b3,
                    p0.Width * b0 + p1.Width * b1 +
                    p2.Width * b2 + p3.Width * b3
                );
            }
        }
        return result;
    }

    private static SmoothedCenterlinePoint[] RotateToClosestStart(
        IReadOnlyList<SmoothedCenterlinePoint> points,
        Vector2 desiredStart
    )
    {
        int startIndex = 0;
        float minimumDistanceSquared = float.MaxValue;
        for (int i = 0; i < points.Count; i++)
        {
            float distanceSquared = Vector2.DistanceSquared(
                points[i].Center,
                desiredStart
            );
            if (distanceSquared >= minimumDistanceSquared)
                continue;
            minimumDistanceSquared = distanceSquared;
            startIndex = i;
        }

        SmoothedCenterlinePoint[] rotated =
            new SmoothedCenterlinePoint[points.Count];
        for (int i = 0; i < points.Count; i++)
            rotated[i] = points[(startIndex + i) % points.Count];
        return rotated;
    }

    private static float ClosedSmoothedLength(
        IReadOnlyList<SmoothedCenterlinePoint> points
    )
    {
        float length = 0f;
        for (int i = 0; i < points.Count; i++)
        {
            length += Vector2.Distance(
                points[i].Center,
                points[(i + 1) % points.Count].Center
            );
        }
        return length;
    }

    private static SmoothedCenterlinePoint[] ResampleSmoothedPoints(
        IReadOnlyList<SmoothedCenterlinePoint> source,
        int sampleCount,
        float totalLength
    )
    {
        SmoothedCenterlinePoint[] result =
            new SmoothedCenterlinePoint[sampleCount];
        int segmentIndex = 0;
        float segmentStartDistance = 0f;
        float segmentLength = Vector2.Distance(
            source[0].Center,
            source[1].Center
        );
        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
        {
            float targetDistance = sampleIndex * TrackData.StepLength;
            while (segmentStartDistance + segmentLength < targetDistance)
            {
                segmentStartDistance += segmentLength;
                segmentIndex = (segmentIndex + 1) % source.Count;
                segmentLength = Vector2.Distance(
                    source[segmentIndex].Center,
                    source[(segmentIndex + 1) % source.Count].Center
                );
            }

            SmoothedCenterlinePoint start = source[segmentIndex];
            SmoothedCenterlinePoint end =
                source[(segmentIndex + 1) % source.Count];
            float t = (targetDistance - segmentStartDistance) /
                      Math.Max(segmentLength, 1e-5f);
            result[sampleIndex] = new SmoothedCenterlinePoint(
                Vector2.Lerp(start.Center, end.Center, t),
                Lerp(start.Width, end.Width, t)
            );
        }
        return result;
    }

    /// <summary>
    /// Supplies the road's out-of-plane shape as a function of distance
    /// along the centreline, sampled once per node at build time. Taking a
    /// function rather than per-segment arguments keeps the geometry
    /// pipeline untouched while still allowing a banking ramp, a hill, or
    /// an oval whose bank steepens toward the wall.
    /// </summary>
    /// <summary>
    /// Supplies the road's out-of-plane shape, sampled once per node at
    /// build time and told how sharply the road turns and how wide it is
    /// there — which is what a construction model needs, since crossfall
    /// scales with the width it must shed water across and a corner's
    /// superelevation follows how tight it is. Taking a function rather
    /// than per-segment arguments keeps the geometry pipeline untouched
    /// while still allowing a banking ramp, a hill, or a speedway whose
    /// bank steepens toward the wall.
    /// </summary>
    public TrackBuilder WithSurface(
        Func<TrackSurfaceContext, TrackSurface> surfaceAt
    )
    {
        ArgumentNullException.ThrowIfNull(surfaceAt);
        _surfaceAt = surfaceAt;
        return this;
    }

    /// <summary>
    /// How sharply the centreline turns here, taken from how the tangent
    /// swings between its neighbours. The racing line's own curvature will
    /// not do for this: a corner taken wide reads as almost straight, and a
    /// road is built to the shape of the road.
    /// </summary>
    /// <summary>
    /// Half the length, in nodes, that the surface's idea of curvature is
    /// averaged over. A road is not banked from the curvature at a point:
    /// superelevation is run in and out over tens of metres, and a
    /// centreline assembled from straights and arcs steps its curvature at
    /// every junction. Reading it over a stretch gives the surface the
    /// transition the geometry does not have, and keeps the dither around
    /// zero on a straight from being read as a corner.
    /// </summary>
    private const int SurfaceCurvatureHalfWindowNodes = 8;

    /// <summary>
    /// How sharply the road bends in the vertical plane, read off the
    /// gradient that has just been laid down rather than asked of whoever
    /// wrote the surface. A road cannot be given a climb and a crest that
    /// disagree, and the car is going to be pressed into whichever of them
    /// is real.
    /// </summary>
    private static void WriteVerticalRate(TrackSurface[] surfaces)
    {
        int count = surfaces.Length;
        if (count < 3)
            return;

        float[] bend = new float[count];
        for (int i = 0; i < count; i++)
        {
            float ahead = surfaces[(i + 1) % count].Grade;
            float behind = surfaces[(i - 1 + count) % count].Grade;
            // The rate the gradient changes at, per metre of plan view --
            // not the curvature of the road in space. The physics is written
            // in the plan view and wants the rate; converting to a curvature
            // here and dividing it back out there would be two errors that
            // only nearly cancel.
            bend[i] = (ahead - behind) / (2f * TrackData.StepLength);
        }

        for (int i = 0; i < count; i++)
            surfaces[i] = surfaces[i] with { VerticalRate = bend[i] };
    }

    private static float CentrelineCurvature(
        IReadOnlyList<RefLineTrackPoint> points,
        int index
    )
    {
        int count = points.Count;
        if (count < 3)
            return 0f;

        int half = Math.Min(SurfaceCurvatureHalfWindowNodes, (count - 1) / 2);
        if (half < 1)
            half = 1;

        // Accumulated node by node rather than as one difference across the
        // window, so a hairpin that turns more than half a circle inside it
        // cannot wrap around and come back as a corner the other way.
        float turn = 0f;
        for (int step = -half; step < half; step++)
        {
            Vector2 a = points[((index + step) % count + count) % count].Tangent;
            Vector2 b = points[((index + step + 1) % count + count) % count].Tangent;
            turn += MathHelper.NormalizeAngle(
                MathF.Atan2(b.Y, b.X) - MathF.Atan2(a.Y, a.X)
            );
        }
        return turn / (2f * half * TrackData.StepLength);
    }

    public TrackBuilder AddStraight(float length, float? targetEndWidth = null, float? targetEndLeftBuffer = null, float? targetEndRightBuffer = null)
    {
        float targetEndWidthNotNull = targetEndWidth ?? currentWidth;
        float targetEndLeftBufferNotNull = targetEndLeftBuffer ?? currentLeftBuffer;
        float targetEndRightBufferNotNull = targetEndRightBuffer ?? currentRightBuffer;

        int steps = (int) (length / TrackData.StepLength);

        float startWidth = currentWidth;
        float startLeftBuffer = currentLeftBuffer;
        float startRightBuffer = currentRightBuffer;

        for (int i = 1; i <= steps; i++)
        {
            Vector2 dir = new(MathF.Cos(currentAngle), MathF.Sin(currentAngle));
            dir *= TrackData.StepLength;
            currentPos += dir;

            float t = i / (float)steps;
            float easeT = SmoothStep(0f, 1f, t);
            float stepWidth = startWidth + easeT * (targetEndWidthNotNull - startWidth);
            float stepLeftBuffer = startLeftBuffer + easeT * (targetEndLeftBufferNotNull - startLeftBuffer);
            float stepRightBuffer = startRightBuffer + easeT * (targetEndRightBufferNotNull - startRightBuffer);

            nodes.Add(
                new(
                    currentPos,
                    stepWidth,
                    stepLeftBuffer,
                    stepRightBuffer
                )
            );
        }

        currentWidth = targetEndWidthNotNull;
        currentLeftBuffer = targetEndLeftBufferNotNull;
        currentRightBuffer = targetEndRightBufferNotNull;

        return this;
    }

    public TrackBuilder AddTurn(float turnAngleDeg, float radius, float? targetEndWidth = null, float? targetEndLeftBuffer = null, float? targetEndRightBuffer = null)
    {
        float targetEndWidthNotNull = targetEndWidth ?? currentWidth;
        float targetEndLeftBufferNotNull = targetEndLeftBuffer ?? currentLeftBuffer;
        float targetEndRightBufferNotNull = targetEndRightBuffer ?? currentRightBuffer;

        float turnAngleRad = MathHelper.DegToRad(turnAngleDeg);
        float arcLength = MathF.Abs(turnAngleRad) * radius;
        int steps = (int) (arcLength / TrackData.StepLength);

        float startWidth = currentWidth;
        float startLeftBuffer = currentLeftBuffer;
        float startRightBuffer = currentRightBuffer;
        float stepAngle = turnAngleRad / steps;

        for (int i = 1; i <= steps; i++)
        {
            currentAngle += stepAngle;
            Vector2 dir = new(MathF.Cos(currentAngle), MathF.Sin(currentAngle));
            dir *= TrackData.StepLength;
            currentPos += dir;

            float t = (float) i / steps;
            float easeT = SmoothStep(0f, 1f, t);
            float stepWidth = startWidth + easeT * (targetEndWidthNotNull - startWidth);
            float stepLeftBuffer = startLeftBuffer + easeT * (targetEndLeftBufferNotNull - startLeftBuffer);
            float stepRightBuffer = startRightBuffer + easeT * (targetEndRightBufferNotNull - startRightBuffer);

            nodes.Add(
                new(
                    currentPos,
                    stepWidth,
                    stepLeftBuffer,
                    stepRightBuffer
                )
            );
        }

        currentWidth = targetEndWidthNotNull;
        currentLeftBuffer = targetEndLeftBufferNotNull;
        currentRightBuffer = targetEndRightBufferNotNull;

        return this;
    }

    public TrackBuilder CloseLoop()
    {
        if (nodes.Count == 0) return this;

        BuilderNode startNode = nodes[0];
        BuilderNode endNode = nodes[^1];

        Vector2 p0 = endNode.Center;
        Vector2 p3 = startNode.Center;

        Vector2 startDir = new(MathF.Cos(_startAngle), MathF.Sin(_startAngle));
        Vector2 endDir = new(MathF.Cos(currentAngle), MathF.Sin(currentAngle));

        float dist = Vector2.Distance(p0, p3);
        float controlLen = dist * 0.4f;

        Vector2 p1 = p0 + (endDir * controlLen);
        Vector2 p2 = p3 - (startDir * controlLen);

        float polyLen = (p1 - p0).Length() + (p2 - p1).Length() + (p3 - p2).Length();
        float estimatedArcLength = (dist + polyLen) / 2.0f;
        int resolution = (int)(estimatedArcLength * 2);
        List<Vector2> highResPoints = [];
        for (int i = 0; i <= resolution; i++)
        {
            float t = (float)i / resolution;
            highResPoints.Add(BezierInterpolate(p0, p1, p2, p3, t));
        }

        float totalTrueLength = 0f;
        for (int i = 0; i < highResPoints.Count - 1; i++)
        {
            totalTrueLength += Vector2.Distance(highResPoints[i], highResPoints[i + 1]);
        }

        List<BuilderNode> closedLoop = new(nodes.Count + highResPoints.Count - 1);
        closedLoop.AddRange(nodes);
        float closingDistance = 0f;
        for (int i = 1; i < highResPoints.Count; i++)
        {
            closingDistance += Vector2.Distance(
                highResPoints[i - 1],
                highResPoints[i]
            );
            float easeT = SmoothStep(0f, 1f, closingDistance / totalTrueLength);
            closedLoop.Add(
                new BuilderNode(
                    highResPoints[i],
                    Lerp(endNode.Width, startNode.Width, easeT),
                    Lerp(endNode.LeftBuffer, startNode.LeftBuffer, easeT),
                    Lerp(endNode.RightBuffer, startNode.RightBuffer, easeT)
                )
            );
        }

        ResampleClosedLoop(closedLoop);

        return this;
    }

    private void ResampleClosedLoop(IReadOnlyList<BuilderNode> closedLoop)
    {
        float totalLength = 0f;
        for (int i = 0; i < closedLoop.Count - 1; i++)
        {
            totalLength += Vector2.Distance(
                closedLoop[i].Center,
                closedLoop[i + 1].Center
            );
        }

        int sampleCount = Math.Max(
            3,
            (int)MathF.Round(totalLength / TrackData.StepLength)
        );
        float sampleSpacing = totalLength / sampleCount;
        List<BuilderNode> samples = new(sampleCount);
        int segmentIndex = 0;
        float segmentStartDistance = 0f;
        float segmentLength = Vector2.Distance(
            closedLoop[0].Center,
            closedLoop[1].Center
        );

        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
        {
            float targetDistance = sampleIndex * sampleSpacing;
            while (
                segmentStartDistance + segmentLength < targetDistance &&
                segmentIndex < closedLoop.Count - 2
            )
            {
                segmentStartDistance += segmentLength;
                segmentIndex++;
                segmentLength = Vector2.Distance(
                    closedLoop[segmentIndex].Center,
                    closedLoop[segmentIndex + 1].Center
                );
            }

            BuilderNode start = closedLoop[segmentIndex];
            BuilderNode end = closedLoop[segmentIndex + 1];
            float t = (targetDistance - segmentStartDistance) /
                      Math.Max(segmentLength, 1e-5f);
            samples.Add(
                new BuilderNode(
                    Vector2.Lerp(start.Center, end.Center, t),
                    Lerp(start.Width, end.Width, t),
                    Lerp(start.LeftBuffer, end.LeftBuffer, t),
                    Lerp(start.RightBuffer, end.RightBuffer, t)
                )
            );
        }

        nodes.Clear();
        nodes.AddRange(samples);
    }

    public TrackData Build(TrackGridConfig startingConfig)
    {
        List<RefLineTrackPoint> refTrackPoints = [];
        for (int i = 0; i < nodes.Count; i++)
        {
            Vector2 p_2 = GetCircular(nodes, i - 2).Center;
            Vector2 p_1 = GetCircular(nodes, i - 1).Center;
            Vector2 p1 = GetCircular(nodes, i + 1).Center;
            Vector2 p2 = GetCircular(nodes, i + 2).Center;

            refTrackPoints.Add(
                new RefLineTrackPoint(
                    nodes[i].Center,
                    MathHelper.FivePointStencil(p_2, p_1, p1, p2),
                    nodes[i].Width
                )
            );
        }
        RefLine refNodes = _refLineSolver.Generate(refTrackPoints);
        TrackSurface[] surfaces = new TrackSurface[nodes.Count];
        for (int i = 0; i < nodes.Count; i++)
        {
            surfaces[i] = _surfaceAt is null
                ? TrackSurface.Flat
                : _surfaceAt(new TrackSurfaceContext(
                    i * TrackData.StepLength,
                    CentrelineCurvature(refTrackPoints, i),
                    refTrackPoints[i].Width * 0.5f,
                    nodes.Count * TrackData.StepLength
                ));
        }
        WriteVerticalRate(surfaces);

        List<TrackNode> resNodes = [];
        for (int i = 0; i < nodes.Count; i++)
        {
            resNodes.Add(
                new TrackNode(
                    refTrackPoints[i].Center,
                    refTrackPoints[i].Tangent,
                    refTrackPoints[i].Width,
                    nodes[i].LeftBuffer,
                    nodes[i].RightBuffer,
                    refNodes[i],
                    surfaces[i]
                )
            );
        }
        return new TrackData(resNodes, startingConfig);
    }

    public static float SmoothStep(float from, float to, float weight)
    {
        if (from.CompareTo(to) == 0)
        {
            return from;
        }

        float num = Math.Clamp((weight - from) / (to - from), 0f, 1f);
        return num * num * (3f - 2f * num);
    }

    public static T GetCircular<T>(IList<T> list, int index)
    {
        int size = list.Count;
        int wrappedIndex = (index % size + size) % size;
        return list[wrappedIndex];
    }

    public static Vector2 BezierInterpolate(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t)
    {
        float u = 1 - t;
        float u2 = u * u;
        float t2 = t * t;
        float u3 = u2 * u;
        float t3 = t2 * t;

        // B(t) = (1-t)^3 * P0 + 3*(1-t)^2*t * P1 + 3*(1-t)*t^2 * P2 + t^3 * P3
        return p0 * u3 + p1 * 3 * u2 * t + p2 * 3 * u * t2 + p3 * t3;
    }

    public static float Lerp(float from, float to, float weight)
    {
        return from + (to - from) * weight;
    }

    public static Vector2 Lerp(Vector2 from, Vector2 to, float weight)
    {
        float x = from.X + (to.X - from.X) * weight;
        float y = from.Y + (to.Y - from.Y) * weight;
        return new Vector2(x, y);
    }
}

public static class TrackFactory
{
    private const float GrandPrixTestBufferMeters = 5f;
    private const float GrandPrixTestTrackWidthMeters = 15f;
    private const int GrandPrixTestStartingLineIndex = 120;
    private const int GrandPrixTestFirstGridIndex = 110;
    private const int GrandPrixTestGridCount = 30;
    private const int GrandPrixTestGridStepMeters = 8;

    private static TrackGridConfig GrandPrixTestGrid(
        float gridOffsetMeters = 5f,
        int startingLineIndex = GrandPrixTestStartingLineIndex,
        int firstGridIndex = GrandPrixTestFirstGridIndex
    )
    {
        return new TrackGridConfig
        {
            StartingLineIdx = startingLineIndex,
            GridCount = GrandPrixTestGridCount,
            GridOffset = gridOffsetMeters,
            FirstGridIdx = firstGridIndex,
            IsFirstGridLeft = true,
            GridStepDist = GrandPrixTestGridStepMeters
        };
    }

    public static TrackData SimpleOvalTrack(float width, float height, float trackWidth) {
        TrackBuilder builder = new(new(), trackWidth, 5, 5);
        builder.AddStraight(width * 2, trackWidth)
                .AddTurn(180, height, trackWidth)
                .AddStraight(width * 2, trackWidth)
                .AddTurn(180, height, trackWidth);
        return builder.Build(
            new()
        );
    }

    public static TrackData SimpleTestTrack(bool isLeft = false) {
        float baseWidth = 18.0f;
        TrackBuilder builder = new(new(), baseWidth, 5, 5);
        if (isLeft) {
            builder.AddStraight(550)
                    .AddTurn(180, 20.0f, 25.0f)
                    .AddStraight(120, baseWidth)
                    .AddTurn(-45, 30.0f)
                    .AddTurn(45, 30.0f)
                    .AddStraight(5)
                    .AddTurn(45, 30.0f)
                    .AddTurn(-45, 30.0f)
                    .AddStraight(500, 12.0f)
                    .AddTurn(180, 80.0f, 12.0f)
                    .AddStraight(20)
                    .AddTurn(90, 15.0f)
                    .CloseLoop()
            ;
        } else {
            builder.AddStraight(550)
                    .AddTurn(-180, 20.0f, 25.0f)
                    .AddStraight(120, baseWidth)
                    .AddTurn(45, 30.0f)
                    .AddTurn(-45, 30.0f)
                    .AddStraight(5)
                    .AddTurn(-45, 30.0f)
                    .AddTurn(45, 30.0f)
                    .AddStraight(500, 12.0f)
                    .AddTurn(-180, 80.0f, 12.0f)
                    .AddStraight(20)
                    .AddTurn(-90, 15.0f)
                    .CloseLoop()
            ;
        }

        builder.WithSurface(SimpleTestTrackSurface);
        return builder.Build(
            new()
            {
                StartingLineIdx = 300,
                GridCount = 30,
                GridOffset = 5,
                FirstGridIdx = 290,
                IsFirstGridLeft = true,
                GridStepDist = 8
            }
        );
    }

    /// <summary>
    /// A banked short track, which is where banking stops being a drainage
    /// detail and becomes the corner. The turns run to about thirty-one
    /// degrees at the wall against eighteen at the apron, so grip grows
    /// with the load the corner itself presses down, and running high buys
    /// bank at the price of distance. The ninety-metre turns are what make
    /// that matter: a gentler speedway's corners are not the limit for
    /// these cars, and banking a corner nobody was slowing for changes
    /// nothing. No Grand Prix circuit in this file offers the trade; this
    /// one exists so the physics that models it has somewhere to be true.
    /// </summary>
    public static TrackData BankedSpeedwayTestTrack()
    {
        TrackBuilder builder = new(
            new Vector2(0f, 0f),
            startWidth: 15f,
            startLeftBuffer: 6f,
            startRightBuffer: 6f
        );
        builder
            .AddStraight(350f)
            .AddTurn(180f, 90f)
            .AddStraight(350f)
            .AddTurn(180f, 90f)
            .CloseLoop()
            .WithSurface(TrackSurfaces.Speedway);
        return builder.Build(
            new TrackGridConfig
            {
                StartingLineIdx = 100,
                GridCount = 20,
                GridOffset = 5,
                FirstGridIdx = 90,
                IsFirstGridLeft = true,
                GridStepDist = 9
            }
        );
    }

    // Where each feature of the simple test layout starts, measured from
    // the start/finish line. The layout is written a few lines above; these
    // follow from it, and a test pins them against the curvature that is
    // actually built so they cannot drift apart unnoticed.
    private const float SimpleStartStraightEnd = 550f;
    private const float SimpleTurn1End = 613f;      // 180 degrees at R20
    private const float SimpleBackStraightStart = 832f;
    private const float SimpleBackStraightEnd = 1332f;
    private const float SimpleBigTurnEnd = 1584f;   // 180 degrees at R80

    /// <summary>
    /// Curvature at which the added bank is fully committed: loose enough
    /// that both banked corners here get essentially all of it, tight enough
    /// that a straight gets essentially none.
    /// </summary>
    private const float SimpleExtraBankReferenceCurvature = 0.006f;

    /// <summary>
    /// The simple test layout given some relief: the start/finish straight
    /// climbs hard, the plateau carries through the first-corner hairpin and
    /// the esses, and the back straight gives every metre of it back. The
    /// hairpin and the long left before the final corner are banked well
    /// past what a road circuit carries, so this layout exercises gradient
    /// and bank together — which is the point of a test track, as against
    /// the Grand Prix circuits, where the surface is modelled after the real
    /// places and is correspondingly mild.
    /// </summary>
    private static TrackSurface SimpleTestTrackSurface(
        TrackSurfaceContext context
    )
    {
        const float summit = 30f;
        TrackSurface section = TrackElevation.ProfileByDistance(
            [
                (0f, 0f),
                (SimpleStartStraightEnd, summit),
                (SimpleBackStraightStart, summit),
                (SimpleBackStraightEnd, 0f)
            ],
            TrackSurfaces.RoadCircuit
        )(context);

        float extra = SimpleExtraBank(
            context.DistanceMeters,
            context.LapLengthMeters
        );
        if (extra <= 0f)
            return section;

        // Added in the direction the corner leans, so the bank helps the turn
        // whichever way it goes -- but weighted by how committed that lean
        // is, not merely by its sign. Taking the sign alone put the whole
        // seventeen degrees on whichever side of zero the curvature happened
        // to be, and flipped all of it in a single metre where the hairpin
        // handed over to the esses.
        float lean = TrackSurfaces.CornerLean(
            context.CentrelineCurvature,
            SimpleExtraBankReferenceCurvature
        );
        return section with
        {
            BankSlope = section.BankSlope + extra * lean
        };
    }

    /// <summary>
    /// How much bank the two chosen corners get beyond the drainage
    /// crossfall, easing in and out over their entries and exits so the
    /// road never steps.
    /// </summary>
    private static float SimpleExtraBank(float distance, float lapLength)
    {
        const float hairpinBank = 0.30f;   // about seventeen degrees
        const float sweeperBank = 0.22f;   // about twelve degrees
        return MathF.Max(
            hairpinBank * TrackSurfaces.SectionWeight(
                distance, SimpleStartStraightEnd, SimpleTurn1End, 25f, lapLength
            ),
            sweeperBank * TrackSurfaces.SectionWeight(
                distance, SimpleBackStraightEnd, SimpleBigTurnEnd, 45f, lapLength
            )
        );
    }

    // FIA Arena Grand Prix layout: the source centerline and widths come from
    // the TUM FTM open racetrack database and are scaled to the FIA-published
    // 5.891 km centreline length.
    public static TrackData SilverstoneStyleTestTrack()
    {
        TrackBuilder builder = TrackBuilder.FromClosedCenterline(
            TrackCenterlineData.Silverstone,
            5_891f,
            GrandPrixTestBufferMeters,
            GrandPrixTestBufferMeters
        );
        builder.WithSurface(TrackElevation.Profile(
            TrackElevation.SilverstoneHeights,
            TrackSurfaces.RoadCircuit
        ));
        return builder.Build(
            GrandPrixTestGrid(startingLineIndex: 0, firstGridIndex: -10)
        );
    }

    // FIA Monaco Grand Prix layout. The public GeoJSON centreline is projected
    // to local metres and scaled to the FIA-published 3.337 km length.
    public static TrackData MonacoStyleTestTrack()
    {
        TrackBuilder builder = TrackBuilder.FromClosedCenterline(
            TrackCenterlineData.Monaco,
            3_337f,
            3f,
            3f
        );
        builder.WithSurface(TrackElevation.Profile(
            TrackElevation.MonacoHeights,
            TrackSurfaces.RoadCircuit
        ));
        return builder.Build(
            GrandPrixTestGrid(
                gridOffsetMeters: 3.5f,
                startingLineIndex: 0,
                firstGridIndex: -10
            )
        );
    }

    // FIA Shanghai Grand Prix layout, including both snail complexes and the
    // 1.2 km back straight, scaled to the published 5.451 km length.
    public static TrackData ShanghaiStyleTestTrack()
    {
        TrackBuilder builder = TrackBuilder.FromClosedCenterline(
            TrackCenterlineData.Shanghai,
            5_451f,
            GrandPrixTestBufferMeters,
            GrandPrixTestBufferMeters,
            controlSpacingMeters: 12f
        );
        builder.WithSurface(TrackElevation.Profile(
            TrackElevation.ShanghaiHeights,
            TrackSurfaces.RoadCircuit
        ));
        return builder.Build(
            GrandPrixTestGrid(startingLineIndex: 0, firstGridIndex: -10)
        );
    }

    // Sepang Grand Prix layout: a non-overlapping high-speed direction-change
    // benchmark replacing Suzuka, whose grade-separated crossover cannot be
    // represented by the current two-dimensional TrackData topology.
    public static TrackData SepangStyleTestTrack()
    {
        TrackBuilder builder = TrackBuilder.FromClosedCenterline(
            TrackCenterlineData.Sepang,
            5_543f,
            GrandPrixTestBufferMeters,
            GrandPrixTestBufferMeters
        );
        builder.WithSurface(TrackElevation.Profile(
            TrackElevation.SepangHeights,
            TrackSurfaces.RoadCircuit
        ));
        return builder.Build(
            GrandPrixTestGrid(startingLineIndex: 0, firstGridIndex: -10)
        );
    }
}
