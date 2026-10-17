using System.Numerics;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Circuits;

/// <summary>
/// A source centreline made into a road: smoothed, scaled to the published
/// length, centred in the road, given run-off, and resampled a metre at a
/// time. Moved here from Core, which no longer does any of it.
/// </summary>
internal static class Centreline
{
    private readonly record struct SmoothedCenterlinePoint(
        Vector2 Center,
        float Width
    );

    public static TrackBuilder FromClosedCenterline(
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
        samples = SmoothHeadingProfile(samples, sampledLength);
        samples = EnsureSymmetricCorridorFeasible(samples, sampledLength);
        (float[] leftBuffers, float[] rightBuffers) = CalculateAdaptiveBuffers(
            samples,
            leftBuffer,
            rightBuffer
        );

        var rows = new CircuitTrackFile.Row[sampleCount];
        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
            rows[sampleIndex] = new CircuitTrackFile.Row(
                samples[sampleIndex].Center, 0f, samples[sampleIndex].Width,
                leftBuffers[sampleIndex], rightBuffers[sampleIndex], 0f, 0f);
        return TrackBuilder.FromRows(rows);
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
                TrackBuilder.Lerp(start.RightWidth, end.RightWidth, t),
                TrackBuilder.Lerp(start.LeftWidth, end.LeftWidth, t)
            );
        }
        return result;
    }

    /// <summary>
    /// How much the road is allowed to turn inside the averaging window.
    ///
    /// The window is not a fixed length, and that is the whole design. A
    /// fixed thirty metres was tried first and rounded real corners off by
    /// as much as forty-six per cent, because these circuits do not hold a
    /// constant radius for thirty metres -- a twenty-three metre hairpin
    /// has an apex about that long in total. But a fixed window short
    /// enough to keep the hairpin is too short to do anything to a
    /// straight.
    ///
    /// The resolution is that the two ends of the road need opposite
    /// treatment for the same reason. Where the road turns hard, the
    /// signal is enormous compared with the survey's error and needs no
    /// help; where it barely turns at all, the error is all there is. So
    /// the window is the distance over which the road turns this much:
    /// tens of metres on a straight, a couple of metres at an apex.
    /// </summary>
    private const float HeadingSmoothingTurnRadians = 0.02f;

    /// <summary>Bounds on that window's half-width, in metres. The upper
    /// one keeps the filter from reaching across a whole straight into the
    /// corners at both ends; the lower one keeps it doing something.</summary>
    private const float HeadingSmoothingMinHalfMetres = 2f;
    private const float HeadingSmoothingMaxHalfMetres = 25f;

    /// <summary>
    /// Average the rate the road turns at, not where it is, and average it
    /// over a distance that shortens as the road tightens.
    ///
    /// Smoothing positions cuts corners: a box filter across a hairpin
    /// pulls the apex metres towards the inside and the road ends up
    /// somewhere the road is not. Smoothing heading increments leaves any
    /// constant-radius arc exactly where it was, because the average of a
    /// constant is that constant. What is left to protect is the apex of a
    /// corner too short to have a constant stretch, and the variable
    /// window does that: see <see cref="HeadingSmoothingTurnRadians"/>.
    ///
    /// Two invariants are structural rather than lucky. The increments are
    /// re-centred after filtering so they sum to exactly what they summed
    /// to before, which is the total turn; and the step between nodes is
    /// untouched, so the lap is exactly as long afterwards. What is not
    /// free is closure -- re-integrating perturbed headings lands a little
    /// away from where it started -- so that error is spread linearly
    /// round the lap and the result is rescaled and resampled. The heading
    /// that ramp costs is the closing error over the lap length, orders
    /// below the dither being removed.
    /// </summary>
    private static SmoothedCenterlinePoint[] SmoothHeadingProfile(
        SmoothedCenterlinePoint[] samples,
        float targetLength
    )
    {
        int count = samples.Length;
        if (count < 8)
            return samples;

        float step = targetLength / count;
        if (step <= 1e-4f)
            return samples;

        float[] heading = new float[count];
        for (int i = 0; i < count; i++)
        {
            Vector2 delta =
                samples[(i + 1) % count].Center - samples[i].Center;
            heading[i] = MathF.Atan2(delta.Y, delta.X);
        }

        float[] turn = new float[count];
        for (int i = 0; i < count; i++)
        {
            turn[i] = MathHelper.NormalizeAngle(
                heading[(i + 1) % count] - heading[i]
            );
        }

        // How hard the road is turning here, read over a fixed stretch
        // before any filtering. It only chooses the window width, so it
        // wants to be robust rather than sharp.
        const int probeHalf = 10;
        float[] magnitude = new float[count];
        for (int i = 0; i < count; i++)
        {
            float accumulated = 0f;
            for (int k = -probeHalf; k < probeHalf; k++)
                accumulated += turn[((i + k) % count + count) % count];
            magnitude[i] =
                MathF.Abs(accumulated) / (2f * probeHalf * step);
        }

        int minHalf = Math.Max(
            1, (int)MathF.Round(HeadingSmoothingMinHalfMetres / step)
        );
        int maxHalf = Math.Max(
            minHalf, (int)MathF.Round(HeadingSmoothingMaxHalfMetres / step)
        );
        if (2 * maxHalf + 1 >= count)
            return samples;

        // Each node spreads its own turn over its own window, rather than
        // each node collecting the average of one. The two are the same
        // operation when the window is a fixed width and different when it
        // is not, and that difference is the reason for writing it this way
        // round: spreading conserves the total turn exactly, node by node,
        // because every node's contribution is divided by exactly the
        // number of places it is divided among. Collecting does not. The
        // first version of this method collected, lost a few per cent of
        // the lap's turn, and the loss came back as a constant bend of
        // twenty-kilometre radius under the straights -- a closing error of
        // tens of metres, and every corner sitting up to sixty metres from
        // where the survey put it.
        float[] smoothed = new float[count];
        for (int i = 0; i < count; i++)
        {
            float reach = magnitude[i] > 1e-7f
                ? HeadingSmoothingTurnRadians / magnitude[i] / step
                : maxHalf;
            int half = (int)MathF.Round(
                Math.Clamp(reach, minHalf, maxHalf)
            );
            float share = turn[i] / (2 * half + 1);
            for (int k = -half; k <= half; k++)
                smoothed[((i + k) % count + count) % count] += share;
        }

        Vector2[] centers = new Vector2[count];
        centers[0] = samples[0].Center;
        float angle = heading[0];
        for (int i = 1; i < count; i++)
        {
            centers[i] = centers[i - 1]
                + step * new Vector2(MathF.Cos(angle), MathF.Sin(angle));
            angle += smoothed[i - 1];
        }

        Vector2 closingError = centers[count - 1]
            + step * new Vector2(MathF.Cos(angle), MathF.Sin(angle))
            - centers[0];
        SmoothedCenterlinePoint[] adjusted =
            new SmoothedCenterlinePoint[count];
        for (int i = 0; i < count; i++)
        {
            adjusted[i] = new SmoothedCenterlinePoint(
                centers[i] - closingError * ((float)i / count),
                samples[i].Width
            );
        }

        float adjustedLength = ClosedSmoothedLength(adjusted);
        if (adjustedLength <= 1e-3f)
            return samples;
        return ResampleSmoothedPoints(adjusted, count, targetLength);
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
            // Right of travel with y to the south, the opposite of Core's
            // left-positive normal.
            Vector2 rightNormal = new(-tangent.Y, tangent.X);
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
                TrackBuilder.Lerp(start.Width, end.Width, t)
            );
        }
        return result;
    }
}
