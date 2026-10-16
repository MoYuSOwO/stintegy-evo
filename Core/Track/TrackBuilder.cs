using System;
using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Track;

public class TrackBuilder
{
    private Func<TrackSurfaceContext, TrackSurface>? _surfaceAt;

    /// <summary>
    /// One metre of road. The buffers are floored on the way in rather
    /// than checked on the way out: run-off narrower than a kerb is not a
    /// narrow run-off, it is a white line with a barrier behind it, and
    /// the edge grammar has no such case. Every path into the builder --
    /// the hand-written straights and arcs, the surveyed centrelines, and
    /// the resampling that runs over both -- goes through here, so this is
    /// the one place the floor has to hold.
    /// </summary>
    private readonly struct BuilderNode(Vector2 center, float width, float leftBuffer, float rightBuffer)
    {
        public readonly Vector2 Center = center;
        public readonly float Width = width;
        public readonly float LeftBuffer =
            Math.Max(leftBuffer, SurfaceGrip.MinimumBufferMeters);
        public readonly float RightBuffer =
            Math.Max(rightBuffer, SurfaceGrip.MinimumBufferMeters);
    }

    private readonly List<BuilderNode> nodes = [];

    private readonly Vector2 _startPos;
    private readonly float _startWidth;
    private readonly float _startLeftBuffer;
    private readonly float _startRightBuffer;
    private readonly float _startAngle;
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
        float startAngleDeg = 0
    )
    {
        _startPos = startPos;
        _startWidth = startWidth;
        _startLeftBuffer = startLeftBuffer;
        _startRightBuffer = startRightBuffer;
        _startAngle = MathHelper.DegToRad(startAngleDeg);

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

    /// <summary>A road already laid out a metre at a time, as a circuit's track file stores it.</summary>
    internal static TrackBuilder FromRows(IReadOnlyList<CircuitTrackFile.Row> rows)
    {
        TrackBuilder builder = new(rows[0].Center, rows[0].Width, rows[0].RunoffLeft, rows[0].RunoffRight);
        for (int i = 1; i < rows.Count; i++)
            builder.nodes.Add(new BuilderNode(rows[i].Center, rows[i].Width, rows[i].RunoffLeft, rows[i].RunoffRight));
        return builder;
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
        IReadOnlyList<Vector2> tangents,
        int index
    )
    {
        int count = tangents.Count;
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
            Vector2 a = tangents[((index + step) % count + count) % count];
            Vector2 b = tangents[((index + step + 1) % count + count) % count];
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
        // Track geometry ends at the physical road centreline.  Tangent and
        // curvature are derived from the sampled road itself; no racing-line
        // solver is consulted or stored in the track model.
        Vector2[] tangents = new Vector2[nodes.Count];
        for (int i = 0; i < nodes.Count; i++)
        {
            Vector2 p_2 = GetCircular(nodes, i - 2).Center;
            Vector2 p_1 = GetCircular(nodes, i - 1).Center;
            Vector2 p1 = GetCircular(nodes, i + 1).Center;
            Vector2 p2 = GetCircular(nodes, i + 2).Center;
            tangents[i] = MathHelper.FivePointStencil(p_2, p_1, p1, p2);
        }

        float[] curvatures = new float[nodes.Count];
        TrackSurface[] surfaces = new TrackSurface[nodes.Count];
        for (int i = 0; i < nodes.Count; i++)
        {
            curvatures[i] = CentrelineCurvature(tangents, i);
            surfaces[i] = _surfaceAt is null
                ? TrackSurface.Flat
                : _surfaceAt(new TrackSurfaceContext(
                    i * TrackData.StepLength,
                    curvatures[i],
                    nodes[i].Width * 0.5f,
                    nodes.Count * TrackData.StepLength
                ));
        }
        WriteVerticalRate(surfaces);

        List<TrackNode> result = new(nodes.Count);
        for (int i = 0; i < nodes.Count; i++)
        {
            result.Add(
                new TrackNode(
                    nodes[i].Center,
                    tangents[i],
                    curvatures[i],
                    nodes[i].Width,
                    nodes[i].LeftBuffer,
                    nodes[i].RightBuffer,
                    surfaces[i]
                )
            );
        }
        return new TrackData(result, startingConfig);
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
