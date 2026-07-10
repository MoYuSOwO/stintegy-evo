using System;
using System.Collections.Generic;
using System.Numerics;
using TheStint.Core.Track.RefLines;
using TheStint.Core.Util;

namespace TheStint.Core.Track;

public class TrackBuilder
{
    private static readonly IRefLineSolver solver = new MinimumCurvatureRefLineSolver();

    private readonly struct BuilderNode(Vector2 center, float width, float leftBuffer, float rightBuffer)
    {
        public readonly Vector2 Center = center;
        public readonly float Width = width;
        public readonly float LeftBuffer = leftBuffer;
        public readonly float RightBuffer = rightBuffer;
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

    public TrackBuilder(Vector2 startPos, float startWidth, float startLeftBuffer = 0, float startRightBuffer = 0, float startAngleDeg = 0)
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

        float distanceWalked = 0f;
        float currentTargetDist = TrackData.StepLength;

        for (int i = 0; i < highResPoints.Count - 1; i++)
        {
            Vector2 segmentStart = highResPoints[i];
            Vector2 segmentEnd = highResPoints[i + 1];
            float segmentLength = Vector2.Distance(segmentStart, segmentEnd);

            while (distanceWalked + segmentLength >= currentTargetDist && currentTargetDist < totalTrueLength)
            {
                float remainDist = currentTargetDist - distanceWalked;
                float lerpFactor = remainDist / segmentLength; 

                Vector2 exactPos = Lerp(segmentStart, segmentEnd, lerpFactor);

                float t = currentTargetDist / totalTrueLength;
                float easeT = SmoothStep(0f, 1f, t);

                float width = Lerp(endNode.Width, startNode.Width, easeT);
                float leftBuffer = Lerp(endNode.LeftBuffer, startNode.LeftBuffer, easeT);
                float rightBuffer = Lerp(endNode.RightBuffer, startNode.RightBuffer, easeT);

                nodes.Add(new BuilderNode(exactPos, width, leftBuffer, rightBuffer));

                currentTargetDist += TrackData.StepLength; 
            }
            distanceWalked += segmentLength;
        }

        return this;
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
        var refNodes = solver.Generate(refTrackPoints);
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
                    refNodes[i]
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
}
