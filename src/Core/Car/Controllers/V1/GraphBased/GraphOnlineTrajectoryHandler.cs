using System;
using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.GraphBased;

public sealed class GraphOnlineTrajectoryHandler(GraphBasedLocalPlanner planner)
{
    private GraphPath? _previousPath;

    public GraphPath? PreviousPath => _previousPath;

    public void Reset()
    {
        _previousPath = null;
    }

    public GraphOnlineTrajectory Plan(GraphPlannerRequest request)
    {
        GraphPlannerConfig config = request.Config ?? request.Cache?.Config ?? new GraphPlannerConfig();
        GraphPath? reusablePreviousPath = null;
        bool reusedPreviousPath = false;
        bool resetPreviousPath = false;
        float previousDistance = float.PositiveInfinity;
        int nearestPreviousSample = -1;
        GraphPath? candidatePreviousPath = _previousPath ?? request.PreviousPath;

        if (CanInspectPreviousPath(candidatePreviousPath))
        {
            nearestPreviousSample = FindNearestPathSample(candidatePreviousPath!, request.Position);
            previousDistance = candidatePreviousPath![nearestPreviousSample].Position.DistanceTo(request.Position);
            if (CanReusePreviousPath(previousDistance, config))
            {
                reusablePreviousPath = candidatePreviousPath;
                reusedPreviousPath = true;
            }
            else
            {
                resetPreviousPath = true;
            }
        }

        GraphPath path = planner.Plan(request with { PreviousPath = reusablePreviousPath });
        _previousPath = path is { IsFallback: false, Count: > 1 } ? path : null;

        return new GraphOnlineTrajectory(
            path,
            reusedPreviousPath,
            resetPreviousPath,
            previousDistance,
            nearestPreviousSample
        );
    }

    private static bool CanInspectPreviousPath(GraphPath? path)
    {
        return path is { IsFallback: false, Count: > 1 } && path.Nodes.Length > 0;
    }

    private static bool CanReusePreviousPath(float distanceMeters, GraphPlannerConfig config)
    {
        return config.PreviousPathReuseMaxDistanceMeters <= 0f ||
               distanceMeters <= config.PreviousPathReuseMaxDistanceMeters;
    }

    private static int FindNearestPathSample(GraphPath path, Vector2 position)
    {
        int bestIndex = 0;
        float bestDistanceSquared = float.PositiveInfinity;
        for (int i = 0; i < path.Count; i++)
        {
            float distanceSquared = path[i].Position.DistanceSquaredTo(position);
            if (distanceSquared >= bestDistanceSquared)
                continue;

            bestDistanceSquared = distanceSquared;
            bestIndex = i;
        }

        return bestIndex;
    }
}
