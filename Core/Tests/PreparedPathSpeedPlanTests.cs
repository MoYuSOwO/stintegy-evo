using System.Numerics;
using StintegyEVO.Core.Drivers;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class PreparedPathSpeedPlanTests
{
    [Fact]
    public void CapturedPlanRestoresOnceForMatchingPathAndPlanningSnapshot()
    {
        VehiclePathPrediction path = BuildPath();
        PreparedPathSpeedPlan prepared = new();
        float[] curvatures = [0.01f, 0.02f];
        float[] segmentLengths = [2f, 0f];
        float[] speedLimits = [31f, 29f];
        float[] speeds = [28f, 27f];
        prepared.Capture(
            path,
            planningSnapshotGeneration: 17,
            curvatures,
            segmentLengths,
            speedLimits,
            speeds,
            maximumAbsoluteCurvature: 0.02f
        );
        float[] restoredCurvatures = new float[2];
        float[] restoredSegmentLengths = new float[2];
        float[] restoredSpeedLimits = new float[2];
        float[] restoredSpeeds = new float[2];

        bool restored = prepared.TryRestore(
            path,
            planningSnapshotGeneration: 17,
            restoredCurvatures,
            restoredSegmentLengths,
            restoredSpeedLimits,
            restoredSpeeds,
            out float maximumAbsoluteCurvature
        );

        Assert.True(restored);
        Assert.Equal(curvatures, restoredCurvatures);
        Assert.Equal(segmentLengths, restoredSegmentLengths);
        Assert.Equal(speedLimits, restoredSpeedLimits);
        Assert.Equal(speeds, restoredSpeeds);
        Assert.Equal(0.02f, maximumAbsoluteCurvature);

        Assert.False(prepared.TryRestore(
            path,
            planningSnapshotGeneration: 17,
            restoredCurvatures,
            restoredSegmentLengths,
            restoredSpeedLimits,
            restoredSpeeds,
            out _
        ));
    }

    [Fact]
    public void CapturedPlanRejectsAnotherPlanningSnapshot()
    {
        VehiclePathPrediction path = BuildPath();
        PreparedPathSpeedPlan prepared = CapturePlan(path);
        float[] restoredCurvatures = new float[2];
        float[] restoredSegmentLengths = new float[2];
        float[] restoredSpeedLimits = new float[2];
        float[] restoredSpeeds = new float[2];

        Assert.False(prepared.TryRestore(
            path,
            planningSnapshotGeneration: 18,
            restoredCurvatures,
            restoredSegmentLengths,
            restoredSpeedLimits,
            restoredSpeeds,
            out _
        ));

        Assert.True(prepared.TryRestore(
            path,
            planningSnapshotGeneration: 17,
            restoredCurvatures,
            restoredSegmentLengths,
            restoredSpeedLimits,
            restoredSpeeds,
            out _
        ));
    }

    [Fact]
    public void CapturedPlanRejectsResetPathBuffer()
    {
        VehiclePathPrediction path = BuildPath();
        PreparedPathSpeedPlan prepared = CapturePlan(path);
        float[] restoredCurvatures = new float[2];
        float[] restoredSegmentLengths = new float[2];
        float[] restoredSpeedLimits = new float[2];
        float[] restoredSpeeds = new float[2];

        path.Reset(2);
        Assert.False(prepared.TryRestore(
            path,
            planningSnapshotGeneration: 17,
            restoredCurvatures,
            restoredSegmentLengths,
            restoredSpeedLimits,
            restoredSpeeds,
            out _
        ));
    }

    private static PreparedPathSpeedPlan CapturePlan(
        VehiclePathPrediction path
    )
    {
        PreparedPathSpeedPlan prepared = new();
        prepared.Capture(
            path,
            planningSnapshotGeneration: 17,
            [0.01f, 0.02f],
            [2f, 0f],
            [31f, 29f],
            [28f, 27f],
            maximumAbsoluteCurvature: 0.02f
        );
        return prepared;
    }

    private static VehiclePathPrediction BuildPath()
    {
        VehiclePathPrediction path = new();
        path.Reset(2);
        path.Add(new VehiclePathPredictionPoint(
            0f,
            Vector2.Zero,
            0f,
            0f,
            0f,
            0f,
            0f,
            0f,
            0f,
            20f
        ));
        path.Add(new VehiclePathPredictionPoint(
            2f,
            new Vector2(2f, 0f),
            0f,
            2f,
            0f,
            0f,
            0f,
            0f,
            0f,
            20f
        ));
        return path;
    }
}
