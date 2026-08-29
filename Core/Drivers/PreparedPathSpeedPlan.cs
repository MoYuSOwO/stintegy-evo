using System;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Retains the exact traffic-free speed solve for a mutable predicted-path
/// buffer until that path is reset. The traffic phase can then begin from the
/// same settled limits it published instead of solving the free path twice.
/// </summary>
internal sealed class PreparedPathSpeedPlan
{
    private float[] _curvatures = [];
    private float[] _segmentLengths = [];
    private float[] _speedLimits = [];
    private float[] _speeds = [];
    private VehiclePathPrediction? _path;
    private int _pathGeneration;
    private long _planningSnapshotGeneration;
    private int _count;
    private float[] _alongTrackGravity = [];
    private float _maximumAbsoluteCurvature;

    public void Capture(
        VehiclePathPrediction path,
        long planningSnapshotGeneration,
        float[] curvatures,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float[] alongTrackGravity,
        float maximumAbsoluteCurvature
    )
    {
        ArgumentNullException.ThrowIfNull(path);
        int count = path.Count;
        EnsureCapacity(count);
        Array.Copy(curvatures, _curvatures, count);
        Array.Copy(segmentLengths, _segmentLengths, count);
        Array.Copy(speedLimits, _speedLimits, count);
        Array.Copy(speeds, _speeds, count);
        Array.Copy(alongTrackGravity, _alongTrackGravity, count);
        _path = path;
        _pathGeneration = path.Generation;
        _planningSnapshotGeneration = planningSnapshotGeneration;
        _count = count;
        _maximumAbsoluteCurvature = maximumAbsoluteCurvature;
    }

    public bool TryRestore(
        VehiclePathPrediction path,
        long planningSnapshotGeneration,
        float[] curvatures,
        float[] segmentLengths,
        float[] speedLimits,
        float[] speeds,
        float[] alongTrackGravity,
        out float maximumAbsoluteCurvature
    )
    {
        ArgumentNullException.ThrowIfNull(path);
        if (!ReferenceEquals(_path, path) ||
            _pathGeneration != path.Generation ||
            _planningSnapshotGeneration != planningSnapshotGeneration ||
            _count != path.Count)
        {
            maximumAbsoluteCurvature = 0f;
            return false;
        }

        EnsureDestinationCapacity(curvatures, nameof(curvatures));
        EnsureDestinationCapacity(segmentLengths, nameof(segmentLengths));
        EnsureDestinationCapacity(speedLimits, nameof(speedLimits));
        EnsureDestinationCapacity(speeds, nameof(speeds));
        EnsureDestinationCapacity(
            alongTrackGravity,
            nameof(alongTrackGravity)
        );
        Array.Copy(_curvatures, curvatures, _count);
        Array.Copy(_segmentLengths, segmentLengths, _count);
        Array.Copy(_speedLimits, speedLimits, _count);
        Array.Copy(_speeds, speeds, _count);
        // The road travels with the plan. Restoring without it would leave
        // whatever the pooled array last held standing in for the gradient,
        // which is both wrong and different depending on who rented it
        // before.
        Array.Copy(_alongTrackGravity, alongTrackGravity, _count);
        maximumAbsoluteCurvature = _maximumAbsoluteCurvature;
        _path = null;
        return true;
    }

    private void EnsureCapacity(int count)
    {
        if (_curvatures.Length >= count)
            return;

        _curvatures = new float[count];
        _segmentLengths = new float[count];
        _speedLimits = new float[count];
        _speeds = new float[count];
        _alongTrackGravity = new float[count];
    }

    private void EnsureDestinationCapacity(float[] destination, string name)
    {
        ArgumentNullException.ThrowIfNull(destination, name);
        if (destination.Length < _count)
        {
            throw new ArgumentException(
                "Destination does not have enough capacity.",
                name
            );
        }
    }
}
