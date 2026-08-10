using System;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Turns a latched racing-room decision into a spatially anchored, quintic
/// handover. The near-field reference is retained, while the farther path is
/// constrained to the side already earned by the car.
/// </summary>
internal sealed class RacingRoomPathConstraint
{
    private const float MinimumHandoverDistanceMeters = 24f;
    private const float MaximumOffsetChangePerMeter = 0.08f;
    private const float LateralAccelerationBudget = 6f;
    private const float MaximumQuinticFirstDerivative = 1.875f;
    private const float MaximumQuinticSecondDerivative = 5.773503f;

    private RacingRoomPairSnapshot[] _fromPairs = [];
    private RacingRoomPairSnapshot[] _toPairs = [];
    private int _fromCount;
    private int _toCount;
    private int _targetSignature;
    private TrackData? _track;
    private string? _carId;
    private float _vehicleHalfWidthMeters;
    private float _handoverStartS;
    private float _handoverLengthMeters;
    private bool _handoverActive;

    public bool HasProfile => _handoverActive || _fromCount > 0 || _toCount > 0;
    public float HandoverLengthMeters => _handoverLengthMeters;

    public void Update(
        TrackData track,
        float currentS,
        float currentSpeedMetersPerSecond,
        in RacingRoomSnapshot snapshot,
        string carId,
        float vehicleHalfWidthMeters
    )
    {
        ArgumentNullException.ThrowIfNull(track);
        ArgumentNullException.ThrowIfNull(carId);
        float halfWidth = MathF.Max(0f, vehicleHalfWidthMeters);
        if (!ReferenceEquals(_track, track) ||
            !string.Equals(_carId, carId, StringComparison.Ordinal) ||
            MathF.Abs(_vehicleHalfWidthMeters - halfWidth) > 1e-4f)
        {
            Reset(track, carId, halfWidth);
        }

        CompleteIfReached(track, currentS);
        int signature = ComputeSignature(in snapshot, carId);
        if (_handoverActive || signature == _targetSignature)
            return;

        EnsureCapacity(snapshot.Count);
        _toCount = CopyRelevantPairs(
            in snapshot,
            carId,
            _toPairs
        );
        _targetSignature = signature;
        float maximumShift = EstimateMaximumShift(
            track,
            currentS,
            halfWidth
        );
        if (maximumShift <= 1e-4f)
        {
            CopyToFrom();
            return;
        }

        float transitionTime = MathF.Sqrt(
            MaximumQuinticSecondDerivative * maximumShift /
            LateralAccelerationBudget
        );
        float speedScaledLength = MathF.Max(0f, currentSpeedMetersPerSecond) *
                                  transitionTime;
        float slopeLimitedLength = MaximumQuinticFirstDerivative *
                                   maximumShift /
                                   MaximumOffsetChangePerMeter;
        _handoverLengthMeters = MathF.Min(
            MathF.Max(
                MinimumHandoverDistanceMeters,
                MathF.Max(speedScaledLength, slopeLimitedLength)
            ),
            track.LengthMeters * 0.45f
        );
        _handoverStartS = track.WrapS(currentS);
        _handoverActive = true;
    }

    public float Apply(
        TrackData track,
        in TrackSample sample,
        float baseOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        if (!HasProfile || !ReferenceEquals(track, _track) || _carId is null)
            return baseOffsetMeters;

        float from = Constrain(
            _fromPairs,
            _fromCount,
            in sample,
            baseOffsetMeters,
            vehicleHalfWidthMeters,
            _carId
        );
        float to = Constrain(
            _toPairs,
            _toCount,
            in sample,
            baseOffsetMeters,
            vehicleHalfWidthMeters,
            _carId
        );
        if (!_handoverActive)
            return to;

        float distance = SignedDistanceFromStart(track, sample.S);
        float progress = Math.Clamp(
            distance / MathF.Max(_handoverLengthMeters, 1e-3f),
            0f,
            1f
        );
        float blend = QuinticSmoothStep(progress);
        return from + (to - from) * blend;
    }

    private float EstimateMaximumShift(
        TrackData track,
        float currentS,
        float vehicleHalfWidthMeters
    )
    {
        float maximum = 0f;
        for (float distance = 0f; distance <= 120f; distance += 10f)
        {
            TrackSample sample = track.Sample(currentS + distance);
            float from = Constrain(
                _fromPairs,
                _fromCount,
                in sample,
                baseOffsetMeters: 0f,
                vehicleHalfWidthMeters,
                _carId!
            );
            float to = Constrain(
                _toPairs,
                _toCount,
                in sample,
                baseOffsetMeters: 0f,
                vehicleHalfWidthMeters,
                _carId!
            );
            maximum = MathF.Max(maximum, MathF.Abs(to - from));
        }
        return maximum;
    }

    private static float Constrain(
        RacingRoomPairSnapshot[] pairs,
        int pairCount,
        in TrackSample sample,
        float baseOffsetMeters,
        float vehicleHalfWidthMeters,
        string carId
    )
    {
        if (pairCount == 0)
            return baseOffsetMeters;

        RacingRoomSnapshot snapshot = new(pairs, pairCount);
        if (!snapshot.TryGetCorridor(
                carId,
                sample.HalfWidth,
                MathF.Max(0f, vehicleHalfWidthMeters) * 2f,
                out RacingRoomCorridor corridor
            ) ||
            !corridor.Feasible)
        {
            return baseOffsetMeters;
        }

        float requestedTrackD = sample.RefOffset + baseOffsetMeters;
        float constrainedTrackD = SmoothClamp(
            requestedTrackD,
            corridor.MinimumTrackD,
            corridor.MaximumTrackD
        );
        return constrainedTrackD - sample.RefOffset;
    }

    private static float SmoothClamp(float value, float minimum, float maximum)
    {
        if (minimum >= maximum)
            return 0.5f * (minimum + maximum);

        float width = maximum - minimum;
        float band = MathF.Min(0.3f, width * 0.24f);
        if (band <= 1e-4f)
            return Math.Clamp(value, minimum, maximum);

        float lowerEnd = minimum + band;
        if (value < lowerEnd)
        {
            if (value <= minimum - band)
                return minimum;
            float t = (value - (minimum - band)) / (2f * band);
            return minimum + band * t * t;
        }

        float upperStart = maximum - band;
        if (value > upperStart)
        {
            if (value >= maximum + band)
                return maximum;
            float t = ((maximum + band) - value) / (2f * band);
            return maximum - band * t * t;
        }

        return value;
    }

    private void CompleteIfReached(TrackData track, float currentS)
    {
        if (!_handoverActive ||
            SignedDistanceFromStart(track, currentS) < _handoverLengthMeters)
        {
            return;
        }

        CopyToFrom();
        _handoverActive = false;
    }

    private void CopyToFrom()
    {
        EnsureCapacity(_toCount);
        Array.Copy(_toPairs, _fromPairs, _toCount);
        _fromCount = _toCount;
    }

    private void EnsureCapacity(int required)
    {
        if (_fromPairs.Length >= required)
            return;
        int capacity = Math.Max(required, Math.Max(2, _fromPairs.Length * 2));
        Array.Resize(ref _fromPairs, capacity);
        Array.Resize(ref _toPairs, capacity);
    }

    private static int CopyRelevantPairs(
        in RacingRoomSnapshot snapshot,
        string carId,
        RacingRoomPairSnapshot[] destination
    )
    {
        int count = 0;
        foreach (RacingRoomPairSnapshot pair in snapshot.Pairs)
        {
            if (pair.Contains(carId))
                destination[count++] = pair;
        }
        return count;
    }

    private static int ComputeSignature(
        in RacingRoomSnapshot snapshot,
        string carId
    )
    {
        int signature = 17;
        int count = 0;
        foreach (RacingRoomPairSnapshot pair in snapshot.Pairs)
        {
            if (!pair.Contains(carId))
                continue;
            signature = unchecked(signature * 31 + pair.Generation);
            count++;
        }
        return count == 0 ? 0 : unchecked(signature * 31 + count);
    }

    private float SignedDistanceFromStart(TrackData track, float s)
    {
        float distance = track.WrapS(s - _handoverStartS);
        if (distance > track.LengthMeters * 0.5f)
            distance -= track.LengthMeters;
        return distance;
    }

    private void Reset(TrackData track, string carId, float halfWidth)
    {
        _track = track;
        _carId = carId;
        _vehicleHalfWidthMeters = halfWidth;
        _fromCount = 0;
        _toCount = 0;
        _targetSignature = 0;
        _handoverStartS = 0f;
        _handoverLengthMeters = 0f;
        _handoverActive = false;
    }

    private static float QuinticSmoothStep(float progress)
    {
        float t = Math.Clamp(progress, 0f, 1f);
        float t2 = t * t;
        float t3 = t2 * t;
        return t3 * (10f + t * (-15f + 6f * t));
    }
}
