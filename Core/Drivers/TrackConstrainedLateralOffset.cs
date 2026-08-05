using System;
using System.Numerics;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Drivers;

internal readonly record struct TrackLateralTargetSample(
    Vector2 Position,
    float Heading,
    float Curvature,
    float OffsetMeters
);

internal sealed class TrackConstrainedLateralOffset
{
    internal const float EdgeMarginMeters = 0.3f;
    private const float MaximumOffsetChangePerMeter = 0.08f;

    /// <summary>
    /// Averaging passes over the offset table. Each spreads a corner a little
    /// further; enough of them and the profile is smooth over the tens of
    /// metres the car covers while its steering settles, which is the scale
    /// that matters. Measured on an empty circuit, running one metre off the
    /// racing line cost 11.8% of the distance covered without this and 0.8%
    /// with it.
    /// </summary>
    private const int SmoothingPasses = 3000;

    private TrackData? _track;
    private float _requestedOffsetMeters = float.NaN;
    private float _vehicleHalfWidthMeters = float.NaN;
    private float[] _offsetByMeter = [];
    private float[] _ceilingByMeter = [];

    public void Prepare(TrackData track)
    {
        ArgumentNullException.ThrowIfNull(track);
        if (_offsetByMeter.Length != track.Length)
            _offsetByMeter = new float[track.Length];

        if (ReferenceEquals(track, _track))
            return;

        _track = null;
        _requestedOffsetMeters = float.NaN;
        _vehicleHalfWidthMeters = float.NaN;
    }

    public float Resolve(
        TrackData track,
        in TrackSample sample,
        float tacticalOffsetMeters,
        float executionOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        float constrainedTacticalOffset = 0f;
        if (tacticalOffsetMeters != 0f)
        {
            EnsureProfile(track, tacticalOffsetMeters, vehicleHalfWidthMeters);
            constrainedTacticalOffset = SampleProfile(track, sample.S);
        }

        return ClampAtSample(
            in sample,
            constrainedTacticalOffset + executionOffsetMeters,
            vehicleHalfWidthMeters
        );
    }

    public TrackLateralTargetSample SampleGeometry(
        TrackData track,
        float s,
        float tacticalOffsetMeters,
        float executionOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        if (tacticalOffsetMeters == 0f && executionOffsetMeters == 0f)
        {
            TrackSample reference = track.Sample(s);
            return new TrackLateralTargetSample(
                reference.RefPosition,
                reference.RefHeading,
                reference.RefCurvature,
                0f
            );
        }

        const float geometryRadiusMeters = 4f;
        TrackSample before = track.Sample(s - geometryRadiusMeters);
        TrackSample center = track.Sample(s);
        TrackSample after = track.Sample(s + geometryRadiusMeters);
        float beforeOffset = Resolve(
            track,
            in before,
            tacticalOffsetMeters,
            executionOffsetMeters,
            vehicleHalfWidthMeters
        );
        float centerOffset = Resolve(
            track,
            in center,
            tacticalOffsetMeters,
            executionOffsetMeters,
            vehicleHalfWidthMeters
        );
        float afterOffset = Resolve(
            track,
            in after,
            tacticalOffsetMeters,
            executionOffsetMeters,
            vehicleHalfWidthMeters
        );
        Vector2 beforePosition = before.RefPosition +
                                 before.Normal * beforeOffset;
        Vector2 centerPosition = center.RefPosition +
                                 center.Normal * centerOffset;
        Vector2 afterPosition = after.RefPosition +
                                after.Normal * afterOffset;
        Vector2 tangent = afterPosition - beforePosition;
        if (tangent.LengthSquared() <= 1e-8f)
            tangent = center.Tangent;

        return new TrackLateralTargetSample(
            centerPosition,
            MathF.Atan2(tangent.Y, tangent.X),
            MathHelper.Curvature(beforePosition, centerPosition, afterPosition),
            centerOffset
        );
    }

    internal static float ClampAtSample(
        in TrackSample sample,
        float requestedOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        float halfWidth = MathF.Max(0f, vehicleHalfWidthMeters);
        float minimumOffset = -sample.HalfWidth +
                              halfWidth +
                              EdgeMarginMeters -
                              sample.RefOffset;
        float maximumOffset = sample.HalfWidth -
                              halfWidth -
                              EdgeMarginMeters -
                              sample.RefOffset;
        if (requestedOffsetMeters > 0f)
            return MathF.Min(requestedOffsetMeters, MathF.Max(0f, maximumOffset));
        if (requestedOffsetMeters < 0f)
            return MathF.Max(requestedOffsetMeters, MathF.Min(0f, minimumOffset));
        return 0f;
    }

    private void EnsureProfile(
        TrackData track,
        float requestedOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        if (ReferenceEquals(track, _track) &&
            requestedOffsetMeters == _requestedOffsetMeters &&
            vehicleHalfWidthMeters == _vehicleHalfWidthMeters)
        {
            return;
        }

        Prepare(track);
        int minimumIndex = 0;
        float minimumMagnitude = float.PositiveInfinity;
        for (int i = 0; i < _offsetByMeter.Length; i++)
        {
            TrackSample sample = track.Sample(i * TrackData.StepLength);
            float magnitude = MathF.Abs(ClampAtSample(
                in sample,
                requestedOffsetMeters,
                vehicleHalfWidthMeters
            ));
            _offsetByMeter[i] = magnitude;
            if (magnitude >= minimumMagnitude)
                continue;

            minimumMagnitude = magnitude;
            minimumIndex = i;
        }

        LimitGrowthFrom(minimumIndex, 1);
        LimitGrowthFrom(minimumIndex, -1);
        Smooth();
        if (requestedOffsetMeters < 0f)
        {
            for (int i = 0; i < _offsetByMeter.Length; i++)
                _offsetByMeter[i] = -_offsetByMeter[i];
        }

        _track = track;
        _requestedOffsetMeters = requestedOffsetMeters;
        _vehicleHalfWidthMeters = vehicleHalfWidthMeters;
    }

    /// <summary>
    /// Rounds the corners the width clamp leaves behind.
    ///
    /// Clamping the offset to the road and then bounding how fast it may grow
    /// produces a line that is continuous in position and in slope, and that
    /// is not enough. Where the clamp stops biting, or where growth starts and
    /// stops being limited, the slope changes abruptly, and an abrupt change
    /// of slope is a spike of curvature. The speed planner reads curvature, so
    /// it finds a corner sharper than anything on the circuit and brakes for
    /// it - several times a lap, which was costing more than a tenth of the
    /// distance the car covered.
    ///
    /// Each pass replaces a sample with a weighted average of itself and its
    /// neighbours, then puts the road's own limit back: averaging alone can
    /// lift a sample over a narrow point it had been clamped under. Repeating
    /// it settles on the smoothest line that still fits, and the corners go
    /// with it. The ceiling is captured before the first pass because it is
    /// what the track allows, not what the last pass produced.
    /// </summary>
    private void Smooth()
    {
        int count = _offsetByMeter.Length;
        if (count < 3)
            return;

        if (_ceilingByMeter.Length != count)
            _ceilingByMeter = new float[count];
        Array.Copy(_offsetByMeter, _ceilingByMeter, count);

        for (int pass = 0; pass < SmoothingPasses; pass++)
        {
            float previous = _offsetByMeter[count - 1];
            for (int i = 0; i < count; i++)
            {
                float current = _offsetByMeter[i];
                float next = _offsetByMeter[(i + 1) % count];
                float smoothed = 0.25f * previous + 0.5f * current + 0.25f * next;
                previous = current;
                _offsetByMeter[i] = MathF.Min(smoothed, _ceilingByMeter[i]);
            }
        }
    }

    private void LimitGrowthFrom(int startIndex, int direction)
    {
        int count = _offsetByMeter.Length;
        int previousIndex = startIndex;
        float maximumStep = MaximumOffsetChangePerMeter * TrackData.StepLength;
        for (int step = 1; step < count; step++)
        {
            int index = (startIndex + direction * step) % count;
            if (index < 0)
                index += count;
            _offsetByMeter[index] = MathF.Min(
                _offsetByMeter[index],
                _offsetByMeter[previousIndex] + maximumStep
            );
            previousIndex = index;
        }
    }

    private float SampleProfile(TrackData track, float s)
    {
        float scaled = track.WrapS(s) / TrackData.StepLength;
        int index = (int)MathF.Floor(scaled);
        float t = scaled - index;
        int nextIndex = (index + 1) % _offsetByMeter.Length;
        return _offsetByMeter[index] +
               (_offsetByMeter[nextIndex] - _offsetByMeter[index]) * t;
    }
}
