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

    private TrackData? _track;
    private float _requestedOffsetMeters = float.NaN;
    private float _vehicleHalfWidthMeters = float.NaN;
    private float[] _offsetByMeter = [];

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
            OffsetCurvature(
                track,
                s,
                in center,
                beforeOffset,
                centerOffset,
                afterOffset,
                geometryRadiusMeters
            ),
            centerOffset
        );
    }

    /// <summary>
    /// Curvature of the offset line, derived rather than measured.
    ///
    /// Fitting a circle through three sampled positions works on the racing
    /// line and fails beside it. An offset point is the reference point plus
    /// the track normal times the offset, so every wobble in the normal is
    /// multiplied by how far out the line sits, and taking a second derivative
    /// from positions multiplies it again. The racing line escapes this
    /// because its curvature is smoothed once when the track is built and read
    /// back, not recomputed from coordinates. Measured, the error averaged
    /// 0.0011 1/m but reached 0.035 - larger than any corner on the circuit,
    /// and inverted - so the speed planner met a hairpin that was not there
    /// several times a lap and braked for it. That alone cost 12% of the
    /// distance covered, at one metre of offset as much as at three, which is
    /// what gave it away: geometry charges for how far out the line is, and
    /// this charged for merely being off the line at all.
    ///
    /// For a line at offset d(s) from a reference of curvature k, writing
    /// a = 1 - k*d and b = d', the Frenet frame gives
    ///   k_off = (a*(k*a + d'') - b*(a' - k*b)) / (a^2 + b^2)^(3/2)
    /// Everything on the right is known exactly except k', which enters only
    /// through a' and multiplied by b, so a coarse difference is enough. The
    /// offset's own derivatives come from the profile, which is slew-limited
    /// and therefore far better behaved than sampled positions.
    /// </summary>
    private static float OffsetCurvature(
        TrackData track,
        float s,
        in TrackSample center,
        float beforeOffset,
        float centerOffset,
        float afterOffset,
        float radiusMeters
    )
    {
        float curvature = center.RefCurvature;
        float first = (afterOffset - beforeOffset) / (2f * radiusMeters);
        float second = (afterOffset - 2f * centerOffset + beforeOffset) /
                       (radiusMeters * radiusMeters);
        float curvatureRate = (
            track.Sample(s + radiusMeters).RefCurvature -
            track.Sample(s - radiusMeters).RefCurvature
        ) / (2f * radiusMeters);

        float a = 1f - curvature * centerOffset;
        float b = first;
        float aRate = -curvatureRate * centerOffset - curvature * first;
        float denominator = MathF.Pow(a * a + b * b, 1.5f);
        if (denominator < 1e-6f)
            return curvature;

        return (a * (curvature * a + second) - b * (aRate - curvature * b)) /
               denominator;
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
        if (requestedOffsetMeters < 0f)
        {
            for (int i = 0; i < _offsetByMeter.Length; i++)
                _offsetByMeter[i] = -_offsetByMeter[i];
        }

        _track = track;
        _requestedOffsetMeters = requestedOffsetMeters;
        _vehicleHalfWidthMeters = vehicleHalfWidthMeters;
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
