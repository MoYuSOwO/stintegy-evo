using System;
using System.Collections.Generic;
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

    // The handover contributes at most about 0.6 g before the track's own
    // curvature is added. The vehicle-specific speed planner still prices the
    // resulting line against the actual car and tires; this only prevents the
    // reference itself from asking for an abrupt lane change.
    private const float HandoverLateralAccelerationBudget = 6f;
    private const float MinimumHandoverDistanceMeters = 24f;

    // Exact maxima of the first and second derivatives of
    // 10t^3 - 15t^4 + 6t^5 on [0, 1].
    private const float MaximumQuinticFirstDerivative = 1.875f;
    private const float MaximumQuinticSecondDerivative = 5.773503f;
    private const float TargetEqualityToleranceMeters = 1e-4f;
    private const float HandoverCurvatureTolerance = 1e-4f;

    /// <summary>
    /// Averaging passes over the offset table. Each spreads a corner a little
    /// further; enough of them and the profile is smooth over the tens of
    /// metres the car covers while its steering settles, which is the scale
    /// that matters. Measured on an empty circuit, running one metre off the
    /// racing line cost 11.8% of the distance covered without this and 0.8%
    /// with it.
    /// </summary>
    private const int SmoothingPasses = 3000;

    /// <summary>
    /// Finished profiles, shared by every car on the circuit.
    ///
    /// Building one is the most expensive thing the racecraft does - the whole
    /// lap is laid out and then walked over several thousand times - and it is
    /// paid whenever a car asks for an offset it was not already holding. Yet
    /// nothing in the answer belongs to the car that asked. The table is the
    /// road's reply to "if I want to run this far off the line, where will you
    /// let me", and the only thing about the car that enters it is how wide it
    /// is. Twenty cars racing each other ask the same handful of questions over
    /// and over: measured over twenty seconds of a full grid, three hundred and
    /// thirty two builds between them had forty one distinct answers.
    ///
    /// Sharing them costs a copy of the table on a change of offset, against a
    /// build that is four orders of magnitude dearer. A car of a width nobody
    /// has raced yet simply builds its own and leaves it here for the next one.
    /// </summary>
    private static readonly Dictionary<(float Offset, float HalfWidth), float[]>
        SharedProfiles = [];

    private static TrackData? _sharedProfilesTrack;

    private TrackData? _track;
    private TrackData? _committedTrack;
    private float _requestedOffsetMeters = float.NaN;
    private float _vehicleHalfWidthMeters = float.NaN;
    private float[] _offsetByMeter = [];
    private float[] _ceilingByMeter = [];
    private float[] _handoverFromByMeter = [];
    private float _committedTargetOffsetMeters;
    private float _committedVehicleHalfWidthMeters = float.NaN;
    private float _handoverStartS;
    private float _handoverLengthMeters;
    private float _minimumHandoverLengthMeters;
    private float _maximumHandoverLengthMeters;
    private bool _handoverActive;

    internal bool HasCommittedProfile =>
        _handoverActive ||
        MathF.Abs(_committedTargetOffsetMeters) >
        TargetEqualityToleranceMeters;
    internal float CommittedTargetOffsetMeters =>
        _committedTargetOffsetMeters;
    internal float CommittedHandoverLengthMeters => _handoverLengthMeters;
    internal float MaximumCommittedHandoverLengthMeters =>
        _maximumHandoverLengthMeters;

    internal void SetCommittedHandoverLength(float lengthMeters)
    {
        if (!_handoverActive)
            return;

        float minimum = MathF.Min(
            _minimumHandoverLengthMeters,
            _maximumHandoverLengthMeters
        );
        float maximum = MathF.Max(
            _minimumHandoverLengthMeters,
            _maximumHandoverLengthMeters
        );
        _handoverLengthMeters = Math.Clamp(
            lengthMeters,
            minimum,
            maximum
        );
    }

    internal bool RequiresHandoverPlanning(
        TrackData track,
        float currentS,
        float requestedOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        float halfWidth = MathF.Max(0f, vehicleHalfWidthMeters);
        if (!ReferenceEquals(track, _committedTrack) ||
            !NearlyEqual(halfWidth, _committedVehicleHalfWidthMeters))
        {
            return MathF.Abs(requestedOffsetMeters) >
                   TargetEqualityToleranceMeters;
        }

        bool activeAtCurrentS = _handoverActive &&
            SignedDistanceFromHandoverStart(track, currentS) <
            _handoverLengthMeters;
        return !activeAtCurrentS &&
               !NearlyEqual(
                   requestedOffsetMeters,
                   _committedTargetOffsetMeters
               );
    }

    public void Prepare(TrackData track)
    {
        ArgumentNullException.ThrowIfNull(track);
        if (_offsetByMeter.Length != track.Length)
            _offsetByMeter = new float[track.Length];
        if (_handoverFromByMeter.Length != track.Length)
            _handoverFromByMeter = new float[track.Length];

        if (ReferenceEquals(track, _track))
            return;

        _track = null;
        _requestedOffsetMeters = float.NaN;
        _vehicleHalfWidthMeters = float.NaN;
    }

    /// <summary>
    /// Commits a spatial handover from the line currently being followed to a
    /// newly requested offset. The handover is anchored once in track space;
    /// repeated frame updates cannot move it underneath the car.
    /// </summary>
    public void UpdateCommittedTarget(
        TrackData track,
        float currentS,
        float currentSpeedMetersPerSecond,
        float requestedOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        UpdateCommittedTargetCore(
            track,
            currentS,
            currentSpeedMetersPerSecond,
            requestedOffsetMeters,
            vehicleHalfWidthMeters,
            constraints: default,
            useSpeedConstraints: false
        );
    }

    public void UpdateCommittedTarget(
        TrackData track,
        float currentS,
        float currentSpeedMetersPerSecond,
        float requestedOffsetMeters,
        float vehicleHalfWidthMeters,
        in ReferenceLineHandoverConstraints constraints
    )
    {
        UpdateCommittedTargetCore(
            track,
            currentS,
            currentSpeedMetersPerSecond,
            requestedOffsetMeters,
            vehicleHalfWidthMeters,
            in constraints,
            useSpeedConstraints: true
        );
    }

    private void UpdateCommittedTargetCore(
        TrackData track,
        float currentS,
        float currentSpeedMetersPerSecond,
        float requestedOffsetMeters,
        float vehicleHalfWidthMeters,
        in ReferenceLineHandoverConstraints constraints,
        bool useSpeedConstraints
    )
    {
        ArgumentNullException.ThrowIfNull(track);
        Prepare(track);
        float halfWidth = MathF.Max(0f, vehicleHalfWidthMeters);
        if (!ReferenceEquals(track, _committedTrack) ||
            !NearlyEqual(halfWidth, _committedVehicleHalfWidthMeters))
        {
            ResetCommittedTarget();
            _committedTrack = track;
            _committedVehicleHalfWidthMeters = halfWidth;
        }

        CompleteHandoverIfReached(track, currentS);
        // A committed line is deliberately not retargeted under the car. Once
        // its spatial prefix ends, this frame's latest request becomes the
        // next handover instead of replaying stale intermediate requests.
        if (_handoverActive)
            return;

        if (NearlyEqual(
                requestedOffsetMeters,
                _committedTargetOffsetMeters))
        {
            return;
        }

        BeginHandover(
            track,
            currentS,
            currentSpeedMetersPerSecond,
            requestedOffsetMeters,
            halfWidth,
            in constraints,
            useSpeedConstraints
        );
    }

    public void ResetCommittedTarget()
    {
        _committedTrack = null;
        _committedTargetOffsetMeters = 0f;
        _committedVehicleHalfWidthMeters = float.NaN;
        _handoverStartS = 0f;
        _handoverLengthMeters = 0f;
        _minimumHandoverLengthMeters = 0f;
        _maximumHandoverLengthMeters = 0f;
        _handoverActive = false;
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
        if (UsesCommittedHandover(
                track,
                tacticalOffsetMeters,
                vehicleHalfWidthMeters))
        {
            constrainedTacticalOffset = SampleCommittedHandover(
                track,
                sample.S
            );
        }
        else if (tacticalOffsetMeters != 0f)
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
        bool usesCommittedHandover = UsesCommittedHandover(
            track,
            tacticalOffsetMeters,
            vehicleHalfWidthMeters
        );
        if (tacticalOffsetMeters == 0f &&
            executionOffsetMeters == 0f &&
            !usesCommittedHandover)
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

    private void BeginHandover(
        TrackData track,
        float currentS,
        float currentSpeedMetersPerSecond,
        float requestedOffsetMeters,
        float vehicleHalfWidthMeters,
        in ReferenceLineHandoverConstraints constraints,
        bool useSpeedConstraints
    )
    {
        if (MathF.Abs(_committedTargetOffsetMeters) <=
            TargetEqualityToleranceMeters)
        {
            Array.Clear(_handoverFromByMeter);
        }
        else
        {
            EnsureProfile(
                track,
                _committedTargetOffsetMeters,
                vehicleHalfWidthMeters
            );
            Array.Copy(
                _offsetByMeter,
                _handoverFromByMeter,
                _offsetByMeter.Length
            );
        }

        if (MathF.Abs(requestedOffsetMeters) >
            TargetEqualityToleranceMeters)
        {
            EnsureProfile(track, requestedOffsetMeters, vehicleHalfWidthMeters);
        }

        float offsetChange = MaximumHandoverOffsetChange(
            requestedOffsetMeters
        );
        float transitionTime = MathF.Sqrt(
            MaximumQuinticSecondDerivative * offsetChange /
            HandoverLateralAccelerationBudget
        );
        float speedScaledLength = MathF.Max(
            0f,
            currentSpeedMetersPerSecond
        ) * transitionTime;
        float slopeLimitedLength =
            MaximumQuinticFirstDerivative * offsetChange /
            MaximumOffsetChangePerMeter;
        float fallbackLength = MathF.Min(
            MathF.Max(
                MinimumHandoverDistanceMeters,
                MathF.Max(speedScaledLength, slopeLimitedLength)
            ),
            track.LengthMeters * 0.45f
        );
        _minimumHandoverLengthMeters = MathF.Max(
            MinimumHandoverDistanceMeters,
            slopeLimitedLength
        );
        _maximumHandoverLengthMeters = fallbackLength;
        _handoverStartS = track.WrapS(currentS);
        _committedTargetOffsetMeters = requestedOffsetMeters;
        _committedVehicleHalfWidthMeters = vehicleHalfWidthMeters;
        _handoverLengthMeters = useSpeedConstraints && constraints.IsUsable
            ? SelectSpeedPreservingHandoverLength(
                track,
                slopeLimitedLength,
                fallbackLength,
                in constraints
            )
            : fallbackLength;
        _handoverActive = true;
    }

    private float SelectSpeedPreservingHandoverLength(
        TrackData track,
        float slopeLimitedLength,
        float fallbackLength,
        in ReferenceLineHandoverConstraints constraints
    )
    {
        float minimumLength = MathF.Max(
            MinimumHandoverDistanceMeters,
            slopeLimitedLength
        );
        float maximumLength = MathF.Min(
            track.LengthMeters * 0.45f,
            constraints.BaselineSpeedPlan.LengthMeters
        );
        if (float.IsFinite(constraints.LatestCompletionDistanceMeters) &&
            constraints.LatestCompletionDistanceMeters > 0f)
        {
            maximumLength = MathF.Min(
                maximumLength,
                constraints.LatestCompletionDistanceMeters
            );
        }
        if (maximumLength < minimumLength)
            return MathF.Min(fallbackLength, track.LengthMeters * 0.45f);

        _minimumHandoverLengthMeters = minimumLength;
        _maximumHandoverLengthMeters = maximumLength;

        if (HandoverRetainsBaselineSpeed(
                track,
                minimumLength,
                in constraints))
        {
            return minimumLength;
        }

        float searchStep = MathF.Max(
            8f,
            constraints.BaselineSpeedPlan.StepLengthMeters * 4f
        );
        float lastFailure = minimumLength;
        for (float candidate = minimumLength + searchStep;
             candidate < maximumLength;
             candidate += searchStep)
        {
            if (!HandoverRetainsBaselineSpeed(
                    track,
                    candidate,
                    in constraints))
            {
                lastFailure = candidate;
                continue;
            }

            return RefineFirstSpeedPreservingLength(
                track,
                lastFailure,
                candidate,
                in constraints
            );
        }

        if (HandoverRetainsBaselineSpeed(
                track,
                maximumLength,
                in constraints))
        {
            return RefineFirstSpeedPreservingLength(
                track,
                lastFailure,
                maximumLength,
                in constraints
            );
        }

        // The target line or the tactical deadline makes some speed loss
        // unavoidable. Use all available distance instead of inventing a
        // sharper transition or silently refusing the requested line.
        return maximumLength;
    }

    private float RefineFirstSpeedPreservingLength(
        TrackData track,
        float knownFailure,
        float knownSuccess,
        in ReferenceLineHandoverConstraints constraints
    )
    {
        float lower = knownFailure;
        float upper = knownSuccess;
        for (int i = 0; i < 5; i++)
        {
            float midpoint = (lower + upper) * 0.5f;
            if (HandoverRetainsBaselineSpeed(
                    track,
                    midpoint,
                    in constraints))
            {
                upper = midpoint;
            }
            else
            {
                lower = midpoint;
            }
        }
        return upper;
    }

    private bool HandoverRetainsBaselineSpeed(
        TrackData track,
        float handoverLengthMeters,
        in ReferenceLineHandoverConstraints constraints
    )
    {
        float sampleStep = Math.Clamp(
            constraints.BaselineSpeedPlan.StepLengthMeters,
            1f,
            4f
        );
        for (float distance = 0f;
             distance < handoverLengthMeters;
             distance += sampleStep)
        {
            if (!HandoverSampleRetainsBaselineSpeed(
                    track,
                    distance,
                    handoverLengthMeters,
                    in constraints))
            {
                return false;
            }
        }
        return HandoverSampleRetainsBaselineSpeed(
            track,
            handoverLengthMeters,
            handoverLengthMeters,
            in constraints
        );
    }

    private bool HandoverSampleRetainsBaselineSpeed(
        TrackData track,
        float distanceMeters,
        float handoverLengthMeters,
        in ReferenceLineHandoverConstraints constraints
    )
    {
        float curvature = SampleHandoverCurvature(
            track,
            _handoverStartS + distanceMeters,
            handoverLengthMeters
        );
        float maximumCurvature = constraints.MaximumCurvatureAt(
            distanceMeters
        );
        return MathF.Abs(curvature) <=
               maximumCurvature + HandoverCurvatureTolerance;
    }

    private float SampleHandoverCurvature(
        TrackData track,
        float s,
        float handoverLengthMeters
    )
    {
        const float geometryRadiusMeters = 4f;
        TrackSample before = track.Sample(s - geometryRadiusMeters);
        TrackSample center = track.Sample(s);
        TrackSample after = track.Sample(s + geometryRadiusMeters);
        Vector2 beforePosition = before.RefPosition + before.Normal *
            SampleCommittedHandover(track, before.S, handoverLengthMeters);
        Vector2 centerPosition = center.RefPosition + center.Normal *
            SampleCommittedHandover(track, center.S, handoverLengthMeters);
        Vector2 afterPosition = after.RefPosition + after.Normal *
            SampleCommittedHandover(track, after.S, handoverLengthMeters);
        return MathHelper.Curvature(
            beforePosition,
            centerPosition,
            afterPosition
        );
    }

    private float MaximumHandoverOffsetChange(float requestedOffsetMeters)
    {
        bool targetIsReferenceLine =
            MathF.Abs(requestedOffsetMeters) <=
            TargetEqualityToleranceMeters;
        float maximum = 0f;
        for (int i = 0; i < _handoverFromByMeter.Length; i++)
        {
            float target = targetIsReferenceLine ? 0f : _offsetByMeter[i];
            maximum = MathF.Max(
                maximum,
                MathF.Abs(target - _handoverFromByMeter[i])
            );
        }
        return maximum;
    }

    private void CompleteHandoverIfReached(
        TrackData track,
        float currentS
    )
    {
        if (!_handoverActive)
            return;

        float distance = SignedDistanceFromHandoverStart(track, currentS);
        if (distance < _handoverLengthMeters)
            return;

        _handoverActive = false;
    }

    private bool UsesCommittedHandover(
        TrackData track,
        float tacticalOffsetMeters,
        float vehicleHalfWidthMeters
    )
    {
        return _handoverActive &&
               ReferenceEquals(track, _committedTrack) &&
               NearlyEqual(
                   tacticalOffsetMeters,
                   _committedTargetOffsetMeters
               ) &&
               NearlyEqual(
                   vehicleHalfWidthMeters,
                   _committedVehicleHalfWidthMeters
               );
    }

    private float SampleCommittedHandover(TrackData track, float s)
    {
        return SampleCommittedHandover(track, s, _handoverLengthMeters);
    }

    private float SampleCommittedHandover(
        TrackData track,
        float s,
        float handoverLengthMeters
    )
    {
        float distance = SignedDistanceFromHandoverStart(track, s);
        float progress = Math.Clamp(
            distance / MathF.Max(handoverLengthMeters, 1e-3f),
            0f,
            1f
        );
        float blend = QuinticSmoothStep(progress);
        float from = SampleProfile(track, _handoverFromByMeter, s);
        float to = MathF.Abs(_committedTargetOffsetMeters) <=
                   TargetEqualityToleranceMeters
            ? 0f
            : SampleProfile(track, _offsetByMeter, s);
        return from + (to - from) * blend;
    }

    private float SignedDistanceFromHandoverStart(TrackData track, float s)
    {
        float distance = track.WrapS(s - _handoverStartS);
        if (distance > track.LengthMeters * 0.5f)
            distance -= track.LengthMeters;
        return distance;
    }

    private static float QuinticSmoothStep(float progress)
    {
        float t = Math.Clamp(progress, 0f, 1f);
        float t2 = t * t;
        float t3 = t2 * t;
        return t3 * (10f + t * (-15f + 6f * t));
    }

    private static bool NearlyEqual(float left, float right) =>
        MathF.Abs(left - right) <= TargetEqualityToleranceMeters;

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
        (float, float) key = (requestedOffsetMeters, vehicleHalfWidthMeters);
        lock (SharedProfiles)
        {
            if (!ReferenceEquals(track, _sharedProfilesTrack))
            {
                // A profile is the shape of one circuit and means nothing on
                // another, and holding the tables of a circuit nobody is racing
                // on holds the circuit itself alive with them.
                SharedProfiles.Clear();
                _sharedProfilesTrack = track;
            }
            else if (SharedProfiles.TryGetValue(key, out float[]? shared))
            {
                Array.Copy(shared, _offsetByMeter, _offsetByMeter.Length);
                _track = track;
                _requestedOffsetMeters = requestedOffsetMeters;
                _vehicleHalfWidthMeters = vehicleHalfWidthMeters;
                return;
            }
        }

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

        // Kept as a copy: the working table is rewritten in place by the next
        // build, and a stored profile has to stay the answer it was. Two
        // circuits being driven at once would otherwise file one's answers
        // under the other's name, and the build is left outside the lock
        // because two cars racing to compute the same table is only wasteful,
        // while holding a lock across it would make every other car wait.
        lock (SharedProfiles)
        {
            if (ReferenceEquals(track, _sharedProfilesTrack))
                SharedProfiles[key] = (float[])_offsetByMeter.Clone();
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
        return SampleProfile(track, _offsetByMeter, s);
    }

    private static float SampleProfile(
        TrackData track,
        float[] profile,
        float s
    )
    {
        float scaled = track.WrapS(s) / TrackData.StepLength;
        int index = (int)MathF.Floor(scaled);
        float t = scaled - index;
        int nextIndex = (index + 1) % profile.Length;
        return profile[index] +
               (profile[nextIndex] - profile[index]) * t;
    }
}
