using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers.Learned;

/// <summary>
/// Layout of the direct-drive observation. Everything is expressed in the
/// ego frame or as local road properties, and nothing identifies the track:
/// the same physical situation on any circuit must produce the same vector.
/// Forbidden by construction: absolute s, global coordinates, track length,
/// lap counts, and the racing line's geometry — the road itself is given,
/// the line is the policy's own problem.
///
/// Two rules the blocks below are built to keep. The road is given as
/// geometry rather than as extracted quantities, because a curvature scalar
/// is a lossy compression that silently drops everything the compressor was
/// not thinking about — which is how gradient, banking, and run-off came to
/// be missing from a car expected to race on gradient, banking, and run-off.
/// And every quantity that belongs to the car rather than the road is
/// dimensionless, a fraction of that car's own envelope, so that a policy
/// is not silently learning one particular car's newtons.
/// </summary>
public static class DirectDriveObservation
{
    /// <summary>
    /// How far ahead the road is drawn, in seconds of travel rather than
    /// metres. A fixed metre array cannot serve both ends of the speed
    /// range: at twenty metres a second its far points are half a minute
    /// away and mean nothing, and at sixty they arrive later than the
    /// braking distance they were supposed to warn about. Scaling by speed
    /// keeps each slot's meaning fixed — slot twelve is always three
    /// seconds off, whatever the car is doing — which is also what stops
    /// the input distribution from sliding as the policy gets faster.
    /// </summary>
    public const float PreviewHorizonSeconds = 6f;

    /// <summary>
    /// Floor on that horizon, so a car crawling out of a spin still sees
    /// far enough to aim at something.
    /// </summary>
    public const float MinimumPreviewMeters = 60f;

    public const int GeometryPointCount = 18;

    /// <summary>
    /// Three points across the road — left edge, centre, right edge — each
    /// with a height, plus the run-off beyond each edge.
    ///
    /// Three points is not a sample of the cross section, it is the whole
    /// of it: the section is <c>z(d) = z0 + BankSlope*d + BankCurvature*d^2</c>,
    /// three coefficients, and three heights at known offsets determine
    /// them exactly. Nothing is lost by giving points instead of the
    /// coefficients, and what is gained is that gradient, banking, camber
    /// and width all arrive through the one channel the network is already
    /// learning to read.
    /// </summary>
    public const int GeometryFloatsPerPoint = 11;

    /// <summary>
    /// Four tyres of surface temperature, core temperature, wear and load,
    /// then the battery's charge. The load is here because Sony's feature
    /// list has it — "load on each tyre" — and because without it the
    /// friction circle a corner is spending is not something the policy can
    /// work out: grip is load times a coefficient, and it was being shown
    /// neither.
    /// </summary>
    public const int TireAndBatterySize = 17;

    /// <summary>
    /// Only the overtake assist. The tyre and battery mode ordinals used to
    /// live here; both are gone. The battery mode is enforced by the
    /// hardware and already folded into the actuator range the policy's
    /// output is stretched onto, so it cannot be disobeyed and need not be
    /// seen. The tyre mode was a five-value index into the one number it
    /// stands for, and that number is now given directly in
    /// <see cref="RoadAndLimitsSize"/>.
    /// </summary>
    public const int ModeSize = 1;

    public const int AeroSize = 3;

    /// <summary>
    /// What the road is doing under this car and what the car can do about
    /// it — thirteen numbers that were all being asked for and none of which
    /// were being supplied.
    ///
    /// Four boundaries: how much room is left to each wall, and how much of
    /// that room is run-off rather than track. A policy that cannot tell
    /// twenty metres of tarmac from a barrier at the white line has no way
    /// to know where running wide is cheap, and ends up pacing itself by
    /// track width, which is what ours was doing.
    ///
    /// Three road: gradient, the cross slope actually under the car at its
    /// own lateral offset, and the vertical bend. These are exactly the
    /// three the physics takes, and the third cannot be recovered from the
    /// preview at all, because the crest that unloads the car is between
    /// the car and the first preview station.
    ///
    /// Five limits and strategy: the drive ceiling as a fraction of this
    /// car's peak, the grip allowance the pit wall set, and the friction
    /// circle each axle is actually using. The mode-excess penalty is
    /// <c>max(frontUse, rearUse) - allowance</c>; both of its operands were
    /// invisible, which is no way to be marked.
    ///
    /// Two indicators: whether the car is against a barrier right now,
    /// since it is charged for the seconds it spends there, and whether
    /// it is off the racing surface, since that is charged too. Both are
    /// on Sony's list. The second is derivable from the edge distances
    /// going negative, but a threshold the network has to discover is a
    /// threshold it can discover late.
    /// </summary>
    public const int RoadAndLimitsSize = 13;

    /// <summary>
    /// The car's own motion, where it sits on the road, what it last asked
    /// for — and two things the tyre model knows that nothing else in here
    /// implies. How badly the rear axle is sliding is not the body's
    /// sideslip angle and cannot be had from it; it comes out of the axle
    /// over-limits and the lateral each end actually delivered. And how much
    /// drive the traction control just took away is buried in the gap
    /// between the acceleration asked for and the one that arrived, along
    /// with drag and grip, with no way to separate the three.
    ///
    /// These stand in for the per-tyre slip angles on Sony's list, which
    /// this physics cannot produce because it resolves forces at the axle.
    /// </summary>
    public const int EgoSize = 14;
    public const int OpponentCount = 6;
    public const int OpponentSize = 16;

    public const int GeometryOffset = 0;
    public const int TireAndBatteryOffset =
        GeometryOffset + GeometryPointCount * GeometryFloatsPerPoint;
    public const int ModeOffset = TireAndBatteryOffset + TireAndBatterySize;
    public const int AeroOffset = ModeOffset + ModeSize;
    public const int RoadAndLimitsOffset = AeroOffset + AeroSize;

    /// <summary>
    /// Ego and opponents sit last and adjacent, because they are the two
    /// blocks the previous frame is kept for and the copy is one contiguous
    /// slice. When they were separated by the tyre, mode and aero blocks
    /// that slice ran off the end of the opponents and the last car and a
    /// bit was silently missing from every previous frame.
    /// </summary>
    public const int EgoOffset = RoadAndLimitsOffset + RoadAndLimitsSize;
    public const int OpponentOffset = EgoOffset + EgoSize;
    public const int DynamicBlockOffset = EgoOffset;
    public const int DynamicBlockSize =
        EgoSize + OpponentCount * OpponentSize;
    public const int PreviousDynamicOffset =
        OpponentOffset + OpponentCount * OpponentSize;
    public const int ObservationSize =
        PreviousDynamicOffset + DynamicBlockSize;

    public const int ActionSize = 2;

    internal const float DistanceScale = 400f;
    internal const float LateralScale = 30f;
    internal const float HalfWidthScale = 12f;

    /// <summary>
    /// Climb along the preview, which over six seconds at racing speed is
    /// tens of metres on a circuit with any relief at all.
    /// </summary>
    internal const float HeightScale = 20f;

    /// <summary>
    /// How much higher an edge sits than the centre — at most the half
    /// width times the bank, so a couple of metres even at Daytona. Kept on
    /// its own scale rather than sharing the climb's, because the banking
    /// is otherwise the small difference between two large numbers and a
    /// network reading it that way is reading noise.
    /// </summary>
    internal const float CrossHeightScale = 3f;

    internal const float BufferScale = 20f;
    internal const float SpeedScale = 100f;
    internal const float AccelerationScale = 20f;
    internal const float YawRateScale = 2f;
    internal const float SideslipScale = 0.5f;

    /// <summary>
    /// Rear slide severity has no upper bound of its own — it is a max over
    /// axle over-limits and delivery imbalances, and full lock at full
    /// throttle takes it past six. Two puts everyday sliding under one and
    /// leaves the extremes where the rest of this vector's extremes are.
    /// </summary>
    internal const float RearSlideScale = 2f;
    internal const float SlopeScale = 0.4f;
    internal const float VerticalRateScale = 0.02f;
    internal const float TemperatureScale = 150f;
    internal const float RelativeLongitudinalScale = 100f;
    internal const float RelativeLateralScale = 20f;
    internal const float RelativeSpeedScale = 50f;
    internal const float AlongsideBodyMeters = 4.8f;
}

/// <summary>
/// What the driver has worked out about its own car this tick and the
/// observation cannot work out for itself.
/// </summary>
internal readonly record struct DirectDriveCarLimits(
    float DriveCeilingFraction,
    float GripAllowance
);

/// <summary>
/// Builds direct-drive observations and keeps the one-tick memory that
/// provides frame-difference information for the dynamic blocks.
/// </summary>
public sealed class DirectDriveObservationBuilder
{
    private readonly float[] _previousDynamic =
        new float[DirectDriveObservation.DynamicBlockSize];
    private bool _hasPrevious;

    public void Reset()
    {
        _hasPrevious = false;
    }

    internal void Build(
        in RaceDriverFrameContext context,
        in DirectDriveCarLimits limits,
        float lastCurvatureNorm,
        float lastAccelerationNorm,
        Span<float> observation
    )
    {
        if (observation.Length < DirectDriveObservation.ObservationSize)
        {
            throw new ArgumentException(
                "Observation span is smaller than the layout.",
                nameof(observation)
            );
        }
        observation[..DirectDriveObservation.ObservationSize].Clear();

        RaceCar car = context.Car;
        CarState state = car.State;
        TrackData track = context.Track;
        TrackPose pose = context.Pose;
        Vector2 forward = new(
            MathF.Cos(state.Heading),
            MathF.Sin(state.Heading)
        );
        Vector2 left = new(-forward.Y, forward.X);

        WriteGeometry(
            observation,
            track,
            pose,
            state.Position,
            state.Speed,
            forward,
            left
        );
        WriteTiresAndBattery(observation, car.CarConfig, state);
        WriteModes(observation, state);
        WriteAero(observation, state);
        WriteRoadAndLimits(observation, car, state, pose, in limits);
        WriteEgo(
            observation,
            state,
            pose,
            lastCurvatureNorm,
            lastAccelerationNorm
        );
        WriteOpponents(
            observation,
            in context,
            state.Position,
            forward,
            left
        );

        Span<float> currentDynamic = observation.Slice(
            DirectDriveObservation.DynamicBlockOffset,
            DirectDriveObservation.DynamicBlockSize
        );
        Span<float> previousDynamic = observation.Slice(
            DirectDriveObservation.PreviousDynamicOffset,
            DirectDriveObservation.DynamicBlockSize
        );
        if (_hasPrevious)
            _previousDynamic.CopyTo(previousDynamic);
        else
            currentDynamic.CopyTo(previousDynamic);
        currentDynamic.CopyTo(_previousDynamic);
        _hasPrevious = true;
    }

    /// <summary>
    /// How far ahead this car is shown the road, given how fast it is
    /// going.
    /// </summary>
    public static float PreviewHorizonMeters(float speed) => MathF.Max(
        DirectDriveObservation.MinimumPreviewMeters,
        speed * DirectDriveObservation.PreviewHorizonSeconds
    );

    private static void WriteGeometry(
        Span<float> observation,
        TrackData track,
        TrackPose pose,
        Vector2 egoPosition,
        float speed,
        Vector2 forward,
        Vector2 left
    )
    {
        float horizon = PreviewHorizonMeters(speed);
        float spacing = horizon / DirectDriveObservation.GeometryPointCount;

        // Height is not carried by a track sample, only the slope is, so the
        // preview climbs by integrating it. The stations are evenly spaced,
        // so a trapezoid between consecutive slopes is exact for a road whose
        // gradient is piecewise linear, which is what the interpolant makes.
        float height = 0f;
        float previousGrade = pose.Sample.Grade;

        int cursor = DirectDriveObservation.GeometryOffset;
        for (int i = 0; i < DirectDriveObservation.GeometryPointCount; i++)
        {
            float distance = spacing * (i + 1);
            TrackSample sample = track.Sample(pose.S + distance);
            height += 0.5f * (previousGrade + sample.Grade) * spacing;
            previousGrade = sample.Grade;

            float halfWidth = sample.HalfWidth;
            // z(d) = z0 + BankSlope*d + BankCurvature*d^2, with the left edge
            // at d = +halfWidth because the normal points left.
            float camber = sample.BankCurvature * halfWidth * halfWidth;
            float lean = sample.BankSlope * halfWidth;

            WritePoint(
                observation,
                ref cursor,
                sample.LeftEdge - egoPosition,
                (lean + camber) / DirectDriveObservation.CrossHeightScale,
                forward,
                left
            );
            WritePoint(
                observation,
                ref cursor,
                sample.Center - egoPosition,
                height / DirectDriveObservation.HeightScale,
                forward,
                left
            );
            WritePoint(
                observation,
                ref cursor,
                sample.RightEdge - egoPosition,
                (camber - lean) / DirectDriveObservation.CrossHeightScale,
                forward,
                left
            );

            observation[cursor++] = sample.LeftBufferWidth /
                                    DirectDriveObservation.BufferScale;
            observation[cursor++] = sample.RightBufferWidth /
                                    DirectDriveObservation.BufferScale;
        }
    }

    /// <summary>
    /// One preview point in the ego frame. The two edges carry their height
    /// relative to the centre and the centre carries the climb, which is
    /// the same three numbers as three absolute heights and is the pair of
    /// scales on which both stay readable.
    /// </summary>
    private static void WritePoint(
        Span<float> observation,
        ref int cursor,
        Vector2 delta,
        float scaledHeight,
        Vector2 forward,
        Vector2 left
    )
    {
        observation[cursor++] = Vector2.Dot(delta, forward) /
                                DirectDriveObservation.DistanceScale;
        observation[cursor++] = Vector2.Dot(delta, left) /
                                DirectDriveObservation.LateralScale;
        observation[cursor++] = scaledHeight;
    }

    private static void WriteRoadAndLimits(
        Span<float> observation,
        RaceCar car,
        CarState state,
        TrackPose pose,
        in DirectDriveCarLimits limits
    )
    {
        TrackSample sample = pose.Sample;
        float halfWidth = sample.HalfWidth;
        CarTelemetry telemetry = state.Telemetry;
        int cursor = DirectDriveObservation.RoadAndLimitsOffset;

        observation[cursor++] = (halfWidth - pose.D + sample.LeftBufferWidth) /
                                DirectDriveObservation.BufferScale;
        observation[cursor++] = (halfWidth + pose.D + sample.RightBufferWidth) /
                                DirectDriveObservation.BufferScale;
        observation[cursor++] = sample.LeftBufferWidth /
                                DirectDriveObservation.BufferScale;
        observation[cursor++] = sample.RightBufferWidth /
                                DirectDriveObservation.BufferScale;

        observation[cursor++] = sample.Grade /
                                DirectDriveObservation.SlopeScale;
        observation[cursor++] = sample.BankSlopeAt(pose.D) /
                                DirectDriveObservation.SlopeScale;
        observation[cursor++] = sample.VerticalRate /
                                DirectDriveObservation.VerticalRateScale;

        observation[cursor++] = limits.DriveCeilingFraction;
        observation[cursor++] = limits.GripAllowance;
        observation[cursor++] = CombinedUse(
            telemetry.FrontLateralUse,
            telemetry.FrontLongitudinalUse
        );
        observation[cursor++] = CombinedUse(
            telemetry.RearLateralUse,
            telemetry.RearLongitudinalUse
        );
        observation[cursor++] = car.BoundaryContactSeconds > 0f ? 1f : 0f;
        observation[cursor] =
            TrackBoundaryResolver.Classify(pose) == TrackRegion.RacingSurface
                ? 0f
                : 1f;
    }

    private static float CombinedUse(float lateral, float longitudinal) =>
        MathF.Min(
            1f,
            MathF.Sqrt(lateral * lateral + longitudinal * longitudinal)
        );

    private static void WriteEgo(
        Span<float> observation,
        CarState state,
        TrackPose pose,
        float lastCurvatureNorm,
        float lastAccelerationNorm
    )
    {
        float roadHeading = MathF.Atan2(
            pose.Sample.Tangent.Y,
            pose.Sample.Tangent.X
        );
        float headingError = MathHelperNormalize(
            state.Heading - roadHeading
        );
        float halfWidth = pose.Sample.HalfWidth;
        int cursor = DirectDriveObservation.EgoOffset;
        observation[cursor++] = state.Speed /
                                DirectDriveObservation.SpeedScale;
        observation[cursor++] = state.FilteredLongitudinalAccel /
                                DirectDriveObservation.AccelerationScale;
        observation[cursor++] = state.FilteredLateralAccel /
                                DirectDriveObservation.AccelerationScale;
        observation[cursor++] = state.YawRateRadiansPerSecond /
                                DirectDriveObservation.YawRateScale;
        observation[cursor++] = state.SideslipAngleRadians /
                                DirectDriveObservation.SideslipScale;
        observation[cursor++] = MathF.Sin(headingError);
        observation[cursor++] = MathF.Cos(headingError);
        observation[cursor++] = pose.D /
                                DirectDriveObservation.HalfWidthScale;
        observation[cursor++] = (halfWidth - pose.D) /
                                DirectDriveObservation.HalfWidthScale;
        observation[cursor++] = (halfWidth + pose.D) /
                                DirectDriveObservation.HalfWidthScale;
        observation[cursor++] = lastCurvatureNorm;
        observation[cursor++] = lastAccelerationNorm;
        observation[cursor++] = state.Telemetry.RearSlideSeverity /
                                DirectDriveObservation.RearSlideScale;
        observation[cursor] = state.Telemetry.TractionControlCutAccel /
                              DirectDriveObservation.AccelerationScale;
    }

    private static void WriteTiresAndBattery(
        Span<float> observation,
        CarConfig config,
        CarState state
    )
    {
        // A quarter of the car's own weight, so the number reads as one at
        // rest on any car rather than as this car's newtons.
        float staticCornerLoad = MathF.Max(
            config.MassKg * 9.81f * 0.25f,
            1f
        );
        int cursor = DirectDriveObservation.TireAndBatteryOffset;
        WriteTire(observation, ref cursor, state.FrontLeft, staticCornerLoad);
        WriteTire(observation, ref cursor, state.FrontRight, staticCornerLoad);
        WriteTire(observation, ref cursor, state.RearLeft, staticCornerLoad);
        WriteTire(observation, ref cursor, state.RearRight, staticCornerLoad);
        observation[cursor] = state.BatterySoc;
    }

    private static void WriteTire(
        Span<float> observation,
        ref int cursor,
        in TireState tire,
        float staticCornerLoad
    )
    {
        observation[cursor++] = tire.SurfaceTempC /
                                DirectDriveObservation.TemperatureScale;
        observation[cursor++] = tire.CoreTempC /
                                DirectDriveObservation.TemperatureScale;
        observation[cursor++] = tire.Wear;
        observation[cursor++] = tire.LoadN / staticCornerLoad;
    }

    private static void WriteModes(Span<float> observation, CarState state)
    {
        observation[DirectDriveObservation.ModeOffset] = state.OvertakeAssist;
    }

    private static void WriteAero(Span<float> observation, CarState state)
    {
        int cursor = DirectDriveObservation.AeroOffset;
        observation[cursor++] = state.AirVelocityDeficit;
        observation[cursor++] = state.DownforceVelocityDeficit;
        observation[cursor] = state.WakeDownforceLoss;
    }

    private static void WriteOpponents(
        Span<float> observation,
        in RaceDriverFrameContext context,
        Vector2 egoPosition,
        Vector2 forward,
        Vector2 left
    )
    {
        if (!context.HasFrameSnapshot)
            return;

        RaceFrameSnapshot frame = context.Frame;
        float trackLength = context.Track.LengthMeters;
        float egoTrackS = context.Pose.S;
        Span<int> selected = stackalloc int[
            DirectDriveObservation.OpponentCount
        ];
        Span<float> selectedDistance = stackalloc float[
            DirectDriveObservation.OpponentCount
        ];
        int selectedCount = 0;
        for (int i = 0; i < frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            float along = WrapSignedDelta(
                frame[i].TrackS - egoTrackS,
                trackLength
            );
            InsertNearest(
                selected,
                selectedDistance,
                ref selectedCount,
                i,
                along
            );
        }

        Vector2 egoVelocity = context.Car.State.Velocity;
        for (int slot = 0; slot < selectedCount; slot++)
        {
            RaceCarSnapshot opponent = frame[selected[slot]];
            int cursor = DirectDriveObservation.OpponentOffset +
                         slot * DirectDriveObservation.OpponentSize;
            Vector2 delta = opponent.Position - egoPosition;
            Vector2 relativeVelocity = opponent.Velocity - egoVelocity;
            float headingDelta = MathHelperNormalize(
                opponent.HeadingRadians - context.Car.State.Heading
            );
            float along = selectedDistance[slot];
            float lateralGap = opponent.TrackD - context.Pose.D;

            observation[cursor++] = 1f;
            observation[cursor++] = Vector2.Dot(delta, forward) /
                DirectDriveObservation.RelativeLongitudinalScale;
            observation[cursor++] = Vector2.Dot(delta, left) /
                DirectDriveObservation.RelativeLateralScale;
            observation[cursor++] = Vector2.Dot(relativeVelocity, forward) /
                DirectDriveObservation.RelativeSpeedScale;
            observation[cursor++] = Vector2.Dot(relativeVelocity, left) /
                DirectDriveObservation.RelativeSpeedScale;
            observation[cursor++] = MathF.Sin(headingDelta);
            observation[cursor++] = MathF.Cos(headingDelta);
            observation[cursor++] = opponent.SpeedMetersPerSecond /
                DirectDriveObservation.SpeedScale;
            observation[cursor++] = opponent.TrackD /
                DirectDriveObservation.HalfWidthScale;
            observation[cursor++] = Math.Clamp(
                along / DirectDriveObservation.RelativeLongitudinalScale,
                -1f,
                1f
            );
            observation[cursor++] = lateralGap /
                DirectDriveObservation.HalfWidthScale;
            observation[cursor++] = MathF.Abs(along) <=
                DirectDriveObservation.AlongsideBodyMeters ? 1f : 0f;
            WritePlanFuture(
                observation,
                ref cursor,
                frame,
                selected[slot],
                egoPosition,
                forward,
                left,
                1f
            );
            WritePlanFuture(
                observation,
                ref cursor,
                frame,
                selected[slot],
                egoPosition,
                forward,
                left,
                2f
            );
        }
    }

    private static void WritePlanFuture(
        Span<float> observation,
        ref int cursor,
        in RaceFrameSnapshot frame,
        int opponentIndex,
        Vector2 egoPosition,
        Vector2 forward,
        Vector2 left,
        float horizonSeconds
    )
    {
        TrafficMotionPlan? plan = frame.GetTrafficMotionPlan(opponentIndex);
        if (plan is not null &&
            plan.TrySample(horizonSeconds, out TrafficMotionPlanPoint point))
        {
            Vector2 delta = point.Position - egoPosition;
            observation[cursor++] = Vector2.Dot(delta, forward) /
                DirectDriveObservation.RelativeLongitudinalScale;
            observation[cursor++] = Vector2.Dot(delta, left) /
                DirectDriveObservation.RelativeLateralScale;
        }
        else
        {
            observation[cursor++] = 0f;
            observation[cursor++] = 0f;
        }
    }

    private static void InsertNearest(
        Span<int> selected,
        Span<float> selectedDistance,
        ref int selectedCount,
        int candidate,
        float along
    )
    {
        float magnitude = MathF.Abs(along);
        int position = selectedCount;
        if (selectedCount < selected.Length)
            selectedCount++;
        else if (magnitude >= MathF.Abs(selectedDistance[^1]))
            return;
        else
            position = selected.Length - 1;

        while (position > 0 &&
               MathF.Abs(selectedDistance[position - 1]) > magnitude)
        {
            selected[position] = selected[position - 1];
            selectedDistance[position] = selectedDistance[position - 1];
            position--;
        }
        selected[position] = candidate;
        selectedDistance[position] = along;
    }

    private static float WrapSignedDelta(float delta, float length)
    {
        if (length <= 0f)
            return delta;
        delta %= length;
        if (delta > length * 0.5f)
            delta -= length;
        else if (delta < -length * 0.5f)
            delta += length;
        return delta;
    }

    private static float MathHelperNormalize(float angle)
    {
        while (angle > MathF.PI)
            angle -= 2f * MathF.PI;
        while (angle < -MathF.PI)
            angle += 2f * MathF.PI;
        return angle;
    }
}
