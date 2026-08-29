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
/// the line is the policy's own problem. The analytic planner appears only
/// as the coach block: a suggestion the policy may copy or ignore.
/// </summary>
public static class DirectDriveObservation
{
    /// <summary>
    /// Log-spaced preview arc lengths in meters: dense where the next
    /// second lives, sparse toward the planning horizon.
    /// </summary>
    public static readonly float[] PreviewDistancesMeters =
    [
        2f, 5f, 9f, 14f, 21f, 30f, 42f, 58f, 80f, 110f,
        150f, 200f, 260f, 330f, 410f, 500f, 600f
    ];

    public const int GeometryPointCount = 17;
    public const int GeometryFloatsPerPoint = 4;
    public const int EgoSize = 12;
    public const int TireAndBatterySize = 13;
    public const int ModeSize = 3;
    public const int AeroSize = 3;
    public const int OpponentCount = 6;
    public const int OpponentSize = 16;

    public const int GeometryOffset = 0;
    public const int EgoOffset =
        GeometryOffset + GeometryPointCount * GeometryFloatsPerPoint;
    public const int TireAndBatteryOffset = EgoOffset + EgoSize;
    public const int ModeOffset = TireAndBatteryOffset + TireAndBatterySize;
    public const int AeroOffset = ModeOffset + ModeSize;
    public const int OpponentOffset = AeroOffset + AeroSize;
    public const int DynamicBlockOffset = EgoOffset;
    public const int DynamicBlockSize =
        EgoSize + OpponentCount * OpponentSize;
    public const int PreviousDynamicOffset =
        OpponentOffset + OpponentCount * OpponentSize;
    public const int ObservationSize =
        PreviousDynamicOffset + DynamicBlockSize;

    public const int ActionSize = 2;

    internal const float DistanceScale = 600f;
    internal const float LateralScale = 30f;
    internal const float HalfWidthScale = 12f;
    internal const float CurvatureScale = 20f;
    internal const float SpeedScale = 100f;
    internal const float AccelerationScale = 20f;
    internal const float YawRateScale = 2f;
    internal const float SideslipScale = 0.5f;
    internal const float TemperatureScale = 150f;
    internal const float RelativeLongitudinalScale = 100f;
    internal const float RelativeLateralScale = 20f;
    internal const float RelativeSpeedScale = 50f;
    internal const float AlongsideBodyMeters = 4.8f;
}

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

        WriteGeometry(observation, track, pose, state.Position, forward, left);
        WriteEgo(
            observation,
            state,
            pose,
            lastCurvatureNorm,
            lastAccelerationNorm
        );
        WriteTiresAndBattery(observation, state);
        WriteModes(observation, car, state);
        WriteAero(observation, state);
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

    private static void WriteGeometry(
        Span<float> observation,
        TrackData track,
        TrackPose pose,
        Vector2 egoPosition,
        Vector2 forward,
        Vector2 left
    )
    {
        int cursor = DirectDriveObservation.GeometryOffset;
        for (int i = 0; i < DirectDriveObservation.GeometryPointCount; i++)
        {
            float distance = DirectDriveObservation.PreviewDistancesMeters[i];
            TrackSample sample = track.Sample(pose.S + distance);
            Vector2 delta = sample.Center - egoPosition;
            observation[cursor++] = Vector2.Dot(delta, forward) /
                                    DirectDriveObservation.DistanceScale;
            observation[cursor++] = Vector2.Dot(delta, left) /
                                    DirectDriveObservation.LateralScale;
            observation[cursor++] = sample.HalfWidth /
                                    DirectDriveObservation.HalfWidthScale;
            observation[cursor++] = sample.RefCurvature *
                                    DirectDriveObservation.CurvatureScale;
        }
    }

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
        observation[cursor] = lastAccelerationNorm;
    }

    private static void WriteTiresAndBattery(
        Span<float> observation,
        CarState state
    )
    {
        int cursor = DirectDriveObservation.TireAndBatteryOffset;
        WriteTire(observation, ref cursor, state.FrontLeft);
        WriteTire(observation, ref cursor, state.FrontRight);
        WriteTire(observation, ref cursor, state.RearLeft);
        WriteTire(observation, ref cursor, state.RearRight);
        observation[cursor] = state.BatterySoc;
    }

    private static void WriteTire(
        Span<float> observation,
        ref int cursor,
        in TireState tire
    )
    {
        observation[cursor++] = tire.SurfaceTempC /
                                DirectDriveObservation.TemperatureScale;
        observation[cursor++] = tire.CoreTempC /
                                DirectDriveObservation.TemperatureScale;
        observation[cursor++] = tire.Wear;
    }

    private static void WriteModes(
        Span<float> observation,
        RaceCar car,
        CarState state
    )
    {
        int cursor = DirectDriveObservation.ModeOffset;
        observation[cursor++] = ((int)car.Strategy.TireMode - 1) / 4f;
        observation[cursor++] = ((int)car.Strategy.BatteryMode - 1) / 4f;
        observation[cursor] = state.OvertakeAssist;
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
