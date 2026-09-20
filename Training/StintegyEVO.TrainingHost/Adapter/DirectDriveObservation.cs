using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.TrainingHost.Adapter;

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
    /// <remarks>
    /// <b>Legacy.</b> This block describes the car's parts rather than its
    /// condition — four wheels of device readings — and its battery
    /// channel now says the same thing the resource slots say in the
    /// contract's own terms. It stays because redundant information is
    /// harmless and every change of dimension voids a generation of
    /// checkpoints, so clearing it is worth doing only alongside a change
    /// that had to happen anyway. The next such change should take it.
    /// </remarks>
    public const int TireAndBatterySize = 17;

    /// <summary>
    /// Only drag reduction (the device formerly named overtake assist). The tyre and battery mode ordinals used to
    /// live here; both are gone. The battery mode is enforced by the
    /// hardware and already folded into the actuator range the policy's
    /// output is stretched onto, so it cannot be disobeyed and need not be
    /// seen. The tyre mode was a five-value index into the one number it
    /// stands for, and that number is now given directly in
    /// <see cref="RoadAndLimitsSize"/>.
    /// </summary>
    /// <remarks>
    /// <b>Legacy.</b> One channel of overtake assist, tied to a mode
    /// ladder whose number of rungs is a property of the machinery. It
    /// goes with <see cref="TireAndBatterySize"/> at the next dimension
    /// change.
    /// </remarks>
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
    /// the combined-grip limiter just took away is buried in the gap
    /// between the acceleration asked for and the one that arrived, along
    /// with drag and grip, with no way to separate the three.
    ///
    /// These stand in for the per-tyre slip angles on Sony's list, which
    /// this physics cannot produce because it resolves forces at the axle.
    /// </summary>
    public const int EgoSize = 14;

    /// <summary>
    /// The stores the car carries and where the pit wall wants them, the
    /// world-v3 contract (PLAN §3.4 items 1 and 2, freeze design 1.2).
    ///
    /// Two resource-slot pairs, each (remaining fraction, capacity): the
    /// state as a fraction of the store, the scale as an absolute capacity
    /// over a world constant, <see cref="ReferenceStoreJoules"/>. A slot the
    /// car does not have reads capacity zero, which is how absence is
    /// written (iron rule four): on the static quantity, never on a reading
    /// that can legitimately fall to zero. A battery car's second slot is
    /// therefore (0, 0) for ever, and a hybrid fills it.
    ///
    /// Then the budget deviation: remaining charge minus the pit wall's
    /// target line at this point of the race, zero when the instruction
    /// carries no line. It is what the budget shaping charges for, and a
    /// charge the driver cannot see is the mode-excess mistake again.
    ///
    ///   0  slot 1 remaining   1  slot 1 capacity
    ///   2  slot 2 remaining   3  slot 2 capacity
    ///   4  budget deviation
    /// </summary>
    public const int ResourceSlotCount = 2;
    public const int ResourceAndBudgetSize = ResourceSlotCount * 2 + 1;

    /// <summary>
    /// The store capacity a capacity channel of one means: the reference
    /// car's pack, 1100 MJ.
    /// </summary>
    internal const float ReferenceStoreJoules = 1.1e9f;

    public const int OpponentCount = 6;
    public const int OpponentSize = 16;

    public const int GeometryOffset = 0;
    public const int TireAndBatteryOffset =
        GeometryOffset + GeometryPointCount * GeometryFloatsPerPoint;
    public const int ModeOffset = TireAndBatteryOffset + TireAndBatterySize;
    public const int AeroOffset = ModeOffset + ModeSize;
    public const int RoadAndLimitsOffset = AeroOffset + AeroSize;
    // Static blocks sit before ego, so that ego and the opponents stay
    // adjacent and last: they are the two the previous frame is kept for,
    // and that copy is one contiguous slice.
    public const int ResourceAndBudgetOffset =
        RoadAndLimitsOffset + RoadAndLimitsSize;

    /// <summary>
    /// Ego and opponents sit last and adjacent, because they are the two
    /// blocks the previous frame is kept for and the copy is one contiguous
    /// slice. When they were separated by the tyre, mode and aero blocks
    /// that slice ran off the end of the opponents and the last car and a
    /// bit was silently missing from every previous frame.
    /// </summary>
    public const int EgoOffset =
        ResourceAndBudgetOffset + ResourceAndBudgetSize;
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
/// Builds direct-drive observations from a frozen race frame and keeps the
/// one-tick memory that provides frame-difference information for the
/// dynamic blocks.
///
/// This is the adapter between the world's contract and the policy's
/// vector. Everything it reads is either in the frame the simulation froze
/// for every controller (<see cref="DriverContext"/>), the read-only track
/// view, or the car's published parameters handed in at construction. It
/// never touches a live car. The layout is the world-v2 layout, unchanged,
/// with two readings re-sourced:
///
/// <list type="bullet">
/// <item>the ego channel that used to carry the traction control's cut now
/// carries the combined-grip limiter's (TC was retired in favour of the
/// limiter on master);</item>
/// <item>the opponents' two planned-future points are always zero.</item>
/// </list>
///
/// <para>
/// The planned-future channels stay zero now that wheel-to-wheel is live,
/// and that is a ruling rather than an omission. What they carried were
/// traffic motion plans, which only the retired analytic driver published;
/// nothing on this contract publishes one, because a controller's plan is
/// its own. A sparring partner's intention is private in the same way a
/// rival driver's is, and what can honestly be known about it — where it
/// is, which way it points, how fast it is closing, whether it is
/// alongside — is already in the twelve channels above them, from which
/// the rest is the ego's to infer.
/// </para>
/// <para>
/// Keeping them zero also keeps the contract at 457 channels with every
/// block where it was, so a parent baked solo drops into the second car
/// without a conversion: the weights are plug-in, and a duel is a
/// different situation rather than a different observation.
/// </para>
/// </summary>
public sealed class DirectDriveObservationBuilder
{
    private readonly float[] _previousDynamic =
        new float[DirectDriveObservation.DynamicBlockSize];
    private readonly CarConfig _config;
    private bool _hasPrevious;

    public DirectDriveObservationBuilder(CarConfig config)
    {
        _config = config ?? throw new ArgumentNullException(nameof(config));
    }

    public void Reset()
    {
        _hasPrevious = false;
    }

    internal void Build(
        in DriverContext context,
        in DirectDriveCarLimits limits,
        float lastCurvatureNorm,
        float lastAccelerationNorm,
        float budgetDeviation,
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

        RaceCarSnapshot car = context.Car;
        DriverTrackView track = context.Track;
        TrackPose pose = track.Project(car.Position);
        Vector2 forward = new(
            MathF.Cos(car.HeadingRadians),
            MathF.Sin(car.HeadingRadians)
        );
        Vector2 left = new(-forward.Y, forward.X);

        WriteGeometry(
            observation,
            track,
            pose,
            car.Position,
            car.SpeedMetersPerSecond,
            forward,
            left
        );
        WriteTiresAndBattery(observation, _config, in car);
        WriteModes(observation, in car);
        WriteAero(observation, in car);
        WriteRoadAndLimits(observation, in car, pose, in limits);
        WriteResourcesAndBudget(observation, _config, in car, budgetDeviation);
        WriteEgo(
            observation,
            _config,
            in car,
            pose,
            lastCurvatureNorm,
            lastAccelerationNorm
        );
        WriteOpponents(
            observation,
            in context,
            pose,
            car.Position,
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
        DriverTrackView track,
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

            // Lateral offsets are scaled by the station's own nominal
            // distance down the road, not by a fixed 30 m: a far station on
            // the other side of a hairpin can sit most of a horizon to the
            // side, which read as 11 against a fixed scale. Divided by how
            // far along the road the station is, lateral becomes roughly the
            // tangent of its bearing, dimensionless at every distance. Near
            // stations keep the 30 m floor and their resolution. The
            // distance is the nominal arc length, fixed by the speed, which
            // the observation carries, so the metres can be recovered.
            float lateralScale = MathF.Max(DirectDriveObservation.LateralScale, distance);
            WritePoint(
                observation,
                ref cursor,
                sample.LeftEdge - egoPosition,
                (lean + camber) / DirectDriveObservation.CrossHeightScale,
                forward,
                left,
                lateralScale
            );
            WritePoint(
                observation,
                ref cursor,
                sample.Center - egoPosition,
                height / DirectDriveObservation.HeightScale,
                forward,
                left,
                lateralScale
            );
            WritePoint(
                observation,
                ref cursor,
                sample.RightEdge - egoPosition,
                (camber - lean) / DirectDriveObservation.CrossHeightScale,
                forward,
                left,
                lateralScale
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
        Vector2 left,
        float lateralScale
    )
    {
        observation[cursor++] = Vector2.Dot(delta, forward) /
                                DirectDriveObservation.DistanceScale;
        observation[cursor++] = Vector2.Dot(delta, left) / lateralScale;
        observation[cursor++] = scaledHeight;
    }

    private static void WriteRoadAndLimits(
        Span<float> observation,
        in RaceCarSnapshot car,
        TrackPose pose,
        in DirectDriveCarLimits limits
    )
    {
        TrackSample sample = pose.Sample;
        float halfWidth = sample.HalfWidth;
        CarTelemetry telemetry = car.Telemetry;
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
        CarConfig config,
        in RaceCarSnapshot car,
        TrackPose pose,
        float lastCurvatureNorm,
        float lastAccelerationNorm
    )
    {
        float roadHeading = MathF.Atan2(
            pose.Sample.Tangent.Y,
            pose.Sample.Tangent.X
        );
        float headingError = NormalizeAngle(car.HeadingRadians - roadHeading);
        float halfWidth = pose.Sample.HalfWidth;
        int cursor = DirectDriveObservation.EgoOffset;
        observation[cursor++] = car.SpeedMetersPerSecond /
                                DirectDriveObservation.SpeedScale;
        // The frame's accelerations are the physics' filtered readings, the
        // same quantities the world-v2 builder read off the live state.
        observation[cursor++] = car.LongitudinalAccelMetersPerSecondSquared /
                                DirectDriveObservation.AccelerationScale;
        observation[cursor++] = car.LateralAccelMetersPerSecondSquared /
                                DirectDriveObservation.AccelerationScale;
        observation[cursor++] = car.YawRateRadiansPerSecond /
                                DirectDriveObservation.YawRateScale;
        observation[cursor++] = car.SideslipAngleRadians /
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
        observation[cursor++] = car.Telemetry.RearSlideSeverity /
                                DirectDriveObservation.RearSlideScale;
        // The limiter's cut as a share of the car's full drive
        // acceleration, its published range (freeze design 1.1). Braking
        // cuts can exceed one; they are the rare hard trims and stay O(1).
        observation[cursor] = car.Telemetry.CombinedGripLimiterCutAccel /
                              MathF.Max(config.MaxDriveAcceleration, 1e-3f);
    }

    private static void WriteResourcesAndBudget(
        Span<float> observation,
        CarConfig config,
        in RaceCarSnapshot car,
        float budgetDeviation
    )
    {
        int cursor = DirectDriveObservation.ResourceAndBudgetOffset;
        var resources = car.Resources;
        for (int slot = 0; slot < DirectDriveObservation.ResourceSlotCount; slot++)
        {
            bool present = !resources.IsDefault && slot < resources.Length;
            observation[cursor++] = present ? resources[slot].Fraction : 0f;
            observation[cursor++] = present
                ? StoreCapacityJoules(config, slot) /
                  DirectDriveObservation.ReferenceStoreJoules
                : 0f;
        }
        observation[cursor] = budgetDeviation;
    }

    /// <summary>
    /// What a store holds when full. The electric powertrain publishes its
    /// pack; a store whose capacity the powertrain does not publish reads
    /// as the reference, present and unit-scaled, rather than as absent.
    /// </summary>
    private static float StoreCapacityJoules(CarConfig config, int slot) =>
        config.Powertrain is ElectricPowertrain electric && slot == 0
            ? electric.BatteryCapacityJoules
            : DirectDriveObservation.ReferenceStoreJoules;

    private static void WriteTiresAndBattery(
        Span<float> observation,
        CarConfig config,
        in RaceCarSnapshot car
    )
    {
        // A quarter of the car's own published weight, so the number reads
        // as one at rest on any car rather than as this car's newtons.
        float staticCornerLoad = MathF.Max(
            config.MassKg * 9.81f * 0.25f,
            1f
        );
        int cursor = DirectDriveObservation.TireAndBatteryOffset;
        WriteTire(observation, ref cursor, car.FrontLeft, staticCornerLoad);
        WriteTire(observation, ref cursor, car.FrontRight, staticCornerLoad);
        WriteTire(observation, ref cursor, car.RearLeft, staticCornerLoad);
        WriteTire(observation, ref cursor, car.RearRight, staticCornerLoad);
        // The primary store only. A car with two of them - a hybrid -
        // needs a second channel here, and adding one moves everything
        // after it, so it waits for a generation of policies to end
        // rather than quietly invalidating the current one.
        observation[cursor] =
            car.Resources.IsDefaultOrEmpty ? 0f : car.Resources[0].Fraction;
    }

    private static void WriteTire(
        Span<float> observation,
        ref int cursor,
        in TireSnapshot tire,
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

    private static void WriteModes(Span<float> observation, in RaceCarSnapshot car)
    {
        observation[DirectDriveObservation.ModeOffset] = car.DragReduction;
    }

    private static void WriteAero(Span<float> observation, in RaceCarSnapshot car)
    {
        int cursor = DirectDriveObservation.AeroOffset;
        observation[cursor++] = car.AirVelocityDeficit;
        observation[cursor++] = car.DownforceVelocityDeficit;
        observation[cursor] = car.WakeDownforceLoss;
    }

    private static void WriteOpponents(
        Span<float> observation,
        in DriverContext context,
        TrackPose egoPose,
        Vector2 egoPosition,
        Vector2 forward,
        Vector2 left
    )
    {
        RaceFrameSnapshot frame = context.Frame;
        float trackLength = context.Track.LengthMeters;
        float egoTrackS = egoPose.S;
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

        RaceCarSnapshot ego = context.Car;
        Vector2 egoVelocity = ego.Velocity;
        for (int slot = 0; slot < selectedCount; slot++)
        {
            RaceCarSnapshot opponent = frame[selected[slot]];
            int cursor = DirectDriveObservation.OpponentOffset +
                         slot * DirectDriveObservation.OpponentSize;
            Vector2 delta = opponent.Position - egoPosition;
            Vector2 relativeVelocity = opponent.Velocity - egoVelocity;
            float headingDelta = NormalizeAngle(
                opponent.HeadingRadians - ego.HeadingRadians
            );
            float along = selectedDistance[slot];
            float lateralGap = opponent.TrackD - egoPose.D;

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
            // The last four channels were two points of the opponent's
            // planned future, sampled from traffic motion plans that only
            // the retired analytic driver published. They stay zero against
            // a learned sparring partner too, by ruling: what a driver
            // means to do next is private, and the motion above is what a
            // real one reads it from (see the class remarks).
            observation[cursor++] = 0f;
            observation[cursor++] = 0f;
            observation[cursor++] = 0f;
            observation[cursor] = 0f;
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

    private static float NormalizeAngle(float angle)
    {
        while (angle > MathF.PI)
            angle -= 2f * MathF.PI;
        while (angle < -MathF.PI)
            angle += 2f * MathF.PI;
        return angle;
    }
}
