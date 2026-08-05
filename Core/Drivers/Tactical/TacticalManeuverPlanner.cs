using System;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Decides where on the road the car wants to be, and hands that out as a
/// single offset from the racing line.
///
/// Two halves, and keeping them apart is the whole design.
///
/// The calculation answers only what a car is capable of, and never starts
/// anything: how long the run to the next corner takes at this car's pace, how
/// long it takes at the pace of the car in front, and therefore whether the
/// ground between them can be made up before the corner arrives. Both figures
/// come from the same speed planner the cars are steering to, so worn tyres, a
/// flat battery and a tow are already in them. Nothing is measured and so
/// nothing is noisy.
///
/// The rules answer what a driver does with that, and they are ordinary
/// racecraft:
///
///   1. Your rival is the car immediately in front. Being quicker than someone
///      four places up is not a reason to do anything.
///   2. If you cannot afford the next place, queue. Sit in the tow on the
///      racing line and wait. This is most of a race and it is not a failure.
///   3. Pull out at the braking zone, not before. A straight is for closing up,
///      not for driving beside somebody.
///   4. If the room beside them is taken, queue instead. Two abreast, not three.
///   5. The corner settles it. Alongside by the apex and the place is yours;
///      not alongside and you drop back in rather than fight through it.
///
/// The rules are what stops a grid of twenty deciding to overtake at once. On
/// the run to the first corner every car is as quick as the one in front, so
/// rule 2 puts the entire field in a queue, which is what a start looks like.
///
/// What this replaces judged an attempt by watching whether the gap was
/// closing. In a queue every car is going the same speed, so the true change is
/// zero and its sign is the noise on the measurement: measured, a coin toss
/// called off ninety six attempts in a hundred.
/// </summary>
internal sealed class TacticalManeuverPlanner
{
    /// <summary>
    /// How far ahead a car still counts as the one being followed. Beyond this
    /// there is nothing to plan a move on: the road between will have changed
    /// before it is reached.
    /// </summary>
    private const float RivalReachMeters = 200f;

    /// <summary>
    /// Speed a corner has to take out of the car before it is somewhere a pass
    /// can be made. A gentle kink slows nobody enough to be passed at, and
    /// treating one as an opportunity is how a car ends up beside another on a
    /// piece of road neither of them was going to lose time on.
    /// </summary>
    private const float PassableSpeedDropMetersPerSecond = 8f;

    /// <summary>
    /// Road used to move across before the braking point.
    ///
    /// Not a decision, a measurement of the machinery: the offset profile is
    /// bounded to eight centimetres of lateral movement per metre travelled, so
    /// three metres of offset needs about forty metres of road. Committing any
    /// later means arriving at the corner still moving sideways.
    /// </summary>
    private const float CommitLeadMeters = 40f;

    /// <summary>
    /// Gap between car bodies before a stretch of road counts as usable.
    ///
    /// This has to clear the speed planner's own following margin with room for
    /// the other car to wander, not merely fit the bodywork. Aimed just past the
    /// bodies, the ego sits within a wobble of the traffic layer's overlap band,
    /// and every time the bands touch the constraint bites and the ego brakes
    /// mid-pass.
    /// </summary>
    private const float LateralClearanceMeters = 1.4f;

    /// <summary>Kerb the planner will not aim inside of.</summary>
    private const float EdgeMarginMeters = 0.5f;

    /// <summary>
    /// How far past the other car the ego has to be for the move to be over.
    /// Measured nose to tail, so it means genuinely clear rather than level.
    /// </summary>
    private const float ClearAheadMeters = 2f;

    /// <summary>Offset below which the car counts as back on the racing line.</summary>
    private const float ReturnedOffsetMeters = 0.2f;

    /// <summary>
    /// Step the asked-for offset is rounded to.
    ///
    /// The room beside another car moves a little every frame as it breathes on
    /// its own line, and passing that straight through would have the target
    /// twitching continuously. The controller would chase it, and the profile
    /// that expands one offset into a line along the track is rebuilt whenever
    /// the number changes at all. Rounding costs a few centimetres of precision
    /// the car cannot steer to anyway.
    /// </summary>
    private const float OffsetQuantumMeters = 0.5f;

    private string? _rival;
    private float _offsetMeters;
    private bool _committed;

    /// <summary>
    /// Where on the track the corner this move is aimed at sits.
    ///
    /// Held as a place on the road and not as a distance ahead, because the
    /// distance ahead is the thing that changes: a braking zone is over a
    /// hundred metres long, so a corner still being far away says nothing
    /// about whether this move has run its course. Only passing the corner
    /// does, and passing it is a fact about where the car is.
    /// </summary>
    private float _committedApexS = float.NaN;

    /// <summary>
    /// The corner this car is currently working towards, as a place on the
    /// road rather than a distance ahead.
    ///
    /// Picking it afresh every frame does not survive contact with the speed
    /// plan it is read from: two corners close together both sit near the
    /// threshold of being worth passing at, so the answer flickers between
    /// them from one frame to the next and the reckoning flickers with it -
    /// measured, the distance to the corner alternated between three hundred
    /// and six hundred metres and the verdict alternated with it. A driver
    /// picks the corner and keeps it, so this does too, until it is behind.
    /// </summary>
    private float _targetApexS = float.NaN;

    public TacticalManeuverPhase Phase { get; private set; } =
        TacticalManeuverPhase.Clear;

    /// <summary>Seconds of road that has to be made up to draw level.</summary>
    public float TimeGapSeconds { get; private set; } = float.NaN;

    /// <summary>
    /// Seconds this car would take out of its rival over the run to the next
    /// corner, on pace alone. A move is affordable when this covers the gap.
    /// </summary>
    public float RunGainSeconds { get; private set; } = float.NaN;

    public float NextApexMeters { get; private set; } = float.NaN;

    public TacticalIntent Update(
        in RaceDriverFrameContext context,
        VehicleSpeedLookahead ownPace
    )
    {
        ArgumentNullException.ThrowIfNull(ownPace);
        TimeGapSeconds = float.NaN;
        RunGainSeconds = float.NaN;
        NextApexMeters = float.NaN;
        if (!context.HasFrameSnapshot)
        {
            Reset();
            return TacticalIntent.Keep;
        }

        // Rule 1: the rival is whoever is immediately in front.
        if (!TryFindRival(in context, out RaceCarSnapshot rival, out int rivalIndex))
            return Release();

        if (!string.Equals(_rival, rival.Id, StringComparison.Ordinal))
        {
            _rival = rival.Id;
            _committed = false;
            _committedApexS = float.NaN;
            _targetApexS = float.NaN;
        }

        if (_committed && IsPast(in context, in rival))
        {
            _committed = false;
            _committedApexS = float.NaN;
            return Queue();
        }

        bool haveCorner = TryTakeAim(
            in context,
            ownPace,
            out float apexMeters,
            out float brakeMeters
        );
        if (haveCorner)
            NextApexMeters = apexMeters;

        RaceCarSnapshot ego = context.CarSnapshot;
        float gap = SignedGap(in context, in ego, in rival);

        // Once the move is under way it is not re-argued every frame. The road
        // beside another car opens and closes constantly as both cars work, and
        // an attempt dropped on the first closed frame is retaken on the next
        // open one, which is how a car crosses the whole track and arrives
        // nowhere.
        if (_committed)
        {
            // Rule 5. The corner settles it, and only once the corner has been
            // reached: level with the other car and the place is taken, still
            // behind and the car drops back in rather than running side by
            // side through a corner it has not earned. Overlapping bodywork
            // keeps the move alive past the apex, because straightening back
            // onto the racing line with a car alongside is not an option.
            if (HasPassedCommittedApex(in context) && gap > 0f)
            {
                _committed = false;
                _committedApexS = float.NaN;
                _offsetMeters = 0f;
                Phase = TacticalManeuverPhase.Yielding;
                return TacticalIntent.Keep;
            }

            return Reach(in context, in rival, hold: true);
        }

        if (!haveCorner)
            return Queue();

        // The calculation: can this car cover the ground before the corner?
        if (gap <= 0f)
            return Reach(in context, in rival, hold: false);

        // Rule 3. A straight is for closing up, so until the braking zone the
        // answer is the same whatever the sums say: sit behind, take the tow.
        //
        // Waiting is not only what a driver does, it is the only place the
        // question can be answered honestly. A car queueing behind another is
        // running at the other car's pace and not its own, so any reckoning
        // made half a kilometre out is done with a pace this car will not be
        // holding - measured, that made a corner five hundred metres away look
        // comfortably winnable and the same corner unwinnable once it arrived.
        // At the braking point the road left is road both cars are about to
        // cover flat out, and the comparison means something.
        if (brakeMeters > CommitLeadMeters)
        {
            _offsetMeters = 0f;
            Phase = TacticalManeuverPhase.Preparing;
            return TacticalIntent.Keep;
        }

        // Rule 2, asked at the braking point: where will the two cars be at the
        // corner? This one takes so long to reach it, the other covers so much
        // ground in that time, and what is left between them settles whether
        // there is anything here worth leaving the racing line for.
        float myRun = TimeOver(ownPace, apexMeters);
        TrafficMotionPlan? rivalPace =
            context.Frame.GetPreviousTrafficMotionPlan(rivalIndex);
        if (rivalPace is null ||
            !rivalPace.TryDistanceTravelled(myRun, out float theirRun))
        {
            return Queue();
        }

        float gapAtCorner = gap + theirRun - apexMeters;
        TimeGapSeconds = gap;
        RunGainSeconds = gapAtCorner;
        if (gapAtCorner > 0f)
            return Queue();

        return Reach(in context, in rival, hold: false);
    }

    public void Reset()
    {
        Phase = TacticalManeuverPhase.Clear;
        _rival = null;
        _offsetMeters = 0f;
        _committed = false;
        _committedApexS = float.NaN;
        _targetApexS = float.NaN;
        TimeGapSeconds = float.NaN;
        RunGainSeconds = float.NaN;
        NextApexMeters = float.NaN;
    }

    /// <summary>Rule 4, and the act of going: take the room or queue for it.</summary>
    private TacticalIntent Reach(
        in RaceDriverFrameContext context,
        in RaceCarSnapshot rival,
        bool hold
    )
    {
        if (!TryFindRoom(in context, in rival, !_committed, out float target))
        {
            if (!hold)
                return Queue();

            // Room closed mid-move. Stop reaching further out, keep the side.
            return new TacticalIntent(_offsetMeters);
        }

        _offsetMeters = Quantise(target);
        if (!_committed)
        {
            _committed = true;
            _committedApexS = float.IsNaN(NextApexMeters)
                ? float.NaN
                : context.Track.WrapS(context.Pose.S + NextApexMeters);
        }
        Phase = TacticalManeuverPhase.Committed;
        return new TacticalIntent(_offsetMeters);
    }

    /// <summary>
    /// Sit behind on the racing line. Wanting to be on the quickest road needs
    /// no reason: the offset goes to zero and the profile eases the car back.
    /// </summary>
    private TacticalIntent Queue()
    {
        _offsetMeters = 0f;
        Phase = TacticalManeuverPhase.Following;
        return TacticalIntent.Keep;
    }

    private TacticalIntent Release()
    {
        _offsetMeters = 0f;
        _committed = false;
        _committedApexS = float.NaN;
        Phase = TacticalManeuverPhase.Clear;
        return TacticalIntent.Keep;
    }

    /// <summary>Whether the corner this move was aimed at is behind the car.</summary>
    private bool HasPassedCommittedApex(in RaceDriverFrameContext context)
    {
        if (float.IsNaN(_committedApexS))
            return true;

        float ahead = context.Track.WrapS(_committedApexS - context.Pose.S);
        return ahead > context.Track.LengthMeters * 0.5f;
    }

    /// <summary>
    /// The next place the road takes enough speed out of the car to pass there,
    /// read off the plan the car is already steering to rather than a table
    /// worked out in advance. Everything that decides where a car brakes - its
    /// tyres, its battery, the tow it is sitting in - is in that plan and is
    /// current, so nothing has to be recomputed when any of it changes.
    /// </summary>
    /// <summary>
    /// The corner being worked towards and how far off it is, holding onto the
    /// one already chosen for as long as it is still in front.
    /// </summary>
    private bool TryTakeAim(
        in RaceDriverFrameContext context,
        VehicleSpeedLookahead pace,
        out float apexMeters,
        out float brakeMeters
    )
    {
        if (!float.IsNaN(_targetApexS))
        {
            float ahead = context.Track.WrapS(_targetApexS - context.Pose.S);
            if (ahead < context.Track.LengthMeters * 0.5f && ahead <= pace.LengthMeters)
            {
                apexMeters = ahead;
                brakeMeters = BrakePointFor(pace, ahead);
                return true;
            }
            _targetApexS = float.NaN;
        }

        if (!TryFindNextCorner(pace, out apexMeters, out brakeMeters))
            return false;

        _targetApexS = context.Track.WrapS(context.Pose.S + apexMeters);
        return true;
    }

    /// <summary>
    /// Where the car is quickest on the run in to a corner it has already
    /// chosen, which is where it starts braking and so where it may pull out.
    /// </summary>
    private static float BrakePointFor(VehicleSpeedLookahead pace, float apexMeters)
    {
        if (pace.Count < 2 || pace.StepLengthMeters <= 0f)
            return apexMeters;

        int apexIndex = Math.Min(
            (int)MathF.Round(apexMeters / pace.StepLengthMeters),
            pace.Count - 1
        );
        int peakIndex = 0;
        float peak = float.NegativeInfinity;
        for (int i = 0; i <= apexIndex; i++)
        {
            if (pace[i].TargetSpeed <= peak)
                continue;

            peak = pace[i].TargetSpeed;
            peakIndex = i;
        }
        return peakIndex * pace.StepLengthMeters;
    }

    private static bool TryFindNextCorner(
        VehicleSpeedLookahead pace,
        out float apexMeters,
        out float brakeMeters
    )
    {
        apexMeters = float.NaN;
        brakeMeters = float.NaN;
        if (pace.Count < 3 || pace.StepLengthMeters <= 0f)
            return false;

        float peak = pace[0].TargetSpeed;
        int peakIndex = 0;
        for (int i = 1; i < pace.Count; i++)
        {
            float speed = pace[i].TargetSpeed;
            if (speed > peak)
            {
                peak = speed;
                peakIndex = i;
                continue;
            }

            if (peak - speed < PassableSpeedDropMetersPerSecond)
                continue;

            int bottom = i;
            while (bottom + 1 < pace.Count &&
                   pace[bottom + 1].TargetSpeed <= pace[bottom].TargetSpeed)
            {
                bottom++;
            }

            apexMeters = bottom * pace.StepLengthMeters;
            brakeMeters = peakIndex * pace.StepLengthMeters;
            return true;
        }
        return false;
    }

    /// <summary>Seconds this plan needs to cover a stretch of road.</summary>
    private static float TimeOver(VehicleSpeedLookahead pace, float distanceMeters)
    {
        if (pace.Count < 2 || distanceMeters <= 0f)
            return 0f;

        float step = MathF.Max(pace.StepLengthMeters, 0.25f);
        float seconds = 0f;
        for (float travelled = 0f; travelled < distanceMeters; travelled += step)
        {
            float span = MathF.Min(step, distanceMeters - travelled);
            float speed = MathF.Max(
                pace.Sample(travelled + span * 0.5f).TargetSpeed,
                1f
            );
            seconds += span / speed;
        }
        return seconds;
    }

    private static bool TryFindRival(
        in RaceDriverFrameContext context,
        out RaceCarSnapshot rival,
        out int rivalIndex
    )
    {
        rival = default;
        rivalIndex = -1;
        RaceCarSnapshot ego = context.CarSnapshot;
        float nearest = float.PositiveInfinity;
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;

            RaceCarSnapshot other = context.Frame[i];
            float gap = SignedGap(in context, in ego, in other);
            if (gap <= -other.LengthMeters ||
                gap > RivalReachMeters ||
                gap >= nearest)
            {
                continue;
            }

            nearest = gap;
            rival = other;
            rivalIndex = i;
        }
        return rivalIndex >= 0;
    }

    private static float SignedGap(
        in RaceDriverFrameContext context,
        in RaceCarSnapshot ego,
        in RaceCarSnapshot other
    )
    {
        float along = context.Track.WrapS(other.TrackS - ego.TrackS);
        if (along > context.Track.LengthMeters * 0.5f)
            along -= context.Track.LengthMeters;
        return along - (ego.LengthMeters + other.LengthMeters) * 0.5f;
    }

    private static float Quantise(float offsetMeters) =>
        MathF.Round(offsetMeters / OffsetQuantumMeters) * OffsetQuantumMeters;

    private static bool IsPast(
        in RaceDriverFrameContext context,
        in RaceCarSnapshot rival
    )
    {
        RaceCarSnapshot ego = context.CarSnapshot;
        float ahead = context.Track.WrapS(ego.TrackS - rival.TrackS);
        float clear = (ego.LengthMeters + rival.LengthMeters) * 0.5f +
                      ClearAheadMeters;
        return ahead > clear && ahead < context.Track.LengthMeters * 0.5f;
    }

    /// <summary>
    /// The nearest stretch of road beside the car ahead that the ego fits
    /// through, as an offset from the racing line.
    ///
    /// Every car between the ego and the one it is going around denies a span
    /// of road, and what those spans leave is what can be driven. A second car
    /// sitting where the ego was heading simply closes that side, so the other
    /// is taken instead - or neither is, and the ego queues. That is rule 4,
    /// and it is what keeps a braking zone to two cars wide.
    ///
    /// Once the move is under way the side is not reconsidered. Changing sides
    /// means crossing the track behind the car being passed, which is slower
    /// than either side and gives back whatever the move had gained.
    /// </summary>
    private bool TryFindRoom(
        in RaceDriverFrameContext context,
        in RaceCarSnapshot ahead,
        bool mayChooseSide,
        out float targetOffsetMeters
    )
    {
        targetOffsetMeters = 0f;
        RaceCarSnapshot ego = context.CarSnapshot;
        TrackData track = context.Track;
        TrackSample at = track.Sample(ahead.TrackS);
        float usable = at.HalfWidth - ego.WidthMeters * 0.5f - EdgeMarginMeters;
        if (usable <= 0f)
            return false;

        // Offsets are measured from the racing line, which does not run down
        // the middle of the road, so the room on either side is not the same.
        float low = -usable - at.RefOffset;
        float high = usable - at.RefOffset;
        float window = track.WrapS(ahead.TrackS - ego.TrackS) +
                       ahead.LengthMeters;

        float best = 0f;
        float bestDistance = float.PositiveInfinity;
        bool found = false;
        for (int side = 0; side < 2; side++)
        {
            bool left = side == 0;
            if (!mayChooseSide && left != _offsetMeters > 0f)
                continue;

            float edge = ProbeSide(in context, in ego, window, left, low, high);
            if (float.IsNaN(edge) || MathF.Abs(edge) >= bestDistance)
                continue;

            bestDistance = MathF.Abs(edge);
            best = edge;
            found = true;
        }

        targetOffsetMeters = best;
        return found;
    }

    /// <summary>
    /// How far to one side the ego has to sit to clear everyone between it and
    /// the car it is passing, or NaN if that side does not fit on the road.
    /// </summary>
    private static float ProbeSide(
        in RaceDriverFrameContext context,
        in RaceCarSnapshot ego,
        float windowMeters,
        bool left,
        float lowOffset,
        float highOffset
    )
    {
        float required = 0f;
        bool any = false;
        foreach (RaceCarSnapshot other in context.Frame.Cars)
        {
            if (string.Equals(other.Id, ego.Id, StringComparison.Ordinal))
                continue;

            float gap = context.Track.WrapS(other.TrackS - ego.TrackS);
            if (gap <= 0f || gap > windowMeters)
                continue;

            TrackSample sample = context.Track.Sample(other.TrackS);
            float theirOffset = other.TrackD - sample.RefOffset;
            float apart = (ego.WidthMeters + other.WidthMeters) * 0.5f +
                          LateralClearanceMeters;
            float edge = left ? theirOffset + apart : theirOffset - apart;
            required = any
                ? (left ? MathF.Max(required, edge) : MathF.Min(required, edge))
                : edge;
            any = true;
        }

        if (!any || required < lowOffset || required > highOffset)
            return float.NaN;

        return required;
    }
}
