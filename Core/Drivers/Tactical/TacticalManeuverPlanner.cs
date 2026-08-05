using System;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Decides where on the road the car wants to be, and hands that out as a
/// single offset from the racing line.
///
/// An overtake is a manoeuvre with a life of its own rather than a line chosen
/// afresh every frame. It begins because the car is being held up, holds its
/// side while it develops, and ends when the car is past - and none of those
/// depend on predicting whether the pass will come off. That prediction cannot
/// honestly be made: whether a move works turns on the seconds after the cars
/// are alongside, on who brakes later and who gets the better exit, and none
/// of that exists in a projection taken from behind. A decision gated on a
/// number that cannot be computed does not come out cautious, it comes out
/// unstable - out, back, out, back, paying for the move each way and
/// finishing nothing.
///
/// What is worth knowing is cheap. Holding three metres off the racing line
/// costs a few tenths of a per cent of a lap, because the inside of one corner
/// is the outside of the next, so any car with a real pace advantage can
/// afford to go and look. The planner therefore never weighs the cost of
/// going. It asks whether the car is being held up, and whether there is room.
/// </summary>
internal sealed class TacticalManeuverPlanner
{
    /// <summary>
    /// Gap between car bodies before a stretch of road counts as usable.
    ///
    /// This has to clear the speed planner's own following margin with room
    /// for the other car to wander, not merely fit the bodywork. Aimed just
    /// past the bodies, the ego sits within a wobble of the traffic layer's
    /// overlap band, and both cars are wandering: the one being passed by
    /// its half metre, the ego by its own tracking error. Every time the
    /// bands touch, the constraint bites and the ego brakes mid-pass -
    /// measured, that bled a seven per cent pace advantage down to nothing
    /// and turned the overtake into minutes of running alongside. The traffic layer still enforces safety on the
    /// path actually driven; this only keeps the planner from aiming
    /// somewhere it can already see will keep tripping it.
    /// </summary>
    private const float LateralClearanceMeters = 1.4f;

    /// <summary>Kerb the planner will not aim inside of.</summary>
    private const float EdgeMarginMeters = 0.5f;

    /// <summary>
    /// How far past the other car the ego has to be for the move to be over.
    /// Measured nose to tail, so it means genuinely clear rather than level.
    /// </summary>
    private const float ClearAheadMeters = 2f;

    /// <summary>
    /// Clear running that ends a spell of being held up. Long enough that the
    /// following constraint letting go for a moment does not read as open
    /// road: it bites as the gap closes and releases as it opens, so a car
    /// holding station behind a slower one trips it several times a second.
    /// </summary>
    private const float ClearRunToReleaseMeters = 40f;

    /// <summary>Offset below which the car counts as back on the racing line.</summary>
    private const float ReturnedOffsetMeters = 0.2f;


    /// <summary>
    /// Step the asked-for offset is rounded to.
    ///
    /// The room beside another car moves a little every frame as it breathes
    /// on its own line, and passing that straight through would have the
    /// target twitching continuously. The controller would chase it, and the
    /// profile that expands one offset into a line along the track is rebuilt
    /// whenever the number changes at all. Rounding costs a few centimetres of
    /// precision the car cannot steer to anyway.
    /// </summary>
    private const float OffsetQuantumMeters = 0.5f;

    private bool _heldUp;
    private float _clearRunMeters;
    private float _lastS;
    private bool _hasLastS;
    private float _offsetMeters;
    private string? _passing;

    public TrafficConflictReport LastObservedConflictReport { get; private set; }

    public TacticalManeuverPhase Phase { get; private set; } =
        TacticalManeuverPhase.Observing;

    public TacticalIntent Update(
        in RaceDriverFrameContext context,
        in TrafficConflictReport previousConflictReport
    )
    {
        LastObservedConflictReport = previousConflictReport;
        if (!context.HasFrameSnapshot)
        {
            Reset();
            return TacticalIntent.Keep;
        }

        UpdateHeldUp(in context, in previousConflictReport);

        if (_passing is not null && IsPast(in context, _passing))
            _passing = null;

        if (_passing is null && _heldUp)
            _passing = BlockerAhead(in context);

        if (_passing is null)
        {
            // Nothing to go around. The racing line is the fastest line there
            // is, so wanting to be back on it needs no reason and no rule to
            // send the car home: the offset goes to zero and the existing
            // profile eases the car across.
            _offsetMeters = 0f;
            Phase = MathF.Abs(context.Pose.D - context.Pose.Sample.RefOffset) <=
                    ReturnedOffsetMeters
                ? TacticalManeuverPhase.Observing
                : TacticalManeuverPhase.Returning;
            return TacticalIntent.Keep;
        }

        bool committing = Phase is TacticalManeuverPhase.Observing
            or TacticalManeuverPhase.Anticipating
            or TacticalManeuverPhase.Returning;
        if (!TryFindRoom(in context, _passing, committing, out float target))
        {
            // No room this frame. That is a reason to stop going further out,
            // not to call the move off: the road beside another car closes and
            // opens constantly as both cars work, and a manoeuvre abandoned on
            // the first closed frame is re-decided on the next open one, which
            // is how a car ends up crossing the whole track behind the one it
            // was passing and arriving nowhere. The side and the car being
            // passed are kept; only the reaching stops.
            Phase = _offsetMeters == 0f
                ? TacticalManeuverPhase.Anticipating
                : TacticalManeuverPhase.Executing;
            return new TacticalIntent(_offsetMeters);
        }

        _offsetMeters = Quantise(target);
        Phase = committing
            ? TacticalManeuverPhase.Committed
            : TacticalManeuverPhase.Executing;
        return new TacticalIntent(_offsetMeters);
    }

    public void Reset()
    {
        LastObservedConflictReport = default;
        Phase = TacticalManeuverPhase.Observing;
        _heldUp = false;
        _clearRunMeters = 0f;
        _hasLastS = false;
        _offsetMeters = 0f;
        _passing = null;
    }

    /// <summary>
    /// Whether the car is running at its own pace or somebody else's.
    ///
    /// The speed planner already answers this on the way past: it times the
    /// path on geometry alone, then again with the traffic applied, and the
    /// difference is what the car in front is costing. Nothing else is being
    /// held up - a car far enough ahead never enters the constraint at all, so
    /// the two times come out equal however slow it is, and no following
    /// distance has to be chosen.
    ///
    /// It is latched because the constraint that produces it is a proximity
    /// test. A car holding station behind a slower one closes to the gap,
    /// eases, drifts out of it and closes again, so the reading is compromised
    /// on one frame and clear on the next while the road tells one continuous
    /// story.
    /// </summary>
    private void UpdateHeldUp(
        in RaceDriverFrameContext context,
        in TrafficConflictReport report
    )
    {
        float s = context.Pose.S;
        float moved = _hasLastS ? context.Track.WrapS(s - _lastS) : 0f;
        _lastS = s;
        _hasLastS = true;
        if (moved < 0f || moved > ClearRunToReleaseMeters)
            moved = 0f;

        float free = report.FreeArrivalTimeSeconds;
        float constrained = report.ConstrainedArrivalTimeSeconds;
        bool compromised = float.IsFinite(free) &&
                           (!float.IsFinite(constrained) ||
                            constrained > free + 1e-3f);
        if (compromised)
        {
            _heldUp = true;
            _clearRunMeters = 0f;
            return;
        }

        if (!_heldUp)
            return;

        _clearRunMeters += moved;
        if (_clearRunMeters >= ClearRunToReleaseMeters)
        {
            _heldUp = false;
            _clearRunMeters = 0f;
        }
    }

    private static float Quantise(float offsetMeters) =>
        MathF.Round(offsetMeters / OffsetQuantumMeters) * OffsetQuantumMeters;

    /// <summary>The nearest slower car ahead, which is the one in the way.</summary>
    private static string? BlockerAhead(in RaceDriverFrameContext context)
    {
        RaceCarSnapshot ego = context.CarSnapshot;
        string? nearest = null;
        float nearestGap = float.PositiveInfinity;
        foreach (RaceCarSnapshot other in context.Frame.Cars)
        {
            if (string.Equals(other.Id, ego.Id, StringComparison.Ordinal))
                continue;
            if (other.SpeedMetersPerSecond >= ego.SpeedMetersPerSecond)
                continue;

            float gap = context.Track.WrapS(other.TrackS - ego.TrackS);
            if (gap <= 0f || gap >= nearestGap)
                continue;

            nearestGap = gap;
            nearest = other.Id;
        }
        return nearest;
    }

    private static bool IsPast(in RaceDriverFrameContext context, string blocker)
    {
        if (!context.Frame.TryGetCar(blocker, out RaceCarSnapshot other))
            return true;

        RaceCarSnapshot ego = context.CarSnapshot;
        float ahead = context.Track.WrapS(ego.TrackS - other.TrackS);
        float clear = (ego.LengthMeters + other.LengthMeters) * 0.5f +
                      ClearAheadMeters;
        return ahead > clear &&
               ahead < context.Track.LengthMeters * 0.5f;
    }

    /// <summary>
    /// The nearest stretch of road beside the car ahead that the ego fits
    /// through, as an offset from the racing line.
    ///
    /// Every car between the ego and the one it is going around denies a span
    /// of road, and what those spans leave is what can be driven. A second car
    /// sitting where the ego was heading simply closes that side, so the other
    /// is taken instead - or neither is, and the ego stays where it is. That
    /// is the answer a driver reaches looking at the same road.
    ///
    /// Once the move is under way the side is not reconsidered. Changing sides
    /// means crossing the track behind the car being passed, which is slower
    /// than either side and gives back whatever the move had gained.
    /// </summary>
    private bool TryFindRoom(
        in RaceDriverFrameContext context,
        string blocker,
        bool mayChooseSide,
        out float targetOffsetMeters
    )
    {
        targetOffsetMeters = 0f;
        if (!context.Frame.TryGetCar(blocker, out RaceCarSnapshot ahead))
            return false;

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
