using StintegyEVO.Core.Racing;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// No-op foundation for the pre-publication tactical phase. Future racecraft
/// decisions may read the preceding frozen plans through context.Frame, but
/// this stage intentionally preserves current driving behavior.
/// </summary>
internal sealed class TacticalManeuverPlanner
{
    public TrafficConflictReport LastObservedConflictReport { get; private set; }
    public TacticalManeuverPhase Phase { get; private set; } =
        TacticalManeuverPhase.Observing;

    public TacticalIntent Update(
        in RaceDriverFrameContext context,
        in TrafficConflictReport previousConflictReport
    )
    {
        LastObservedConflictReport = previousConflictReport;
        return TacticalIntent.Keep;
    }

    public void Reset()
    {
        LastObservedConflictReport = default;
        Phase = TacticalManeuverPhase.Observing;
    }
}
