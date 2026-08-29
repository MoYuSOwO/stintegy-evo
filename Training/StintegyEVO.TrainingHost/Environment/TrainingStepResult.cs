namespace StintegyEVO.TrainingHost.Environment;

public enum TrainingTerminalReason : byte
{
    None,
    Passed,
    Contact,
    Wall,
    Stalled,
    Timeout
}

public readonly record struct TrainingStepResult(
    TrainingTerminalReason TerminalReason,
    float OwnProgressReward,
    float RelativeProgressReward,
    float PassReward,
    float ContactPenalty,
    float WallPenalty,
    float ActionMagnitudePenalty,
    float ActionDeltaPenalty,
    float TimePenalty,
    float TimeoutOutcome,
    float ModeExcessPenalty
)
{
    public const int ComponentCount = 10;

    public bool Done => TerminalReason != TrainingTerminalReason.None;

    public float Reward =>
        OwnProgressReward +
        RelativeProgressReward +
        PassReward +
        ContactPenalty +
        WallPenalty +
        ActionMagnitudePenalty +
        ActionDeltaPenalty +
        TimePenalty +
        TimeoutOutcome +
        ModeExcessPenalty;

    public float GetComponent(int index) => index switch
    {
        0 => OwnProgressReward,
        1 => RelativeProgressReward,
        2 => PassReward,
        3 => ContactPenalty,
        4 => WallPenalty,
        5 => ActionMagnitudePenalty,
        6 => ActionDeltaPenalty,
        7 => TimePenalty,
        8 => TimeoutOutcome,
        9 => ModeExcessPenalty,
        _ => throw new ArgumentOutOfRangeException(nameof(index))
    };
}
