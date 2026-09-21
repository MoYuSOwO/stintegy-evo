namespace StintegyEVO.TrainingHost.Environment;

public enum TrainingTerminalReason : byte
{
    None,
    Passed,
    Contact,
    Stalled,
    Timeout,
    Finished
}

public readonly record struct TrainingStepResult(
    TrainingTerminalReason TerminalReason,
    float OwnProgressReward,
    float RelativeProgressReward,
    float PassReward,
    float ContactPenalty,
    float WallPenalty,
    float OffCoursePenalty,
    float TyreSlipPenalty,
    float TimePenalty,
    float TimeoutOutcome,
    float ModeExcessPenalty,
    float RetirementPenalty,
    float BudgetShaping = 0f,
    float SteeringReversalPenalty = 0f,
    float SteeringChangePenalty = 0f
)
{
    public const int ComponentCount = 14;

    public bool Done => TerminalReason != TrainingTerminalReason.None;

    public float Reward =>
        OwnProgressReward +
        RelativeProgressReward +
        PassReward +
        ContactPenalty +
        WallPenalty +
        OffCoursePenalty +
        TyreSlipPenalty +
        TimePenalty +
        TimeoutOutcome +
        ModeExcessPenalty +
        RetirementPenalty +
        BudgetShaping +
        SteeringReversalPenalty +
        SteeringChangePenalty;

    public float GetComponent(int index) => index switch
    {
        0 => OwnProgressReward,
        1 => RelativeProgressReward,
        2 => PassReward,
        3 => ContactPenalty,
        4 => WallPenalty,
        5 => OffCoursePenalty,
        6 => TyreSlipPenalty,
        7 => TimePenalty,
        8 => TimeoutOutcome,
        9 => ModeExcessPenalty,
        10 => RetirementPenalty,
        11 => BudgetShaping,
        12 => SteeringReversalPenalty,
        13 => SteeringChangePenalty,
        _ => throw new ArgumentOutOfRangeException(nameof(index))
    };
}
