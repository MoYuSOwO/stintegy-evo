namespace StintegyEVO.TrainingHost.Protocol;

public enum TrainingMessageKind : ushort
{
    Hello = 1,
    HelloResponse = 2,
    Reset = 3,
    ResetResponse = 4,
    Step = 5,
    StepResponse = 6,
    Close = 7,
    CloseResponse = 8,
    MaskedReset = 9,
    MaskedResetResponse = 10,
    Error = ushort.MaxValue
}
