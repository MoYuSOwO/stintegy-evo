namespace StintegyEVO.Nodes.Race;

public interface IEnvironment
{
    public float EnvTemp { get; }
}

public class DummyEnvironment : IEnvironment
{
    public static readonly DummyEnvironment Instance = new();

    public float EnvTemp => 25.0f;

    private DummyEnvironment() {}
}