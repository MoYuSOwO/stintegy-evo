namespace StintegyEVO.Core.Track;

/// <summary>
/// The shipped circuits by name. Each one is a pack under data/official/circuits/,
/// written by Tools/Circuits; these only load them, so the many callers
/// that ask for a circuit by its old name keep working.
/// </summary>
public static class TrackFactory
{
    public static TrackData SimpleTestTrack(bool isLeft = false) =>
        CircuitLibrary.Load(isLeft ? "simple_left" : "simple");

    public static TrackData BankedSpeedwayTestTrack() => CircuitLibrary.Load("banked_speedway");
    public static TrackData BankedSweeperTestTrack() => CircuitLibrary.Load("banked_sweeper");
    public static TrackData FlatSweeperTestTrack() => CircuitLibrary.Load("flat_sweeper");
    public static TrackData DaytonaStyleTestTrack() => CircuitLibrary.Load("daytona");

    public static TrackData SilverstoneStyleTestTrack() => CircuitLibrary.Load("silverstone");
    public static TrackData MonacoStyleTestTrack() => CircuitLibrary.Load("monaco");
    public static TrackData ZandvoortStyleTestTrack() => CircuitLibrary.Load("zandvoort");
    public static TrackData BakuStyleTestTrack() => CircuitLibrary.Load("baku");
    public static TrackData SpaStyleTestTrack() => CircuitLibrary.Load("spa");
    public static TrackData MonzaStyleTestTrack() => CircuitLibrary.Load("monza");
    public static TrackData InterlagosStyleTestTrack() => CircuitLibrary.Load("interlagos");
    public static TrackData SingaporeStyleTestTrack() => CircuitLibrary.Load("singapore");
    public static TrackData PortimaoStyleTestTrack() => CircuitLibrary.Load("portimao");
    public static TrackData ShanghaiStyleTestTrack() => CircuitLibrary.Load("shanghai");
    public static TrackData SepangStyleTestTrack() => CircuitLibrary.Load("sepang");
}
