using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;

namespace StintegyEVO.Drivers.Racecraft;

/// <summary>What one quarter second of a duel was worth, and what it was made of.</summary>
public readonly record struct RacecraftStep(
    float Reward,
    bool Ended,
    bool OutOfTime,
    float Gain,
    float Passes,
    float Contact,
    float Off,
    float Wall,
    float Spin
)
{
    public const int ComponentCount = 6;
}

/// <summary>
/// One training episode of racecraft: two cars on a circuit, the one the
/// racecraft layer draws for starting a few dozen metres behind a car that
/// is a little slower and drives its own race on the racing line. Each
/// step the layer draws, the race runs a quarter of a second, and the step
/// is scored on the ground gained on the other car, places taken and lost,
/// and contact, the grass, the wall and spins, the risks weighed by how bold
/// the driver is.
/// </summary>
public sealed class RacecraftDuel
{
    public const float StepSeconds = 0.25f;
    public const float SimDt = 1f / 60f;
    public const float EpisodeSeconds = 60f;

    // Reward: a point for every ten metres taken out of the other car, five
    // for each place taken and lost, and for the risks, at the bold driver's
    // half and the timid one's one and a half times: one a second touching
    // the other car, one a second off the road, three a second on the wall,
    // and twenty and the end of the episode for a spin.
    public const float GainPerMeter = 0.1f;
    public const float PassReward = 5f;
    public const float ContactPerSecond = 1f;
    public const float OffPerSecond = 1f;
    public const float WallPerSecond = 3f;
    public const float SpinPenalty = 20f;

    /// <summary>
    /// How much slower the car ahead is, drawn afresh each duel between
    /// these: at one it plans on <see cref="MaxShareGiven"/> less of every
    /// share of the grip than the plain driver, at nought it is the plain driver.
    /// </summary>
    public (float Min, float Max) RivalSlack { get; init; } = (0.5f, 1f);

    public const float MaxShareGiven = 0.1f;
    private static readonly LaneShares Plain = new();

    private readonly TrackData _track;
    private readonly LaneTrackFile _file;
    private readonly LaneSet _lanes;
    private readonly RacecraftObservation _observation;
    private readonly float[] _plannedSpeed;
    private readonly float[] _previous = new float[RacecraftAction.Size];
    private readonly float[] _offsets = new float[RacecraftAction.Points];

    private RaceSimulation _simulation = null!;
    private RaceCar _ego = null!, _rival = null!;
    private LaneDriver _egoDriver = null!;
    private float _boldness;
    private float _time;
    private float _lead;

    /// <summary>Duels on <paramref name="track"/>, whose lanes are those of <paramref name="circuit"/>.</summary>
    public RacecraftDuel(string circuit, TrackData track)
    {
        _track = track;
        _file = LaneTrackFile.ForCircuit(circuit, _track);
        _lanes = _file.LanesFor(_track);
        _observation = new RacecraftObservation(_lanes);
        _plannedSpeed = PlannedSpeeds();
    }

    public static int ObservationSize => RacecraftObservation.Size;
    public static int ActionSize => RacecraftAction.Size;

    /// <summary>A new duel drawn from <paramref name="random"/>; its first row into <paramref name="row"/>.</summary>
    public void Reset(Random random, Span<float> row)
    {
        _simulation = new RaceSimulation(_track, seed: (ulong)random.NextInt64());
        string compound = DevelopmentTyres.SuperId;
        float length = _track.LengthMeters;
        float rivalS = (float)random.NextDouble() * length;
        float gap = 10f + 50f * (float)random.NextDouble();
        float egoS = ((rivalS - gap) % length + length) % length;
        // The car ahead slower, by a different amount each time: it plans
        // on less of the grip in the corners, the braking and the drive.
        float slack = RivalSlack.Min + (RivalSlack.Max - RivalSlack.Min) * (float)random.NextDouble();
        var rivalShares = new LaneShares
        {
            Lateral = Plain.Lateral - MaxShareGiven * slack,
            Brake = Plain.Brake - MaxShareGiven * slack,
            Drive = Plain.Drive - MaxShareGiven * slack
        };
        (_rival, _) = LaneGrid.Seat(_simulation, _file, _lanes, "rival", StockCar.Build(compound), compound,
            rivalS, 0f, Planned(rivalS), shares: rivalShares);
        _boldness = 100f * (float)random.NextDouble();
        var egoAbilities = new DriverAbilities { Boldness = _boldness };
        (_ego, _egoDriver) = LaneGrid.Seat(_simulation, _file, _lanes, "ego", StockCar.Build(compound), compound,
            egoS, 0f, Planned(egoS), abilities: egoAbilities);
        RacecraftAction.RacingLine(_previous);
        _time = 0f;
        _lead = Lead();
        Observe(row);
    }

    /// <summary>A quarter second on <paramref name="action"/>; the next row into <paramref name="row"/>.</summary>
    public RacecraftStep Step(ReadOnlySpan<float> action, Span<float> row)
    {
        float brake = RacecraftAction.Decode(action, _offsets);
        _egoDriver.DrawLine(_offsets, brake);
        action.CopyTo(_previous);

        float risk = 1.5f - _boldness / 100f;
        float contact = 0f, off = 0f, wall = 0f, spin = 0f;
        int spinsBefore = Spins(_ego);
        int frames = (int)MathF.Round(StepSeconds / SimDt);
        for (int f = 0; f < frames; f++)
        {
            _simulation.Step(SimDt);
            if (_ego.HitCarThisStep)
                contact += SimDt;
            if (_ego.Progress.Region != TrackRegion.RacingSurface)
                off += SimDt;
            wall += _ego.BoundaryContactSeconds;
            if (Spins(_ego) > spinsBefore)
            {
                spin = 1f;
                break;
            }
        }
        _time += StepSeconds;

        float lead = Lead();
        float gain = lead - _lead;
        float passes = (MathF.Sign(lead) - MathF.Sign(_lead)) * 0.5f;
        _lead = lead;

        float reward = GainPerMeter * gain + PassReward * passes -
                       risk * (ContactPerSecond * contact + OffPerSecond * off + WallPerSecond * wall + SpinPenalty * spin);
        bool outOfTime = _time >= EpisodeSeconds - 1e-3f;
        Observe(row);
        return new RacecraftStep(reward, spin > 0f || outOfTime, outOfTime && spin <= 0f, gain, passes, contact, off, wall, spin);
    }

    private void Observe(Span<float> row)
    {
        RaceFrameSnapshot frame = _simulation.CaptureFrame();
        int self = 0;
        for (int i = 0; i < frame.Count; i++)
            if (frame[i].Id == _ego.Id)
                self = i;
        _observation.Write(frame, self, _previous, _boldness, row);
    }

    // How far ahead of the other car the car is along the lap, behind negative.
    private float Lead()
    {
        float length = _track.LengthMeters;
        float lead = ((_ego.Progress.CurrentS - _rival.Progress.CurrentS) % length + length) % length;
        return lead > length * 0.5f ? lead - length : lead;
    }

    private static int Spins(RaceCar car) => ((ICarMotion)car.Car).Motion(car.State).Spins;

    private float Planned(float s) => _plannedSpeed[_lanes.Wrap((int)MathF.Round(_lanes.StationOf(s)))];

    // The speed the driver does round the lap alone, to start each car
    // rolling at the speed it would be doing there: driven, not read off a
    // plan that has only been made for the road just ahead.
    private float[] PlannedSpeeds()
    {
        LaneLapResult lap = LaneLapRunner.Run(_track, _file);
        float[] speeds = lap.StationSpeed.ToArray();
        for (int i = 0; i < speeds.Length; i++)
            if (speeds[i] <= 1f)
                speeds[i] = speeds[_lanes.Wrap(i - 1)];
        return speeds;
    }
}
