using System.Globalization;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// One circuit's tuning file: the four lanes and, per station, lane and grip
/// level, the learned share of the envelope the plan may spend there.
///
/// Factors are multipliers on the live envelope, never speeds, so a worn
/// tyre or a flat battery still moves the plan. They are kept per grip
/// level because what a corner allows on top of the envelope is not the same
/// with the car at full grip as with it at two thirds: cold tyres, worn
/// tyres and a green track are all the same thing to the driver, less grip,
/// and each level is learned on its own so the slippery cases do not drag
/// the quick one down.
/// </summary>
public sealed class LaneTrackFile
{
    /// <summary>Grip, as a share of the car on ideal-temperature new tyres on a clean track.</summary>
    public static readonly float[] GripLevels = [0.55f, 0.70f, 0.85f, 1.00f];

    private LaneSet? _lanes;
    private float[][][] _corner = [];
    private float[][][] _brake = [];
    private float[][] _steer = [];
    private readonly LaneBakeOptions _bake;

    private LaneTrackFile(LaneSet? lanes, float[][][]? corner, float[][][]? brake, LaneBakeOptions? bake)
    {
        _lanes = lanes;
        _bake = bake ?? new LaneBakeOptions();
        if (lanes is not null)
        {
            _corner = corner ?? NewFactors(lanes.Count);
            _brake = brake ?? NewFactors(lanes.Count);
            _steer = NewSteer(lanes.Count);
        }
    }

    private static float[][] NewSteer(int count) =>
        Enumerable.Range(0, LaneSet.LaneCount).Select(_ => new float[count]).ToArray();

    /// <summary>
    /// Per station, curvature added to the steering feedforward (1/m), learned
    /// from where the car keeps ending up off its lane.
    /// </summary>
    public float[] Steer(LaneId lane) => _steer[(int)lane];

    public static int TopLevel => GripLevels.Length - 1;

    /// <summary>A file with nothing learned yet; lanes are baked against the first track it meets.</summary>
    public static LaneTrackFile Fresh(LaneBakeOptions? bake = null) => new(null, null, null, bake);

    public static LaneTrackFile For(LaneSet lanes) => new(lanes, null, null, null);

    /// <summary>A fresh file on lanes from the offline 3D bake; nothing learned yet.</summary>
    public static LaneTrackFile FromBakedLanes(string bakedPath, TrackData track, LaneBakeOptions? fallbackBake = null) =>
        new(LaneSet.ImportBaked(bakedPath, track, LaneSet.Bake(track, fallbackBake)), null, null, null);

    public LaneSet LanesFor(DriverTrackView track) => LanesFor(track.LengthMeters, () => LaneSet.Bake(track, _bake));

    public LaneSet LanesFor(TrackData track) => LanesFor(track.LengthMeters, () => LaneSet.Bake(track, _bake));

    private LaneSet LanesFor(float lengthMeters, Func<LaneSet> bake)
    {
        if (_lanes is null || MathF.Abs(_lanes.LengthMeters - lengthMeters) > 1f)
        {
            _lanes = bake();
            _corner = NewFactors(_lanes.Count);
            _brake = NewFactors(_lanes.Count);
            _steer = NewSteer(_lanes.Count);
        }
        return _lanes;
    }

    /// <summary>Corner factors for one lane at one grip level, for the learner to change in place.</summary>
    public float[] Factors(LaneId lane, int level) => _corner[level][(int)lane];

    /// <summary>Braking factors for one lane at one grip level.</summary>
    public float[] BrakeFactors(LaneId lane, int level) => _brake[level][(int)lane];

    /// <summary>Corner factors at full grip, the level a quick lap is driven at.</summary>
    public float[] Factors(LaneId lane) => Factors(lane, TopLevel);

    public float[] BrakeFactors(LaneId lane) => BrakeFactors(lane, TopLevel);

    /// <summary>
    /// The factors for a lane at this grip, interpolated between the two
    /// levels either side and held at the ends, written into
    /// <paramref name="corner"/> and <paramref name="brake"/>, the whole lap.
    /// </summary>
    public void FactorsAtGrip(LaneId lane, float grip, Span<float> corner, Span<float> brake) =>
        FactorsAtGrip(lane, grip, 0, corner.Length, corner, brake);

    /// <summary>
    /// The same for <paramref name="count"/> stations from
    /// <paramref name="start"/>, wrapped round the lap, written at their own
    /// stations in lap-long <paramref name="corner"/> and <paramref name="brake"/>.
    /// </summary>
    public void FactorsAtGrip(LaneId lane, float grip, int start, int count, Span<float> corner, Span<float> brake)
    {
        int n = corner.Length;
        count = Math.Min(count, n);
        start = ((start % n) + n) % n;
        (int lo, int hi, float t) = Bracket(grip);
        float[] cLo = _corner[lo][(int)lane], cHi = _corner[hi][(int)lane];
        float[] bLo = _brake[lo][(int)lane], bHi = _brake[hi][(int)lane];
        for (int k = 0, i = start; k < count; k++, i = i + 1 == n ? 0 : i + 1)
        {
            corner[i] = cLo[i] + (cHi[i] - cLo[i]) * t;
            brake[i] = bLo[i] + (bHi[i] - bLo[i]) * t;
        }

        // Less grip never buys a faster corner. The envelope already takes
        // the grip off; the factor is what it gets wrong about the line and
        // the car, and that does not grow more hopeful on a worse tyre. A
        // level learned on an evenly slippery road can still find a corner
        // it got away with that the full-grip level was wary of, and worn
        // tyres are not evenly slippery: the rear goes first. So every level
        // with more grip is a ceiling on this one.
        int above = t > 0f ? hi : lo + 1;
        for (int k = above; k < GripLevels.Length; k++)
        {
            float[] cCap = _corner[k][(int)lane], bCap = _brake[k][(int)lane];
            for (int j = 0, i = start; j < count; j++, i = i + 1 == n ? 0 : i + 1)
            {
                corner[i] = MathF.Min(corner[i], cCap[i]);
                brake[i] = MathF.Min(brake[i], bCap[i]);
            }
        }
    }

    internal static (int Lo, int Hi, float T) Bracket(float grip)
    {
        if (grip <= GripLevels[0])
            return (0, 0, 0f);
        for (int k = 1; k < GripLevels.Length; k++)
        {
            if (grip <= GripLevels[k])
                return (k - 1, k, (grip - GripLevels[k - 1]) / (GripLevels[k] - GripLevels[k - 1]));
        }
        return (TopLevel, TopLevel, 0f);
    }

    public LaneTrackFile Clone()
    {
        static float[][][] Copy(float[][][] source) =>
            source.Select(level => level.Select(lane => (float[])lane.Clone()).ToArray()).ToArray();
        var copy = new LaneTrackFile(_lanes, Copy(_corner), Copy(_brake), _bake);
        if (_steer.Length == LaneSet.LaneCount)
            copy._steer = _steer.Select(lane => (float[])lane.Clone()).ToArray();
        copy.Energy = Energy;
        copy.LearnedFor = LearnedFor;
        copy.Tyres = Tyres;
        copy.Calibration = Calibration;
        return copy;
    }

    /// <summary>
    /// Where each level starts before anything is learned: the slippery ones
    /// conservatively, and learning is what earns them more. A car on cold
    /// tyres meets the bottom level before it has any data at all.
    /// </summary>
    public static readonly float[] StartingFactors = [0.80f, 0.88f, 0.95f, 1.00f];

    private static float[][][] NewFactors(int count)
    {
        var factors = new float[GripLevels.Length][][];
        for (int level = 0; level < GripLevels.Length; level++)
        {
            factors[level] = new float[LaneSet.LaneCount][];
            for (int lane = 0; lane < LaneSet.LaneCount; lane++)
            {
                factors[level][lane] = new float[count];
                Array.Fill(factors[level][lane], StartingFactors[level]);
            }
        }
        return factors;
    }

    /// <summary>Puts one grip level back to its conservative start, for relearning it.</summary>
    public void ResetLevel(int level)
    {
        for (int lane = 0; lane < LaneSet.LaneCount; lane++)
        {
            Array.Fill(_corner[level][lane], StartingFactors[level]);
            Array.Fill(_brake[level][lane], StartingFactors[level]);
        }
    }

    private static readonly string[] LaneNames = ["free", "left", "right", "middle"];

    private static string Level(int level) => GripLevels[level].ToString("F2", CultureInfo.InvariantCulture);

    public void Save(string path)
    {
        LaneSet lanes = _lanes ?? throw new InvalidOperationException("Nothing baked yet.");
        using var writer = new StreamWriter(path);
        writer.WriteLine(
            $"# lanes length_m={lanes.LengthMeters.ToString(CultureInfo.InvariantCulture)} " +
            $"spacing_m={lanes.Spacing.ToString(CultureInfo.InvariantCulture)}");
        var header = new List<string> { "station", "s_m", "free_m", "left_m", "right_m", "half_width_m", "middle_m" };
        for (int level = 0; level < GripLevels.Length; level++)
        {
            foreach (string lane in LaneNames)
                header.Add($"corner_{lane}@{Level(level)}");
            foreach (string lane in LaneNames)
                header.Add($"brake_{lane}@{Level(level)}");
        }
        foreach (string lane in LaneNames)
            header.Add($"steer_{lane}");
        writer.WriteLine(string.Join(',', header));
        for (int i = 0; i < lanes.Count; i++)
        {
            var row = new List<string>
            {
                i.ToString(CultureInfo.InvariantCulture),
                (i * lanes.Spacing).ToString("F2", CultureInfo.InvariantCulture),
                lanes.Offset(LaneId.Free, i).ToString("F4", CultureInfo.InvariantCulture),
                lanes.Offset(LaneId.Left, i).ToString("F4", CultureInfo.InvariantCulture),
                lanes.Offset(LaneId.Right, i).ToString("F4", CultureInfo.InvariantCulture),
                lanes.HalfWidth(i).ToString("F3", CultureInfo.InvariantCulture),
                lanes.Offset(LaneId.Middle, i).ToString("F4", CultureInfo.InvariantCulture)
            };
            for (int level = 0; level < GripLevels.Length; level++)
            {
                for (int lane = 0; lane < LaneSet.LaneCount; lane++)
                    row.Add(_corner[level][lane][i].ToString("F4", CultureInfo.InvariantCulture));
                for (int lane = 0; lane < LaneSet.LaneCount; lane++)
                    row.Add(_brake[level][lane][i].ToString("F4", CultureInfo.InvariantCulture));
            }
            for (int lane = 0; lane < LaneSet.LaneCount; lane++)
                row.Add(_steer[lane][i].ToString("F6", CultureInfo.InvariantCulture));
            writer.WriteLine(string.Join(',', row));
        }
    }

    /// <summary>
    /// The energy rungs as this circuit's lift seconds, Save to Attack, or
    /// null where the circuit has none set and the driver's general table
    /// stands in.
    /// </summary>
    public LaneEnergyRungs? Energy { get; set; }

    /// <summary>What the car was when this was learned; null for a file that never said.</summary>
    public LaneLearnedFor? LearnedFor { get; set; }

    /// <summary>The tyre rungs as this circuit's styles, or null where the driver's general table stands in.</summary>
    public LaneTyreRungs? Tyres { get; set; }

    /// <summary>What the last calibration of the rungs measured.</summary>
    public LaneCalibration? Calibration { get; set; }

    /// <summary>
    /// Whether this was learned on the car as it is now: the stock car on
    /// the stock tyre, as far as their numbers tell. A file from before a
    /// change to the physics still drives, on notes about another car.
    /// </summary>
    public bool IsCurrent => LearnedFor is not null && LearnedFor.Physics == PhysicsFingerprint.Stock;

    /// <summary>Stamps the file as learned today on the car as it is now.</summary>
    public void MarkLearnedNow() =>
        LearnedFor = new LaneLearnedFor
        {
            Physics = PhysicsFingerprint.Stock,
            Date = DateTime.Now.ToString("yyyy-MM-dd", CultureInfo.InvariantCulture)
        };

    /// <summary>
    /// This car's shipped folder for the circuit, <c>Cars/{car}/{id}/</c>,
    /// or a fresh file baked on first use when it has none.
    /// </summary>
    public static LaneTrackFile ForCircuit(string id, TrackData track, string car = DriverFiles.StockCar)
    {
        string? manifestPath = DriverFiles.FindFile($"{car}/{id}/{LaneCircuitManifest.FileName}");
        if (manifestPath is null)
            return Fresh();
        LaneCircuitManifest manifest = LaneCircuitManifest.Read(manifestPath);
        string lanesPath = DriverFiles.FindFile($"{car}/{id}/{manifest.Lanes}")
                           ?? throw new FileNotFoundException($"{car}/{id}/{LaneCircuitManifest.FileName} names {manifest.Lanes}, which is not there.");
        return WithManifest(Load(lanesPath, track), manifest);
    }

    /// <summary>Reads a circuit folder from disk: its <c>driver.json</c> and what it names.</summary>
    public static LaneTrackFile LoadCircuit(string folder, TrackData track)
    {
        LaneCircuitManifest manifest = LaneCircuitManifest.Read(Path.Combine(folder, LaneCircuitManifest.FileName));
        return WithManifest(Load(Path.Combine(folder, manifest.Lanes), track), manifest);
    }

    private static LaneTrackFile WithManifest(LaneTrackFile file, LaneCircuitManifest manifest)
    {
        if (manifest.Format > LaneCircuitManifest.CurrentFormat)
            throw new InvalidDataException($"{manifest.Circuit}: format {manifest.Format} is newer than this driver reads.");
        file.Energy = manifest.Energy;
        file.LearnedFor = manifest.LearnedFor;
        file.Tyres = manifest.Tyres;
        file.Calibration = manifest.Calibration;
        return file;
    }

    /// <summary>Writes the circuit folder: <c>driver.json</c> and the lanes table beside it.</summary>
    public void SaveCircuit(string folder, string id)
    {
        Directory.CreateDirectory(folder);
        var manifest = new LaneCircuitManifest
        {
            Circuit = id, LearnedFor = LearnedFor, Energy = Energy, Tyres = Tyres, Calibration = Calibration
        };
        Save(Path.Combine(folder, manifest.Lanes));
        manifest.Write(Path.Combine(folder, LaneCircuitManifest.FileName));
    }

    public static LaneTrackFile Load(string path, TrackData track) =>
        Load(path, track.Sample, track.LengthMeters);

    /// <summary>
    /// Reads either layout: per-level columns (<c>corner_free@0.85</c>), or the
    /// older single set (<c>factor_free</c>, <c>brake_free</c>), which is taken
    /// as the full-grip level with the others left at one.
    /// </summary>
    public static LaneTrackFile Load(string path, Func<float, TrackSample> sample, float lengthMeters)
    {
        LaneSet lanes = LaneSet.Load(path, sample, lengthMeters);
        float[][][] corner = NewFactors(lanes.Count);
        float[][][] brake = NewFactors(lanes.Count);
        float[][] steer = NewSteer(lanes.Count);
        Dictionary<string, int>? columns = null;
        int row = 0;
        foreach (string line in File.ReadLines(path))
        {
            if (line.StartsWith('#'))
                continue;
            string[] cols = line.Split(',');
            if (columns is null)
            {
                columns = cols.Select((name, index) => (name, index)).ToDictionary(c => c.name, c => c.index);
                continue;
            }
            for (int lane = 0; lane < LaneSet.LaneCount; lane++)
            {
                string name = LaneNames[lane];
                for (int level = 0; level < GripLevels.Length; level++)
                {
                    if (columns.TryGetValue($"corner_{name}@{Level(level)}", out int c))
                        corner[level][lane][row] = float.Parse(cols[c], CultureInfo.InvariantCulture);
                    if (columns.TryGetValue($"brake_{name}@{Level(level)}", out int b))
                        brake[level][lane][row] = float.Parse(cols[b], CultureInfo.InvariantCulture);
                }
                if (columns.TryGetValue($"factor_{name}", out int oldCorner))
                    corner[TopLevel][lane][row] = float.Parse(cols[oldCorner], CultureInfo.InvariantCulture);
                if (columns.TryGetValue($"brake_{name}", out int oldBrake))
                    brake[TopLevel][lane][row] = float.Parse(cols[oldBrake], CultureInfo.InvariantCulture);
                if (columns.TryGetValue($"steer_{name}", out int st))
                    steer[lane][row] = float.Parse(cols[st], CultureInfo.InvariantCulture);
            }
            row++;
        }
        // A file from before the middle lane: what it knows of the side lanes
        // stands in, the more careful of the two, until the middle is learned.
        if (columns is not null && !columns.ContainsKey($"corner_middle@{Level(TopLevel)}"))
        {
            int middle = (int)LaneId.Middle, left = (int)LaneId.Left, right = (int)LaneId.Right;
            for (int level = 0; level < GripLevels.Length; level++)
            {
                for (int i = 0; i < lanes.Count; i++)
                {
                    corner[level][middle][i] = MathF.Min(corner[level][left][i], corner[level][right][i]);
                    brake[level][middle][i] = MathF.Min(brake[level][left][i], brake[level][right][i]);
                }
            }
            steer[middle] = LaneSet.Between(steer[left], steer[right]);
        }
        var file = new LaneTrackFile(lanes, corner, brake, null);
        file._steer = steer;
        return file;
    }
}
