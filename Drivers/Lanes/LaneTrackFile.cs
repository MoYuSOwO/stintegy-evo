using System.Globalization;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// One circuit's lanes file: the four lanes, baked offline, with the stamp of
/// the car they were baked against. Nothing is learned beforehand; what a
/// driver learns, it learns as it drives (<see cref="LaneFamiliarity"/>).
/// </summary>
public sealed class LaneTrackFile
{
    private LaneSet? _lanes;
    private readonly LaneBakeOptions _bake;

    private LaneTrackFile(LaneSet? lanes, LaneBakeOptions? bake)
    {
        _lanes = lanes;
        _bake = bake ?? new LaneBakeOptions();
    }

    /// <summary>A file with no lanes yet; they are baked against the first track it meets.</summary>
    public static LaneTrackFile Fresh(LaneBakeOptions? bake = null) => new(null, bake);

    public static LaneTrackFile For(LaneSet lanes) => new(lanes, null);

    /// <summary>A file on lanes from the offline 3D bake.</summary>
    public static LaneTrackFile FromBakedLanes(string bakedPath, TrackData track, LaneBakeOptions? fallbackBake = null) =>
        new(LaneSet.ImportBaked(bakedPath, track, LaneSet.Bake(track, fallbackBake)), null);

    public LaneSet LanesFor(DriverTrackView track) => LanesFor(track.LengthMeters, () => LaneSet.Bake(track, _bake));

    public LaneSet LanesFor(TrackData track) => LanesFor(track.LengthMeters, () => LaneSet.Bake(track, _bake));

    private LaneSet LanesFor(float lengthMeters, Func<LaneSet> bake)
    {
        if (_lanes is null || MathF.Abs(_lanes.LengthMeters - lengthMeters) > 1f)
            _lanes = bake();
        return _lanes;
    }

    public LaneTrackFile Clone() => new(_lanes, _bake) { Stamp = Stamp };

    /// <summary>The same stamp on lanes drawn differently over the same road.</summary>
    public LaneTrackFile WithLanes(LaneSet lanes)
    {
        if (_lanes is null || lanes.Count != _lanes.Count)
            throw new ArgumentException("The lanes must lie on the same stations.", nameof(lanes));
        LaneTrackFile copy = Clone();
        copy._lanes = lanes;
        return copy;
    }

    public void Save(string path)
    {
        LaneSet lanes = _lanes ?? throw new InvalidOperationException("Nothing baked yet.");
        using var writer = new StreamWriter(path);
        writer.WriteLine(
            $"# lanes length_m={lanes.LengthMeters.ToString(CultureInfo.InvariantCulture)} " +
            $"spacing_m={lanes.Spacing.ToString(CultureInfo.InvariantCulture)}");
        writer.WriteLine("station,s_m,free_m,left_m,right_m,half_width_m,middle_m");
        for (int i = 0; i < lanes.Count; i++)
        {
            writer.WriteLine(string.Join(',',
                i.ToString(CultureInfo.InvariantCulture),
                (i * lanes.Spacing).ToString("F2", CultureInfo.InvariantCulture),
                lanes.Offset(LaneId.Free, i).ToString("F4", CultureInfo.InvariantCulture),
                lanes.Offset(LaneId.Left, i).ToString("F4", CultureInfo.InvariantCulture),
                lanes.Offset(LaneId.Right, i).ToString("F4", CultureInfo.InvariantCulture),
                lanes.HalfWidth(i).ToString("F3", CultureInfo.InvariantCulture),
                lanes.Offset(LaneId.Middle, i).ToString("F4", CultureInfo.InvariantCulture)));
        }
    }

    /// <summary>Which car these lanes were baked for, and when; null for a file that never said.</summary>
    public NotesStamp? Stamp { get; set; }

    /// <summary>
    /// Whether this was baked for the stock car as it is now.
    /// </summary>
    public bool IsCurrent => Stamp?.CarFingerprint == StockCar.Loaded.Fingerprint;

    /// <summary>Stamps the file as baked today by this driver on the stock car as it is now.</summary>
    public void MarkLearnedNow() =>
        Stamp = new NotesStamp
        {
            Driver = LanesPack.Id.ToString(),
            Car = StockCar.Id,
            CarFingerprint = StockCar.Loaded.Fingerprint,
            Learned = DateTime.Now.ToString("yyyy-MM-dd", CultureInfo.InvariantCulture)
        };

    /// <summary>
    /// The lanes this driver has for <paramref name="car"/> (the stock car
    /// unless said) at the circuit, from the car's pack, or a fresh file
    /// baked on first use where it has none.
    /// </summary>
    public static LaneTrackFile ForCircuit(string id, TrackData track, ModCar? car = null) =>
        FromNotes(DriverNotes.For(LanesPack.Id, car ?? StockCar.Loaded, ResourceId.Parse(id)), track);

    /// <summary>The driver's notes as a lane file, or a fresh one where there are none.</summary>
    public static LaneTrackFile FromNotes(DriverNotes notes, TrackData track)
    {
        if (notes.ReadText(LaneCircuitManifest.FileName) is not { } json)
            return Fresh();
        LaneCircuitManifest manifest = LaneCircuitManifest.Parse(json);
        if (notes.ReadText(manifest.Lanes) is null)
            throw new FileNotFoundException($"{notes.Folder}/{LaneCircuitManifest.FileName} names {manifest.Lanes}, which is not there.");
        LaneTrackFile file = WithManifest(Load(Path.Combine(notes.Folder, manifest.Lanes), track), manifest);
        file.Stamp = notes.Stamp;
        return file;
    }

    /// <summary>Reads a circuit folder from disk: its <c>lanes.json</c>, what it names, and its stamp.</summary>
    public static LaneTrackFile LoadCircuit(string folder, TrackData track)
    {
        LaneCircuitManifest manifest = LaneCircuitManifest.Parse(File.ReadAllText(Path.Combine(folder, LaneCircuitManifest.FileName)));
        LaneTrackFile file = WithManifest(Load(Path.Combine(folder, manifest.Lanes), track), manifest);
        string stamp = Path.Combine(folder, DriverNotes.StampFile);
        file.Stamp = File.Exists(stamp) ? NotesStamp.Parse(File.ReadAllText(stamp)) : null;
        return file;
    }

    private static LaneTrackFile WithManifest(LaneTrackFile file, LaneCircuitManifest manifest)
    {
        if (manifest.Format > LaneCircuitManifest.CurrentFormat)
            throw new InvalidDataException($"{manifest.Circuit}: format {manifest.Format} is newer than this driver reads.");
        return file;
    }

    /// <summary>Writes the circuit folder: <c>lanes.json</c>, the lanes table beside it, and the stamp.</summary>
    public void SaveCircuit(string folder, string id)
    {
        Directory.CreateDirectory(folder);
        var manifest = new LaneCircuitManifest
        {
            Circuit = id
        };
        Save(Path.Combine(folder, manifest.Lanes));
        File.WriteAllText(Path.Combine(folder, LaneCircuitManifest.FileName), manifest.ToJson());
        if (Stamp is not null)
            File.WriteAllText(Path.Combine(folder, DriverNotes.StampFile), Stamp.ToJson());
    }

    public static LaneTrackFile Load(string path, TrackData track) =>
        Load(path, track.Sample, track.LengthMeters);

    /// <summary>Reads the lanes table: station, s_m, free_m, left_m, right_m, half_width_m and, where there is one, middle_m.</summary>
    public static LaneTrackFile Load(string path, Func<float, TrackSample> sample, float lengthMeters) =>
        new(LaneSet.Load(path, sample, lengthMeters), null);
}
