using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Text;

namespace StintegyEVO.Core.Track;

/// <summary>Where the pit boxes stand along a pit lane, as a circuit folder writes it down.</summary>
public sealed record CircuitPitBoxes
{
    /// <summary>How many boxes the lane has.</summary>
    public int Count { get; init; }

    /// <summary>How far along the lane the first box is, in metres from where the lane leaves the track.</summary>
    public float FirstMeters { get; init; }

    /// <summary>Box to box along the lane.</summary>
    public float SpacingMeters { get; init; }

    /// <summary>How far to the side of the lane's middle a car stops, positive to the left: the working lane, by the garages.</summary>
    public float OffsetMeters { get; init; }
}

/// <summary>
/// The pit lane in a circuit's <c>circuit.json</c>. The lane itself, its
/// middle and its width, is a file of its own beside the track file, one
/// row a metre from where it leaves the track to where it comes back.
/// </summary>
public sealed record CircuitPitLane
{
    /// <summary>The lane, relative to the folder.</summary>
    public string Track { get; init; } = "pitlane.csv";

    /// <summary>The lane, in metres: one less than the number of rows, since the lane has two ends.</summary>
    public int LengthMeters { get; init; }

    /// <summary>The row of the track file where the lane leaves the track: its first row is that row's point of the centreline.</summary>
    public int EntryMeters { get; init; }

    /// <summary>The row of the track file where the lane comes back: its last row is that row's point of the centreline.</summary>
    public int ExitMeters { get; init; }

    /// <summary>The speed limit between the two lines, km/h. Each circuit's own: sixty at Monaco, eighty at most.</summary>
    public float SpeedLimitKph { get; init; }

    /// <summary>How far along the lane the limit begins.</summary>
    public float SpeedLimitFromMeters { get; init; }

    /// <summary>How far along the lane the limit ends.</summary>
    public float SpeedLimitToMeters { get; init; }

    public CircuitPitBoxes Boxes { get; init; } = new();
}

/// <summary>A point of a pit lane: how far along it, where, which way, how wide and how high.</summary>
public readonly record struct PitLaneSample(float U, Vector2 Center, Vector2 Tangent, float Width, float Z)
{
    /// <summary>Left of the direction of travel, as on the track.</summary>
    public Vector2 Normal => new(Tangent.Y, -Tangent.X);
    public float HalfWidth => Width * 0.5f;
}

/// <summary>
/// Where a position sits against a pit lane: how far along it (<see cref="U"/>,
/// held to the lane's two ends), how far to the side of its middle
/// (<see cref="D"/>, positive left), and how far past an end it is, which
/// is zero for any position alongside the lane.
/// </summary>
public readonly record struct PitLanePose(float U, float D, float BeyondEnd, PitLaneSample Sample)
{
    /// <summary>On the lane's surface: alongside it and within its width.</summary>
    public bool OnLane => BeyondEnd <= 0f && MathF.Abs(D) <= Sample.HalfWidth;
}

/// <summary>Where a car stops: its box, counted from one at the lane's entry end.</summary>
public readonly record struct PitBox(int Number, float U, Vector2 Position, Vector2 Tangent);

/// <summary>
/// A circuit's pit lane: a road of its own beside the track that leaves it
/// at <see cref="EntryS"/> and comes back at <see cref="ExitS"/>, with a
/// speed limit between two lines and boxes where cars stop.
///
/// Like the track, the lane is the file, finished: its middle a row a
/// metre, its width at every row. Nothing here smooths or moves it. What
/// it answers is where a position is on it, whether that is on it at all,
/// and what the limit is there. How a car gets into it, and what it does
/// once it is in, is not the lane's business.
/// </summary>
public sealed class PitLane
{
    private readonly Vector2[] _centers;
    private readonly float[] _widths;
    private readonly float[] _heights;
    private readonly Vector2 _min;
    private readonly Vector2 _max;

    /// <summary>The lane from end to end, in metres along its middle.</summary>
    public float LengthMeters => (_centers.Length - 1) * TrackData.StepLength;

    /// <summary>Where on the track the lane leaves it.</summary>
    public float EntryS { get; }

    /// <summary>Where on the track the lane comes back.</summary>
    public float ExitS { get; }

    /// <summary>The limit, metres a second.</summary>
    public float SpeedLimit { get; }

    /// <summary>How far along the lane the limit begins and ends.</summary>
    public float SpeedLimitFromU { get; }
    public float SpeedLimitToU { get; }

    public IReadOnlyList<PitBox> Boxes { get; }

    internal PitLane(
        IReadOnlyList<PitLaneFile.Row> rows,
        float entryS,
        float exitS,
        float speedLimit,
        float limitFrom,
        float limitTo,
        CircuitPitBoxes boxes
    )
    {
        _centers = new Vector2[rows.Count];
        _widths = new float[rows.Count];
        _heights = new float[rows.Count];
        Vector2 min = new(float.PositiveInfinity), max = new(float.NegativeInfinity);
        float widest = 0f;
        for (int i = 0; i < rows.Count; i++)
        {
            _centers[i] = rows[i].Center;
            _widths[i] = rows[i].Width;
            _heights[i] = rows[i].Z;
            min = Vector2.Min(min, rows[i].Center);
            max = Vector2.Max(max, rows[i].Center);
            widest = MathF.Max(widest, rows[i].Width);
        }
        _min = min - new Vector2(widest);
        _max = max + new Vector2(widest);
        EntryS = entryS;
        ExitS = exitS;
        SpeedLimit = speedLimit;
        SpeedLimitFromU = limitFrom;
        SpeedLimitToU = limitTo;

        var list = new List<PitBox>(boxes.Count);
        for (int i = 0; i < boxes.Count; i++)
        {
            PitLaneSample at = Sample(boxes.FirstMeters + i * boxes.SpacingMeters);
            list.Add(new PitBox(i + 1, at.U, at.Center + at.Normal * boxes.OffsetMeters, at.Tangent));
        }
        Boxes = list;
    }

    /// <summary>The lane at <paramref name="u"/> metres from its entry end, held to its two ends.</summary>
    public PitLaneSample Sample(float u)
    {
        float scaled = Math.Clamp(u, 0f, LengthMeters) / TrackData.StepLength;
        int i = Math.Min((int)scaled, _centers.Length - 2);
        float t = scaled - i;
        return new PitLaneSample(
            scaled * TrackData.StepLength,
            Vector2.Lerp(_centers[i], _centers[i + 1], t),
            Tangent(i, t),
            _widths[i] + (_widths[i + 1] - _widths[i]) * t,
            _heights[i] + (_heights[i + 1] - _heights[i]) * t
        );
    }

    /// <summary>
    /// Where a position is against the lane. Defined anywhere: a position
    /// nowhere near it is simply far to its side or past an end.
    /// </summary>
    public PitLanePose Project(Vector2 position)
    {
        float best = float.PositiveInfinity;
        int bestSegment = 0;
        float bestT = 0f;
        for (int i = 0; i + 1 < _centers.Length; i++)
        {
            Vector2 a = _centers[i], ab = _centers[i + 1] - a;
            float t = Math.Clamp(Vector2.Dot(position - a, ab) / ab.LengthSquared(), 0f, 1f);
            float distance = Vector2.DistanceSquared(position, a + ab * t);
            if (distance < best)
            {
                best = distance;
                bestSegment = i;
                bestT = t;
            }
        }
        PitLaneSample sample = Sample((bestSegment + bestT) * TrackData.StepLength);
        Vector2 offset = position - sample.Center;
        float along = Vector2.Dot(offset, sample.Tangent);
        float beyond = sample.U <= 0f ? -along : sample.U >= LengthMeters ? along : 0f;
        return new PitLanePose(sample.U, Vector2.Dot(offset, sample.Normal), MathF.Max(0f, beyond), sample);
    }

    /// <summary>Whether a position is on the lane's surface.</summary>
    public bool Contains(Vector2 position) =>
        position.X >= _min.X && position.X <= _max.X && position.Y >= _min.Y && position.Y <= _max.Y
        && Project(position).OnLane;

    /// <summary>The limit at <paramref name="u"/> along the lane, or null outside the two lines.</summary>
    public float? SpeedLimitAt(float u) =>
        u >= SpeedLimitFromU && u <= SpeedLimitToU ? SpeedLimit : null;

    private Vector2 Tangent(int segment, float t)
    {
        // The direction of each row is the chord through its neighbours,
        // as on the track; between rows it turns smoothly from one to the next.
        Vector2 At(int i)
        {
            int a = Math.Max(0, i - 1), b = Math.Min(_centers.Length - 1, i + 1);
            return Vector2.Normalize(_centers[b] - _centers[a]);
        }
        Vector2 direction = Vector2.Lerp(At(segment), At(segment + 1), t);
        return direction.LengthSquared() > 1e-8f ? Vector2.Normalize(direction) : At(segment);
    }
}

/// <summary>
/// A pit lane as it is stored, and the one way it becomes a <see cref="PitLane"/>:
/// checked against the track it belongs to, so a lane that does not start
/// and end on points of the track's centreline, or a limit that is not on
/// the lane, is refused when the circuit loads rather than found out in a race.
/// </summary>
public static class PitLaneFile
{
    public static readonly string[] Columns = ["x_m", "y_m", "z_m", "width_m"];

    public readonly record struct Row(Vector2 Center, float Z, float Width);

    /// <summary>
    /// How far a lane's first or last row may be from the centreline point
    /// it leaves or rejoins at: the two files' rounding, and nothing more.
    /// The ends are those points.
    /// </summary>
    public const float EndTolerance = 0.01f;

    public static PitLane Parse(CircuitPitLane lane, string csv, TrackData track, string id = "")
    {
        ArgumentNullException.ThrowIfNull(lane);
        ArgumentNullException.ThrowIfNull(track);
        IReadOnlyList<Row> rows = ReadRows(csv, id);
        string Where() => $"Pit lane of '{id}'";
        if (lane.LengthMeters != rows.Count - 1)
            throw new InvalidDataException($"{Where()} says it is {lane.LengthMeters} m and its file has {rows.Count} rows.");
        for (int i = 0; i + 1 < rows.Count; i++)
        {
            float step = Vector2.Distance(rows[i].Center, rows[i + 1].Center);
            if (MathF.Abs(step - TrackData.StepLength) > CircuitTrackFile.SpacingTolerance)
                throw new InvalidDataException($"{Where()}: row {i} is {step:F3} m from the next, not a metre.");
            if (rows[i].Width <= 0f)
                throw new InvalidDataException($"{Where()}: row {i} has no width.");
        }
        if (lane.EntryMeters < 0 || lane.EntryMeters >= track.LengthMeters || lane.ExitMeters < 0 || lane.ExitMeters >= track.LengthMeters)
            throw new InvalidDataException($"{Where()} leaves or rejoins the track off the end of its {track.LengthMeters} m.");
        EndOnRoad(rows[0].Center, lane.EntryMeters, "leaves");
        EndOnRoad(rows[^1].Center, lane.ExitMeters, "rejoins");
        float length = lane.LengthMeters;
        if (lane.SpeedLimitKph <= 0f)
            throw new InvalidDataException($"{Where()} has no speed limit.");
        if (lane.SpeedLimitFromMeters < 0f || lane.SpeedLimitToMeters > length || lane.SpeedLimitFromMeters >= lane.SpeedLimitToMeters)
            throw new InvalidDataException($"{Where()}: the limit lines are not on the lane.");
        CircuitPitBoxes boxes = lane.Boxes;
        if (boxes.Count < 0 || (boxes.Count > 0 && (boxes.FirstMeters < lane.SpeedLimitFromMeters
            || boxes.FirstMeters + (boxes.Count - 1) * boxes.SpacingMeters > lane.SpeedLimitToMeters)))
            throw new InvalidDataException($"{Where()}: the boxes are not between the limit lines.");

        return new PitLane(
            rows,
            lane.EntryMeters * TrackData.StepLength,
            lane.ExitMeters * TrackData.StepLength,
            lane.SpeedLimitKph / 3.6f,
            lane.SpeedLimitFromMeters,
            lane.SpeedLimitToMeters,
            boxes
        );

        void EndOnRoad(Vector2 end, int s, string what)
        {
            Vector2 centre = track.Sample(s * TrackData.StepLength).Center;
            if (Vector2.Distance(end, centre) > EndTolerance)
                throw new InvalidDataException($"{Where()} {what} the track {Vector2.Distance(end, centre):F2} m from the centreline's point at {s} m.");
        }
    }

    public static IReadOnlyList<Row> ReadRows(string csv, string id = "")
    {
        ArgumentNullException.ThrowIfNull(csv);
        var rows = new List<Row>();
        int[]? index = null;
        using var reader = new StringReader(csv);
        while (reader.ReadLine() is { } line)
        {
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith('#'))
                continue;
            string[] fields = line.Split(',');
            if (index is null)
            {
                index = new int[Columns.Length];
                for (int c = 0; c < Columns.Length; c++)
                {
                    index[c] = Array.IndexOf(fields, Columns[c]);
                    if (index[c] < 0)
                        throw new InvalidDataException($"Pit lane file of '{id}' has no '{Columns[c]}' column.");
                }
                continue;
            }
            float F(int c) => float.Parse(fields[index[c]], NumberStyles.Float, CultureInfo.InvariantCulture);
            rows.Add(new Row(new Vector2(F(0), F(1)), F(2), F(3)));
        }
        if (rows.Count < 2)
            throw new InvalidDataException($"Pit lane file of '{id}' has {rows.Count} rows; a lane needs two ends.");
        return rows;
    }

    /// <summary>Writes rows in the form <see cref="ReadRows"/> reads.</summary>
    public static string Write(IReadOnlyList<Row> rows, string header = "")
    {
        var text = new StringBuilder();
        if (header.Length > 0)
            text.Append("# ").Append(header).Append('\n');
        text.Append(string.Join(',', Columns)).Append('\n');
        foreach (Row row in rows)
        {
            text.Append(row.Center.X.ToString("F5", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.Center.Y.ToString("F5", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.Z.ToString("F4", CultureInfo.InvariantCulture)).Append(',')
                .Append(row.Width.ToString("F4", CultureInfo.InvariantCulture)).Append('\n');
        }
        return text.ToString();
    }
}
