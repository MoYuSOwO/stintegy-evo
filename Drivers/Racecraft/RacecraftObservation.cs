using StintegyEVO.Core.Racing;
using StintegyEVO.Drivers.Lanes;

namespace StintegyEVO.Drivers.Racecraft;

/// <summary>
/// What the racecraft layer sees each time it draws, as one row of numbers,
/// built here and only here: the training host and the game read the same
/// row. Everything is the car's own view: its own state and the line it drew
/// last, where it is round the lap, the road's shape ahead and behind, the
/// nearest cars about it, and how bold its driver is. The racing line itself
/// is not in it: the line drawn is an offset from it, and the layer learns
/// where it is from where on the lap it is.
/// </summary>
public sealed class RacecraftObservation
{
    /// <summary>The road ahead, every this many metres, this far.</summary>
    public const float ShapeStepMeters = 10f;
    public const float ShapeAheadMeters = 400f;
    public const float ShapeBehindMeters = 100f;

    /// <summary>The nearest cars within this far ahead or behind, nearest first.</summary>
    public const int CarSlots = 4;
    public const float CarRangeMeters = 150f;

    // Scales that bring each number to about one.
    private const float SpeedScale = 80f;
    private const float OffsetScale = 5f;
    private const float RoomScale = 10f;
    private const float CurvatureScale = 50f;
    private const float AccelScale = 20f;
    private const float RelativeSpeedScale = 20f;

    private const int OwnSize = 7;
    private const int CarSize = 6;
    private static readonly int AheadPoints = (int)(ShapeAheadMeters / ShapeStepMeters);
    private static readonly int BehindPoints = (int)(ShapeBehindMeters / ShapeStepMeters);

    /// <summary>How many numbers the row has.</summary>
    public static int Size =>
        OwnSize + RacecraftAction.Size + 2 + 2 * (AheadPoints + BehindPoints + 1) + CarSlots * CarSize + 1;

    private readonly LaneSet _lanes;
    private readonly (float Along, int Index)[] _near = new (float, int)[64];

    public RacecraftObservation(LaneSet lanes)
    {
        _lanes = lanes;
    }

    /// <summary>
    /// The row for car <paramref name="self"/> of <paramref name="frame"/>,
    /// which last drew <paramref name="previous"/> (zeros before its first
    /// drawing), its driver as bold as <paramref name="boldness"/> (0..100).
    /// </summary>
    public void Write(RaceFrameSnapshot frame, int self, ReadOnlySpan<float> previous, float boldness, Span<float> row)
    {
        if (row.Length != Size)
            throw new ArgumentException($"The row has {Size} numbers, not {row.Length}.", nameof(row));
        if (previous.Length != RacecraftAction.Size)
            throw new ArgumentException($"The last drawing has {RacecraftAction.Size} numbers.", nameof(previous));
        RaceCarSnapshot car = frame[self];
        float length = MathF.Max(car.TrackLengthMeters, 1f);
        float station = _lanes.StationOf(car.TrackS);
        int here = (int)MathF.Round(station);
        float spacing = _lanes.Spacing;
        int k = 0;

        // Its own state: speed, where it is across the road against the
        // racing line and the edges, which way it heads against the road and
        // how fast it is turning, and how hard it is braking or driving.
        float free = _lanes.Offset(LaneId.Free, here);
        float half = _lanes.HalfWidth(here);
        float roadHeading = Heading(here);
        row[k++] = car.SpeedMetersPerSecond / SpeedScale;
        row[k++] = (car.TrackD - free) / OffsetScale;
        row[k++] = (half - car.TrackD) / RoomScale;
        row[k++] = (half + car.TrackD) / RoomScale;
        row[k++] = Wrap(car.VelocityHeadingRadians - roadHeading);
        row[k++] = car.YawRateRadiansPerSecond;
        row[k++] = (car.Motion?.StepLongitudinalAccel ?? 0f) / AccelScale;

        // The line it drew last, as it drew it.
        previous.CopyTo(row[k..]);
        k += previous.Length;

        // Where round the lap, as an angle, so the end of the lap meets its start.
        float angle = 2f * MathF.PI * car.TrackS / length;
        row[k++] = MathF.Sin(angle);
        row[k++] = MathF.Cos(angle);

        // The road's shape, from behind the car to ahead of it: how much the
        // middle of the road bends, and how wide it is, every ten metres.
        for (int p = -BehindPoints; p <= AheadPoints; p++)
        {
            int at = here + (int)MathF.Round(p * ShapeStepMeters / spacing);
            int chord = (int)MathF.Round(ShapeStepMeters / spacing);
            row[k++] = CurvatureScale * LaneSet.Curvature(_lanes.Centre(at - chord), _lanes.Centre(at), _lanes.Centre(at + chord));
            row[k++] = _lanes.HalfWidth(at) / RoomScale;
        }

        // The nearest cars about it: whether the slot has one, how far ahead
        // (behind negative), how far aside of it and of the racing line,
        // how much faster, and which way it heads against this car.
        int count = 0;
        for (int i = 0; i < frame.Count && count < _near.Length; i++)
        {
            if (i == self)
                continue;
            float along = ((frame[i].TrackS - car.TrackS) % length + length) % length;
            if (along > length * 0.5f)
                along -= length;
            if (MathF.Abs(along) <= CarRangeMeters)
                _near[count++] = (along, i);
        }
        Array.Sort(_near, 0, count, Comparer<(float Along, int Index)>.Create((a, b) => MathF.Abs(a.Along).CompareTo(MathF.Abs(b.Along))));
        for (int slot = 0; slot < CarSlots; slot++)
        {
            if (slot >= count)
            {
                row.Slice(k, CarSize).Clear();
                k += CarSize;
                continue;
            }
            RaceCarSnapshot other = frame[_near[slot].Index];
            int theirs = (int)MathF.Round(_lanes.StationOf(other.TrackS));
            row[k++] = 1f;
            row[k++] = _near[slot].Along / CarRangeMeters;
            row[k++] = (other.TrackD - car.TrackD) / OffsetScale;
            row[k++] = (other.TrackD - _lanes.Offset(LaneId.Free, theirs)) / OffsetScale;
            row[k++] = (other.SpeedMetersPerSecond - car.SpeedMetersPerSecond) / RelativeSpeedScale;
            row[k++] = Wrap(other.VelocityHeadingRadians - car.VelocityHeadingRadians);
        }

        row[k++] = boldness / 100f;
    }

    private float Heading(int station)
    {
        var d = _lanes.Centre(station + 1) - _lanes.Centre(station - 1);
        return MathF.Atan2(d.Y, d.X);
    }

    private static float Wrap(float angle) => MathF.Atan2(MathF.Sin(angle), MathF.Cos(angle));
}
