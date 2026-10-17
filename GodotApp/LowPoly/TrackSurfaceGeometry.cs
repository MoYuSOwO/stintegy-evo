using System;
using System.Numerics;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>Presentation-only reconstruction of the surface already used by Core.</summary>
public sealed class TrackSurfaceGeometry
{
    /// <summary>How far apart the coarse stations the meadow measures from are.</summary>
    private const float StationStepMeters = 5f;

    /// <summary>
    /// How wide the verge is: the band beyond the barrier over which the
    /// ground eases from the road's edge down to the meadow, and inside
    /// which the meadow itself does not roll.
    /// </summary>
    public const float VergeMeters = 24f;

    /// <summary>
    /// How far round a point of the meadow the road it must stay under
    /// reaches. The meadow is a coarse grid, and where two parts of the
    /// lap pass close at different heights — Monaco's hillside — one cell
    /// can have a corner by the high road and a corner by the low one;
    /// sloped between them it cuts through the low road. Taking the lowest
    /// road within this reach keeps every corner of any cell the road
    /// crosses beneath it: it covers a cell's diagonal and a station step.
    /// </summary>
    private const float BeneathMeters = 30f;

    private readonly float[] _heights;
    private readonly Vector2[] _stations;
    private readonly float _step;
    public TrackData Track { get; }
    public Vector3 Minimum { get; }
    public Vector3 Maximum { get; }

    public TrackSurfaceGeometry(TrackData track)
    {
        Track = track ?? throw new ArgumentNullException(nameof(track));
        int count = Math.Max(2, (int)MathF.Ceiling(track.LengthMeters));
        _step = track.LengthMeters / count;
        _heights = new float[count + 1];
        for (int i = 1; i <= count; i++)
            _heights[i] = _heights[i - 1] + 0.5f * _step *
                (track.Sample((i - 1) * _step).Grade + track.Sample(i * _step).Grade);
        // Grade integration can retain a tiny residual on a closed circuit.
        float drift = _heights[count];
        for (int i = 0; i <= count; i++)
            _heights[i] -= drift * i / count;
        Vector3 min = new(float.PositiveInfinity), max = new(float.NegativeInfinity);
        for (int i = 0; i < count; i++)
        {
            var point = Point(i * _step, 0f);
            min = Vector3.Min(min, point);
            max = Vector3.Max(max, point);
        }
        Minimum = min;
        Maximum = max;
        _stations = new Vector2[
            Math.Max(2, (int)MathF.Ceiling(track.LengthMeters / StationStepMeters))
        ];
        for (int i = 0; i < _stations.Length; i++)
            _stations[i] = track.Sample(i * StationStepMeters).Center;
    }

    public float Height(float s, float d)
    {
        float position = Track.WrapS(s) / _step;
        int i = Math.Min((int)position, _heights.Length - 2);
        float height = _heights[i] + (_heights[i + 1] - _heights[i]) * (position - i);
        var sample = Track.Sample(s);
        return height + sample.BankSlope * d + sample.BankCurvature * d * d;
    }

    /// <summary>
    /// The ground under a point of the meadow, which is most of the world
    /// and nearly all of it far outside the barriers.
    ///
    /// It does not ask Core where that point is on the track, because for
    /// a point out in the fields there is no answer: the projection index
    /// covers the road plus the width a car's body can reach, and a query
    /// past that throws rather than scanning the whole circuit for a node
    /// that is not there. That is the right behaviour for a car and the
    /// wrong question for scenery, so the scenery answers its own: the
    /// nearest of a coarse ring of centreline stations, which is all a
    /// grass height needs and cannot fail anywhere on the map.
    /// </summary>
    public float MeadowHeight(float x, float z)
    {
        var point = new Vector2(x, z);
        (float s, float d) = NearestStation(point);
        float road = Height(s, 0f);
        for (int i = 0; i < _stations.Length; i++)
            if (Vector2.DistanceSquared(_stations[i], point) < BeneathMeters * BeneathMeters)
                road = MathF.Min(road, Height(i * StationStepMeters, 0f));
        float t = Math.Clamp((MathF.Abs(d) - VergeMeters) / 82f, 0f, 1f);
        float away = t * t * (3f - 2f * t);
        return road - 2.6f + away *
            (MathF.Sin(x * 0.008f) * MathF.Cos(z * 0.006f) * 2f - 1.5f);
    }

    /// <summary>
    /// Where a point sits against the coarse centreline: the station it is
    /// nearest to, and how far to the side of it. Exact enough for ground
    /// that is about to have a sine wave added to it, and — unlike Core's
    /// projection — defined for a point anywhere on the map, which is what
    /// scenery placement needs.
    /// </summary>
    public (float S, float D) NearestStation(Vector2 point)
    {
        int nearest = 0;
        float best = float.PositiveInfinity;
        for (int i = 0; i < _stations.Length; i++)
        {
            float distance = Vector2.DistanceSquared(_stations[i], point);
            if (distance < best)
            {
                best = distance;
                nearest = i;
            }
        }
        float s = nearest * StationStepMeters;
        var sample = Track.Sample(s);
        return (s, Vector2.Dot(point - sample.Center, sample.Normal));
    }

    /// <summary>
    /// The ground at a point off the road: at the barrier, the road's own
    /// edge height, so the terrain mesh and the run-off meet in one line;
    /// across the verge it eases down to the meadow. The terrain's
    /// vertices are set from this, and scenery stands on it.
    /// </summary>
    public float GroundHeight(float x, float z)
    {
        (float s, float d) = Project(new Vector2(x, z));
        int side = d >= 0f ? 1 : -1;
        float barrier = BarrierOffset(s, side);
        float t = Math.Clamp((MathF.Abs(d) - barrier) / VergeMeters, 0f, 1f);
        float ease = t * t * (3f - 2f * t);
        float edge = Height(s, side * barrier);
        return edge + (MeadowHeight(x, z) - edge) * ease;
    }

    /// <summary>How far from the centreline the barrier stands on this side: the road and its buffer.</summary>
    public float BarrierOffset(float s, int side)
    {
        var sample = Track.Sample(s);
        return sample.HalfWidth + (side > 0 ? sample.LeftBufferWidth : sample.RightBufferWidth);
    }

    /// <summary>
    /// Where a point sits against the centreline, to well under a metre:
    /// the nearest coarse station, then slid along the tangent onto the
    /// foot of the perpendicular. Defined anywhere on the map.
    /// </summary>
    public (float S, float D) Project(Vector2 point)
    {
        float s = NearestStation(point).S;
        for (int i = 0; i < 3; i++)
        {
            var sample = Track.Sample(s);
            float along = Math.Clamp(Vector2.Dot(point - sample.Center, sample.Tangent),
                -StationStepMeters, StationStepMeters);
            s = Track.WrapS(s + along);
        }
        var foot = Track.Sample(s);
        return (s, Vector2.Dot(point - foot.Center, foot.Normal));
    }

    public Vector3 Point(float s, float d)
    {
        var sample = Track.Sample(s);
        var point = sample.Center + sample.Normal * d;
        return new Vector3(point.X, Height(s, d), point.Y);
    }
}
