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
    /// How far out the circuit's own verge reaches. Inside this band the
    /// meadow is flat ground a fixed drop below the road, so the verge's
    /// rim and the meadow are the same surface where they meet; the
    /// meadow's own rolling starts here rather than under the barrier,
    /// which is where it used to rise through the verge and show sky.
    /// </summary>
    public const float VergeMeters = 24f;

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
        (float s, float d) = NearestStation(new Vector2(x, z));
        float t = Math.Clamp((MathF.Abs(d) - VergeMeters) / 82f, 0f, 1f);
        float away = t * t * (3f - 2f * t);
        return Height(s, 0f) - 2.6f + away *
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

    public Vector3 Point(float s, float d)
    {
        var sample = Track.Sample(s);
        var point = sample.Center + sample.Normal * d;
        return new Vector3(point.X, Height(s, d), point.Y);
    }
}
