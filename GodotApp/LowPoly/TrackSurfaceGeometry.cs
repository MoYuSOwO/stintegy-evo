using System;
using System.Numerics;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>Presentation-only reconstruction of the surface already used by Core.</summary>
public sealed class TrackSurfaceGeometry
{
    private readonly float[] _heights;
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
    }

    public float Height(float s, float d)
    {
        float position = Track.WrapS(s) / _step;
        int i = Math.Min((int)position, _heights.Length - 2);
        float height = _heights[i] + (_heights[i + 1] - _heights[i]) * (position - i);
        var sample = Track.Sample(s);
        return height + sample.BankSlope * d + sample.BankCurvature * d * d;
    }

    public float MeadowHeight(float x, float z)
    {
        var pose = Track.Project(new Vector2(x, z));
        float t = Math.Clamp((MathF.Abs(pose.D) - 18f) / 82f, 0f, 1f);
        float away = t * t * (3f - 2f * t);
        return Height(pose.S, 0f) - 2.6f + away *
            (MathF.Sin(x * 0.008f) * MathF.Cos(z * 0.006f) * 2f - 1.5f);
    }

    public Vector3 Point(float s, float d)
    {
        var sample = Track.Sample(s);
        var point = sample.Center + sample.Normal * d;
        return new Vector3(point.X, Height(s, d), point.Y);
    }
}
