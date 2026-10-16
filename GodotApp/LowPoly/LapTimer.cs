using System;
using System.Collections.Generic;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>
/// One car's lap times, from samples of its race distance and the race
/// clock. The line is crossed between two samples, and the crossing time
/// is put where the distance says it was rather than at the sample after
/// it, so a lap read off a screen refreshed a few times a second, or off a
/// simulation running ahead of the screen, is right to the millisecond.
///
/// It also keeps the best lap's time at every few metres, which is what the
/// running delta reads: where this lap is against the best at the same
/// point of the road.
/// </summary>
public sealed class LapTimer
{
    private const float TraceSpacingMeters = 10f;

    private readonly float _lapLength;
    private float _lastTime = float.NaN;
    private float _lastDistance;
    private float _lapStartTime = float.NaN;
    private int _lapIndex = int.MinValue;
    private readonly List<float> _trace = new();
    private float[] _bestTrace = [];

    public LapTimer(float lapLengthMeters) => _lapLength = MathF.Max(lapLengthMeters, 1f);

    /// <summary>Laps completed since timing began.</summary>
    public int Completed { get; private set; }
    public float LastLap { get; private set; }
    public float BestLap { get; private set; }

    /// <summary>Seconds into the lap being driven, or zero before the first crossing.</summary>
    public float Current(float raceTime) => float.IsNaN(_lapStartTime) ? 0f : MathF.Max(0f, raceTime - _lapStartTime);

    /// <summary>
    /// This lap against the best at the same distance: negative is quicker.
    /// Null until there is a best lap to read against.
    /// </summary>
    public float? Delta(float raceTime, float raceDistance)
    {
        if (_bestTrace.Length == 0 || float.IsNaN(_lapStartTime))
            return null;
        float into = raceDistance - _lapIndex * _lapLength;
        float at = into / TraceSpacingMeters;
        int i = (int)MathF.Floor(at);
        if (i < 0 || i >= _bestTrace.Length - 1)
            return null;
        float best = _bestTrace[i] + (_bestTrace[i + 1] - _bestTrace[i]) * (at - i);
        return Current(raceTime) - best;
    }

    public void Sample(float raceTime, float raceDistance)
    {
        int lap = (int)MathF.Floor(raceDistance / _lapLength);
        if (!float.IsNaN(_lastTime) && lap > _lapIndex && _lapIndex != int.MinValue)
        {
            // Where between the two samples the line was.
            float line = lap * _lapLength;
            float span = raceDistance - _lastDistance;
            float share = span > 1e-4f ? Math.Clamp((line - _lastDistance) / span, 0f, 1f) : 1f;
            float crossing = _lastTime + (raceTime - _lastTime) * share;
            if (!float.IsNaN(_lapStartTime))
            {
                LastLap = crossing - _lapStartTime;
                Completed++;
                if (BestLap <= 0f || LastLap < BestLap)
                {
                    BestLap = LastLap;
                    _trace.Add(LastLap);
                    _bestTrace = _trace.ToArray();
                }
            }
            _lapStartTime = crossing;
            _trace.Clear();
        }
        if (lap != _lapIndex)
            _lapIndex = lap;

        if (!float.IsNaN(_lapStartTime))
        {
            // The trace: time into the lap at every TraceSpacingMeters.
            float into = raceDistance - _lapIndex * _lapLength;
            while (_trace.Count * TraceSpacingMeters <= into)
            {
                float target = _trace.Count * TraceSpacingMeters + _lapIndex * _lapLength;
                float span = raceDistance - _lastDistance;
                float share = span > 1e-4f ? Math.Clamp((target - _lastDistance) / span, 0f, 1f) : 1f;
                _trace.Add(_lastTime + (raceTime - _lastTime) * share - _lapStartTime);
            }
        }
        _lastTime = raceTime;
        _lastDistance = raceDistance;
    }
}
