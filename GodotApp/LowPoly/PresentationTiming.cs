using System;
using System.Collections.Generic;

namespace StintegyEVO.GodotApp.LowPoly;

public sealed class FixedStepBudget
{
    public const double StepSeconds = 1.0 / 60.0;
    private double _seconds;
    // Bound catch-up after long stalls without discarding ordinary fractional frames.
    public void Advance(double delta) => _seconds = Math.Min(_seconds + delta, 6 * StepSeconds);
    public int TakeSteps()
    {
        int count = (int)Math.Floor((_seconds + 1e-9) / StepSeconds);
        _seconds = Math.Max(0, _seconds - count * StepSeconds);
        return count;
    }
    public void Reset() => _seconds = 0;
}

public sealed class SnapshotTimeline<T>
{
    private readonly Queue<(double Time, T Value)> _next = new();
    private (double Time, T Value) _previous;
    public SnapshotTimeline(T initial) => _previous = (0, initial);
    public void Add(double time, T value) => _next.Enqueue((time, value));
    public (T Previous, T Next, float Fraction) Sample(double time)
    {
        while (_next.TryPeek(out var frame) && frame.Time <= time)
            _previous = _next.Dequeue();
        if (!_next.TryPeek(out var next)) return (_previous.Value, _previous.Value, 0);
        float fraction = (float)Math.Clamp((time - _previous.Time) / (next.Time - _previous.Time), 0, 1);
        return (_previous.Value, next.Value, fraction);
    }
}
