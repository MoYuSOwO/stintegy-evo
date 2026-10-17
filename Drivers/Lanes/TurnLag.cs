using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// How long this car takes to take up a turn: from the wheel asking for a
/// curvature to the car's path having it, read as the delay that a steadily
/// tightening or opening corner sees. A driver who reads the corner that far
/// ahead has the car on the line; one who reads it further ahead turns in
/// early and runs wide on the way out, by as much as the feedback needs to
/// pull the difference back.
///
/// Measured once, on the car itself, by asking a car already in a corner
/// for a little more of it at a few speeds and timing the path coming
/// round: no number to tune, and a different car brings its own.
/// </summary>
public sealed class TurnLag
{
    private static readonly float[] Speeds = [15f, 25f, 40f, 60f, 85f];
    private const float Dt = 1f / 60f;
    // A corner taken at about half of what the car holds, and a turn more
    // of the wheel small enough to stay well inside the rest.
    private const float SettledLateralAccel = 12f;
    private const float StepLateralAccel = 2f;
    private const float SettleSeconds = 1.5f;
    private const float WindowSeconds = 1f;

    private readonly float[] _lag;

    private TurnLag(float[] lag) => _lag = lag;

    public static TurnLag Measure(CarConfig car, TireConfig tires)
    {
        var lag = new float[Speeds.Length];
        for (int i = 0; i < Speeds.Length; i++)
            lag[i] = At(car, tires, Speeds[i]);
        return new TurnLag(lag);
    }

    /// <summary>The delay at <paramref name="speed"/>, between the speeds it was measured at.</summary>
    public float Seconds(float speed)
    {
        if (speed <= Speeds[0])
            return _lag[0];
        for (int i = 1; i < Speeds.Length; i++)
        {
            if (speed <= Speeds[i])
            {
                float u = (speed - Speeds[i - 1]) / (Speeds[i] - Speeds[i - 1]);
                return _lag[i - 1] + (_lag[i] - _lag[i - 1]) * u;
            }
        }
        return _lag[^1];
    }

    // Already in a corner, the car asked for a little more of it: the path's
    // curvature moves from where it had settled to where it settles next,
    // and the area between that move and an instant one, over the size of
    // the move, is exactly how far behind a steadily changing corner the
    // path runs. Measured from a settled corner rather than from straight,
    // because that is where the line is followed: a car going straight has
    // its slip still to build, and takes several times as long to answer the
    // first turn of the wheel. Measured against what the car settles to and
    // not what was asked for, because at speed the car takes up less of the
    // ask than it was given, and that shortfall is the feedback's to close,
    // not a delay.
    private static float At(CarConfig car, TireConfig tires, float speed)
    {
        var state = new CarState { Position = Vector2.Zero, Heading = 0f, Speed = speed, Energy = PowertrainState.Filled(0.8f) };
        state.InstallFreshTires(tires);
        float settled = SettledLateralAccel / (speed * speed);
        float ask = StepLateralAccel / (speed * speed);
        int settle = (int)(SettleSeconds / Dt), window = (int)(WindowSeconds / Dt);
        var path = new float[window];
        float heading = 0f, start = 0f;
        for (int i = 0; i < settle + window; i++)
        {
            bool stepped = i >= settle;
            var input = new CarPhysicsStepInput(new DriverInput(settled + (stepped ? ask : 0f), 0f), CarStrategy.Default, 25f);
            Vector2 from = state.Position;
            CarPhysics.Step(state, car, tires, input, Dt);
            Vector2 move = state.Position - from;
            float length = move.Length();
            if (length < 1e-4f)
                return 0f;
            float now = MathF.Atan2(move.Y, move.X);
            float curvature = MathHelper.NormalizeAngle(now - heading) / length;
            heading = now;
            if (stepped)
                path[i - settle] = curvature;
            else
                start = curvature;
        }
        // Where it settles: the second half of the window, by when the step is long done.
        float end = 0f;
        for (int k = window / 2; k < window; k++)
            end += path[k];
        end /= window - window / 2;
        float taken = end - start;
        if (MathF.Abs(taken) < 0.1f * ask)
            return 0f;
        float behind = 0f;
        for (int k = 0; k < window / 2; k++)
            behind += (1f - Math.Clamp((path[k] - start) / taken, 0f, 1f)) * Dt;
        return behind;
    }
}
