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
/// Measured once, on the car itself, by asking a car for more corner at a
/// few speeds and timing the path coming round: no number to tune, and a
/// different car brings its own. The delay is not one number per speed.
/// Already in a corner and asked for a little more, the car answers in 0.04
/// to 0.08 s; turned in hard from straight it takes 0.07 or so, and turned
/// in under the brakes up to 0.13, the rear lightened and the tyres already
/// busy slowing the car. Read at the first alone, the hands turned in for
/// Maggotts' right-hander under the brakes too late, the car fell behind the
/// line by half a metre before the tyres were even busy, and at the corner's
/// speed had nothing left to pull it back: 130 of 300 random lines ran more
/// than 1.2 m wide there, none with the delay read for what the car is doing.
/// So it is measured coasting and braking, asked for a little or a lot, and
/// read for the braking and the turn the plan has ahead.
/// </summary>
public sealed class TurnLag
{
    private static readonly float[] Speeds = [15f, 25f, 40f, 60f, 85f];
    private static readonly float[] Brakings = [0f, 15f, 30f];
    private const float Dt = 1f / 60f;
    // A corner taken at about half of what the car holds, and a turn more
    // of the wheel small enough to stay well inside the rest.
    private const float SettledLateralAccel = 12f;
    private const float StepLateralAccel = 2f;
    // A turn-in: from straight, most of a fast corner's pull at once.
    private const float TurnInLateralAccel = 20f;
    private const float SettleSeconds = 1.5f;
    private const float WindowSeconds = 1f;
    // Braking begins this long before the wheel turns, so the car is already
    // pitched onto its nose as it is at a turn-in under the brakes, and is held
    // light enough that the car is still rolling at the end of the window:
    // slow, the braking measured at is less than asked, and is read as what it was.
    private const float BrakeLeadSeconds = 0.3f;
    private const float KeptSpeedShare = 0.4f;

    // [speed][braking][a little more, a turn-in], and the braking each speed was measured at.
    private readonly float[,,] _lag;
    private readonly float[,] _braking;

    private TurnLag(float[,,] lag, float[,] braking)
    {
        _lag = lag;
        _braking = braking;
    }

    private static float BrakingAt(float speed, float braking) =>
        MathF.Min(braking, (1f - KeptSpeedShare) * speed / (WindowSeconds + BrakeLeadSeconds));

    /// <summary>Measured on the driver's own copy of the car, which must have tyres and an envelope.</summary>
    public static TurnLag Measure(ICar car)
    {
        var lag = new float[Speeds.Length, Brakings.Length, 2];
        var braking = new float[Speeds.Length, Brakings.Length];
        for (int i = 0; i < Speeds.Length; i++)
            for (int b = 0; b < Brakings.Length; b++)
            {
                braking[i, b] = BrakingAt(Speeds[i], Brakings[b]);
                lag[i, b, 0] = b == 0 ? At(car, Speeds[i]) : Under(car, Speeds[i], SettledLateralAccel, StepLateralAccel, Brakings[b]);
                lag[i, b, 1] = Under(car, Speeds[i], 0f, TurnInLateralAccel, Brakings[b]);
            }
        return new TurnLag(lag, braking);
    }

    /// <summary>
    /// How far ahead a turn counts as one ask of the wheel: the time the car
    /// was measured to answer one in. A corner that builds over longer than
    /// this is a series of smaller asks.
    /// </summary>
    public const float AskSeconds = WindowSeconds / 2f;

    /// <summary>The delay at <paramref name="speed"/> asked for a little more of a corner, coasting.</summary>
    public float Seconds(float speed) => Seconds(speed, 0f, StepLateralAccel);

    /// <summary>
    /// The delay at <paramref name="speed"/>, braking at
    /// <paramref name="braking"/> (m/s², positive) and asked to change its
    /// sideways pull by <paramref name="turn"/> (m/s²), between what was measured.
    /// </summary>
    public float Seconds(float speed, float braking, float turn)
    {
        (int s0, int s1, float ts) = Between(i => Speeds[i], Speeds.Length, speed);
        float tt = Math.Clamp((MathF.Abs(turn) - StepLateralAccel) / (TurnInLateralAccel - StepLateralAccel), 0f, 1f);
        float At(int s, int b) => _lag[s, b, 0] + (_lag[s, b, 1] - _lag[s, b, 0]) * tt;
        float Over(int s)
        {
            (int b0, int b1, float tb) = Between(b => _braking[s, b], Brakings.Length, MathF.Max(0f, braking));
            return At(s, b0) + (At(s, b1) - At(s, b0)) * tb;
        }
        return Over(s0) + (Over(s1) - Over(s0)) * ts;
    }

    private static (int Lo, int Hi, float T) Between(Func<int, float> at, int count, float x)
    {
        if (x <= at(0))
            return (0, 0, 0f);
        for (int i = 1; i < count; i++)
            if (x <= at(i))
                return (i - 1, i, at(i) > at(i - 1) ? (x - at(i - 1)) / (at(i) - at(i - 1)) : 0f);
        return (count - 1, count - 1, 0f);
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
    private static float At(ICar car, float speed)
    {
        var state = new CarState { Position = Vector2.Zero, Heading = 0f, Speed = speed };
        (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
        var tyres = (ICarTires)car;
        var envelope = (ICarEnvelope)car;
        tyres.InstallFreshTires(state);
        tyres.WarmTires(state, 90f, 90f);
        float settled = SettledLateralAccel / (speed * speed);
        float ask = StepLateralAccel / (speed * speed);
        int settle = (int)(SettleSeconds / Dt), window = (int)(WindowSeconds / Dt);
        var path = new float[window];
        float heading = 0f, start = 0f;
        for (int i = 0; i < settle + window; i++)
        {
            bool stepped = i >= settle;
            float wanted = settled + (stepped ? ask : 0f);
            var input = new CarPhysicsStepInput(
                new DriverInput(envelope.EstimateSteering(state, RoadAttitude.Flat).AngleFor(wanted), 0f),
                CarStrategy.Default, 25f);
            Vector2 from = state.Position;
            car.Step(state, input, Dt);
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

    // As At, but read as sideways pull rather than curvature, so a car
    // slowing under the brakes is not read as turning tighter, and with the
    // brakes on from a moment before the wheel turns.
    private static float Under(ICar car, float speed, float settledAccel, float stepAccel, float braking)
    {
        var state = new CarState { Position = Vector2.Zero, Heading = 0f, Speed = speed };
        (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
        var tyres = (ICarTires)car;
        var envelope = (ICarEnvelope)car;
        tyres.InstallFreshTires(state);
        tyres.WarmTires(state, 90f, 90f);
        float brake = BrakingAt(speed, braking);
        int settle = (int)(SettleSeconds / Dt), window = (int)(WindowSeconds / Dt), lead = (int)(BrakeLeadSeconds / Dt);
        var path = new float[window];
        float heading = 0f, start = 0f;
        for (int i = 0; i < settle + window; i++)
        {
            bool stepped = i >= settle;
            float v = MathF.Max(state.Speed, 5f);
            float wanted = (settledAccel + (stepped ? stepAccel : 0f)) / (v * v);
            var input = new CarPhysicsStepInput(
                new DriverInput(envelope.EstimateSteering(state, RoadAttitude.Flat).AngleFor(wanted), i >= settle - lead ? -brake : 0f),
                CarStrategy.Default, 25f);
            Vector2 from = state.Position;
            car.Step(state, input, Dt);
            Vector2 move = state.Position - from;
            float length = move.Length();
            if (length < 1e-4f)
                return 0f;
            float now = MathF.Atan2(move.Y, move.X);
            float pull = MathHelper.NormalizeAngle(now - heading) / length * MathF.Max(state.Speed, 5f) * MathF.Max(state.Speed, 5f);
            heading = now;
            if (stepped)
                path[i - settle] = pull;
            else
                start = pull;
        }
        float end = 0f;
        for (int k = window / 2; k < window; k++)
            end += path[k];
        end /= window - window / 2;
        float taken = end - start;
        if (MathF.Abs(taken) < 0.1f * stepAccel)
            return 0f;
        float behind = 0f;
        for (int k = 0; k < window / 2; k++)
            behind += (1f - Math.Clamp((path[k] - start) / taken, 0f, 1f)) * Dt;
        return behind;
    }
}
