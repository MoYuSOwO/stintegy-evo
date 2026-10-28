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
    private readonly float[,] _braked;

    private TurnLag(float[] lag, float[,] braked)
    {
        _lag = lag;
        _braked = braked;
    }

    // Shares of all the braking the car has at the speed.
    private static readonly float[] BrakeShares = [0f, 0.3f, 0.6f];

    /// <summary>Measured on the driver's own copy of the car, which must have tyres and an envelope.</summary>
    public static TurnLag Measure(ICar car)
    {
        var lag = new float[Speeds.Length];
        var braked = new float[Speeds.Length, BrakeShares.Length];
        for (int i = 0; i < Speeds.Length; i++)
        {
            lag[i] = At(car, Speeds[i]);
            for (int j = 0; j < BrakeShares.Length; j++)
                braked[i, j] = TurningIn(car, Speeds[i], BrakeShares[j]);
        }
        return new TurnLag(lag, braked);
    }

    /// <summary>
    /// The delay of a turn-in the size corners are turned into, the rack's
    /// travel in it, at <paramref name="speed"/> with this share of all the
    /// braking the car has on: between where it was measured.
    /// </summary>
    public float Seconds(float speed, float brakeShare)
    {
        float i = Position(Speeds, speed), j = Position(BrakeShares, Math.Clamp(brakeShare, 0f, 1f));
        int i0 = (int)i, j0 = (int)j, i1 = Math.Min(i0 + 1, Speeds.Length - 1), j1 = Math.Min(j0 + 1, BrakeShares.Length - 1);
        float u = i - i0, w = j - j0;
        return (_braked[i0, j0] * (1f - w) + _braked[i0, j1] * w) * (1f - u) + (_braked[i1, j0] * (1f - w) + _braked[i1, j1] * w) * u;
    }

    private static float Position(float[] at, float x)
    {
        if (x <= at[0])
            return 0f;
        for (int i = 1; i < at.Length; i++)
            if (x <= at[i])
                return i - 1 + (x - at[i - 1]) / (at[i] - at[i - 1]);
        return at.Length - 1;
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

    // A turn-in the size corners are turned into, from a corner the other
    // way, as out of one bend into the next: the wheel has its whole travel
    // to make at the rack's speed, and the path its slip to build the other
    // way. Under the brakes from a moment before, at a share of all the
    // braking the car has: the rear lightened and the tyres partly spent
    // lengthways, the car takes longer. Read by the same area as the small
    // step, against where the path gets to in the window rather than the ask.
    private const float TurnInFromLateralAccel = -8f;
    private const float TurnInToLateralAccel = 16f;
    private const float BrakeLeadSeconds = 0.15f;
    private const float TurnInWindowSeconds = 0.6f;

    private static float TurningIn(ICar car, float speed, float brakeShare)
    {
        var state = new CarState { Position = Vector2.Zero, Heading = 0f, Speed = speed };
        (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
        var tyres = (ICarTires)car;
        var envelope = (ICarEnvelope)car;
        tyres.InstallFreshTires(state);
        tyres.WarmTires(state, 90f, 90f);
        float from = TurnInFromLateralAccel / (speed * speed), to = TurnInToLateralAccel / (speed * speed);
        float braking = brakeShare * envelope.EstimatePerformanceLimits(state, CarStrategy.Default, speed, from).MaximumBrakeDeceleration;
        int settle = (int)(SettleSeconds / Dt), lead = (int)(BrakeLeadSeconds / Dt), window = (int)(TurnInWindowSeconds / Dt);
        var path = new float[window];
        float heading = 0f, start = 0f;
        for (int i = 0; i < settle + lead + window; i++)
        {
            bool stepped = i >= settle + lead;
            float wanted = stepped ? to : from;
            var input = new CarPhysicsStepInput(
                new DriverInput(envelope.EstimateSteering(state, RoadAttitude.Flat).AngleFor(wanted), i >= settle ? -braking : 0f),
                CarStrategy.Default, 25f);
            Vector2 was = state.Position;
            car.Step(state, input, Dt);
            // The settle is for the corner, not the drag.
            if (i < settle)
                state.Speed = speed;
            Vector2 move = state.Position - was;
            float length = move.Length();
            if (length < 1e-4f)
                return 0f;
            float now = MathF.Atan2(move.Y, move.X);
            float curvature = MathHelper.NormalizeAngle(now - heading) / length;
            heading = now;
            if (stepped)
                path[i - settle - lead] = curvature;
            else
                start = curvature;
        }
        float end = 0f;
        for (int k = window * 2 / 3; k < window; k++)
            end += path[k];
        end /= window - window * 2 / 3;
        float taken = end - start;
        if (MathF.Abs(taken) < 0.1f * MathF.Abs(to - from))
            return 0f;
        float behind = 0f;
        for (int k = 0; k < window * 2 / 3; k++)
            behind += (1f - Math.Clamp((path[k] - start) / taken, 0f, 1f)) * Dt;
        return behind;
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
}
