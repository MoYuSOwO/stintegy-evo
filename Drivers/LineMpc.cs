using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Drivers;

/// <summary>
/// Receding-horizon follow on the live car. Candidates come from the
/// geometric line and the remaining friction circle; CarPhysics.Step is the
/// plant that accepts or rejects them. No kinematic bicycle as the live law.
/// </summary>
internal sealed class LineMpc
{
    public const int PreviewSteps = 12;
    public const float Dt = 1f / 60f;
    private const float SpinCost = 50_000f;
    private const float SpeedGain = 2.8f;

    private readonly CarState _start = new();
    private readonly CarState _scratch = new();
    private readonly float[] _kappas = new float[8];
    private readonly float[] _accels = new float[3];

    public readonly record struct Command(float Curvature, float Accel);

    public Command Solve(
        CarState live,
        CarConfig car,
        TireConfig tires,
        CarStrategy strategy,
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        Func<Vector2, TrackPose> project,
        GeometricRacingLine line,
        LineSpeedProfile profile,
        Func<float, float> ggAt,
        float carHalfWidth,
        float ayLimit,
        float lookAheadMeters = 0f
    )
    {
        _start.CopyFrom(live);
        TrackPose here = project(_start.Position);
        GeometricRacingLine.Sample target = line.At(sample, wrap, here.S);
        LineActuators.RoadLimits(
            target.OffsetMeters,
            here.Sample.HalfWidth,
            carHalfWidth,
            out float nLo,
            out float nHi
        );
        float v = MathF.Max(_start.Speed, 0.5f);
        float look = MathF.Max(lookAheadMeters, 0f);
        float vStar = look > 0.5f
            ? profile.MinAhead(wrap, here.S, look)
            : profile.At(wrap, here.S);
        float pathK = target.PathCurvature;
        float holdK = MathF.Tan(_start.SteerAngleRadians) / MathF.Max(car.WheelBaseMeters, 1.5f);
        float e = here.D - target.OffsetMeters;
        float psi = HeadingError(here.S, sample, wrap, line, _start);
        float gripK = MathF.Min(
            car.MaxCurvatureRequest,
            MathF.Max(ggAt(here.S) * ayLimit, 1f) / MathF.Max(v * v, 1f)
        );
        float extra = MathF.Min(0.008f, 0.40f * gripK);
        LineActuators.Steer follow = LineActuators.CurvatureForLine(
            here.D,
            target.OffsetMeters,
            psi,
            v,
            pathK,
            here.Sample.HalfWidth,
            carHalfWidth,
            gripK
        );
        float followK = pathK + Math.Clamp(follow.Curvature - pathK, -extra, extra);
        float pK = LineActuators.LimitToBand(
            pathK
                - Math.Clamp(0.010f * e, -extra, extra)
                + Math.Clamp(0.06f * psi, -0.04f, 0.04f),
            here.D,
            target.OffsetMeters,
            psi,
            v,
            pathK,
            here.Sample.HalfWidth,
            carHalfWidth,
            gripK
        );

        int kappaCount = 0;
        kappaCount = PushKappa(kappaCount, followK, gripK);
        kappaCount = PushKappa(kappaCount, pathK + 0.5f * (followK - pathK), gripK);
        kappaCount = PushKappa(kappaCount, pathK, gripK);
        kappaCount = PushKappa(kappaCount, holdK, gripK);
        if (v >= 16f)
        {
            kappaCount = PushKappa(kappaCount, pathK - extra, gripK);
            kappaCount = PushKappa(kappaCount, pathK + extra, gripK);
            kappaCount = PushKappa(kappaCount, pK, gripK);
        }

        CarPerformanceLimits straight = CarPhysics.EstimatePerformanceLimits(
            _start, car, tires, strategy, v, pathK
        );
        Command best = new(pathK, -straight.MaximumBrakeDeceleration);
        float bestCost = float.PositiveInfinity;

        for (int i = 0; i < kappaCount; i++)
        {
            float k = _kappas[i];
            CarPerformanceLimits lim = CarPhysics.EstimatePerformanceLimits(
                _start, car, tires, strategy, v, k
            );
            float brakeBudget = MathF.Abs(e) > 1.0f && v < 28f
                ? 0.60f * lim.MaximumBrakeDeceleration
                : lim.MaximumBrakeDeceleration;
            bool catchUp = v > 32f && (MathF.Abs(e) > 1.8f || MathF.Abs(psi) > 0.15f);
            float trackAx = LineActuators.RemainingAccel(
                v,
                vStar,
                SpeedGain,
                0f,
                lim.MaximumDriveAcceleration,
                brakeBudget,
                0f,
                allowDrive: v <= vStar + 0.4f && !catchUp
            );
            int accelCount = 1;
            _accels[0] = trackAx;
            if (v <= vStar + 0.8f)
            {
                accelCount = 2;
                _accels[1] = 0f;
            }
            else
            {
                accelCount = 2;
                _accels[1] = -brakeBudget;
            }

            for (int j = 0; j < accelCount; j++)
            {
                float ax = Math.Clamp(
                    _accels[j],
                    -lim.MaximumBrakeDeceleration,
                    lim.MaximumDriveAcceleration
                );
                float cost = Preview(
                    k,
                    ax,
                    car,
                    tires,
                    strategy,
                    sample,
                    wrap,
                    project,
                    line,
                    vStar,
                    nLo,
                    nHi
                );
                if (cost < bestCost)
                {
                    bestCost = cost;
                    best = new Command(k, ax);
                }
            }
        }

        return best;
    }

    private int PushKappa(int count, float kappa, float gripK)
    {
        float k = Math.Clamp(kappa, -gripK, gripK);
        for (int i = 0; i < count; i++)
        {
            if (MathF.Abs(_kappas[i] - k) < 4e-4f)
                return count;
        }

        _kappas[count] = k;
        return count + 1;
    }

    private float Preview(
        float kappa,
        float ax,
        CarConfig car,
        TireConfig tires,
        CarStrategy strategy,
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        Func<Vector2, TrackPose> project,
        GeometricRacingLine line,
        float vStar,
        float nLo,
        float nHi
    )
    {
        _scratch.CopyFrom(_start);
        int spins0 = _scratch.SpinEvents;
        float cost = 0f;
        for (int i = 0; i < PreviewSteps; i++)
        {
            CarPhysics.Step(
                _scratch,
                car,
                tires,
                PhysicsInput(kappa, ax, strategy, car, project, _scratch),
                Dt
            );
            if (_scratch.Spinning || _scratch.SpinEvents > spins0)
                return SpinCost;
            float slip = MathF.Abs(_scratch.SideslipAngleRadians);
            if (slip > 0.14f)
                return SpinCost * 0.5f;

            TrackPose pose = project(_scratch.Position);
            GeometricRacingLine.Sample target = line.At(sample, wrap, pose.S);
            float e = pose.D - target.OffsetMeters;
            float psi = HeadingError(pose.S, sample, wrap, line, _scratch);
            float dv = _scratch.Speed - vStar;
            cost += 8f * e * e + 12f * psi * psi + 0.18f * dv * dv
                + 80f * slip * slip;
            if (dv > 1.0f)
                cost += 1.6f * dv * dv;
            if (slip > 0.10f)
                cost += 400f * (slip - 0.10f) * (slip - 0.10f);
            if (pose.D > nHi)
                cost += 90f * (pose.D - nHi) * (pose.D - nHi);
            if (pose.D < nLo)
                cost += 90f * (nLo - pose.D) * (nLo - pose.D);
        }

        return cost;
    }

    private static float HeadingError(
        float s,
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        GeometricRacingLine line,
        CarState state
    )
    {
        GeometricRacingLine.Sample target = line.At(sample, wrap, s);
        return MathHelper.NormalizeAngle(target.Heading - state.VelocityHeading);
    }

    private static CarPhysicsStepInput PhysicsInput(
        float kappa,
        float ax,
        CarStrategy strategy,
        CarConfig car,
        Func<Vector2, TrackPose> project,
        CarState state
    )
    {
        TrackPose pose = project(state.Position);
        TrackSample here = pose.Sample;
        return new CarPhysicsStepInput(
            new DriverInput(kappa, ax),
            strategy,
            AirTempC: 25f
        )
        {
            RoadAttitude = new RoadAttitude(
                here.Grade,
                here.BankSlopeAt(pose.D),
                here.VerticalRate
            ),
            SurfaceGrip = WheelGrips(car, here, pose, state.Heading)
        };
    }

    private static WheelSurfaceGrip WheelGrips(
        CarConfig car,
        TrackSample sample,
        TrackPose pose,
        float heading
    )
    {
        Vector2 forward = new(MathF.Cos(heading), MathF.Sin(heading));
        Vector2 left = new(-forward.Y, forward.X);
        Vector2 normal = sample.Normal;
        float halfTrack = MathF.Max(car.TrackWidthMeters, 0f) * 0.5f;
        float centre = pose.D;
        float OffsetOf(Vector2 fromCentre) => centre + Vector2.Dot(fromCentre, normal);
        Vector2 front = forward * car.FrontAxleOffsetMeters;
        Vector2 rear = -forward * car.RearAxleOffsetMeters;
        Vector2 side = left * halfTrack;
        return new WheelSurfaceGrip(
            SurfaceGrip.At(sample, OffsetOf(front + side)),
            SurfaceGrip.At(sample, OffsetOf(front - side)),
            SurfaceGrip.At(sample, OffsetOf(rear + side)),
            SurfaceGrip.At(sample, OffsetOf(rear - side))
        );
    }
}
