using StintegyEVO.Core.Cars;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// What this car can do right now, read off the published envelope at four
/// speeds and interpolated in v², piece by piece: with tyres whose grip falls
/// with load the cornering grip bends over as the downforce grows, so no one
/// straight line in v² fits it, and a line through the ends promised the
/// fastest corners more than the car has. The lateral figure is taken twice: once
/// coasting and once under braking, because braking unloads the rear and the
/// rear is the axle that runs out of corner first.
/// </summary>
internal readonly struct LaneEnvelope
{
    private static readonly float[] Anchors = [15f, 40f, 70f, 90f];

    /// <summary>The speeds the envelope is read at.</summary>
    public static float[] AnchorSpeeds => Anchors;

    private readonly float[] _lateral;
    private readonly float[] _lateralBraking;
    private readonly float[] _brake;
    private readonly float[] _drive;
    private readonly float[] _loss;

    // Each axle's grip, as an acceleration of the whole car, with the car
    // cornering at its limit: coasting, and braking as hard as it can.
    private readonly AxleGrips? _axles;

    // The same car in a left-hand and a right-hand corner: weight thrown onto
    // the outside tyres, so the corner is held by those two, at their
    // temperature and wear, not by an average of all four.
    private readonly LaneEnvelope[]? _turns;

    private LaneEnvelope(float[] lateral, float[] lateralBraking, float[] brake, float[] drive, float[] loss, float loadExponent, LaneEnvelope[]? turns = null, AxleGrips? axles = null)
    {
        _turns = turns;
        _axles = axles;
        LoadExponent = loadExponent;
        _lateral = lateral;
        _lateralBraking = lateralBraking;
        _brake = brake;
        _drive = drive;
        _loss = loss;
        // The split between what the car's weight gives and what the wings
        // give, from the ends: the part that scales with the road's
        // apparent gravity.
        int last = Anchors.Length - 1;
        float span = Anchors[last] * Anchors[last] - Anchors[0] * Anchors[0];
        B = (lateral[last] - lateral[0]) / span;
        A = lateral[0] - B * Anchors[0] * Anchors[0];
        float brakeB = (brake[last] - brake[0]) / span;
        BrakeStatic = MathF.Max(0f, brake[0] - brakeB * Anchors[0] * Anchors[0]);
    }

    /// <summary>The part of the cornering grip that comes from the car's weight, at one g.</summary>
    public float A { get; }

    /// <summary>The part that comes from downforce, per v².</summary>
    public float B { get; }

    /// <summary>The part of the braking grip that comes from the car's weight, at one g.</summary>
    public float BrakeStatic { get; }

    /// <summary>
    /// How a tyre's grip grows with what presses it into the road: as the
    /// load to this power, one less the tyre's load sensitivity, as the
    /// physics has it (<see cref="Tire.GripForce"/>).
    /// </summary>
    public float LoadExponent { get; }

    /// <summary>
    /// The corner the tyres hold at <paramref name="v"/> where the road
    /// presses the car down <paramref name="load"/> times its weight.
    /// </summary>
    public float Lateral(float v, float load) => WithLoad(Lateral(v), A, load, LoadExponent);

    /// <summary>The braking the tyres hold where the road presses the car down <paramref name="load"/> times its weight.</summary>
    public float Brake(float v, float load) => WithLoad(Brake(v), BrakeStatic, load, LoadExponent);

    /// <summary>
    /// Grip read on the level, <paramref name="level"/>, of which the
    /// weight gives <paramref name="weight"/>, where the road presses the car
    /// down <paramref name="load"/> times its weight instead of once. A tyre
    /// does not grip in proportion to its load: each extra newton is worth
    /// less than the one before (load sensitivity), so the load the level
    /// grip stands for is found by undoing the power, the road's share is
    /// added to it in weights, and the power is put back. Counted straight,
    /// a steep bank's load grew the grip as fast as the corner grew the
    /// demand, and on Zandvoort's 19 degree Hugenholtz the plan found no
    /// speed the corner could not be taken at.
    /// </summary>
    public static float WithLoad(float level, float weight, float load, float exponent)
    {
        if (load == 1f || weight <= 0f || level <= 0f || exponent <= 0f)
            return level + weight * (load - 1f);
        float loads = MathF.Pow(level / weight, 1f / exponent) + load - 1f;
        return loads <= 0f ? 0f : weight * MathF.Pow(loads, exponent);
    }

    /// <summary>
    /// This envelope for a corner turning the way <paramref name="curvature"/>
    /// does: the outside tyres' figures where they have been built, the
    /// four-tyre average on a straight or where they have not.
    /// </summary>
    public LaneEnvelope ForTurn(float curvature)
    {
        if (_turns is null || MathF.Abs(curvature) < 1e-4f)
            return this;
        return curvature > 0f ? _turns[0] : _turns[1];
    }

    public static LaneEnvelope From(CarState plan, ICarEnvelope car, ICarGripFeel feel, CarStrategy strategy, bool turnAware)
    {
        LaneEnvelope straight = From(plan, car, feel, strategy);
        if (!turnAware)
            return straight;
        var turns = new[] { From(plan, car, strategy, +1f, straight), From(plan, car, strategy, -1f, straight) };
        return new LaneEnvelope(straight._lateral, straight._lateralBraking, straight._brake, straight._drive, straight._loss, straight.LoadExponent, turns, straight._axles);
    }

    /// <summary>
    /// The envelope at the corner it describes: each anchor's limit read with
    /// the car already at that limit, turning the way <paramref name="sign"/>
    /// says, so the load sits on the outside wheels while it is read.
    /// </summary>
    private static LaneEnvelope From(CarState plan, ICarEnvelope car, CarStrategy strategy, float sign, LaneEnvelope straight)
    {
        float cornerGrip = car.GripUsage(strategy);
        var lateral = new float[Anchors.Length];
        var lateralBraking = new float[Anchors.Length];
        for (int i = 0; i < Anchors.Length; i++)
        {
            float v = Anchors[i];
            CarPerformanceLimits coast = car.EstimatePerformanceLimits(
                plan, strategy, v, curvature: 0f, cornerGrip);
            // Settled on itself: the limit read at the curvature it implies,
            // so the load on the outside wheels is the corner's own.
            float curvature = sign * straight._lateral[i] / (v * v);
            CarPerformanceLimits corner = default;
            for (int k = 0; k < 3; k++)
            {
                corner = car.EstimatePerformanceLimits(
                    plan, strategy, v, curvature, cornerGrip);
                curvature = sign * MathF.Max(1f, corner.LateralAccelerationLimit) / (v * v);
            }
            CarPerformanceLimits cornerBraking = car.EstimatePerformanceLimits(
                plan, strategy, v, curvature, cornerGrip,
                assumedLongitudinalAcceleration: -coast.MaximumBrakeDeceleration);
            lateral[i] = MathF.Max(1f, corner.LateralAccelerationLimit);
            lateralBraking[i] = MathF.Max(1f, cornerBraking.LateralAccelerationLimit);
        }
        return new LaneEnvelope(lateral, lateralBraking, straight._brake, straight._drive, straight._loss, straight.LoadExponent,
            axles: AxleGrips.Read(plan, car, sign, lateral, lateralBraking, straight._brake));
    }

    public static LaneEnvelope From(CarState plan, ICarEnvelope car, ICarGripFeel feel, CarStrategy strategy)
    {
        // The tyre rung is an allowance the combined-grip limiter enforces on
        // braking and drive; it does not give the tyre more corner. So the
        // corner is read at no more than the Normal rung, and the rung's own
        // allowance only moves the longitudinal figures. Below Normal the
        // corner comes down with it, which is what the wall asked for.
        float grip = car.GripUsage(strategy);
        // The tyre rung is the driver's authority over the circle; the plan
        // spends what the rung allows, corner included.
        float cornerGrip = grip;
        var lateral = new float[Anchors.Length];
        var lateralBraking = new float[Anchors.Length];
        var brake = new float[Anchors.Length];
        var drive = new float[Anchors.Length];
        var loss = new float[Anchors.Length];
        for (int i = 0; i < Anchors.Length; i++)
        {
            float v = Anchors[i];
            CarPerformanceLimits coast = car.EstimatePerformanceLimits(
                plan, strategy, v, curvature: 0f, grip);
            CarPerformanceLimits braking = car.EstimatePerformanceLimits(
                plan, strategy, v, curvature: 0f, grip,
                assumedLongitudinalAcceleration: -coast.MaximumBrakeDeceleration);
            CarPerformanceLimits corner = car.EstimatePerformanceLimits(
                plan, strategy, v, curvature: 0f, cornerGrip);
            CarPerformanceLimits cornerBraking = car.EstimatePerformanceLimits(
                plan, strategy, v, curvature: 0f, cornerGrip,
                assumedLongitudinalAcceleration: -coast.MaximumBrakeDeceleration);
            lateral[i] = MathF.Max(1f, corner.LateralAccelerationLimit);
            lateralBraking[i] = MathF.Max(1f, cornerBraking.LateralAccelerationLimit);
            brake[i] = MathF.Max(1f, braking.MaximumBrakeDeceleration);
            drive[i] = MathF.Max(0f, coast.MaximumDriveAcceleration);
            loss[i] = MathF.Max(0f, coast.LossAcceleration);
        }

        return new LaneEnvelope(lateral, lateralBraking, brake, drive, loss, 1f - Math.Clamp(feel.LoadSensitivity(plan), 0f, 1f),
            axles: AxleGrips.Read(plan, car, 1f, lateral, lateralBraking, brake));
    }

    /// <summary>
    /// The most the tyres may brake, as an acceleration of the whole car,
    /// at <paramref name="v"/> cornering at <paramref name="ay"/> with the
    /// rear using no more than <paramref name="rearShare"/> of its grip, the
    /// brakes split as the driver dials them: by what each axle's corner
    /// leaves of it. None when nothing holds the axles' figures.
    ///
    /// Reckoned forward, from what each axle is asked for, not read back off
    /// a slip curve: the corner split between the axles by where the weight
    /// sits, as the car turns, and each axle's grip at the braking asked.
    /// Braking moves the weight forward and the rear, carrying more than its
    /// share of the corner on less than its share of the grip, is the end
    /// that runs out: there the car turns a great deal for very little
    /// lock, and the hands cannot place it.
    /// </summary>
    public float? BrakeHoldingRear(float v, float ay, float braking, float rearShare) =>
        BrakeOnRear(v, ay, braking, _ => rearShare);

    /// <summary>
    /// The most the tyres may brake (as <see cref="BrakeHoldingRear"/>) with
    /// the brakes adding no more than <paramref name="adds"/> to what the
    /// corner alone asks of the rear, once the corner asks for more than
    /// <paramref name="from"/> of it: below that the brakes are free, and
    /// between nothing and there the limit closes in. The corner itself
    /// may use the rear to the full; it is the brakes on top of it, while
    /// the car is being turned in, that leave it nothing to spare.
    /// </summary>
    public float? BrakeAddingToRear(float v, float ay, float braking, float adds, float from) =>
        BrakeOnRear(v, ay, braking, corner =>
            MathF.Min(1f, corner + adds + (1f - adds) * MathF.Max(0f, 1f - corner / MathF.Max(from, 1e-3f))));

    private float? BrakeOnRear(float v, float ay, float braking, Func<float, float> allowed)
    {
        if (_axles is not { } axles)
            return null;
        float share = Math.Clamp(braking / MathF.Max(Brake(v), 1f), 0f, 1f);
        (float front, float rear) = axles.At(v, share);
        if (front <= 1f || rear <= 1f)
            return null;
        float rearCorner = (1f - axles.FrontCornerShare) * ay / rear;
        float rearShare = allowed(rearCorner);
        if (rearCorner >= rearShare)
            return 0f;
        float frontCorner = Math.Min(axles.FrontCornerShare * ay / front, 1f);
        float frontLeft = front * MathF.Sqrt(1f - frontCorner * frontCorner);
        float rearLeft = rear * MathF.Sqrt(1f - rearCorner * rearCorner);
        float rearBrake = rear * MathF.Sqrt(rearShare * rearShare - rearCorner * rearCorner);
        return rearBrake * (frontLeft + rearLeft) / MathF.Max(rearLeft, 1e-3f);
    }

    /// <summary>
    /// The same car on a road worth <paramref name="scale"/> of a clean one.
    /// The published envelope does not read the day's surface grip, so the
    /// driver folds it in here: corner and brakes go with the road, the motor
    /// does not.
    /// </summary>
    public LaneEnvelope WithRoadGrip(float scale)
    {
        float s = MathF.Max(0.05f, scale);
        return new LaneEnvelope(
            _lateral.Select(x => x * s).ToArray(),
            _lateralBraking.Select(x => x * s).ToArray(),
            _brake.Select(x => x * s).ToArray(),
            _drive,
            _loss,
            LoadExponent,
            _turns?.Select(e => e.WithRoadGrip(scale)).ToArray(),
            _axles?.Scaled(s));
    }

    public float Lateral(float v) => Interpolate(_lateral, v, extrapolate: true);
    public float LateralBraking(float v) => Interpolate(_lateralBraking, v, extrapolate: true);
    public float Brake(float v) => Interpolate(_brake, v, extrapolate: false);
    public float Drive(float v) => Interpolate(_drive, v, extrapolate: false);
    public float Loss(float v) => Interpolate(_loss, v, extrapolate: true);

    private static float Interpolate(float[] values, float v, bool extrapolate)
    {
        float vv = v * v;
        if (vv <= Anchors[0] * Anchors[0])
            return values[0];
        int last = Anchors.Length - 1;
        for (int i = 1; i <= last; i++)
        {
            float hi = Anchors[i] * Anchors[i];
            if (vv <= hi || i == last)
            {
                if (vv > hi && !extrapolate)
                    return values[last];
                float lo = Anchors[i - 1] * Anchors[i - 1];
                return values[i - 1] + (values[i] - values[i - 1]) * (vv - lo) / (hi - lo);
            }
        }
        return values[last];
    }
}

/// <summary>
/// Each axle's grip at the envelope's anchors, cornering at the limit, as an
/// acceleration of the whole car: coasting, and under the hardest braking.
/// </summary>
internal sealed record AxleGrips(float[] Speeds, float[] Front, float[] Rear, float[] FrontBraking, float[] RearBraking, float FrontCornerShare)
{
    /// <summary>
    /// Read off a copy of the plan's car, put at each anchor's speed,
    /// cornering at its limit and braking or not, by the car's own estimate
    /// of its axles (<see cref="ICarEnvelope.EstimateSteering"/>).
    /// </summary>
    public static AxleGrips Read(CarState plan, ICarEnvelope car, float sign, float[] lateral, float[] lateralBraking, float[] brake)
    {
        int n = lateral.Length;
        float[] speeds = LaneEnvelope.AnchorSpeeds;
        var front = new float[n];
        var rear = new float[n];
        var frontBraking = new float[n];
        var rearBraking = new float[n];
        CarState copy = plan.Clone();
        var motion = (ICarMotion)car;
        for (int i = 0; i < n; i++)
        {
            copy.Speed = speeds[i];
            motion.Assume(copy, motion.Motion(copy) with { LongitudinalAccel = 0f, LateralAccel = sign * lateral[i] });
            SteeringEstimate coast = car.EstimateSteering(copy, RoadAttitude.Flat);
            motion.Assume(copy, motion.Motion(copy) with { LongitudinalAccel = -brake[i], LateralAccel = sign * lateralBraking[i] });
            SteeringEstimate braking = car.EstimateSteering(copy, RoadAttitude.Flat);
            front[i] = coast.FrontGripAccel;
            rear[i] = coast.RearGripAccel;
            frontBraking[i] = braking.FrontGripAccel;
            rearBraking[i] = braking.RearGripAccel;
        }
        return new AxleGrips(speeds, front, rear, frontBraking, rearBraking, Math.Clamp((car as ICarAxles)?.Layout.FrontWeightShare ?? 0.5f, 0.05f, 0.95f));
    }

    /// <summary>Front and rear at <paramref name="v"/>, braking <paramref name="share"/> of the hardest.</summary>
    public (float Front, float Rear) At(float v, float share)
    {
        float f = Interpolate(Front, v) + (Interpolate(FrontBraking, v) - Interpolate(Front, v)) * share;
        float r = Interpolate(Rear, v) + (Interpolate(RearBraking, v) - Interpolate(Rear, v)) * share;
        return (f, r);
    }

    public AxleGrips Scaled(float s) =>
        this with { Front = Front.Select(x => x * s).ToArray(), Rear = Rear.Select(x => x * s).ToArray(),
            FrontBraking = FrontBraking.Select(x => x * s).ToArray(), RearBraking = RearBraking.Select(x => x * s).ToArray() };

    // In v², as the envelope is.
    private float Interpolate(float[] values, float v)
    {
        float vv = v * v;
        if (vv <= Speeds[0] * Speeds[0])
            return values[0];
        for (int i = 1; i < Speeds.Length; i++)
        {
            float hi = Speeds[i] * Speeds[i];
            if (vv <= hi || i == Speeds.Length - 1)
            {
                float lo = Speeds[i - 1] * Speeds[i - 1];
                return values[i - 1] + (values[i] - values[i - 1]) * (vv - lo) / (hi - lo);
            }
        }
        return values[^1];
    }
}

/// <summary>How much of the envelope the plan spends. The rest belongs to the tracker.</summary>
public sealed record LaneShares
{
    public float Lateral { get; init; } = 0.95f;
    public float Brake { get; init; } = 0.92f;
    public float Drive { get; init; } = 1f;

    /// <summary>Whether braking takes lateral grip from the rear. Off only for probes.</summary>
    public bool BrakingCoupling { get; init; } = true;

    /// <summary>
    /// The most of the rear's grip the plan brakes into a corner on: past
    /// it, the braking is done before the turn instead. None, braking and
    /// corner share the whole car's ellipse however the rear fares.
    /// </summary>
    public float? RearTurnIn { get; init; }

    /// <summary>
    /// The most the brakes may add to the share of the rear's grip the
    /// corner alone asks of it, braking into a corner, once that share is
    /// past <see cref="RearBrakeFrom"/>. None, the brakes add what the
    /// whole car's ellipse allows.
    /// </summary>
    public float? RearBrakeAdds { get; init; }

    /// <summary>Where the rear's corner share starts to hold the brakes back (<see cref="RearBrakeAdds"/>).</summary>
    public float RearBrakeFrom { get; init; } = 0.5f;
}

/// <summary>
/// Speed along a chain of stations: corner limits from curvature, then
/// braking backwards and driving forwards through a friction ellipse.
/// </summary>
/// <summary>
/// How the road lies at one station: the physics' own description, so the
/// plan and the car agree on what a bank or a crest is worth.
/// </summary>
internal readonly record struct LaneRoad(float Grade, float Bank, float Vertical)
{
    /// <summary>Probe switch: credit the bank only for the corner it takes off the tyres, not the load it adds.</summary>
    internal static bool BankAddsLoad = true;

    public bool IsFlat => Grade == 0f && Bank == 0f && Vertical == 0f;
    public RoadAttitude Attitude => new(Grade, Bank, Vertical);
}

/// <summary>
/// Speed along a chain of stations: corner limits from curvature and the
/// road's lie, then braking backwards and driving forwards through a friction
/// ellipse, with gravity along the road paid or collected.
/// </summary>
internal static class LaneSpeedPlanner
{
    public const float TopSpeed = 95f;
    private const float Gravity = 9.80665f;

    /// <summary>
    /// The fastest the corner can be taken on a level road: where what it
    /// asks for, v² times the curvature, meets what the envelope gives at
    /// that speed. Solved by bisection on the interpolated envelope, which
    /// is not one line in v² (LaneEnvelope).
    /// </summary>
    public static float CornerLimit(in LaneEnvelope envelope, float absCurvature, float factor)
    {
        LaneEnvelope e = envelope;
        return Solve(v => absCurvature * v * v <= factor * e.Lateral(v));
    }

    private static float Solve(Func<float, bool> holds)
    {
        if (holds(TopSpeed))
            return TopSpeed;
        float lo = 1f, hi = TopSpeed;
        for (int k = 0; k < 22; k++)
        {
            float mid = 0.5f * (lo + hi);
            if (holds(mid)) lo = mid; else hi = mid;
        }
        return lo;
    }

    /// <summary>
    /// The fastest the corner can be taken where the road leans or climbs:
    /// the bank takes part of the corner off the tyres and presses them into
    /// the road, a crest lifts the car off them. Solved by bisection because
    /// the two no longer fold into one closed form.
    /// </summary>
    public static float CornerLimit(in LaneEnvelope e, float curvature, float factor, LaneRoad road)
    {
        if (road.IsFlat)
            return CornerLimit(e, MathF.Abs(curvature), factor);
        RoadAttitude att = road.Attitude;
        LaneEnvelope env = e;
        bool Holds(float v)
        {
            float demand = MathF.Abs(att.CurvatureDemandScale * v * v * curvature + att.LateralGravityDemand(Gravity, v));
            float load = LaneRoad.BankAddsLoad
                ? att.NormalGravity(Gravity, v, curvature) / Gravity
                : new RoadAttitude(road.Grade, 0f, road.Vertical).NormalGravity(Gravity, v, curvature) / Gravity;
            // The envelope read on the level, with the weight's share moved
            // by how hard the road presses the car into it.
            return demand <= factor * env.Lateral(v, load);
        }
        return Solve(Holds);
    }

    /// <summary>A closed lap: the chain wraps, so it is swept twice each way.</summary>
    public static void PlanClosed(
        in LaneEnvelope envelope,
        ReadOnlySpan<float> curvature,
        ReadOnlySpan<float> stepLength,
        ReadOnlySpan<float> factor,
        ReadOnlySpan<float> brakeFactor,
        ReadOnlySpan<LaneRoad> road,
        LaneShares shares,
        Span<float> speed
    )
    {
        int count = speed.Length;
        for (int i = 0; i < count; i++)
            speed[i] = CornerLimit(envelope.ForTurn(curvature[i]), curvature[i], factor[i] * shares.Lateral, Road(road, i));

        for (int pass = 0; pass < 2; pass++)
        {
            for (int k = count - 1; k >= 0; k--)
                speed[k] = MathF.Min(speed[k], BrakeFrom(envelope, speed[(k + 1) % count], curvature[k], stepLength[k], factor[k], brakeFactor[k], Road(road, k), shares));
        }

        for (int pass = 0; pass < 2; pass++)
        {
            for (int k = 0; k < count; k++)
            {
                int next = (k + 1) % count;
                speed[next] = MathF.Min(speed[next], DriveFrom(envelope, speed[k], curvature[k], stepLength[k], factor[k], Road(road, k), shares));
            }
        }
    }

    /// <summary>An open window ahead of the car, ending at a speed it must be able to meet.</summary>
    public static void PlanWindow(
        in LaneEnvelope envelope,
        ReadOnlySpan<float> curvature,
        ReadOnlySpan<float> stepLength,
        ReadOnlySpan<float> factor,
        ReadOnlySpan<float> brakeFactor,
        ReadOnlySpan<LaneRoad> road,
        LaneShares shares,
        float terminalSpeed,
        Span<float> speed
    )
    {
        int count = speed.Length;
        for (int i = 0; i < count; i++)
            speed[i] = CornerLimit(envelope.ForTurn(curvature[i]), curvature[i], factor[i] * shares.Lateral, Road(road, i));
        speed[count - 1] = MathF.Min(speed[count - 1], terminalSpeed);
        for (int k = count - 2; k >= 0; k--)
            speed[k] = MathF.Min(speed[k], BrakeFrom(envelope, speed[k + 1], curvature[k], stepLength[k], factor[k], brakeFactor[k], Road(road, k), shares));
    }

    /// <summary>
    /// The stretch just ahead of the car, replanned on fresher grip: braking
    /// back from a speed it must meet at the far end, driving on from the
    /// speed it already had planned at the near one.
    /// </summary>
    public static void PlanAhead(
        in LaneEnvelope envelope,
        ReadOnlySpan<float> curvature,
        ReadOnlySpan<float> stepLength,
        ReadOnlySpan<float> factor,
        ReadOnlySpan<float> brakeFactor,
        ReadOnlySpan<LaneRoad> road,
        LaneShares shares,
        float startSpeed,
        float terminalSpeed,
        Span<float> speed
    )
    {
        PlanWindow(envelope, curvature, stepLength, factor, brakeFactor, road, shares, terminalSpeed, speed);
        speed[0] = MathF.Min(speed[0], startSpeed);
        for (int k = 0; k < speed.Length - 1; k++)
            speed[k + 1] = MathF.Min(speed[k + 1], DriveFrom(envelope, speed[k], curvature[k], stepLength[k], factor[k], Road(road, k), shares));
    }

    private static LaneRoad Road(ReadOnlySpan<LaneRoad> road, int i) => road.IsEmpty ? default : road[i];

    /// <summary>The tyres' share of the corner at this speed, and how hard the road presses on them.</summary>
    private static (float Demand, float Load) Corner(float v, float curvature, LaneRoad road)
    {
        if (road.IsFlat)
            return (v * v * MathF.Abs(curvature), 1f);
        RoadAttitude att = road.Attitude;
        float demand = MathF.Abs(att.CurvatureDemandScale * v * v * curvature + att.LateralGravityDemand(Gravity, v));
        float load = LaneRoad.BankAddsLoad
            ? att.NormalGravity(Gravity, v, curvature) / Gravity
            : new RoadAttitude(road.Grade, 0f, road.Vertical).NormalGravity(Gravity, v, curvature) / Gravity;
        return (demand, load);
    }

    private static float BrakeFrom(in LaneEnvelope envelope, float vNext, float curvature, float ds, float factor, float brakeFactor, LaneRoad road, LaneShares shares)
    {
        LaneEnvelope e = envelope.ForTurn(curvature);
        RoadAttitude att = road.Attitude;
        // The step is driven at every speed between the one it starts at and
        // vNext, so the corner it carries and the grip it has are read at
        // their middle, the start found by asking again, as Simplix does
        // (TFixCarParam::CalcBraking). Read at vNext, the slower end, the
        // corner was charged for less than the car was turning and the plan
        // braked harder into it than the tyres would carry.
        float v0 = vNext;
        for (int pass = 0; pass < 3; pass++)
        {
            float v = 0.5f * (v0 + vNext);
            // Braking unloads the rear by as much as it brakes, not by the most
            // it could: take the lateral limit part way to the full-braking
            // one, in proportion to how hard the first guess brakes, and ask again.
            (float ay, float load) = Corner(v, curvature, road);
            float lateral = e.Lateral(v, load);
            float brakingShare = e.LateralBraking(v) / MathF.Max(e.Lateral(v), 1f);
            float brake = brakeFactor * shares.Brake * MathF.Max(1f, e.Brake(v, load));
            float coast = factor * shares.Lateral * lateral;
            float full = shares.BrakingCoupling ? coast * brakingShare : coast;
            float ax = brake * Ellipse(ay, coast);
            for (int k = 0; k < 2; k++)
            {
                float limit = coast + (full - coast) * (ax / MathF.Max(brake, 1e-3f));
                ax = brake * Ellipse(ay, limit);
            }
            if (shares.RearTurnIn is { } rearShare && ay > 1f &&
                e.BrakeHoldingRear(v, ay, ax, rearShare) is { } holding)
                ax = MathF.Min(ax, holding);
            if (shares.RearBrakeAdds is { } adds && ay > 1f &&
                e.BrakeAddingToRear(v, ay, ax, adds, shares.RearBrakeFrom) is { } adding)
                ax = MathF.Min(ax, adding);

            // What the road does to the car along it, the physics' way round:
            // the axles are scaled onto the slope, gravity is added on top.
            float decel = (ax + e.Loss(v)) * att.LongitudinalDemandScale - att.AlongTrackGravity(Gravity, v);
            v0 = MathF.Sqrt(vNext * vNext + 2f * MathF.Max(0.1f, decel) * ds);
        }
        return v0;
    }

    private static float Ellipse(float used, float limit)
    {
        float share = MathF.Min(1f, used / MathF.Max(limit, 1f));
        return MathF.Sqrt(MathF.Max(0f, 1f - share * share));
    }

    /// <summary>
    /// The speed full drive brings the car to over <paramref name="ds"/>
    /// from <paramref name="v"/> on this road: the plan's own forward step,
    /// so a driver can tell a stretch the plan drives flat out -- where its
    /// speed is the motor's curve, rising or, up a climb, falling -- from one
    /// it holds back on.
    /// </summary>
    internal static float FlatOutSpeed(in LaneEnvelope envelope, float v, float curvature, float ds, float factor, LaneRoad road, LaneShares shares) =>
        DriveFrom(envelope, v, curvature, ds, factor, road, shares);

    private static float DriveFrom(in LaneEnvelope envelope, float v, float curvature, float ds, float factor, LaneRoad road, LaneShares shares)
    {
        LaneEnvelope e = envelope.ForTurn(curvature);
        (float ay, float load) = Corner(v, curvature, road);
        float lateralLimit = factor * shares.Lateral * e.Lateral(v, load);
        // The motor's drive or what the tyres have left beside the corner,
        // whichever is less. Scaling the motor by the ellipse charged it for
        // grip it never asked for: up a fast bend at 240 km/h the motor gives
        // nine metres a second squared against tyres good for fifty, and the
        // plan had it give seven, so the car was held to part throttle with
        // half its grip unused.
        float traction = lateralLimit * Ellipse(ay, lateralLimit);
        float push = shares.Drive * MathF.Min(e.Drive(v), traction);
        RoadAttitude att = road.Attitude;
        float accel = (push - e.Loss(v)) * att.LongitudinalDemandScale + att.AlongTrackGravity(Gravity, v);
        return MathF.Sqrt(MathF.Max(0f, v * v + 2f * accel * ds));
    }
}
