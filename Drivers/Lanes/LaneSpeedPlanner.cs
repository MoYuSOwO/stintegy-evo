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

    // How much of the corner the tyres are carrying, as a share of what they
    // hold, at which the loss is read in a corner.
    private static readonly float[] Uses = [0f, 0.4f, 0.6f, 0.75f, 0.85f, 0.92f, 1f];

    private readonly float[] _lateral;
    private readonly float[] _lateralBraking;
    private readonly float[] _brake;
    private readonly float[] _drive;
    private readonly float[] _loss;

    // Per share of the corner (Uses), per speed (Anchors): the drag the
    // corner adds to the air's.
    private readonly float[][] _cornerDrag;

    // The same car in a left-hand and a right-hand corner: weight thrown onto
    // the outside tyres, so the corner is held by those two, at their
    // temperature and wear, not by an average of all four.
    private readonly LaneEnvelope[]? _turns;

    private LaneEnvelope(float[] lateral, float[] lateralBraking, float[] brake, float[] drive, float[] loss,
        float[][] cornerDrag, float loadExponent, LaneEnvelope[]? turns = null)
    {
        _turns = turns;
        _cornerDrag = cornerDrag;
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
        return new LaneEnvelope(straight._lateral, straight._lateralBraking, straight._brake, straight._drive, straight._loss,
            straight._cornerDrag, straight.LoadExponent, turns);
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
        return new LaneEnvelope(lateral, lateralBraking, straight._brake, straight._drive, straight._loss,
            CornerDrag(plan, car, strategy, sign, lateral, straight._loss), straight.LoadExponent);
    }

    /// <summary>
    /// The drag a corner adds, read off the car's own estimate at each share
    /// of the corner: the slip angles a corner takes tilt the tyres' force
    /// back against the car. Left out, the plan had the car at Luffield's
    /// apex losing what the air takes and no more, held its speed there
    /// while the coasting car lost four metres a second squared, and drove
    /// out of it later than the plan said by a tenth and a half.
    /// </summary>
    private static float[][] CornerDrag(
        CarState plan, ICarEnvelope car, CarStrategy strategy, float sign, float[] lateral, float[] straightLoss)
    {
        float cornerGrip = car.GripUsage(strategy);
        var drag = new float[Uses.Length][];
        for (int u = 0; u < Uses.Length; u++)
        {
            drag[u] = new float[Anchors.Length];
            for (int i = 0; i < Anchors.Length; i++)
            {
                float v = Anchors[i];
                float curvature = sign * Uses[u] * lateral[i] / (v * v);
                CarPerformanceLimits corner = car.EstimatePerformanceLimits(
                    plan, strategy, v, curvature, cornerGrip);
                drag[u][i] = MathF.Max(0f, corner.LossAcceleration - straightLoss[i]);
            }
        }
        return drag;
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

        return new LaneEnvelope(lateral, lateralBraking, brake, drive, loss,
            CornerDrag(plan, car, strategy, +1f, lateral, loss), 1f - Math.Clamp(feel.LoadSensitivity(plan), 0f, 1f));
    }

    /// <summary>
    /// The same car on a road worth <paramref name="scale"/> of a clean one.
    /// The published envelope does not read the day's surface grip, so the
    /// driver folds it in here: corner and brakes go with the road, the motor
    /// does not. The corner's drag goes with the road too: it is the
    /// corner's force at its angle.
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
            _cornerDrag.Select(row => row.Select(x => x * s).ToArray()).ToArray(),
            LoadExponent,
            _turns?.Select(e => e.WithRoadGrip(scale)).ToArray());
    }

    public float Lateral(float v) => Interpolate(_lateral, v, extrapolate: true);
    public float LateralBraking(float v) => Interpolate(_lateralBraking, v, extrapolate: true);
    public float Brake(float v) => Interpolate(_brake, v, extrapolate: false);
    public float Drive(float v) => Interpolate(_drive, v, extrapolate: false);
    public float Loss(float v) => Interpolate(_loss, v, extrapolate: true);

    /// <summary>The drag a corner adds to <see cref="Loss(float)"/> while the tyres carry <paramref name="use"/> of what they hold.</summary>
    public float CornerDrag(float v, float use)
    {
        // Past the limit the last stretch carries on: the notes take some
        // corners a little over what the envelope says the tyres hold.
        float u = MathF.Max(use, 0f);
        int hi = 1;
        while (hi < Uses.Length - 1 && Uses[hi] < u)
            hi++;
        float t = (u - Uses[hi - 1]) / (Uses[hi] - Uses[hi - 1]);
        float low = Interpolate(_cornerDrag[hi - 1], v, extrapolate: true);
        return low + (Interpolate(_cornerDrag[hi], v, extrapolate: true) - low) * t;
    }

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

/// <summary>How much of the envelope the plan spends. The rest belongs to the tracker.</summary>
public sealed record LaneShares
{
    public float Lateral { get; init; } = 0.95f;
    public float Brake { get; init; } = 0.92f;
    public float Drive { get; init; } = 1f;

    /// <summary>
    /// The share of a tyre's grip the throttle may take it to, corner and
    /// drive together: the circle the driver's foot holds the driven axle
    /// to. Out of a corner the plan drives no sooner than the foot will;
    /// planned on its own corner share, it had the car holding speed round
    /// a slow apex the foot was coasting through, and fell behind by the exit.
    /// </summary>
    public float Traction { get; init; } = 0.90f;

    /// <summary>Whether braking takes lateral grip from the rear. Off only for probes.</summary>
    public bool BrakingCoupling { get; init; } = true;
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

    /// <summary>
    /// The stretch just ahead of the car with its speed chosen, not only
    /// swept: at every station each speed the car can be doing there, from
    /// braking all the way to driving all the way out of the one before,
    /// charged the time it takes and <paramref name="extra"/>, what the cars
    /// around add to a station's step (infinite where it runs into one); the
    /// far end charged <paramref name="rest"/>, the seconds from there on at
    /// the speed it ends at. Speeds are kept in bins of
    /// <paramref name="speedStep"/>, each holding the speed it was reached at
    /// as it is, so a station's few hundredths of drive at the top are not
    /// lost to the bin. With nothing around, the fastest way is the sweeps'
    /// own. How hard the car can drive or brake at a station is read every
    /// <see cref="ChooseSample"/> m/s and taken in a straight line between:
    /// it bends gently with the speed. Speeds below
    /// <paramref name="slowest"/> are not looked at. The seconds it all
    /// takes; the speed at each station in <paramref name="speed"/>.
    /// </summary>
    public static float ChooseAhead(
        in LaneEnvelope envelope,
        ReadOnlySpan<float> curvature,
        ReadOnlySpan<float> stepLength,
        ReadOnlySpan<float> factor,
        ReadOnlySpan<float> brakeFactor,
        ReadOnlySpan<LaneRoad> road,
        LaneShares shares,
        float startSpeed,
        Func<float, float> rest,
        Func<int, float, float, float>? extra,
        Span<float> speed,
        float speedStep = 0.5f,
        ReadOnlySpan<float> slowest = default,
        float sample = ChooseSample
    )
    {
        int count = speed.Length;
        int bins = (int)MathF.Ceiling(TopSpeed / speedStep) + 2;
        Choice[] grid = ChoiceGrid(count * bins);
        Array.Fill(grid, Choice.None, 0, count * bins);
        var lo = new int[count];
        var hi = new int[count];
        Array.Fill(lo, int.MaxValue);
        Array.Fill(hi, -1);
        int Bin(float v) => Math.Clamp((int)MathF.Round(v / speedStep), 0, bins - 1);
        void Reach(int k, int bin, float seconds, float v, int from)
        {
            ref Choice c = ref grid[k * bins + bin];
            if (seconds >= c.Seconds)
                return;
            c = new Choice(seconds, v, from);
            lo[k] = Math.Min(lo[k], bin);
            hi[k] = Math.Max(hi[k], bin);
        }
        Reach(0, Bin(startSpeed), 0f, startSpeed, -1);
        for (int k = 0; k + 1 < count; k++)
        {
            float limit = CornerLimit(envelope.ForTurn(curvature[k + 1]), curvature[k + 1], factor[k + 1] * shares.Lateral, Road(road, k + 1));
            float floor = slowest.IsEmpty ? 0f : slowest[k + 1];
            float ds = stepLength[k];
            // Full drive and full braking out of this station, read at every
            // sample speed the reached speeds lie between.
            float vMin = float.PositiveInfinity, vMax = 0f;
            for (int b = lo[k]; b <= hi[k]; b++)
                if (grid[k * bins + b].From != Choice.Unreached)
                {
                    vMin = MathF.Min(vMin, grid[k * bins + b].Speed);
                    vMax = MathF.Max(vMax, grid[k * bins + b].Speed);
                }
            if (vMax <= 0f)
                break;
            bool exact = sample <= 0f;
            float every = exact ? ChooseSample : sample;
            int s0 = (int)MathF.Floor(vMin / every), s1 = (int)MathF.Ceiling(vMax / every);
            if (exact)
                s1 = s0 + 1;
            if (s1 == s0)
                s1++;
            Span<float> drive = stackalloc float[s1 - s0 + 1];
            Span<float> brake = stackalloc float[s1 - s0 + 1];
            for (int i = 0; i <= s1 - s0; i++)
            {
                if (exact)
                    break;
                float vs = MathF.Max((s0 + i) * every, 0.5f);
                drive[i] = DriveFrom(envelope, vs, curvature[k], ds, factor[k], Road(road, k), shares);
                // Braked all the way, the braking read at the step's middle
                // speed, as the backward sweep reads it.
                float braked = vs;
                for (int pass = 0; pass < 3; pass++)
                {
                    float mid = 0.5f * (vs + braked);
                    braked = MathF.Sqrt(MathF.Max(0f, vs * vs - 2f * Deceleration(envelope, mid, curvature[k], factor[k], brakeFactor[k], Road(road, k), shares) * ds));
                }
                brake[i] = braked;
            }
            for (int b = lo[k]; b <= hi[k]; b++)
            {
                Choice here = grid[k * bins + b];
                if (here.From == Choice.Unreached)
                    continue;
                float v = here.Speed;
                float high, low;
                if (exact)
                {
                    high = DriveFrom(envelope, v, curvature[k], ds, factor[k], Road(road, k), shares);
                    low = v;
                    for (int pass = 0; pass < 3; pass++)
                    {
                        float mid = 0.5f * (v + low);
                        low = MathF.Sqrt(MathF.Max(0f, v * v - 2f * Deceleration(envelope, mid, curvature[k], factor[k], brakeFactor[k], Road(road, k), shares) * ds));
                    }
                }
                else
                {
                    float x = v / every - s0;
                    int i0 = Math.Clamp((int)x, 0, s1 - s0 - 1);
                    float u = x - i0;
                    high = drive[i0] + (drive[i0 + 1] - drive[i0]) * u;
                    low = brake[i0] + (brake[i0 + 1] - brake[i0]) * u;
                }
                high = MathF.Min(high, limit);
                low = MathF.Max(low, floor);
                if (low > high)
                    continue;
                Try(k, b, here.Seconds, v, high, ds);
                if (low < high)
                    Try(k, b, here.Seconds, v, low, ds);
                for (int b1 = Bin(low) + 1; b1 * speedStep < high; b1++)
                    Try(k, b, here.Seconds, v, b1 * speedStep, ds);
            }
        }
        void Try(int k, int b, float before, float v, float v1, float ds)
        {
            float seconds = before + ds / MathF.Max(0.5f * (v + v1), 1f);
            if (extra is not null)
                seconds += extra(k, v, v1);
            if (seconds < float.PositiveInfinity)
                Reach(k + 1, Bin(v1), seconds, v1, b);
        }
        int last = count - 1, best = -1;
        float bestSeconds = float.PositiveInfinity;
        for (int b = lo[last]; b <= hi[last]; b++)
        {
            Choice end = grid[last * bins + b];
            if (end.From == Choice.Unreached)
                continue;
            float total = end.Seconds + rest(end.Speed);
            if (total < bestSeconds)
                (bestSeconds, best) = (total, b);
        }
        if (best < 0)
            return float.PositiveInfinity;
        for (int k = last, b = best; k >= 0; k--)
        {
            Choice c = grid[k * bins + b];
            speed[k] = c.Speed;
            b = c.From;
        }
        return bestSeconds;
    }

    /// <summary>How far apart in speed the drive and braking a choice reads are taken, m/s.</summary>
    public const float ChooseSample = 2f;

    private readonly record struct Choice(float Seconds, float Speed, int From)
    {
        public const int Unreached = -2;
        public static readonly Choice None = new(float.PositiveInfinity, 0f, Unreached);
    }

    [ThreadStatic] private static Choice[]? _choiceGrid;

    private static Choice[] ChoiceGrid(int size)
    {
        if (_choiceGrid is null || _choiceGrid.Length < size)
            _choiceGrid = new Choice[size];
        return _choiceGrid;
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
            v0 = MathF.Sqrt(vNext * vNext + 2f * Deceleration(envelope, v, curvature, factor, brakeFactor, road, shares) * ds);
        }
        return v0;
    }

    /// <summary>
    /// The braking the car has at <paramref name="v"/> on this stretch, as
    /// the backward sweep reckons it at the middle of a step: for a search
    /// that runs forwards, so the two stop the car in the same road.
    /// </summary>
    internal static float Deceleration(in LaneEnvelope envelope, float v, float curvature, float factor, float brakeFactor, LaneRoad road, LaneShares shares)
    {
        LaneEnvelope e = envelope.ForTurn(curvature);
        RoadAttitude att = road.Attitude;
        (float ay, float load) = Corner(v, curvature, road);
        float lateral = e.Lateral(v, load);
        // Braking unloads the rear by as much as it brakes, not by the most
        // it could: take the lateral limit part way to the full-braking
        // one, in proportion to how hard the first guess brakes, and ask again.
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
        // What the road does to the car along it, the physics' way round:
        // the axles are scaled onto the slope, gravity is added on top.
        float loss = e.Loss(v) + e.CornerDrag(v, ay / MathF.Max(lateral, 1f));
        return MathF.Max(0.1f, (ax + loss) * att.LongitudinalDemandScale - att.AlongTrackGravity(Gravity, v));
    }

    /// <summary>Full drive over one step, from <paramref name="v"/>: the forward sweep's own step.</summary>
    internal static float Driven(in LaneEnvelope envelope, float v, float curvature, float ds, float factor, LaneRoad road, LaneShares shares) =>
        DriveFrom(envelope, v, curvature, ds, factor, road, shares);

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
        float grip = MathF.Max(factor * e.Lateral(v, load), 1f);
        // The motor's drive or what the tyres have left beside the corner,
        // whichever is less. Scaling the motor by the ellipse charged it for
        // grip it never asked for: up a fast bend at 240 km/h the motor gives
        // nine metres a second squared against tyres good for fifty, and the
        // plan had it give seven, so the car was held to part throttle with
        // half its grip unused. What is left is read in the foot's own
        // circle (LaneShares.Traction).
        float use = ay / grip;
        float traction = grip * MathF.Sqrt(MathF.Max(0f, shares.Traction * shares.Traction - use * use));
        float push = shares.Drive * MathF.Min(e.Drive(v), traction);
        RoadAttitude att = road.Attitude;
        // The corner's drag is the tyres', at how full they are: of what
        // they hold, not of what the notes let the corner be taken at.
        float loss = e.Loss(v) + e.CornerDrag(v, ay / MathF.Max(e.Lateral(v, load), 1f));
        float accel = (push - loss) * att.LongitudinalDemandScale + att.AlongTrackGravity(Gravity, v);
        return MathF.Sqrt(MathF.Max(0f, v * v + 2f * accel * ds));
    }
}
