using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// How the road lies under a car: how steeply it climbs along the direction
/// of travel, how far it leans across it, and how sharply it bends in the
/// vertical plane. All three are held as rates — rise over run — rather than
/// angles, because that is the form the track surface is defined in and the
/// form the physics consumes.
///
/// Everything here is written in the plan view. The track is a
/// two-dimensional curve with a height laid over it: s is arc length on that
/// curve, the car's position moves in it, so speed is horizontal speed and
/// curvature is the curvature of the horizontal path. Getting that straight
/// matters, because the balance the model has to satisfy is then the
/// horizontal-and-vertical one:
///
///     N sin(phi) + f cos(phi) = m v^2 / R
///     N cos(phi) - f sin(phi) = m g
///
/// Written generally, with the surface normal (-grade, -bank, 1) / L and the
/// car's acceleration (a, -v^2 k, v^2 r + grade a) — the last term being what
/// following the road does to it vertically — the required force resolves to
///
///     N/m       = [ g + v^2 (k bank + r) ] / L
///     lateral/m = (G/L) v^2 k  -  bank (g + v^2 r) / (L G)
///     along/m   = a G  +  grade (g + v^2 r) / G
///
/// with G = sqrt(1 + grade^2) and L = sqrt(1 + grade^2 + bank^2). The terms
/// in a cancel out of the normal force: accelerating up a slope lifts the car
/// at exactly the rate that offsets the load it would otherwise shed.
///
/// The G/L on the cornering demand is the part that is easy to lose, and
/// losing it is expensive. A model can have the normal force exactly right —
/// so every test of grip passes — and still hand the tyres the whole of
/// v^2 k instead of its share along the surface, which at Daytona's banking
/// asks for fourteen percent more grip than the corner needs.
///
/// This is the "2.5D" of the model: the car stays a body on a surface rather
/// than a rigid body in three dimensions, so the road enters only by tilting
/// what gravity does and by changing how hard the car is pressed down. That
/// is enough for the three things that matter to racing — a climb costs
/// drive, a banked corner carries part of its own load, and a crest takes the
/// weight off the tyres just as you want to brake.
/// </summary>
public readonly record struct RoadAttitude(
    float GradeTangent,
    float BankTangent,
    float VerticalRate = 0f
)
{
    public static readonly RoadAttitude Flat = new(0f, 0f, 0f);

    /// <summary>
    /// The least of its weight the model will let a car keep. Past this the
    /// car has left the road, which a surface-bound model has nothing to say
    /// about, so it is held here instead.
    /// </summary>
    public const float MinimumNormalShare = 0.1f;

    /// <summary>How much longer a metre of road is than its plan view.</summary>
    private float AlongShape
    {
        get
        {
            float grade = Sanitize(GradeTangent);
            return MathF.Sqrt(1f + grade * grade);
        }
    }

    /// <summary>The same for the surface, leaning included.</summary>
    private float SurfaceShape
    {
        get
        {
            float grade = Sanitize(GradeTangent);
            float bank = Sanitize(BankTangent);
            return MathF.Sqrt(1f + grade * grade + bank * bank);
        }
    }

    /// <summary>
    /// Cosine of the surface's total tilt: the share of gravity left pressing
    /// straight into the road once the slope has taken its part.
    /// </summary>
    public float NormalCosine => 1f / SurfaceShape;

    /// <summary>
    /// What the corner's own demand becomes once resolved onto the surface
    /// the car is driving on. One on a flat road; cos(phi) on a pure bank.
    /// </summary>
    public float CurvatureDemandScale => AlongShape / SurfaceShape;

    /// <summary>
    /// What a force the tyres put down along the road becomes in the plan
    /// view. One on the level; less than one on a climb, because part of the
    /// push goes into lifting the car rather than moving it along the map.
    /// </summary>
    public float LongitudinalDemandScale => 1f / AlongShape;

    /// <summary>
    /// Gravity's pull along the direction of travel, negative uphill, in the
    /// plan view the rest of the simulation is written in.
    /// </summary>
    public float AlongTrackGravity(float gravity, float speedMetersPerSecond)
    {
        float grade = Sanitize(GradeTangent);
        float shape = AlongShape;
        return -grade * WeightPlusRoad(gravity, speedMetersPerSecond) /
               (shape * shape);
    }

    /// <summary>
    /// What the road adds to the tyres' lateral duty, in the same sense as
    /// curvature: negative where the bank leans into the corner and carries
    /// some of it. The track normal points right, so a surface rising to the
    /// right raises the outside of a left-hand corner and gravity then pulls
    /// the car the way the corner already wants to go.
    /// </summary>
    public float LateralGravityDemand(float gravity, float speedMetersPerSecond)
    {
        float bank = Sanitize(BankTangent);
        return -bank * WeightPlusRoad(gravity, speedMetersPerSecond) /
               (SurfaceShape * AlongShape);
    }

    /// <summary>
    /// What presses the car into the road: gravity's normal share, plus the
    /// part of the cornering load the banking turns into downward force, plus
    /// what the road's own vertical bend adds or takes away.
    ///
    /// This is why a steeply banked oval is quick — the faster the car goes
    /// the harder the bank pushes it down, and the grip grows with the demand.
    /// The vertical bend counts the same way and for the same reason: a
    /// compression presses the car down hard, which is the whole character of
    /// Eau Rouge, and over a crest the sign turns round and the brake pedal
    /// means less. Both scale with the square of the speed, so neither is felt
    /// slowly.
    /// </summary>
    public float NormalGravity(
        float gravity,
        float speedMetersPerSecond,
        float curvature
    )
    {
        float speedSquared = speedMetersPerSecond * speedMetersPerSecond;
        float fromRoad = speedSquared *
                         (Sanitize(curvature) * Sanitize(BankTangent) +
                          Sanitize(VerticalRate));
        return MathF.Max(
            (gravity + fromRoad) / SurfaceShape,
            gravity * MinimumNormalShare
        );
    }

    /// <summary>
    /// Weight as the road's vertical bend has left it: what both the along-
    /// track and across-track pulls are actually a share of.
    /// </summary>
    private float WeightPlusRoad(float gravity, float speedMetersPerSecond)
    {
        float speed = Sanitize(speedMetersPerSecond);
        return gravity + speed * speed * Sanitize(VerticalRate);
    }

    private static float Sanitize(float value) =>
        float.IsFinite(value) ? value : 0f;
}
