using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// How the road is lying under a car: how steeply it climbs along the
/// direction of travel, and how far it leans across it. Both are stored as
/// tangents — rise over run — rather than angles, because that is the form
/// the track surface is defined in and the form the physics consumes.
///
/// This is the "2.5D" part of the model: the car remains a body moving on a
/// surface rather than a rigid body in three dimensions, so the road's
/// attitude enters only by tilting gravity and by changing how hard the car
/// is pressed into the tarmac. That is enough for the two effects that
/// matter to racing — a climb costs acceleration, and a banked corner both
/// carries part of the cornering load and adds grip while it does so.
/// </summary>
public readonly record struct RoadAttitude(
    float GradeTangent,
    float BankTangent,
    float VerticalCurvature = 0f
)
{
    public static readonly RoadAttitude Flat = new(0f, 0f, 0f);

    /// <summary>
    /// The least of its weight the model will let a car keep. Beyond this
    /// the car has left the road, which is a thing a surface-bound model
    /// cannot represent, so it is held here instead.
    /// </summary>
    public const float MinimumNormalShare = 0.1f;

    /// <summary>
    /// Cosine of the surface's total tilt: the share of gravity left
    /// pressing straight into the road once the slope has taken its part.
    /// </summary>
    public float NormalCosine
    {
        get
        {
            float grade = Sanitize(GradeTangent);
            float bank = Sanitize(BankTangent);
            return 1f / MathF.Sqrt(1f + grade * grade + bank * bank);
        }
    }

    /// <summary>
    /// Gravity's pull along the direction of travel, negative uphill.
    /// </summary>
    public float AlongTrackGravity(float gravity) =>
        -gravity * Sanitize(GradeTangent) * NormalCosine;

    /// <summary>
    /// The lateral acceleration the tyres must add on top of what the
    /// corner itself demands, in the same sense as curvature: positive is
    /// the way a positive curvature turns. The track normal points right,
    /// so a surface rising to the right raises the outside of a left-hand
    /// corner and gravity then pulls the car the way the corner already
    /// wants to go — which is why the sign here is negative, and why the
    /// same bank costs the tyres extra on a right-hander.
    /// </summary>
    public float LateralGravityDemand(float gravity) =>
        -gravity * Sanitize(BankTangent) * NormalCosine;

    /// <summary>
    /// What presses the car into the road: gravity's normal share, plus the
    /// part of the cornering load the banking turns into downward force.
    /// This is why a steeply banked oval is quick — the faster the car goes
    /// the harder the bank pushes it down, and the grip grows with the
    /// demand. It is clamped at a floor because a corner steep and fast
    /// enough to unload the car entirely has thrown it off the surface,
    /// which this reduced-order model does not represent.
    ///
    /// The road's own vertical bend counts the same way and for the same
    /// reason. Following a road that curves upward takes force beyond
    /// holding the car up, and the tarmac supplies it: a compression presses
    /// the car down hard, which is the whole character of Eau Rouge. Over a
    /// crest the sign turns round and the car goes light, which is why a
    /// brake pedal means less at the top of a hill. Both scale with the
    /// square of the speed, so neither is felt slowly and both arrive at
    /// once.
    /// </summary>
    public float NormalGravity(
        float gravity,
        float speedMetersPerSecond,
        float curvature
    )
    {
        float cosine = NormalCosine;
        // Curvature and bank sharing a sign means the road is leaning the
        // way the corner turns, and the cornering load then presses the car
        // down instead of sideways.
        float speedSquared = speedMetersPerSecond * speedMetersPerSecond;
        float fromBanking = speedSquared *
                            Sanitize(curvature) * Sanitize(BankTangent) *
                            cosine;
        float fromVerticalBend = speedSquared * Sanitize(VerticalCurvature);
        return MathF.Max(
            gravity * cosine + fromBanking + fromVerticalBend,
            gravity * MinimumNormalShare
        );
    }

    private static float Sanitize(float value) =>
        float.IsFinite(value) ? value : 0f;
}
