using System;
using StintegyEVO.Core.Cars;

namespace Official.SingleTrack;

/// <summary>
/// Everything that makes one car physically different from another.
/// </summary>
/// <remarks>
/// Device slots are the whole vocabulary. Core carries every vehicle mechanism
/// it knows -- the combined-grip limiter, push-to-pass, a powertrain's
/// resource slots -- and a car without one of them has that mechanism's
/// parameters at zero rather than a different interface: strength zero is a
/// limiter not fitted, a push-to-pass nobody presses does nothing, and a
/// resource slot with nothing in it is <see cref="PowertrainResourceClass.None"/>.
/// A second car is new values in these boxes, never a new shape.
/// </remarks>
public sealed class CarConfig
{
    /// <summary>
    /// What the car weighs without anything it burns off.
    ///
    /// Total mass is this plus whatever is left in the stores, which for an
    /// electric car is the same number all race and for a petrol one is not.
    /// Ask <see cref="CarPhysics.TotalMassKg"/> rather than reading this
    /// wherever the answer wanted is how much car there is right now.
    /// </summary>
    public float MassKg { get; init; } = 900f;

    /// <summary>
    /// What turns the wheels. Owns the stores the strategist spends, the
    /// ladder they are spent on, and every number about either.
    /// </summary>
    public IPowertrain Powertrain { get; init; } = RangeExtenderPowertrain.Default;

    /// <summary>
    /// The wings, the floor and the body against the air: downforce, drag
    /// and how a car in front changes both.
    /// </summary>
    public IAero Aero { get; init; } = new StandardAero();
    public float WheelBaseMeters { get; init; } = 3.1f;
    public float TrackWidthMeters { get; init; } = 1.65f;

    /// <summary>
    /// The body other cars and the walls meet (<see cref="CarBody"/>). The
    /// chassis's own is the box every car collided as before cars declared
    /// theirs: 4.8 by 1.9 m.
    /// </summary>
    public CarBody Body { get; init; } = new()
    {
        LengthMeters = 4.8f,
        WidthMeters = 1.9f,
        Restitution = 0.05f,
        Friction = 0.35f,
        WallRestitution = 0.1f,
        WallFriction = 0.5f,
    };
    public float CenterOfGravityHeightMeters { get; init; } = 0.32f;
    public float FrontStaticLoadShare { get; init; } = 0.47f;

    /// <summary>
    /// The front axle's share of the load a corner moves from the inside
    /// wheels to the outside ones: the roll stiffness split, set with the
    /// anti-roll bars. With tyres whose grip falls with load, the axle that
    /// takes more of it loses more grip, so a front share above the
    /// weight's makes the front the end that runs out first at the limit,
    /// and a car that runs wide rather than spinning when its tyres fade.
    /// </summary>
    public float FrontLateralLoadTransferShare { get; init; } = 0.55f;
    /// <summary>
    /// How much lateral deformation the front axle needs for the same force,
    /// relative to the rear axle. One means equal compliance; a larger value
    /// makes the front tyres do a larger share of the car's lateral rubber
    /// work without changing the force they deliver or how the car moves.
    /// </summary>
    public float FrontLateralComplianceRatio { get; init; } = 1.4f;

    /// <summary>
    /// The front motor's share of the car's peak drive: 0.42 is a 250 kW
    /// front motor beside a 350 kW rear one. Zero is a rear-driven car.
    ///
    /// With <see cref="AdaptiveDriveSplit"/> (the default) this is a ceiling,
    /// not a split: each step the drive is shared between the axles by the
    /// grip each has left after its corner, the way the brakes are, and each
    /// axle takes no more than its own motor can give. Without it, it is the
    /// fixed share of every request that goes to the front.
    /// </summary>
    public float FrontDriveShare { get; init; } = 0.42f;

    /// <summary>
    /// Share drive between the axles by the grip each has left, within each
    /// motor's share, rather than in the fixed <see cref="FrontDriveShare"/>.
    /// Two motors can do that at every instant, so a car that has them does.
    /// </summary>
    public bool AdaptiveDriveSplit { get; init; } = true;

    /// <summary>
    /// When push-to-pass may run. What it does is the powertrain's (more
    /// power) and the active aero's (<see cref="ActiveAeroWakeDownforceRecovery"/>).
    /// </summary>
    public PushToPassRules PushToPass { get; init; } = new();
    /// <summary>
    /// Where the axles sit relative to the centre of mass. The front arm is
    /// the longer one on this car, because the front carries less of the
    /// weight and so the mass sits nearer the rear.
    ///
    /// Named here rather than recomputed wherever it is wanted: the yaw
    /// balance needs it, and so does anything asking where a wheel is - and
    /// two copies of the same formula is one copy waiting to disagree.
    /// </summary>
    public float RearAxleOffsetMeters =>
        WheelBaseMeters * Math.Clamp(FrontStaticLoadShare, 0f, 1f);

    public float FrontAxleOffsetMeters =>
        WheelBaseMeters - RearAxleOffsetMeters;

    public float YawInertiaKgM2 { get; init; } = 1450f;

    /// <summary>
    /// How the car used to fake a yaw response and how quickly it used to
    /// straighten itself out. Both are dead numbers in the car: the yaw
    /// response is now whatever the front and rear slip angles say it is,
    /// and nothing straightens the car except the front tyres and the
    /// driver's hands. They survive because the analytic driver's motion
    /// predictor still carries the old kinematic approximation, and that
    /// driver is an instrument now rather than a car the model owes
    /// anything to.
    /// </summary>
    public float YawResponseTimeSeconds { get; init; } = 0.15f;
    public float SideslipRecoveryTimeSeconds { get; init; } = 0.15f;

    /// <summary>
    /// How far the front wheels can be turned. Set from the tightest corner
    /// the driver is allowed to ask for - a curvature request of
    /// <see cref="MaxCurvatureRequest"/> needs about forty five degrees on
    /// this wheelbase - with room above it for the angle a driver adds
    /// catching a slide, which is the whole reason the wheel angle is a
    /// state rather than an algebraic result.
    /// </summary>
    /// <summary>
    /// Where the front tyres peak, as a share of where the rears do.
    ///
    /// This is the car's balance at the limit, and it is the difference
    /// between a car that can be raced and one that cannot. Both axles have
    /// the grip their share of the weight gives them, so both run out at
    /// the same instant however this is set - what this decides is what
    /// happens after that instant. A front that peaks earlier is a front
    /// that is already sliding when the rear reaches its own peak, so the
    /// car washes wide, the yaw rate drops, the rear's slip angle falls
    /// with it, and the slide puts itself out.
    ///
    /// At one the car is neutral: both ends give up together, whichever one
    /// is nudged past first keeps going, and the result is a spin every
    /// time the driver asks for a few percent too much. Quicker on paper,
    /// unraceable in fact, and not what anything in this class runs.
    /// </summary>
    public float FrontPeakSlipAngleRatio { get; init; } = 1.35f;

    public float MaxSteerAngleRadians { get; init; } = 0.8f;

    /// <summary>
    /// How fast the front wheels can be moved, at the wheels themselves.
    /// Six hundred degrees a second at the driver's hands through a ten to
    /// one rack, which is a quick but human input: full opposite lock takes
    /// something over a third of a second, and that delay is exactly what
    /// makes a slide catchable or not.
    ///
    /// This is the first thing that makes the decision rate bite. A policy
    /// deciding fifteen times a second used to have its curvature appear
    /// instantly; now it appears at the speed a driver's arms can put it
    /// there.
    /// </summary>
    public float SteerRateLimitRadiansPerSecond { get; init; } = 1.047f;

    /// <summary>
    /// How hard the steering chases a curvature it is not getting. The
    /// driver asks for a corner, the wheels are set to the angle that
    /// geometry says would draw it, and this closes the difference the
    /// tyres' own slip angles leave behind - which is what a driver does
    /// when the car understeers and they wind on more lock.
    ///
    /// Deliberately small. A high gain here would be a driver who saws at
    /// the wheel, and it would hide the understeer this model exists to
    /// show.
    /// </summary>
    public float SteerCurvatureFeedbackGain { get; init; } = 0.5f;

    public float MaxCurvatureRequest { get; init; } = 0.32f;
    public float MaxDriveAcceleration { get; init; } = 12f;
    /// <summary>
    /// What the brakes themselves can do, before the tyres are asked whether
    /// they will take it.
    ///
    /// It is not what stops the car, the tyres are: set where a Formula One
    /// car's heaviest stops peak in Brembo's figures (5.5 to 5.9 g, from
    /// above 300 km/h), so at the top of the speed range the brakes and the
    /// tyres run out together and everywhere else the tyres go first.
    /// </summary>
    public float MaxBrakeAccel { get; init; } = 57f;
    /// <summary>
    /// The combined-grip limiter: an electronic stability device on the car,
    /// which this class of racing permits. It trims each axle's longitudinal
    /// force -- drive and brakes alike -- so that the tyre's combined use,
    /// slip angle and pedals together, ends the step inside the share of the
    /// friction circle the tyre allows, which is all of it unless the host
    /// asks for less (<see cref="CarPhysics.GripUsage"/>). It does
    /// not steer, so it never takes a correction away.
    ///
    /// On an axle that spends a share s of its circle on its slip angle, a
    /// longitudinal share u brings the combined use to
    ///
    ///     use^2 = s^2 + u^2 (1 - s^2)
    ///
    /// so the limiter holds u^2 to (A^2 - s^2) / (1 - s^2) for an
    /// authorisation A, and to nothing once s alone reaches A. An
    /// authorisation of one -- the Attack rung -- leaves nothing to limit.
    ///
    /// Past the tyre's peak slip angle the curve's share falls as the slide
    /// deepens, so a ceiling read off the share would rise with it. Above
    /// <see cref="CombinedGripLimiterPastPeakMinimumSpeed"/> an axle at or
    /// past its peak therefore gets no drive at all. Brakes are exempt: a
    /// sliding car always keeps its brakes. Below that speed the slip angle
    /// is estimated from a floored speed and reads past the peak on a car
    /// that is simply pulling away, so the lockout stays off there.
    ///
    /// Strength blends between the raw request (zero) and the limited one
    /// (one). Zero is the device not fitted, and is bit-for-bit the same car
    /// as one that never had it.
    /// </summary>
    public float CombinedGripLimiterStrength { get; init; } = 1f;

    /// <summary>
    /// The brake balance as a fixed share of the braking at the front, as a
    /// real car's is: set by the driver's dial (DriverInput.Axles
    /// moves it), not by the car. Where it does not match the grip each axle
    /// has, one of them locks first: the front and the car runs on, the rear
    /// and it steps out. Null is the old car, whose brakes shared themselves
    /// by the grip each axle had left at every instant and never locked one
    /// before the other.
    /// </summary>
    public float? FixedFrontBrakeShare { get; init; }
    public float CombinedGripLimiterPastPeakMinimumSpeed { get; init; } = 10f;
    public float RollingDragAccel { get; init; } = 0.18f;

    /// <summary>
    /// How much of its own speed the air is still carrying directly behind
    /// this car, as a fraction, before any of it has been left behind.
    ///
    /// This is the size of the hole the car punches, and it is written as a
    /// speed rather than as a share of drag because that is what it physically
    /// is. Drag is fought against the air the car is moving through, not
    /// against the ground: a car in air already travelling at a fifth of its
    /// speed meets four fifths of the wind, and pays the square of that.
    ///
    /// Zero leaves the car alone on the circuit however close it gets.
    ///
    /// The default is set from the one figure about a tow that can be checked
    /// without a wind tunnel: what it is worth at the end of a long straight,
    /// which for these cars is ten to fifteen km/h. Top speed goes as the cube
    /// root of the drag, so that is a tenth of it escaped at a couple of car
    /// lengths and no more.
    ///
    /// Deliberately not the far larger reductions quoted for drafting, which
    /// belong to cars with their wheels covered and no wings to lose. An open
    /// wheeled car following closely gives up downforce as it gains slipstream,
    /// and the tow that survives is the modest one.
    /// </summary>
    public float WakeVelocityDeficit { get; init; } = 0.11f;

    /// <summary>
    /// Additional share of downforce which the car's turbulent wake can make
    /// unusable for a following car at zero separation.
    ///
    /// The low-energy part of the wake already removes downforce through its
    /// reduced air speed. This is only the remainder: vortices and changing
    /// flow direction upsetting surfaces which still see some air. Keeping it
    /// separate lets that disturbance fade more slowly down the road without
    /// pretending the straight-line tow remains equally strong.
    /// </summary>
    public float WakeDownforceDisruption { get; init; } = 0.08f;

    /// <summary>
    /// Half distance of the drag relief this car's wake hands to a follower,
    /// as a hyperbolic tail. The two faces of a wake part company with
    /// distance: the wake rises off the road as it ages, but a following
    /// car's body stays sunk in the slowed air, so the tow keeps a long
    /// tail - at fifty metres roughly a seventh of the full effect survives,
    /// which is what makes slipstreaming from far back on a long straight a
    /// real tactic.
    /// </summary>
    public float WakeTowHalfDistanceMeters { get; init; } = 11f;

    /// <summary>
    /// Gaussian decay length of the dynamic-pressure share of the follower's
    /// downforce loss. Wings and floor ride close to the road and climb out
    /// of the rising wake within a couple of car lengths, so this face is
    /// short where the tow is long. Together with the disruption decay below
    /// it is set so the total downforce loss lands near the published
    /// post-2022 figures: roughly 16 % at ten metres and 3 % at twenty
    /// against the cited 18 % and 4 %.
    /// </summary>
    public float WakeDownforceDecayLengthMeters { get; init; } = 12f;

    /// <summary>
    /// Gaussian decay length of the turbulent disruption term. It outlives
    /// the downforce share of the dynamic pressure a little, but it is the
    /// same short-range creature: it also lives in the risen wake.
    /// </summary>
    public float WakeDirtyAirDecayLengthMeters { get; init; } = 16f;

    /// <summary>
    /// How strongly this car's own aerodynamics feel a wake's downforce
    /// effects. The drag relief of a tow is universal, but how much of the
    /// disturbed air reaches the working surfaces is a property of the
    /// following car: a ground-effect concept living below the risen wake
    /// runs lower than a car that leans on over-body wings.
    /// </summary>
    public float DirtyAirSensitivity { get; init; } = 1f;

    /// <summary>
    /// What is left of an axle's braking or drive once it is being asked
    /// for more than it has. Lateral force no longer passes through here -
    /// the slip curve is what takes cornering away from a tyre dragged past
    /// its peak - but a wheel asked to stop the car harder than the road
    /// will let it still gives back less than the road would have, and that
    /// is the same longitudinal chain the car has always had.
    /// </summary>
    public float OverLimitMinGripEfficiency { get; init; } = 0.8f;
    public float OverLimitCostCap { get; init; } = 0.2f;

    public float LoadTransferResponse { get; init; } = 8f;
    public float MinimumWheelLoadShare { get; init; } = 0.08f;
}
