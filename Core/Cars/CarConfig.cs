namespace StintegyEVO.Core.Cars;

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
    public float MassKg { get; init; } = 820f;

    /// <summary>
    /// What turns the wheels. Owns the stores the strategist spends, the
    /// ladder they are spent on, and every number about either.
    /// </summary>
    public IPowertrain Powertrain { get; init; } = ElectricPowertrain.Default;
    public float WheelBaseMeters { get; init; } = 3.1f;
    public float TrackWidthMeters { get; init; } = 1.65f;
    public float CenterOfGravityHeightMeters { get; init; } = 0.32f;
    public float FrontStaticLoadShare { get; init; } = 0.47f;
    /// <summary>
    /// How much lateral deformation the front axle needs for the same force,
    /// relative to the rear axle. One means equal compliance; a larger value
    /// makes the front tyres do a larger share of the car's lateral rubber
    /// work without changing the force they deliver or how the car moves.
    /// </summary>
    public float FrontLateralComplianceRatio { get; init; } = 1.4f;
    public float FrontDriveShare { get; init; } = 0f;
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
    /// Raised from 1.2g when downforce arrived, because it had quietly become
    /// the thing that stopped the car. With wings the tyres will bear five g at
    /// speed, and a cap below that means the model brakes like a road car at
    /// the one moment a racing car does not. Calibrated with the wings and the
    /// drag together against published lap times: the three test circuits land
    /// within one per cent, cornering settles at 3.5g, braking at 4.1g and top
    /// speed at 301 km/h, all of which is where a Formula 2 car lives.
    /// </summary>
    public float MaxBrakeAccel { get; init; } = 40f;
    /// <summary>
    /// Traction control and, below it, anti-lock.
    ///
    /// Read these as a stand-in for the driver's reflexes rather than as
    /// electronics bolted to the car. What they model is the thing a driver
    /// does between the pedal and the tyre - easing off the instant an axle
    /// starts to go - and they live here because that reflex has to exist
    /// for the car to be driveable at all, not because this class of car
    /// carries the boxes.
    ///
    /// Which means their precision is a driver trait and will be modulated
    /// by the ability ratings when those arrive: a great pair of hands
    /// catches it early and gives back little, a poor pair catches it late
    /// and gives back a lot. Anyone tuning these for realism should be
    /// tuning a driver, not a control unit.
    /// </summary>
    public float TractionControlActivationUse { get; init; } = 0.99f;
    public float TractionControlStrength { get; init; } = 0.65f;

    /// <summary>
    /// Where the anti-lock starts holding the brakes back, and how much of the
    /// excess it takes away, in the same terms as the traction control beside
    /// it. Both axles, because both of them lock.
    ///
    /// Strength deliberately short of one: what it removes is braking the car
    /// does not get back, so an anti-lock that held the tyre exactly at its
    /// limit would be leaving the corner entry to a system that cannot see the
    /// corner. Taking most of the excess and no more lets the driver keep
    /// asking for slightly too much, which is what they do.
    ///
    /// Zero switches it off.
    /// </summary>
    public float AntiLockActivationUse { get; init; } = 0.99f;
    public float AntiLockStrength { get; init; } = 0.65f;
    public float RollingDragAccel { get; init; } = 0.18f;
    public float AeroDragAccelPerSpeedSquared { get; init; } = 0.0009f;

    /// <summary>
    /// Downforce, as the acceleration it would add to gravity, per squared
    /// metre per second.
    ///
    /// Written this way because that is how it is used: the wheels are pressed
    /// down by weight and by air, and only their sum matters to the tyre. It
    /// also makes the number readable - multiplied by the square of a speed it
    /// gives an acceleration to set beside 9.81, so a car making its own weight
    /// in downforce at fifty metres a second is carrying a coefficient of about
    /// four thousandths.
    ///
    /// Zero is a car with no wings at all, which is what the model had before
    /// this existed: grip that does not care how fast the car is going, so a
    /// fast corner is worth no more than a slow one of the same radius.
    ///
    /// The default is a Formula 2 car, and the drag beside it was raised to
    /// suit: a lift to drag ratio of about four, which is where real
    /// single-seaters sit, and 301 km/h down the longest straight.
    /// </summary>
    public float DownforceAccelPerSpeedSquared { get; init; } = 0.0035f;

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
    /// Fraction of aerodynamic drag removed while the car's overtake mode
    /// runs. A drag reduction system is the model: the published worth of one
    /// is ten to twelve km/h of top speed, and with top speed going as the
    /// cube root of drag, three metres per second at eighty needs roughly a
    /// tenth of the drag gone. This is a genuine straight-line gain over the
    /// car's own clean-air pace, exactly as it is for the real device.
    /// Eight percent is the measured sweet spot on top of a real tow: in the
    /// reference duel six percent converts no passes at all, ten converts
    /// every start at the same second, and eight wins some starts and not
    /// others - possible, never guaranteed.
    /// </summary>
    public float OvertakeAssistDragReduction { get; init; } = 0.08f;

    /// <summary>
    /// Fraction of the downforce lost to a leading car's wake that the
    /// overtake mode hands back. It scales what the wake removed, so in clean
    /// air it does nothing, and at one it would erase the cornering penalty
    /// of following entirely; half keeps dirty air a real cost while letting
    /// a committed follower live close enough to strike.
    /// </summary>
    public float OvertakeAssistDownforceRecovery { get; init; } = 0.5f;

    public float CorneringScrubAccel { get; init; } = 1.15f;
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
