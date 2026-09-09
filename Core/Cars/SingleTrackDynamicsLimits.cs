namespace StintegyEVO.Core.Cars;

/// <summary>
/// The numerical edges of the single-track lateral model, and what happens
/// at the one edge the model cannot represent.
///
/// There is no sideslip clamp here and no yaw acceleration clamp. Both used
/// to exist, and both were doing dynamical work that the tyres do now: a
/// car that will not come back is a car whose rear slip angle is past its
/// peak and staying there, which is a thing this model says out loud
/// instead of a thing a clamp used to hide.
///
/// What remains is a referee. Past thirty five degrees of body sideslip the
/// single-track assumptions have run out - the small-angle force directions
/// are wrong and the state cannot describe a car facing back down the road
/// - so the car is taken off the driver and a spin is played out rather
/// than integrated. Time and places are lost for real; nothing else is
/// pretended.
/// </summary>
internal static class SingleTrackDynamicsLimits
{
    /// <summary>
    /// A numerical guard, not a model: nothing on a race track rotates at
    /// a hundred and forty degrees a second except a car that is already
    /// spinning, and one that is spinning is being played rather than
    /// solved by then.
    /// </summary>
    public const float MaximumYawRateRadiansPerSecond = 2.5f;

    /// <summary>
    /// Body sideslip past which the model is no longer describing a car
    /// being driven, and how long it has to stay there. Sustained rather
    /// than instantaneous, because a flick through thirty five degrees that
    /// comes straight back is a save, and a save should be allowed to be
    /// spectacular.
    /// </summary>
    public const float SpinVerdictSideslipRadians = 0.610865238f;
    public const float SpinVerdictHoldSeconds = 0.25f;

    /// <summary>
    /// How hard a spinning car scrubs off speed. Four tyres sliding sideways
    /// is ploughing rather than braking, so this sits near the run-off
    /// figure rather than the racing one.
    /// </summary>
    public const float SpinScrubDecelerationMetersPerSecondSquared = 9f;

    /// <summary>
    /// Time constant the rotation the car arrived with is spent over, so a
    /// car that was barely rotating when it went does not pirouette.
    /// </summary>
    public const float SpinYawDecayTimeSeconds = 1.2f;

    /// <summary>
    /// Below this speed the car is being gathered up rather than spun: the
    /// body is steered back onto its direction of travel over
    /// <see cref="SpinGatherTimeSeconds"/> and handed back once it is
    /// pointing there, within <see cref="SpinReleaseSideslipRadians"/>.
    /// It is handed back moving, because a spin should cost a race and not
    /// end one.
    /// </summary>
    public const float SpinReleaseSpeedMetersPerSecond = 12f;
    /// <summary>
    /// How fast a spinning car's path bends towards the side of it that
    /// still has grip, at full asymmetry -- one pair of tyres on tarmac
    /// and the other on grass.
    ///
    /// Half a radian a second, which over the few seconds a spin lasts is
    /// enough to bring a car back towards the edge of the road rather than
    /// leaving it square in the middle of a run-off, and not enough to
    /// look like steering. A spun driver is not driving; the road is
    /// dragging one side of the car harder than the other, and this is the
    /// size of that.
    /// </summary>
    public const float SpinRecoveryBendRateRadiansPerSecond = 0.5f;

    public const float SpinGatherTimeSeconds = 0.45f;
    public const float SpinReleaseSideslipRadians = 0.034906585f;

    /// <summary>
    /// The lateral states are stiff at low speed - both time constants
    /// scale with it - so the sideslip and yaw pair gets its own subdivided
    /// clock inside a physics step. The count is worked out from the
    /// stiffness actually present rather than assumed, and capped, because
    /// past the cap the kinematic blend has taken over anyway.
    /// </summary>
    public const int MaximumLateralSubsteps = 4;
    public const float LateralSubstepSafetyFactor = 0.4f;
}
