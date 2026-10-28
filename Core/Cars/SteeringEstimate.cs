using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// The road-wheel angle that turns this car onto a curvature, read off the
/// car as it is this step: what a good driver's hands do with the wheel.
///
/// The physics turns the wheels to the angle it is given and no further
/// (<see cref="DriverInput.SteerAngleRadians"/>); how much lock a corner
/// wants is the driver's to judge. This is the judgement a driver who knew
/// the car exactly would make, for a driver to start from and get wrong in
/// its own way. Geometry is the start of the answer: the wheels have to be
/// turned further than the corner's own angle by the difference between the
/// two ends' slip angles, faded in as the tyres take up the load, with the
/// path the car is actually on fed back.
/// </summary>
public readonly record struct SteeringEstimate(
    float SpeedMetersPerSecond,
    float WheelBaseMeters,
    float FrontArmMeters,
    float RearArmMeters,
    float FrontGripAccel,
    float RearGripAccel,
    float FrontPeakSlipAngleRatio,
    float CurvatureFeedbackGain,
    float MaxSteerAngleRadians,
    float ActualLateralAccel,
    float ActualCurvature,
    float CurvatureDemandScale,
    float RoadLateralDemand)
{
    private const float Epsilon = 1e-4f;

    /// <summary>
    /// How loaded the car has to be, as a share of what its tyres can give
    /// laterally, before the slip-angle compensation is fully applied. Below
    /// it the extra fades away, so a turn from a straight line starts with
    /// the geometric angle and converges as the tyres load.
    /// </summary>
    public const float FullSlipCompensationShare = 0.35f;

    /// <summary>
    /// The most of an axle's grip the slip curve is read backwards for.
    /// Above it the curve is too flat near its peak to say which slip angle
    /// a share wants: a few hundredths of share invert to twice the angle,
    /// and asked for more than the rear could hold, the end nearer its peak
    /// read as a slip angle far larger than the other's and "front minus
    /// rear" turned the wheel off. Asked for more, both ends are asked for
    /// this and the curvature feedback finds the rest, which on the skidpad
    /// is nearly all of it (<see cref="TireSlipCurve.SustainablePeakShare"/>).
    /// </summary>
    public const float InversionShare = 0.9f;

    /// <summary>The lateral acceleration the tyres are asked for on this road to follow <paramref name="curvature"/>.</summary>
    public float LateralDemand(float curvature) =>
        CurvatureDemandScale * SpeedMetersPerSecond * SpeedMetersPerSecond * curvature + RoadLateralDemand;

    /// <summary>The angle to turn the road wheels to for <paramref name="curvature"/>, within full lock.</summary>
    public float AngleFor(float curvature)
    {
        float wheelBase = MathF.Max(WheelBaseMeters, Epsilon);
        float frontArm = MathF.Max(FrontArmMeters, Epsilon);
        float rearArm = MathF.Max(RearArmMeters, Epsilon);
        float frontCapacity = MathF.Max(FrontGripAccel, Epsilon);
        float rearCapacity = MathF.Max(RearGripAccel, Epsilon);
        float holdable = InversionShare * MathF.Min(
            frontCapacity * wheelBase / rearArm,
            rearCapacity * wheelBase / frontArm);
        float corner = Math.Clamp(LateralDemand(curvature), -holdable, holdable);
        float frontShare = rearArm / wheelBase * corner;
        float rearShare = frontArm / wheelBase * corner;
        float slipCompensation =
            TireSlipCurve.InverseEvaluate(frontShare / frontCapacity, FrontPeakSlipAngleRatio) -
            TireSlipCurve.InverseEvaluate(rearShare / rearCapacity);

        // Faded in with the load, because the inversion above is a steady
        // state and the car is not in one yet. Turning in from a straight,
        // none of those slip angles exist yet, and a car with a worn rear
        // under fresh fronts wants a smaller angle than geometry; applied
        // from the first frame, that asks for opposite lock before any slide
        // exists and then makes one the other way. Measured against what the
        // tyres can give rather than against what was asked for, so it is
        // one through any corner worth the name and zero on the straight.
        float loadedShare = FullSlipCompensationShare * MathF.Max(FrontGripAccel + RearGripAccel, Epsilon);
        float lateralLoad = Math.Clamp(MathF.Abs(ActualLateralAccel) / loadedShare, 0f, 1f);
        float kinematic = MathF.Atan(wheelBase * curvature);
        float feedforward = kinematic + slipCompensation * lateralLoad;
        float feedback = CurvatureFeedbackGain * wheelBase * (curvature - ActualCurvature);
        float maximum = MathF.Max(MaxSteerAngleRadians, 0f);
        return Math.Clamp(feedforward + feedback, -maximum, maximum);
    }
}
