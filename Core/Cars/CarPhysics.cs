using System;
using System.Numerics;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Cars;

public static class CarPhysics
{
    private const float Gravity = 9.80665f;
    private const float Epsilon = 1e-5f;
    /// <summary>
    /// Below this speed the lateral model is kinematic: sideslip is not
    /// carried and the body turns as its wheels point, speed times the
    /// tangent of the lock over the wheelbase. Contact resolvers read it to
    /// know where an attitude they hand over would simply be discarded.
    ///
    /// It is where the slip-angle model has to stop, not anywhere it could
    /// carry on: a slip angle divides by speed, and the lateral pair's time
    /// constants shrink with it until a physics step cut into its most
    /// pieces cannot follow them, which for this car is about 2.5 m/s. Any
    /// racing corner, the slowest hairpin included, is well above it.
    /// </summary>
    internal const float DynamicYawMinimumSpeed = 3f;
    private const float DynamicYawBlendRange = 2f;

    /// <summary>
    /// Speed below which a slip angle stops meaning anything, because the
    /// yaw-rate term divided by it stops meaning anything. The kinematic
    /// blend owns everything under here.
    /// </summary>
    private const float MinimumSlipAngleSpeed = 3f;

    /// <summary>
    /// How loaded the car has to be, as a share of what its tyres can give
    /// laterally, before the slip-angle compensation is fully applied. Below
    /// it the extra fades away, so a curvature command from a straight line
    /// starts with the geometric angle and converges as the tyres load.
    /// </summary>
    private const float FullSlipCompensationShare = 0.35f;

    /// <summary>
    /// The chassis's own ladders are its powertrain's output ladder and
    /// nothing else, so that is the first one a strategy sets for it.
    /// </summary>
    internal const int OutputLadderIndex = 0;

    private static int OutputRung(CarConfig config, in CarStrategy strategy) =>
        strategy.Rung(OutputLadderIndex, config.Powertrain.OutputLadder);

    private static readonly WheelId[] Wheels =
    {
        WheelId.FrontLeft,
        WheelId.FrontRight,
        WheelId.RearLeft,
        WheelId.RearRight
    };

    /// <summary>
    /// How much car there is right now: the chassis plus whatever is left in
    /// the stores. A constant for a car that carries a battery, which weighs
    /// the same flat as full, and a falling number for a car that carries
    /// fuel - which is most of why a petrol car's last lap is its quickest.
    /// </summary>
    public static float TotalMassKg(CarConfig config, in PowertrainState energy)
    {
        return config.MassKg + config.Powertrain.ConsumableMassKg(energy);
    }

    /// <summary>Downforce in newtons per squared metre per second, after the wake and the active aero.</summary>
    internal static float EffectiveDownforcePerSpeedSquared(
        CarState state,
        CarConfig config
    )
    {
        return EffectiveDownforcePerSpeedSquared(
            config,
            state.DownforceVelocityDeficit,
            state.WakeDownforceLoss,
            state.ActiveAero
        );
    }

    /// <summary>
    /// The steady corner this car can hold at this speed, read with that
    /// corner's own load on the outside wheels.
    ///
    /// The envelope read at no curvature has every wheel evenly loaded,
    /// and a tyre whose friction falls with load grips more that way than
    /// it ever will in a bend; asking for a share of that figure asks for
    /// more than the car has. This settles the corner on itself: the limit
    /// read at the curvature the limit implies, a few times over.
    /// </summary>
    public static float EstimateCorneringLimit(
        CarState state,
        CarConfig config,
        TireSet tires,
        CarStrategy strategy,
        float speed,
        float gripUsage = 1f
    )
    {
        float v = MathF.Max(speed, MinimumSlipAngleSpeed);
        float lateral = EstimatePerformanceLimits(
            state, config, tires, strategy, v, 0f, gripUsage).LateralAccelerationLimit;
        for (int i = 0; i < 6; i++)
            lateral = EstimatePerformanceLimits(
                state, config, tires, strategy, v, lateral / (v * v), gripUsage).LateralAccelerationLimit;
        return lateral;
    }

    /// <summary>
    /// The published performance envelope: what this car can do right now,
    /// computed only from its public parameters (<paramref name="config"/>,
    /// <paramref name="tires"/>), the team's strategy, and physical state.
    /// Controllers outside Core use it to scale their commands to the car
    /// they are driving; it applies nothing and changes nothing.
    /// </summary>
    /// <param name="gripUsage">The share of the friction circle to plan
    /// against, usually the tyre rung's authorisation.</param>
    public static CarPerformanceLimits EstimatePerformanceLimits(
        CarState state,
        CarConfig config,
        TireSet tires,
        CarStrategy strategy,
        float speed,
        float curvature,
        float gripUsage = 1f,
        float assumedLongitudinalAcceleration = 0f,
        float frontBrakeBiasOffset = 0f
    )
    {
        float lateralAcceleration = speed * speed * curvature;
        float massKg = TotalMassKg(config, state.Energy);
        WheelLoads loads = CalculateWheelLoads(
            config,
            massKg,
            assumedLongitudinalAcceleration,
            lateralAcceleration,
            speed,
            state.DownforceVelocityDeficit,
            state.WakeDownforceLoss,
            state.ActiveAero
        );
        float usage = Math.Clamp(gripUsage, 0.05f, 1f);
        float frontGrip = (
            WheelGripForce(config, tires, state.FrontLeft, loads.FrontLeft, front: true) +
            WheelGripForce(config, tires, state.FrontRight, loads.FrontRight, front: true)
        ) / Math.Max(massKg, Epsilon) * usage;
        float rearGrip = (
            WheelGripForce(config, tires, state.RearLeft, loads.RearLeft, front: false) +
            WheelGripForce(config, tires, state.RearRight, loads.RearRight, front: false)
        ) / Math.Max(massKg, Epsilon) * usage;

        // What the car can hold, not what it can touch. See
        // TireSlipCurve.SustainablePeakShare: planning against the whole
        // circle means arriving at every apex a tenth over what the tyres
        // will give, and running wide by exactly that.
        float lateralLimit = AxleLateralCeiling(config, frontGrip, rearGrip);

        // Charge the tyre for the full requested corner. Driver skill belongs
        // upstream of this vehicle boundary; the physical estimate receives
        // only the curvature that was actually requested.
        float chargedLateralAcceleration = lateralAcceleration;
        (float frontLateral, float rearLateral) = AxleLateralDemand(
            config,
            chargedLateralAcceleration,
            frontGrip,
            rearGrip
        );
        float frontLongitudinal = RemainingLongitudinalGrip(
            frontGrip,
            frontLateral
        );
        float rearLongitudinal = RemainingLongitudinalGrip(
            rearGrip,
            rearLateral
        );

        float powertrainDriveLimit = config.Powertrain.DriveAccelerationLimit(
            state.Energy,
            OutputRung(config, strategy),
            speed,
            massKg,
            config.MaxDriveAcceleration,
            state.PushToPass.Active
        );
        float gripDriveLimit = config.AdaptiveDriveSplit
            ? SharedDriveLimit(frontLongitudinal, rearLongitudinal, powertrainDriveLimit, config.FrontDriveShare)
            : DistributedLongitudinalLimit(
                frontLongitudinal,
                rearLongitudinal,
                config.FrontDriveShare
            );
        float maximumDrive = Math.Min(
            config.MaxDriveAcceleration,
            Math.Min(gripDriveLimit, powertrainDriveLimit)
        );
        float frontBrakeShare = BiasedFrontBrakeShare(
            frontLongitudinal,
            rearLongitudinal,
            frontBrakeBiasOffset
        );
        float maximumBrake = Math.Min(
            config.MaxBrakeAccel,
            DistributedLongitudinalLimit(
                frontLongitudinal,
                rearLongitudinal,
                frontBrakeShare
            )
        );
        float loss = CalculateLossAccel(
            config,
            speed,
            state.AirVelocityDeficit,
            massKg
        ) + SteadyInducedDragAccel(
            config, speed, frontLateral, rearLateral, frontGrip, rearGrip
        );

        return new CarPerformanceLimits(
            Math.Max(0f, lateralLimit),
            Math.Max(0f, maximumDrive),
            Math.Max(0f, maximumBrake),
            Math.Max(0f, loss)
        );
    }

    /// <summary>
    /// Only how hard the car will corner, at a speed, round a curvature, with
    /// the weight where a given longitudinal acceleration puts it.
    ///
    /// The same figure the full estimate returns, and reached the same way, but
    /// without the drive limit, the brake distribution or the losses beside it.
    /// A speed plan that wants to know whether a corner survives the braking it
    /// is planning has to ask this once per point of the horizon, every frame,
    /// for every car, and everything the full answer carries is thrown away
    /// unread - including the battery model, which is the dearest part of it.
    /// </summary>
    internal static float EstimateLateralAccelerationLimit(
        CarState state,
        CarConfig config,
        TireSet tires,
        float speed,
        float curvature,
        float gripUsage,
        float assumedLongitudinalAcceleration
    )
    {
        float massKg = TotalMassKg(config, state.Energy);
        WheelLoads loads = CalculateWheelLoads(
            config,
            massKg,
            assumedLongitudinalAcceleration,
            speed * speed * curvature,
            speed,
            state.DownforceVelocityDeficit,
            state.WakeDownforceLoss,
            state.ActiveAero
        );
        float usage = Math.Clamp(gripUsage, 0.05f, 1f);
        float mass = Math.Max(massKg, Epsilon);
        float frontGrip = (
            WheelGripForce(config, tires, state.FrontLeft, loads.FrontLeft, front: true) +
            WheelGripForce(config, tires, state.FrontRight, loads.FrontRight, front: true)
        ) / mass * usage;
        float rearGrip = (
            WheelGripForce(config, tires, state.RearLeft, loads.RearLeft, front: false) +
            WheelGripForce(config, tires, state.RearRight, loads.RearRight, front: false)
        ) / mass * usage;

        return MathF.Max(
            0f,
            AxleLateralCeiling(config, frontGrip, rearGrip)
        );
    }

    /// <summary>The game's own tyre, built to <paramref name="tires"/>, all round.</summary>
    public static float EstimateCorneringLimit(
        CarState state, CarConfig config, TireConfig tires, CarStrategy strategy, float speed, float gripUsage = 1f) =>
        EstimateCorneringLimit(state, config, TireSet.Of(tires), strategy, speed, gripUsage);

    /// <summary>The game's own tyre, built to <paramref name="tires"/>, all round.</summary>
    public static CarPerformanceLimits EstimatePerformanceLimits(
        CarState state, CarConfig config, TireConfig tires, CarStrategy strategy, float speed, float curvature,
        float gripUsage = 1f, float assumedLongitudinalAcceleration = 0f, float frontBrakeBiasOffset = 0f) =>
        EstimatePerformanceLimits(state, config, TireSet.Of(tires), strategy, speed, curvature, gripUsage,
            assumedLongitudinalAcceleration, frontBrakeBiasOffset);

    /// <summary>The game's own tyre, built to <paramref name="tires"/>, all round.</summary>
    public static void Step(CarState state, CarConfig config, TireConfig tires, CarPhysicsStepInput input, float dt) =>
        Step(state, config, TireSet.Of(tires), input, dt);

    public static void Step(
        CarState state,
        CarConfig config,
        TireSet tires,
        CarPhysicsStepInput input,
        float dt
    )
    {
        if (dt <= 0f)
            return;

        state.Normalize();

        // The button and the clock first, so everything below sees the push
        // as it stands this step. A car that has been declared lost is not
        // pressing anything.
        state.PushToPass = PushToPassState.Advance(
            state.PushToPass,
            config.PushToPass,
            input.Strategy.PushToPass && !state.Spinning,
            dt
        );
        state.ActiveAero = state.PushToPass.Active ? 1f : 0f;

        // What each wheel is standing on, before anything asks how much
        // grip there is.
        foreach (WheelId wheel in Wheels)
            state.GetTire(wheel).SurfaceGrip = input.SurfaceGrip[wheel];

        // A car that has been declared lost is not being driven, so nothing
        // below this line runs: no request is read, no axle is resolved,
        // and the rotation is played rather than integrated.
        if (state.Spinning)
        {
            StepScriptedSpin(state, config, tires, input, dt);
            return;
        }

        RoadAttitude road = input.RoadAttitude;
        float roadNormalGravity = road.NormalGravity(
            Gravity,
            state.Speed,
            state.Telemetry.ActualCurvature
        );
        float roadAlongGravity = road.AlongTrackGravity(Gravity, state.Speed);
        float roadLateralDemand =
            road.LateralGravityDemand(Gravity, state.Speed);
        float curvatureDemandScale = road.CurvatureDemandScale;
        float longitudinalDemandScale = road.LongitudinalDemandScale;

        float massKg = TotalMassKg(config, state.Energy);
        WheelLoads loads = CalculateWheelLoads(
            state,
            config,
            massKg,
            roadNormalGravity
        );
        ApplyWheelLoads(state, loads);

        float frontGrip = CalculateAxleGripAccel(massKg, config, tires, state.FrontLeft, state.FrontRight, front: true);
        float rearGrip = CalculateAxleGripAccel(massKg, config, tires, state.RearLeft, state.RearRight, front: false);
        float totalGrip = Math.Max(Epsilon, frontGrip + rearGrip);

        float desiredCurvature = Math.Clamp(
            input.DriverInput.DesiredCurvature,
            -config.MaxCurvatureRequest,
            config.MaxCurvatureRequest
        );
        float desiredAccel = Math.Clamp(
            input.DriverInput.DesiredAccel,
            -config.MaxBrakeAccel,
            config.MaxDriveAcceleration
        );

        // The corner is asked for in the plan view; the tyres answer along
        // the surface, and on a bank those are not the same size. Handing
        // over the whole of v^2 k asks for more grip than the corner needs --
        // fourteen percent more at Daytona's angle.
        //
        // This is still worth naming because it is what the DriverInput asked
        // for and what the telemetry reports against. Nothing downstream
        // obeys it any more: the tyres deliver what their slip angles say,
        // and the difference between the two is understeer.
        float requestedLateralAccel =
            curvatureDemandScale * state.Speed * state.Speed * desiredCurvature +
            roadLateralDemand;
        float referenceYawRate = state.Speed * desiredCurvature;
        float dynamicYawBlend = CalculateDynamicYawBlend(state.Speed);

        float steerAngle = UpdateSteerAngle(
            state,
            config,
            desiredCurvature,
            requestedLateralAccel,
            frontGrip,
            rearGrip,
            dt
        );

        // What each axle is giving laterally right now with its whole
        // circle available. The brake allocator needs to know how much of
        // each axle is already spoken for, and that is no longer a request
        // to be granted: it is a measurement of what the rubber is doing at
        // the angle it is at.
        float frontLatRequest = frontGrip *
                                TireSlipCurve.Evaluate(
                                    FrontSlipAngle(
                                        config,
                                        steerAngle,
                                        state.SideslipAngleRadians,
                                        state.YawRateRadiansPerSecond,
                                        state.Speed
                                    ),
                                    config.FrontPeakSlipAngleRatio
                                );
        float rearLatRequest = rearGrip *
                               TireSlipCurve.Evaluate(
                                   RearSlipAngle(
                                       config,
                                       state.SideslipAngleRadians,
                                       state.YawRateRadiansPerSecond,
                                       state.Speed
                                   )
                               );

        float frontLongRequest = 0f;
        float rearLongRequest = 0f;
        float requestedLongitudinalAccel;

        if (desiredAccel >= 0f)
        {
            float available = config.Powertrain.DriveAccelerationLimit(
                state.Energy,
                OutputRung(config, input.Strategy),
                state.Speed,
                massKg,
                config.MaxDriveAcceleration,
                state.PushToPass.Active
            );
            float driveAccel = Math.Min(desiredAccel, available);
            if (config.AdaptiveDriveSplit)
            {
                AllocateDriveRequest(
                    driveAccel,
                    available,
                    frontLatRequest,
                    rearLatRequest,
                    frontGrip,
                    rearGrip,
                    config.FrontDriveShare,
                    input.DriverInput.FrontDriveBiasOffset,
                    out frontLongRequest,
                    out rearLongRequest
                );
            }
            else
            {
                frontLongRequest = driveAccel * config.FrontDriveShare;
                rearLongRequest = driveAccel - frontLongRequest;
            }
            requestedLongitudinalAccel = driveAccel;
        }
        else
        {
            float brakeRequest = Math.Min(-desiredAccel, config.MaxBrakeAccel);
            AllocateBrakeRequest(
                brakeRequest,
                frontLatRequest,
                rearLatRequest,
                frontGrip,
                rearGrip,
                input.DriverInput.FrontBrakeBiasOffset,
                out float frontBrake,
                out float rearBrake
            );
            frontLongRequest = -frontBrake;
            rearLongRequest = -rearBrake;
            requestedLongitudinalAccel = -(frontBrake + rearBrake);
        }

        // Sideslip and yaw rate get their own subdivided clock: both of
        // their time constants shrink with speed, so the model is stiffest
        // exactly where the car is slowest. Everything else - the wheel
        // angle, the grip, what the brakes are doing - is held across the
        // subdivision, because those are set once per physics step by
        // things outside this loop.
        LateralIntegration lateral = IntegrateLateral(
            state,
            config,
            massKg,
            frontGrip,
            rearGrip,
            frontLongRequest,
            rearLongRequest,
            steerAngle,
            roadLateralDemand,
            curvatureDemandScale,
            dynamicYawBlend,
            tires.Front.GripUsage(input.Strategy),
            dt
        );
        AxleResult front = lateral.Front;
        AxleResult rear = lateral.Rear;

        float actualLateralAccel = front.LateralAccel + rear.LateralAccel;
        float driveAccelActual = Math.Max(0f, front.LongitudinalAccel) + Math.Max(0f, rear.LongitudinalAccel);
        float brakeAccelActual = Math.Max(0f, -front.LongitudinalAccel) + Math.Max(0f, -rear.LongitudinalAccel);
        float axleLongitudinalAccel = front.LongitudinalAccel + rear.LongitudinalAccel;

        // What the tyre was worked, not only what came out of it. Lateral
        // slip still spends rubber even when the resulting force is reduced,
        // so wear follows physical work rather than a controller's extraction.
        //
        // Past the peak that stops being a share of the force at all, and
        // charging it as one gets the sign wrong: force falls away beyond
        // the peak, so a car ploughing at three times its best slip angle
        // would be billed less rubber than one sitting neatly on it. What a
        // tyre actually spends is frictional work - force times how fast the
        // rubber is being dragged across the road - and the dragging goes
        // with the slip angle. Below the peak the two readings are the same
        // number and every tyre figure ever calibrated still stands; above
        // it, sliding gets expensive, which is what sliding is.
        float frontLateralUse = front.LateralUse *
                                ScrubWeight(
                                    lateral.FrontSlipAngle,
                                    config.FrontPeakSlipAngleRatio
                                );
        float rearLateralUse = rear.LateralUse *
                               ScrubWeight(lateral.RearSlipAngle, 1f);
        // The heat half of the same argument. Wear is force times sliding
        // and so is heat, but they are not the same function of it: wear
        // goes with the square root of the slip angle (above), while the
        // rubber's power dissipation goes with the sliding speed itself.
        // So heat gets its own reading -- force share times how fast the
        // contact patch is being dragged sideways -- computed here, where
        // the slip angles are, rather than reconstructed downstream from a
        // utilisation that has already lost them.
        float frontLateralSlipWork =
            Math.Clamp(front.LateralUse, 0f, 1f) *
            SlipSpeedShare(
                lateral.FrontSlipAngle,
                config.FrontPeakSlipAngleRatio
            );
        float rearLateralSlipWork =
            Math.Clamp(rear.LateralUse, 0f, 1f) *
            SlipSpeedShare(lateral.RearSlipAngle, 1f);
        float frontLongitudinalUse = front.LongitudinalUse;
        float rearLongitudinalUse = rear.LongitudinalUse;
        float frontBrakeUse = front.LongitudinalAccel < 0f
            ? frontLongitudinalUse
            : 0f;
        float rearBrakeUse = rear.LongitudinalAccel < 0f
            ? rearLongitudinalUse
            : 0f;
        AxleLateralWorkScales lateralWorkScales =
            CalculateAxleLateralWorkScales(
                state,
                config,
                tires,
                front,
                rear,
                frontLateralSlipWork,
                rearLateralSlipWork
            );
        float lateralUse = Math.Abs(actualLateralAccel) / totalGrip;
        float overLimit = Math.Max(front.OverLimit, rear.OverLimit);
        float actualYawAcceleration = lateral.YawAcceleration;
        float rearSlideSeverity = CalculateRearSlideSeverity(
            config,
            lateral.FrontSlipAngle,
            lateral.RearSlipAngle
        );

        // What the corner costs, read off the axles that are paying it.
        float inducedDragAccel = InducedDragAccel(
            front.LateralAccel,
            lateral.FrontSlipAngle,
            rear.LateralAccel,
            lateral.RearSlipAngle,
            dynamicYawBlend
        );
        float lossAccel = CalculateLossAccel(
            config,
            state.Speed,
            state.AirVelocityDeficit,
            massKg
        ) + inducedDragAccel;
        float actualLongitudinalAccel =
            (axleLongitudinalAccel - lossAccel) * longitudinalDemandScale +
            roadAlongGravity;

        float oldSpeed = state.Speed;
        float newSpeed = Math.Max(0f, oldSpeed + actualLongitudinalAccel * dt);
        float averageSpeed = (oldSpeed + newSpeed) * 0.5f;

        // Gravity bends the path as surely as the tyres do. The bank was
        // taken off what the tyres delivered, so it has to be added back
        // here or the car would corner only as hard as the tyres alone and
        // run wide on exactly the surface built to hold it in. Undone in
        // the same order it was applied, so the curvature that comes back
        // out is the one the plan view will actually see.
        float actualCurvature = averageSpeed > 0.5f
            ? lateral.PathLateralAccel /
              Math.Max(averageSpeed * averageSpeed, Epsilon)
            : 0f;
        referenceYawRate = averageSpeed * desiredCurvature;

        float velocityHeading = state.VelocityHeading;
        float nextBodyHeading = MathHelper.NormalizeAngle(
            state.Heading + lateral.HeadingDelta
        );
        float nextSideslipAngle = lateral.Sideslip;
        float nextVelocityHeading = MathHelper.NormalizeAngle(
            nextBodyHeading + nextSideslipAngle
        );
        float travelHeading = velocityHeading + MathHelper.NormalizeAngle(
            nextVelocityHeading - velocityHeading
        ) * 0.5f;
        Vector2 travelDirection = new(
            MathF.Cos(travelHeading),
            MathF.Sin(travelHeading)
        );

        state.Position += travelDirection * averageSpeed * dt;
        state.SideslipAngleRadians = nextSideslipAngle;
        state.YawRateRadiansPerSecond = lateral.YawRate;
        state.Heading = nextBodyHeading;
        state.SteerAngleRadians = steerAngle;
        state.Speed = newSpeed;
        UpdateSpinVerdict(state, dt);
        PowertrainSettlement settlement = config.Powertrain.Settle(
            state.Energy,
            driveAccelActual,
            brakeAccelActual,
            averageSpeed,
            massKg,
            dt,
            state.PushToPass.Active
        );
        state.Energy = settlement.Energy;
        float drivePowerWatts = settlement.DrawnPowerWatts;
        float regenPowerWatts = config.Powertrain.RecoveredPowerWatts(
            brakeAccelActual,
            averageSpeed,
            massKg
        );

        float costedFrontOverLimit = CostedOverLimit(config, front.OverLimit);
        float costedRearOverLimit = CostedOverLimit(config, rear.OverLimit);
        float frontCombinedUse = Math.Clamp(
            MathF.Sqrt(
                frontLateralUse * frontLateralUse +
                frontLongitudinalUse * frontLongitudinalUse
            ),
            0f,
            1f
        );
        float rearCombinedUse = Math.Clamp(
            MathF.Sqrt(
                rearLateralUse * rearLateralUse +
                rearLongitudinalUse * rearLongitudinalUse
            ),
            0f,
            1f
        );
        // Combined tire demand creates a common intensity for this instant.
        // Each axle still receives heat according
        // to its own force and compliance; sharing only the intensity avoids
        // turning a rear-limited car into an artificial rear-tyre heater.
        float directionalHeatDemandUse = Math.Max(
            frontCombinedUse,
            rearCombinedUse
        );
        float coolingAirSpeed = averageSpeed *
                                (
                                    1f - Math.Clamp(
                                        state.AirVelocityDeficit,
                                        0f,
                                        1f
                                    )
                                );
        float tireWakeDownforceLoss =
            EffectiveTireWakeDownforceLoss(state, config);
        UpdateTires(
            state.FrontLeft,
            true,
            config,
            tires,
            frontLateralUse,
            frontLateralSlipWork,
            frontLongitudinalUse,
            frontBrakeUse,
            directionalHeatDemandUse,
            lateralWorkScales.FrontHeat,
            lateralWorkScales.FrontWear,
            costedFrontOverLimit,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            tireWakeDownforceLoss,
            dt
        );
        UpdateTires(
            state.FrontRight,
            true,
            config,
            tires,
            frontLateralUse,
            frontLateralSlipWork,
            frontLongitudinalUse,
            frontBrakeUse,
            directionalHeatDemandUse,
            lateralWorkScales.FrontHeat,
            lateralWorkScales.FrontWear,
            costedFrontOverLimit,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            tireWakeDownforceLoss,
            dt
        );
        UpdateTires(
            state.RearLeft,
            false,
            config,
            tires,
            rearLateralUse,
            rearLateralSlipWork,
            rearLongitudinalUse,
            rearBrakeUse,
            directionalHeatDemandUse,
            lateralWorkScales.RearHeat,
            lateralWorkScales.RearWear,
            costedRearOverLimit,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            tireWakeDownforceLoss,
            dt
        );
        UpdateTires(
            state.RearRight,
            false,
            config,
            tires,
            rearLateralUse,
            rearLateralSlipWork,
            rearLongitudinalUse,
            rearBrakeUse,
            directionalHeatDemandUse,
            lateralWorkScales.RearHeat,
            lateralWorkScales.RearWear,
            costedRearOverLimit,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            tireWakeDownforceLoss,
            dt
        );

        float response = 1f - MathF.Exp(-config.LoadTransferResponse * dt);
        state.FilteredLongitudinalAccel = Lerp(state.FilteredLongitudinalAccel, actualLongitudinalAccel, response);
        state.FilteredLateralAccel = Lerp(state.FilteredLateralAccel, actualLateralAccel, response);

        state.Telemetry = new CarTelemetry(
            input.DriverInput,
            input.Strategy,
            requestedLateralAccel,
            actualLateralAccel,
            requestedLongitudinalAccel,
            actualLongitudinalAccel,
            lossAccel,
            actualCurvature,
            overLimit,
            drivePowerWatts,
            regenPowerWatts,
            state.SideslipAngleRadians,
            referenceYawRate,
            state.YawRateRadiansPerSecond,
            actualYawAcceleration
        );
        SingleTrackState.Of(state).Axles = new AxleTelemetry(
            frontGrip,
            rearGrip,
            frontLateralUse,
            rearLateralUse,
            frontLongitudinalUse,
            rearLongitudinalUse,
            lateral.Front.LimiterCut + lateral.Rear.LimiterCut,
            inducedDragAccel,
            rearSlideSeverity
        );

        state.Normalize();
    }

    /// <summary>
    /// The referee. Past the angle where the single-track model stops
    /// describing a car being driven, held there rather than flashed
    /// through, the car is declared lost and handed to the choreography.
    ///
    /// Sustained rather than instantaneous on purpose: a flick through
    /// thirty five degrees that comes straight back is a save, and a save
    /// should be allowed to be spectacular. What is not allowed is sitting
    /// there, which is the one thing the old ten degree clamp used to make
    /// both free and invisible.
    /// </summary>
    private static void UpdateSpinVerdict(CarState state, float dt)
    {
        float sideslip = MathF.Abs(state.SideslipAngleRadians);
        state.SideslipHoldSeconds =
            sideslip >= SingleTrackDynamicsLimits.SpinVerdictSideslipRadians
                ? state.SideslipHoldSeconds + dt
                : 0f;

        if (state.SideslipHoldSeconds <=
            SingleTrackDynamicsLimits.SpinVerdictHoldSeconds)
        {
            return;
        }

        state.Spinning = true;
        state.SpinSeconds = 0f;
        state.SideslipHoldSeconds = 0f;
        state.SpinEvents++;
    }

    /// <summary>
    /// A spin, played rather than solved.
    ///
    /// The car keeps travelling the way it was travelling and scrubs speed
    /// off at run-off rather than racing rate; the body turns at the
    /// rotation it arrived with, bleeding away; the front wheels drift onto
    /// the direction of travel, which is where a spinning car's wheels end
    /// up; and once it is slow enough to be collected the body is steered
    /// back onto its course and handed to the driver.
    ///
    /// Nothing here integrates a spin, and nothing here needs to. Past this
    /// angle the single-track force directions are wrong and the state
    /// cannot describe a car facing back down the road. What a spin costs
    /// is seconds and places, and seconds and places are exactly what this
    /// spends.
    /// </summary>
    private static void StepScriptedSpin(
        CarState state,
        CarConfig config,
        TireSet tires,
        CarPhysicsStepInput input,
        float dt
    )
    {
        state.SpinSeconds += dt;

        // Down to the speed it is handed back at and no further. A spin
        // that parked the car would retire it, and a spin is meant to cost
        // a driver the race, not end it.
        float oldSpeed = state.Speed;
        float floorSpeed = MathF.Min(
            oldSpeed,
            SingleTrackDynamicsLimits.SpinReleaseSpeedMetersPerSecond
        );
        float newSpeed = MathF.Max(
            floorSpeed,
            oldSpeed -
            SingleTrackDynamicsLimits
                .SpinScrubDecelerationMetersPerSecondSquared * dt
        );
        float averageSpeed = (oldSpeed + newSpeed) * 0.5f;

        // Where the car is going bends towards whichever side of it can
        // still find grip, which on a circuit means back towards the road.
        //
        // It used to travel in a dead straight line while it rotated, and
        // that is how a spun car ends up stopped square in the middle of a
        // run-off with nothing to do but retire. A real spin does not do
        // that: the pair of tyres still on tarmac drags harder than the
        // pair on grass, the car pivots and slides towards the harder
        // pair, and drivers come back out of a spin near the edge of the
        // road far more often than in the middle of the field.
        //
        // The car does not have to know where the track is to do this --
        // it can feel it. Nothing here reads a circuit; it reads the four
        // grips it is already given, which is what a tyre knows.
        float leftGrip = input.SurfaceGrip[WheelId.FrontLeft] +
                         input.SurfaceGrip[WheelId.RearLeft];
        float rightGrip = input.SurfaceGrip[WheelId.FrontRight] +
                          input.SurfaceGrip[WheelId.RearRight];
        float gripSum = leftGrip + rightGrip;
        float bodyAsymmetry = gripSum > Epsilon
            ? (leftGrip - rightGrip) / gripSum
            : 0f;
        // The asymmetry is measured across the car, and the car is not
        // pointing where it is going. Only the part of it that lies across
        // the direction of travel can bend the path.
        float travelAsymmetry = bodyAsymmetry *
                                MathF.Cos(state.SideslipAngleRadians);
        // Bent here and carried through: the sideslip written back at the
        // end of the step is measured against this, so the new direction
        // of travel is what the car leaves the step with.
        float velocityHeading = MathHelper.NormalizeAngle(
            state.VelocityHeading +
            SingleTrackDynamicsLimits.SpinRecoveryBendRateRadiansPerSecond *
            travelAsymmetry * dt
        );
        Vector2 travelDirection = new(
            MathF.Cos(velocityHeading),
            MathF.Sin(velocityHeading)
        );
        state.Position += travelDirection * averageSpeed * dt;

        bool gathering = newSpeed <= floorSpeed + Epsilon;
        float previousYawRate = state.YawRateRadiansPerSecond;
        float yawRate;
        float heading;
        if (gathering)
        {
            float weight = 1f - MathF.Exp(
                -dt / MathF.Max(
                    SingleTrackDynamicsLimits.SpinGatherTimeSeconds,
                    Epsilon
                )
            );
            heading = LerpAngle(state.Heading, velocityHeading, weight);
            yawRate = MathHelper.NormalizeAngle(heading - state.Heading) / dt;
        }
        else
        {
            yawRate = previousYawRate * MathF.Exp(
                -dt / MathF.Max(
                    SingleTrackDynamicsLimits.SpinYawDecayTimeSeconds,
                    Epsilon
                )
            );
            heading = MathHelper.NormalizeAngle(
                state.Heading + (previousYawRate + yawRate) * 0.5f * dt
            );
        }

        state.Heading = heading;
        state.YawRateRadiansPerSecond = yawRate;
        state.Speed = newSpeed;
        state.SideslipAngleRadians = MathHelper.NormalizeAngle(
            velocityHeading - heading
        );

        // Nobody is steering. The wheels wander onto the direction of
        // travel at the speed they can move, which on a car this far
        // sideways means full opposite lock - and that is what a spinning
        // car looks like from the outside.
        float maximumSteer = MathF.Max(config.MaxSteerAngleRadians, 0f);
        float steerTarget = Math.Clamp(
            state.SideslipAngleRadians,
            -maximumSteer,
            maximumSteer
        );
        float steerStep = MathF.Max(config.SteerRateLimitRadiansPerSecond, 0f) * dt;
        state.SteerAngleRadians += Math.Clamp(
            steerTarget - state.SteerAngleRadians,
            -steerStep,
            steerStep
        );

        float massKg = TotalMassKg(config, state.Energy);
        float roadNormalGravity = input.RoadAttitude.NormalGravity(
            Gravity,
            averageSpeed,
            0f
        );
        ApplyWheelLoads(
            state,
            CalculateWheelLoads(state, config, massKg, roadNormalGravity)
        );

        // The tyres are charged for the spin, on the same terms as any
        // other sliding: force times how fast the rubber is being dragged
        // across the road.
        //
        // They used to be charged nothing, on the argument that the seconds
        // were the bill and a set of flat spots would be charging the same
        // mistake twice. That was wrong, and wrong in the way that matters:
        // a spin that costs no rubber is a free reset button, both false to
        // the thing being modelled and available to be leant on. The force
        // is what the road is taking off the car and the dragging is total,
        // so the scrub weight sits at its ceiling; flat spots as a thing
        // with a shape of their own belong to a damage model that does not
        // exist yet, and one lump charge stands in until it does.
        float coolingAirSpeed = averageSpeed *
                                (1f - Math.Clamp(state.AirVelocityDeficit, 0f, 1f));
        float tireWakeDownforceLoss =
            EffectiveTireWakeDownforceLoss(state, config);
        float frontSpinGrip = CalculateAxleGripAccel(
            massKg, config, tires, state.FrontLeft, state.FrontRight, front: true
        );
        float rearSpinGrip = CalculateAxleGripAccel(
            massKg, config, tires, state.RearLeft, state.RearRight, front: false
        );
        foreach (WheelId wheel in Wheels)
        {
            bool front = wheel is WheelId.FrontLeft or WheelId.FrontRight;
            float axleGrip = front ? frontSpinGrip : rearSpinGrip;
            float scrubForce = MathF.Min(
                1f,
                SingleTrackDynamicsLimits
                    .SpinScrubDecelerationMetersPerSecondSquared /
                MathF.Max(axleGrip, Epsilon)
            );
            // A spin is billed by the same rules as everything else. It
            // used to have its own flat rate -- the scrub weight pinned at
            // its cap and a sideslip channel switched fully on -- which
            // made a spin cost a fixed amount no matter how sideways or
            // how fast it was. The choreographed trajectory has a real
            // sideslip angle and a real speed, so the ordinary formula
            // applies to it: force times the square root of the slip for
            // wear, force times the sliding speed for heat. What the flat
            // rate used to charge is now something the physics arrives at,
            // and the two landing in the same place is the check.
            float spinSlip = state.SideslipAngleRadians;
            UpdateTires(
                state.GetTire(wheel),
                front,
                config,
                tires,
                scrubForce * ScrubWeight(spinSlip, 1f),
                scrubForce * SlipSpeedShare(spinSlip, 1f),
                0f,
                0f,
                1f,
                1f,
                1f,
                0f,
                input.AirTempC,
                input.TrackTempC,
                averageSpeed,
                coolingAirSpeed,
                tireWakeDownforceLoss,
                dt
            );
        }

        PowertrainSettlement settlement = config.Powertrain.Settle(
            state.Energy,
            0f,
            0f,
            averageSpeed,
            massKg,
            dt
        );
        state.Energy = settlement.Energy;

        float actualLongitudinalAccel = (newSpeed - oldSpeed) / dt;
        float response = 1f - MathF.Exp(-config.LoadTransferResponse * dt);
        state.FilteredLongitudinalAccel = Lerp(
            state.FilteredLongitudinalAccel,
            actualLongitudinalAccel,
            response
        );
        state.FilteredLateralAccel = Lerp(state.FilteredLateralAccel, 0f, response);

        state.Telemetry = new CarTelemetry(
            input.DriverInput,
            input.Strategy,
            0f,
            0f,
            0f,
            actualLongitudinalAccel,
            0f,
            0f,
            0f,
            settlement.DrawnPowerWatts,
            0f,
            state.SideslipAngleRadians,
            0f,
            yawRate,
            (yawRate - previousYawRate) / dt
        );
        SingleTrackState.Of(state).Axles = new AxleTelemetry(
            CalculateAxleGripAccel(massKg, config, tires, state.FrontLeft, state.FrontRight, front: true),
            CalculateAxleGripAccel(massKg, config, tires, state.RearLeft, state.RearRight, front: false),
            0f, 0f, 0f, 0f, 0f, 0f,
            1f
        );

        if (gathering &&
            MathF.Abs(state.SideslipAngleRadians) <=
            SingleTrackDynamicsLimits.SpinReleaseSideslipRadians)
        {
            state.Spinning = false;
            state.SpinSeconds = 0f;
            state.SideslipHoldSeconds = 0f;
        }

        state.Normalize();
    }

    /// <summary>
    /// How a corner's lateral demand actually falls on the two axles.
    ///
    /// The share is the single-track model's own yaw-moment balance -
    /// l_f * F_f = l_r * F_r - and not the static weight distribution. On
    /// this chassis the two numbers coincide, because the moment arms are
    /// built from the load share; they would not on a car whose weight and
    /// wheelbase were quoted independently, and reading the balance is the
    /// answer the physics would give either way.
    ///
    /// The clamp is the part that matters. An axle cannot carry more than
    /// it has, so a corner past what one end will hold does not leave that
    /// end with imaginary circle to brake on - which is exactly what a
    /// static apportionment told anyone who asked, and what let a plan
    /// arrive at an apex having budgeted for grip that was never there.
    /// </summary>
    private static (float Front, float Rear) AxleLateralDemand(
        CarConfig config,
        float lateralAcceleration,
        float frontGrip,
        float rearGrip
    )
    {
        float wheelBase = MathF.Max(config.WheelBaseMeters, Epsilon);
        float frontShare = Math.Clamp(RearMomentArm(config) / wheelBase, 0f, 1f);
        float usableFront = frontGrip * TireSlipCurve.SustainablePeakShare;
        float usableRear = rearGrip * TireSlipCurve.SustainablePeakShare;
        float magnitude = MathF.Abs(lateralAcceleration);
        return (
            MathF.Min(magnitude * frontShare, usableFront),
            MathF.Min(magnitude * (1f - frontShare), usableRear)
        );
    }

    /// <summary>
    /// The most lateral acceleration this pair of axles will hold, which is
    /// whichever of them runs out first at the balance above.
    /// </summary>
    private static float AxleLateralCeiling(
        CarConfig config,
        float frontGrip,
        float rearGrip
    )
    {
        float wheelBase = MathF.Max(config.WheelBaseMeters, Epsilon);
        float frontShare = Math.Clamp(RearMomentArm(config) / wheelBase, 0f, 1f);
        float rearShare = 1f - frontShare;
        float front = frontShare <= Epsilon
            ? float.PositiveInfinity
            : frontGrip / frontShare;
        float rear = rearShare <= Epsilon
            ? float.PositiveInfinity
            : rearGrip / rearShare;
        return MathF.Min(front, rear) * TireSlipCurve.SustainablePeakShare;
    }

    /// <summary>
    /// Distance from the centre of mass to each axle. The front carries
    /// less of the weight, so the centre of mass sits nearer the rear and
    /// the front arm is the longer one.
    /// </summary>
    private static float RearMomentArm(CarConfig config)
    {
        return MathF.Max(config.RearAxleOffsetMeters, Epsilon);
    }

    private static float FrontMomentArm(CarConfig config)
    {
        return MathF.Max(config.FrontAxleOffsetMeters, Epsilon);
    }

    /// <summary>
    /// The angle between where a front wheel points and where it is going.
    ///
    /// Everything the car does laterally comes from this and its twin at
    /// the rear. Turn the wheels further than the tyre can use and the
    /// front runs past its peak and the car goes wide; get the rear's angle
    /// past its peak and the tail keeps going.
    /// </summary>
    private static float FrontSlipAngle(
        CarConfig config,
        float steerAngle,
        float sideslip,
        float yawRate,
        float speed
    )
    {
        return steerAngle - sideslip -
               FrontMomentArm(config) * yawRate /
               MathF.Max(speed, MinimumSlipAngleSpeed);
    }

    private static float RearSlipAngle(
        CarConfig config,
        float sideslip,
        float yawRate,
        float speed
    )
    {
        return -sideslip +
               RearMomentArm(config) * yawRate /
               MathF.Max(speed, MinimumSlipAngleSpeed);
    }

    /// <summary>
    /// Where the front wheels end up this step.
    ///
    /// DriverInput carries curvature rather than a wheel angle. Geometry gives
    /// the actuator's starting angle, while the physical slip-angle estimate
    /// and curvature feedback close the tracking error.
    ///
    /// Then the wheels have to get there, and they can only move so fast.
    /// That rate limit is the whole reason this is a state: a slide is
    /// caught by lock that arrives in time and not caught by the same lock
    /// arriving late, and a policy deciding fifteen times a second now has
    /// to live with the difference.
    /// </summary>
    private static float UpdateSteerAngle(
        CarState state,
        CarConfig config,
        float desiredCurvature,
        float requestedLateralAccel,
        float frontGrip,
        float rearGrip,
        float dt
    )
    {
        float wheelBase = MathF.Max(config.WheelBaseMeters, Epsilon);
        float maximum = MathF.Max(config.MaxSteerAngleRadians, 0f);
        float frontArm = FrontMomentArm(config);
        float rearArm = RearMomentArm(config);

        // Geometry is the start of the answer and not the whole of it. The
        // wheels have to be turned further than the corner's own angle by
        // the difference between the two ends' slip angles. Past the peak the
        // inverse runs
        // out, which is the point at which no amount of lock buys any more
        // corner - understeer, arrived at honestly.
        float frontShare = rearArm / wheelBase * requestedLateralAccel;
        float rearShare = frontArm / wheelBase * requestedLateralAccel;
        float frontCapacity = MathF.Max(frontGrip, Epsilon);
        float rearCapacity = MathF.Max(rearGrip, Epsilon);
        float slipCompensation =
            TireSlipCurve.InverseEvaluate(
                frontShare / frontCapacity,
                config.FrontPeakSlipAngleRatio
            ) -
            TireSlipCurve.InverseEvaluate(rearShare / rearCapacity);

        // Faded in with the load, because the inversion above is a steady
        // state and the car is not in one yet.
        //
        // Turning into a corner from a straight line, none of those slip
        // angles exist: the tyres are pointed where the car is going, and
        // the extra lock they will want is a thing to add as they take up
        // load. Applied in full from the first frame it is worse than useless
        // - a car with a worn rear
        // under fresh fronts wants a smaller angle than geometry, and on a
        // strong enough asymmetry it wants a negative one, so the
        // arithmetic asks for opposite lock before any slide exists and
        // then makes one the other way.
        // Measured against what the tyres can give rather than against what
        // was asked for. Against the request it would never quite reach one
        // - at the limit the car is always a little short of what it wanted
        // - and the compensation would be permanently under-applied, which
        // costs real cornering. Against the car's own capability it is one
        // through any corner worth the name and zero only on the straight.
        float loadedShare = FullSlipCompensationShare *
                            MathF.Max(frontGrip + rearGrip, Epsilon);
        float lateralLoad = Math.Clamp(
            MathF.Abs(state.Telemetry.ActualLateralAccel) / loadedShare,
            0f,
            1f
        );
        float kinematic = MathF.Atan(wheelBase * desiredCurvature);
        float feedforward = kinematic + slipCompensation * lateralLoad;
        float feedback = config.SteerCurvatureFeedbackGain * wheelBase *
                         (desiredCurvature - state.Telemetry.ActualCurvature);
        float target = Math.Clamp(feedforward + feedback, -maximum, maximum);

        float step = MathF.Max(config.SteerRateLimitRadiansPerSecond, 0f) * dt;
        float change = Math.Clamp(target - state.SteerAngleRadians, -step, step);
        return Math.Clamp(state.SteerAngleRadians + change, -maximum, maximum);
    }

    /// <summary>
    /// One axle, at the angle it is at, with whatever the brakes or the
    /// motor have left it.
    ///
    /// Longitudinal first, because that is commanded directly: a locked
    /// wheel steers nothing, and the combined-grip limiter exists precisely
    /// to keep braking within the authorised share of the circle. What remains
    /// of the circle is what the slip angle gets to work with.
    ///
    /// The tyre is charged for what it was worked at and the car receives
    /// the force left by the tire model: lateral slip still spends rubber
    /// even when it produces less useful force.
    /// </summary>
    private static AxleResult ResolveAxleSlip(
        CarConfig config,
        float grip,
        float slipAngle,
        float peakScale,
        float longitudinalRequest,
        float gripAllowance,
        float speed
    )
    {
        if (grip <= Epsilon)
            return default;

        float unlimitedRequest = longitudinalRequest;
        longitudinalRequest = CombinedGripLimitedLongitudinal(
            config,
            longitudinalRequest,
            grip,
            slipAngle,
            peakScale,
            gripAllowance,
            speed
        );
        float limiterCut =
            MathF.Abs(unlimitedRequest) - MathF.Abs(longitudinalRequest);
        float longitudinalDemand = MathF.Abs(longitudinalRequest) / grip;
        float longitudinalUse = MathF.Min(longitudinalDemand, 1f);
        float longitudinalEfficiency = OverLimitGripEfficiency(
            config,
            MathF.Max(0f, longitudinalDemand - 1f)
        );
        float longitudinalAccel =
            Math.Clamp(longitudinalRequest, -grip, grip) * longitudinalEfficiency;
        float circle = MathF.Sqrt(
            MathF.Max(0f, 1f - longitudinalUse * longitudinalUse)
        );

        float shape = TireSlipCurve.Evaluate(slipAngle, peakScale);
        float lateralUse = circle * MathF.Abs(shape);
        float lateralAccel = grip * circle * shape;

        // Two ways to be past what the axle has, and they mean different
        // things: brakes asked for more than the tyre can hold, and rubber
        // dragged past the angle where it stops paying. Both are scrub, so
        // both are charged here.
        float overLimit = MathF.Max(0f, longitudinalDemand - 1f) +
                          TireSlipCurve.PastPeak(slipAngle, peakScale);
        float combinedRequest = MathF.Sqrt(
            lateralUse * lateralUse +
            longitudinalDemand * longitudinalDemand
        );
        return new AxleResult(
            lateralAccel,
            longitudinalAccel,
            overLimit,
            combinedRequest,
            lateralUse,
            longitudinalUse,
            limiterCut
        );
    }

    /// <summary>
    /// How many pieces the sideslip and yaw pair has to be advanced in to
    /// stay stable, worked out from the stiffness actually present rather
    /// than from a speed threshold - so a change to the tyre curve cannot
    /// silently outrun the clock.
    ///
    /// Capped, because below the speed where the cap would not be enough
    /// the kinematic blend has taken the lateral model over anyway.
    /// </summary>
    private static int LateralSubstepCount(
        CarConfig config,
        float massKg,
        float frontGrip,
        float rearGrip,
        float speed,
        float dt
    )
    {
        float velocity = MathF.Max(speed, MinimumSlipAngleSpeed);
        float stiffness = TireSlipCurve.NormalizedCorneringStiffness;
        float front = massKg * frontGrip * stiffness;
        float rear = massKg * rearGrip * stiffness;
        float frontArm = FrontMomentArm(config);
        float rearArm = RearMomentArm(config);
        float yawDamping =
            (frontArm * frontArm * front + rearArm * rearArm * rear) / velocity;
        float sideslipDamping = (front + rear) / velocity;
        float yawTime = MathF.Max(config.YawInertiaKgM2, Epsilon) /
                        MathF.Max(yawDamping, Epsilon);
        float sideslipTime = MathF.Max(massKg, Epsilon) /
                             MathF.Max(sideslipDamping, Epsilon);
        float allowed = SingleTrackDynamicsLimits.LateralSubstepSafetyFactor *
                        MathF.Min(yawTime, sideslipTime);
        if (allowed <= Epsilon)
            return SingleTrackDynamicsLimits.MaximumLateralSubsteps;

        return Math.Clamp(
            (int)MathF.Ceiling(dt / allowed),
            1,
            SingleTrackDynamicsLimits.MaximumLateralSubsteps
        );
    }

    /// <summary>
    /// Advance the two states the tyres own - which way the car is pointing
    /// relative to where it is going, and how fast that is changing - and
    /// report what the axles did on the way.
    ///
    /// Below DynamicYawMinimumSpeed the slip angles are meaningless, so the
    /// kinematic blend takes over: the body turns as its wheels point and
    /// carries no sideslip. Above it the car is free to be out of shape,
    /// and there is no clamp anywhere that says how far.
    /// </summary>
    private static LateralIntegration IntegrateLateral(
        CarState state,
        CarConfig config,
        float massKg,
        float frontGrip,
        float rearGrip,
        float frontLongRequest,
        float rearLongRequest,
        float steerAngle,
        float roadLateralDemand,
        float curvatureDemandScale,
        float dynamicYawBlend,
        float gripAllowance,
        float dt
    )
    {
        float speed = state.Speed;
        float frontPeakScale = MathF.Max(config.FrontPeakSlipAngleRatio, 0.05f);
        int substeps = LateralSubstepCount(
            config, massKg, frontGrip, rearGrip, speed, dt
        );
        float h = dt / substeps;
        float demandScale = MathF.Max(curvatureDemandScale, 1e-3f);
        float sideslip = state.SideslipAngleRadians;
        float yawRate = state.YawRateRadiansPerSecond;
        float headingDelta = 0f;

        float frontLateral = 0f, rearLateral = 0f;
        float frontUse = 0f, rearUse = 0f;
        float frontOver = 0f, rearOver = 0f;
        float frontCombined = 0f, rearCombined = 0f;
        float pathLateral = 0f, yawAcceleration = 0f;
        float frontSlip = 0f, rearSlip = 0f;
        AxleResult front = default;
        AxleResult rear = default;

        for (int i = 0; i < substeps; i++)
        {
            // Below DynamicYawMinimumSpeed a slip angle means nothing (it is
            // read off a floored speed) and the body turns as its wheels
            // point, so it fades out with the dynamic model. Read anyway, a
            // car crawling round in a circle measured its rear at the peak
            // and the grip limiter took all of its drive away.
            frontSlip = dynamicYawBlend * FrontSlipAngle(
                config, steerAngle, sideslip, yawRate, speed
            );
            rearSlip = dynamicYawBlend * RearSlipAngle(config, sideslip, yawRate, speed);
            front = ResolveAxleSlip(
                config,
                frontGrip,
                frontSlip,
                frontPeakScale,
                frontLongRequest,
                gripAllowance,
                speed
            );
            rear = ResolveAxleSlip(
                config,
                rearGrip,
                rearSlip,
                1f,
                rearLongRequest,
                gripAllowance,
                speed
            );

            float tyreLateral = front.LateralAccel + rear.LateralAccel;
            float pathStep = (tyreLateral - roadLateralDemand) / demandScale;
            float yawStep = CalculateYawAcceleration(
                config, massKg, front.LateralAccel, rear.LateralAccel
            );

            // The velocity vector turns at the rate the path curves; the
            // body turns at the yaw rate. Sideslip is the gap between the
            // two, and it grows whenever the body is turning faster than
            // the car is actually going round.
            float velocityYawRate = speed > 0.5f ? pathStep / speed : 0f;
            // Kinematic, the body turns as the wheels point. Not from the
            // tyre force over the speed: the force is read off a slip angle
            // floored at MinimumSlipAngleSpeed, so it does not shrink as the
            // car slows, and divided by a speed that does it turned a
            // degree of lock at walking pace into the body spinning round,
            // which the driver then fought to full lock off the line.
            float kinematicYawRate = speed *
                MathF.Tan(steerAngle) / MathF.Max(config.WheelBaseMeters, Epsilon);
            float dynamicYawRate = Math.Clamp(
                yawRate + yawStep * h,
                -SingleTrackDynamicsLimits.MaximumYawRateRadiansPerSecond,
                SingleTrackDynamicsLimits.MaximumYawRateRadiansPerSecond
            );
            float dynamicSideslip = sideslip + (velocityYawRate - yawRate) * h;
            float nextYawRate = Lerp(
                kinematicYawRate, dynamicYawRate, dynamicYawBlend
            );
            pathStep = Lerp(speed * kinematicYawRate, pathStep, dynamicYawBlend);
            float nextSideslip = dynamicSideslip * dynamicYawBlend;

            headingDelta += (yawRate + nextYawRate) * 0.5f * h;
            yawRate = nextYawRate;
            sideslip = MathHelper.NormalizeAngle(nextSideslip);

            frontLateral += front.LateralAccel;
            rearLateral += rear.LateralAccel;
            frontUse += front.LateralUse;
            rearUse += rear.LateralUse;
            frontOver += front.OverLimit;
            rearOver += rear.OverLimit;
            frontCombined += front.CombinedRequest;
            rearCombined += rear.CombinedRequest;
            pathLateral += pathStep;
            yawAcceleration += yawStep;
        }

        float share = 1f / substeps;
        return new LateralIntegration(
            new AxleResult(
                frontLateral * share,
                front.LongitudinalAccel,
                frontOver * share,
                frontCombined * share,
                frontUse * share,
                front.LongitudinalUse,
                front.LimiterCut
            ),
            new AxleResult(
                rearLateral * share,
                rear.LongitudinalAccel,
                rearOver * share,
                rearCombined * share,
                rearUse * share,
                rear.LongitudinalUse,
                rear.LimiterCut
            ),
            pathLateral * share,
            yawAcceleration * share,
            sideslip,
            yawRate,
            headingDelta,
            frontSlip,
            rearSlip,
            substeps
        );
    }

    /// <summary>
    /// How much rubber an axle is spending per unit of force it delivers.
    ///
    /// One up to the peak, and from there it climbs with the slip angle:
    /// the tyre is being dragged across the road faster and faster for a
    /// force that has stopped growing, which is what a scrubbed set of
    /// fronts at the end of an understeering stint is. Capped so that a
    /// spin bills a set of tyres for a spin and not for the race.
    /// </summary>

    /// <summary>
    /// How fast the contact patch is being dragged across the road, as a
    /// share of what it does at the peak.
    ///
    /// The lateral sliding speed is <c>v sin(alpha)</c>, so this is
    /// <c>sin(alpha)</c> normalised by its value at the tyre's own peak
    /// slip angle -- the speed factor the heat term is multiplied by
    /// carries the <c>v</c>. Sine rather than the angle because a car at
    /// sixty degrees of slip is not dragging four times as fast as one at
    /// fifteen; below the peak the two agree to the first order and the
    /// calibration is unaffected either way.
    /// </summary>
    private static float SlipSpeedShare(float slipAngle, float peakScale)
    {
        float peak = TireSlipCurve.PeakSlipAngleRadians *
                     MathF.Max(peakScale, Epsilon);
        return MathF.Abs(MathF.Sin(slipAngle)) /
               MathF.Max(MathF.Sin(peak), Epsilon);
    }

    private static float ScrubWeight(float slipAngle, float peakScale)
    {
        float peak = TireSlipCurve.PeakSlipAngleRadians *
                     MathF.Max(peakScale, 0.05f);
        // The square root is not decoration. Wear and heat go as the square
        // of this figure downstream, and what a tyre spends is force times
        // sliding speed - one power of each. Handed the slip ratio whole it
        // would come out squared as well, and a car fifteen percent over
        // the limit would destroy a set of tyres in a minute rather than
        // ruin them over a stint.
        return Math.Clamp(
            MathF.Sqrt(MathF.Abs(slipAngle) / MathF.Max(peak, Epsilon)),
            1f,
            MaximumScrubWeight
        );
    }

    /// <summary>
    /// How much more than its force share a sliding tyre may be charged,
    /// as a multiplier before the squaring downstream. Two is a tyre being
    /// dragged at four times the angle it pays best at - well past anything
    /// held on purpose - and the ceiling is there so a spin bills a set of
    /// tyres for a spin rather than for the race.
    /// </summary>
    private const float MaximumScrubWeight = 2f;

    /// <summary>
    /// How much more the rear is letting go than the front.
    ///
    /// With real slip angles this is one subtraction. The axle further past
    /// its peak is the axle that is going; if that is the rear, the car is
    /// oversteering.
    /// </summary>
    private static float CalculateRearSlideSeverity(
        CarConfig config,
        float frontSlipAngle,
        float rearSlipAngle
    )
    {
        // Compared uncapped and clamped afterwards. Capping each side first
        // would make this read zero in the middle of the worst slide there
        // is, which is the one moment it exists to report.
        return Math.Clamp(
            TireSlipCurve.UncappedPastPeak(rearSlipAngle) -
            TireSlipCurve.UncappedPastPeak(
                frontSlipAngle,
                config.FrontPeakSlipAngleRatio
            ),
            0f,
            1f
        );
    }

    private static float RemainingLongitudinalGrip(float grip, float lateralAcceleration)
    {
        float remainingSquared = grip * grip - lateralAcceleration * lateralAcceleration;
        return MathF.Sqrt(Math.Max(0f, remainingSquared));
    }

    /// <summary>
    /// The combined-grip limiter's longitudinal ceiling for one axle; see
    /// <see cref="CarConfig.CombinedGripLimiterStrength"/> for the device.
    ///
    /// The arithmetic is one square root of quantities the step already
    /// has. The axle spends s of its circle on the slip angle it is at and
    /// u of it on the pedals, and because the lateral capacity left is the
    /// remainder of the circle, what it ends up using is
    ///
    ///     use^2 = s^2 + u^2 (1 - s^2)
    ///
    /// so use stays inside an authorisation A exactly when
    ///
    ///     u^2 &lt;= (A^2 - s^2) / (1 - s^2)
    ///
    /// and past a slip angle whose own share is already A, no pedal
    /// position satisfies it: the tyre is over the authorisation on steering
    /// alone. The limiter cuts the pedals to nothing there, which is all it
    /// can do; the excess that remains is the steering's, and the device
    /// leaves steering alone.
    ///
    /// Past the peak the share falls as the slide deepens, and a ceiling
    /// read off it would rise with it -- at 1.8 times the peak angle it
    /// would hand back three quarters of the throttle. An axle on the far
    /// side of its peak therefore gets no drive, above the configured speed
    /// only (the slip angle below it is read off a floored speed) and never
    /// on the brakes. There is no step at the peak: wherever the limiter
    /// trims at all, the share reaches A before the peak and the ceiling is
    /// already zero there.
    /// </summary>
    internal static float CombinedGripLimitedLongitudinal(
        CarConfig config,
        float longitudinalRequest,
        float grip,
        float slipAngle,
        float peakScale,
        float gripAllowance,
        float speed
    )
    {
        float strength = Math.Clamp(config.CombinedGripLimiterStrength, 0f, 1f);
        float allowance = Math.Clamp(gripAllowance, 0f, 1f);
        if (strength <= 0f || allowance >= 1f || grip <= Epsilon)
            return longitudinalRequest;

        return Lerp(
            longitudinalRequest,
            FullyLimitedLongitudinal(
                longitudinalRequest,
                grip,
                slipAngle,
                peakScale,
                allowance,
                speed,
                config.CombinedGripLimiterPastPeakMinimumSpeed
            ),
            strength
        );
    }

    private static float FullyLimitedLongitudinal(
        float longitudinalRequest,
        float grip,
        float slipAngle,
        float peakScale,
        float allowance,
        float speed,
        float pastPeakMinimumSpeed
    )
    {
        float peak = TireSlipCurve.PeakSlipAngleRadians *
                     MathF.Max(peakScale, 0.05f);
        if (longitudinalRequest > 0f &&
            speed > pastPeakMinimumSpeed &&
            MathF.Abs(slipAngle) >= peak)
            return 0f;

        float lateralShare = MathF.Min(
            1f,
            MathF.Abs(TireSlipCurve.Evaluate(slipAngle, peakScale))
        );
        float headroom = 1f - lateralShare * lateralShare;
        if (headroom <= Epsilon)
            return 0f;

        float share = (allowance * allowance - lateralShare * lateralShare) /
                      headroom;
        if (share <= 0f)
            return 0f;

        float ceiling = MathF.Sqrt(share) * grip;
        return Math.Clamp(longitudinalRequest, -ceiling, ceiling);
    }

    private static float DistributedLongitudinalLimit(
        float frontCapacity,
        float rearCapacity,
        float frontShare
    )
    {
        float normalizedFrontShare = Math.Clamp(frontShare, 0f, 1f);
        float rearShare = 1f - normalizedFrontShare;
        float limit = float.PositiveInfinity;

        if (normalizedFrontShare > Epsilon)
            limit = Math.Min(limit, frontCapacity / normalizedFrontShare);
        if (rearShare > Epsilon)
            limit = Math.Min(limit, rearCapacity / rearShare);

        return float.IsFinite(limit) ? Math.Max(0f, limit) : 0f;
    }

    /// <summary>
    /// The most drive the two axles can put down between them: each the grip
    /// it has left after its corner, and no more than its own motor's share
    /// of what the powertrain can give.
    /// </summary>
    private static float SharedDriveLimit(
        float frontCapacity,
        float rearCapacity,
        float powertrainLimit,
        float frontMotorShare
    )
    {
        float share = Math.Clamp(frontMotorShare, 0f, 1f);
        return MathF.Min(frontCapacity, powertrainLimit * share) +
               MathF.Min(rearCapacity, powertrainLimit * (1f - share));
    }

    /// <summary>
    /// Drive shared between the axles as the brakes are: by the grip each
    /// has left after the corner it is carrying, so an axle busy turning the
    /// car is asked for little and one with grip to spare is asked for more.
    /// Each axle is held to its own motor's share of the drive available; a
    /// request one cannot take goes to the other. A rear-driven car (front
    /// share zero) is all rear, exactly as before. The driver's offset moves
    /// the split from the one the grip gives, as the brake bias offset does.
    /// </summary>
    private static void AllocateDriveRequest(
        float driveAccel,
        float available,
        float frontLateralRequest,
        float rearLateralRequest,
        float frontGrip,
        float rearGrip,
        float frontMotorShare,
        float frontDriveBiasOffset,
        out float frontDrive,
        out float rearDrive
    )
    {
        float motorShare = Math.Clamp(frontMotorShare, 0f, 1f);
        float frontMotor = available * motorShare;
        float rearMotor = available * (1f - motorShare);
        float frontCapacity = RemainingLongitudinalGrip(frontGrip, frontLateralRequest);
        float rearCapacity = RemainingLongitudinalGrip(rearGrip, rearLateralRequest);
        float total = frontCapacity + rearCapacity;
        float share = total > Epsilon ? frontCapacity / total : motorShare;
        share = Math.Clamp(share + frontDriveBiasOffset, 0f, 1f);

        frontDrive = MathF.Min(driveAccel * share, frontMotor);
        rearDrive = driveAccel - frontDrive;
        if (rearDrive > rearMotor)
        {
            rearDrive = rearMotor;
            frontDrive = MathF.Min(driveAccel - rearDrive, frontMotor);
        }
    }

    private static void AllocateBrakeRequest(
        float brakeRequest,
        float frontLateralRequest,
        float rearLateralRequest,
        float frontGrip,
        float rearGrip,
        float frontBrakeBiasOffset,
        out float frontBrake,
        out float rearBrake
    )
    {
        float frontCapacity = RemainingLongitudinalGrip(
            frontGrip,
            frontLateralRequest
        );
        float rearCapacity = RemainingLongitudinalGrip(
            rearGrip,
            rearLateralRequest
        );
        float frontShare = BiasedFrontBrakeShare(
            frontCapacity,
            rearCapacity,
            frontBrakeBiasOffset
        );
        if (frontCapacity + rearCapacity <= Epsilon)
        {
            float totalGrip = frontGrip + rearGrip;
            if (totalGrip <= Epsilon)
            {
                frontBrake = 0f;
                rearBrake = 0f;
                return;
            }

            // Both axles are already at their lateral limit. Preserve the
            // requested combined-demand behavior and let ResolveAxle clip it.
            frontShare = BiasedFrontBrakeShare(
                frontGrip,
                rearGrip,
                frontBrakeBiasOffset
            );
            frontBrake = brakeRequest * frontShare;
            rearBrake = brakeRequest - frontBrake;
            return;
        }

        frontBrake = brakeRequest * frontShare;
        rearBrake = brakeRequest - frontBrake;
    }

    private static float BiasedFrontBrakeShare(
        float frontCapacity,
        float rearCapacity,
        float frontBrakeBiasOffset
    )
    {
        float totalCapacity = frontCapacity + rearCapacity;
        float optimalFrontShare = totalCapacity <= Epsilon
            ? 0.5f
            : frontCapacity / totalCapacity;
        float finiteOffset = float.IsFinite(frontBrakeBiasOffset)
            ? Math.Clamp(frontBrakeBiasOffset, -0.25f, 0.25f)
            : 0f;
        return Math.Clamp(optimalFrontShare + finiteOffset, 0f, 1f);
    }

    /// <summary>
    /// The yaw moment produced by the two physical axle forces.
    /// </summary>
    private static float CalculateYawAcceleration(
        CarConfig config,
        float massKg,
        float frontLateralAcceleration,
        float rearLateralAcceleration
    )
    {
        float wheelBase = Math.Max(config.WheelBaseMeters, Epsilon);
        float rearMomentArm = wheelBase * Math.Clamp(
            config.FrontStaticLoadShare,
            0f,
            1f
        );
        float frontMomentArm = wheelBase - rearMomentArm;
        float yawMoment = massKg * (
            frontMomentArm * frontLateralAcceleration -
            rearMomentArm * rearLateralAcceleration
        );
        return yawMoment / Math.Max(config.YawInertiaKgM2, Epsilon);
    }

    private static float CalculateDynamicYawBlend(float speed)
    {
        float t = Math.Clamp(
            (speed - DynamicYawMinimumSpeed) / DynamicYawBlendRange,
            0f,
            1f
        );
        return t * t * (3f - 2f * t);
    }

    /// <summary>
    /// What the car gives back to the air, the road and its own tyres.
    ///
    /// Air resistance is fought against the air the car is moving through and
    /// not against the ground, so a car whose air is already being dragged
    /// along by somebody in front meets less wind and pays the square of what
    /// is left. Writing it as a share of drag removed instead would be writing
    /// down the answer; written this way the squaring is the reason a tow is
    /// worth so much more at a car length than at ten.
    ///
    /// Only that part is reduced. Rolling resistance does not care what is in
    /// front. What cornering costs is not here at all: see
    /// <see cref="InducedDragAccel"/>, which is the tyres' business and not
    /// the air's.
    /// </summary>
    private static float CalculateLossAccel(
        CarConfig config,
        float speed,
        float airVelocityDeficit,
        float massKg
    )
    {
        if (speed <= 0.01f)
            return 0f;

        return
            config.RollingDragAccel +
            config.Aero.DragForce(speed, new AeroAir(airVelocityDeficit, 0f, 0f, 0f)) /
            Math.Max(massKg, Epsilon);
    }

    /// <summary>
    /// What cornering costs in speed, from where it actually comes from.
    ///
    /// A tyre at a slip angle makes its force perpendicular to the wheel,
    /// not to the car's travel. The component of that force pointing back
    /// down the road is drag, and it is the whole of the cornering loss:
    /// <c>sum over axles of |F_lat| * sin(|alpha|)</c>, with nothing to
    /// calibrate — no coefficient, no scale, no rate. Turning is slow
    /// because the rubber is pointing slightly the wrong way, and how
    /// wrong is a number the model already carries.
    ///
    /// This replaces two approximations that used to be charged side by
    /// side. One was a rate times the square of lateral utilisation, which
    /// is the steady-state answer written as a curve fit; the other was the
    /// body's own sideslip against the lateral acceleration, which is the
    /// same mechanism seen from the chassis. They overlapped in the middle
    /// and, more to the point, they were both blind to the same thing: a
    /// front axle sawn back and forth at a large slip angle while the body
    /// stays pointed straight. Utilisation averages out over the sawing and
    /// the body angle never appears, so shaking the wheel cost a car
    /// nothing. It costs it plenty here, because the angle is read per axle
    /// at the moment it is held.
    /// </summary>
    private static float InducedDragAccel(
        float frontLateralAccel,
        float frontSlipAngle,
        float rearLateralAccel,
        float rearSlipAngle,
        float dynamicYawBlend
    )
    {
        // Faded out with the rest of the slip-angle model. Below
        // DynamicYawMinimumSpeed the car does not carry a sideslip at all:
        // it follows its wheels, so there is no angle between where the
        // rubber points and where it is going, and nothing for this to
        // charge. Reading the angles there anyway charges a crawling car
        // the whole of its grip as drag -- a car nosed into a barrier at
        // walking pace could not pull away from it, which is the deadlock
        // this model has been dug out of twice.
        return dynamicYawBlend * (
            Math.Abs(frontLateralAccel) * MathF.Sin(Math.Abs(frontSlipAngle)) +
            Math.Abs(rearLateralAccel) * MathF.Sin(Math.Abs(rearSlipAngle))
        );
    }

    /// <summary>
    /// The same cost, for a car that is not being simulated: what the
    /// induced drag would settle at if this corner were held.
    ///
    /// The angles are not known to a planner, so they are solved from the
    /// force each axle is being asked for: the slip curve read backwards
    /// gives the angle that delivers that share, and the drag follows. It
    /// is the steady-state reading of the same mechanism the step charges,
    /// so a plan and the physics quote one price list rather than two.
    /// </summary>
    private static float SteadyInducedDragAccel(
        CarConfig config,
        float speed,
        float frontLateralAccel,
        float rearLateralAccel,
        float frontGrip,
        float rearGrip
    )
    {
        float frontShare = Math.Abs(frontLateralAccel) /
                           Math.Max(frontGrip, Epsilon);
        float rearShare = Math.Abs(rearLateralAccel) /
                          Math.Max(rearGrip, Epsilon);
        float frontAngle = TireSlipCurve.InverseEvaluate(
            frontShare, config.FrontPeakSlipAngleRatio
        );
        float rearAngle = TireSlipCurve.InverseEvaluate(rearShare);
        return InducedDragAccel(
            frontLateralAccel, frontAngle, rearLateralAccel, rearAngle,
            CalculateDynamicYawBlend(speed)
        );
    }

    /// <summary>
    /// What is left of a longitudinal demand that is past what the axle
    /// has. Falls from all of it to the vehicle's configured floor over the
    /// cost cap.
    /// </summary>
    private static float OverLimitGripEfficiency(CarConfig config, float overLimit)
    {
        float floor = Math.Clamp(config.OverLimitMinGripEfficiency, 0f, 1f);
        float t = Math.Clamp(
            overLimit / MathF.Max(config.OverLimitCostCap, Epsilon), 0f, 1f
        );
        return 1f + (floor - 1f) * t;
    }

    private static float CostedOverLimit(CarConfig config, float overLimit)
    {
        return Math.Clamp(overLimit, 0f, Math.Max(0f, config.OverLimitCostCap));
    }

    private static void UpdateTires(
        TireState tire,
        bool front,
        CarConfig config,
        TireSet tires,
        float lateralUse,
        float lateralSlipWork,
        float longitudinalUse,
        float brakeUse,
        float directionalHeatDemandUse,
        float lateralHeatScale,
        float lateralWearScale,
        float overLimit,
        float airTempC,
        float trackTempC,
        float speed,
        float coolingAirSpeed,
        float wakeDownforceLoss,
        float dt
    )
    {
        tires.For(front).Update(
            tire,
            new TireWork(
                SizedWheelLoad(config, front), lateralUse, lateralSlipWork, longitudinalUse, brakeUse,
                directionalHeatDemandUse, lateralHeatScale, lateralWearScale, overLimit,
                airTempC, trackTempC, speed, coolingAirSpeed, wakeDownforceLoss),
            dt);
    }

    private static AxleLateralWorkScales CalculateAxleLateralWorkScales(
        CarState state,
        CarConfig config,
        TireSet tires,
        AxleResult front,
        AxleResult rear,
        float frontSlipWork,
        float rearSlipWork
    )
    {
        float frontLoadWeight =
            CalculateTireForceScale(state.FrontLeft, config, tires, front: true) +
            CalculateTireForceScale(state.FrontRight, config, tires, front: true);
        float rearLoadWeight =
            CalculateTireForceScale(state.RearLeft, config, tires, front: false) +
            CalculateTireForceScale(state.RearRight, config, tires, front: false);

        // Lateral rubber work is force times deformation. In this reduced
        // model deformation is not a state of its own, so use the small-angle
        // relation deformation ~= force * compliance. The common mass and
        // rear compliance cancel when only the front/rear share is needed.
        float frontForce = Math.Abs(front.LateralAccel);
        float rearForce = Math.Abs(rear.LateralAccel);
        float frontCompliance = Math.Max(
            0f,
            config.FrontLateralComplianceRatio
        );
        float physicalFrontWeight =
            frontForce * frontForce * frontCompliance;
        float physicalRearWeight = rearForce * rearForce;
        float physicalTotalWeight =
            physicalFrontWeight + physicalRearWeight;
        if (physicalTotalWeight <= Epsilon)
            return AxleLateralWorkScales.Identity;

        float targetFrontShare = physicalFrontWeight / physicalTotalWeight;
        float normalizedFrontUse = Math.Clamp(front.LateralUse, 0f, 1f);
        float normalizedRearUse = Math.Clamp(rear.LateralUse, 0f, 1f);
        // The redistribution has to be computed on the same quantity it
        // is about to scale, or it does not conserve anything. Heat is
        // force times sliding now, so its shares come from that; wear is
        // still force times the square root of the slip and keeps its own.
        // Scaling one by a factor derived from the other was a real defect
        // for exactly as long as the two happened to be the same function.
        AxleScales heat = RedistributeAxleWork(
            Math.Max(0f, frontSlipWork) * frontLoadWeight,
            Math.Max(0f, rearSlipWork) * rearLoadWeight,
            targetFrontShare
        );
        AxleScales wear = RedistributeAxleWork(
            front.LateralUse * front.LateralUse * frontLoadWeight,
            rear.LateralUse * rear.LateralUse * rearLoadWeight,
            targetFrontShare
        );
        return new AxleLateralWorkScales(
            heat.Front,
            heat.Rear,
            wear.Front,
            wear.Rear
        );
    }

    private static AxleScales RedistributeAxleWork(
        float currentFront,
        float currentRear,
        float targetFrontShare
    )
    {
        float total = currentFront + currentRear;
        if (total <= Epsilon)
            return AxleScales.Identity;

        // A multiplicative scale cannot move a finite target onto an axle
        // whose current contribution is numerically zero. In that degenerate
        // frame, keep both sides unchanged so the total work remains exact.
        if (currentFront <= Epsilon || currentRear <= Epsilon)
            return AxleScales.Identity;

        float frontTarget = total * Math.Clamp(targetFrontShare, 0f, 1f);
        float rearTarget = total - frontTarget;
        float frontScale = frontTarget / currentFront;
        float rearScale = rearTarget / currentRear;
        return new AxleScales(frontScale, rearScale);
    }

    /// <summary>The force a tyre gives at its load against what it gives at the load it was sized for (<see cref="ITireModel.ForceScale"/>).</summary>
    private static float CalculateTireForceScale(
        TireState tire,
        CarConfig config,
        TireSet tires,
        bool front
    ) => tires.For(front).ForceScale(tire, SizedWheelLoad(config, front));

    /// <summary>What one wheel of this axle carries with the car standing: the load its tyre is sized for.</summary>
    private static float SizedWheelLoad(CarConfig config, bool front)
    {
        float axleShare = Math.Clamp(config.FrontStaticLoadShare, 0.05f, 0.95f);
        return config.MassKg * Gravity * 0.5f * (front ? axleShare : 1f - axleShare);
    }

    private static WheelLoads CalculateWheelLoads(
        CarState state,
        CarConfig config,
        float massKg,
        float normalGravity
    )
    {
        return CalculateWheelLoads(
            config,
            massKg,
            state.FilteredLongitudinalAccel,
            state.FilteredLateralAccel,
            state.Speed,
            state.DownforceVelocityDeficit,
            state.WakeDownforceLoss,
            state.ActiveAero,
            normalGravity
        );
    }

    /// <summary>
    /// What each tyre is being pressed into the road with.
    ///
    /// Weight, plus what the air is pushing down with, moved about by
    /// accelerating, braking and cornering. The air's share is the reason a
    /// quick corner is worth more than a slow one of the same radius, and
    /// leaving it out makes every corner the same corner: measured against
    /// published lap times, a circuit whose character is its fast corners came
    /// out slower than one whose character is a long straight, which is the
    /// wrong way round.
    ///
    /// Downforce is shared front to rear as the aero is trimmed
    /// (<see cref="IAero.FrontDownforceShare"/>), as the weight is unless
    /// it says otherwise.
    /// </summary>
    private static WheelLoads CalculateWheelLoads(
        CarConfig config,
        float massKg,
        float longitudinalAcceleration,
        float lateralAcceleration,
        float speed,
        float downforceVelocityDeficit,
        float wakeDownforceLoss,
        float activeAero,
        float normalGravity = Gravity
    )
    {
        float downforce = EffectiveDownforcePerSpeedSquared(
            config,
            downforceVelocityDeficit,
            wakeDownforceLoss,
            activeAero
        ) * speed * speed;
        float totalLoad = massKg * normalGravity + downforce;
        // The downforce sits on the axles as the weight does unless the aero
        // is trimmed away from it, and only the difference moves.
        float aeroBalance = config.Aero.FrontDownforceShare ?? config.FrontStaticLoadShare;
        float frontLoad = totalLoad * config.FrontStaticLoadShare +
                          downforce * (aeroBalance - config.FrontStaticLoadShare);
        frontLoad -= massKg * longitudinalAcceleration * config.CenterOfGravityHeightMeters /
                     Math.Max(config.WheelBaseMeters, Epsilon);
        frontLoad = Math.Clamp(frontLoad, 0f, totalLoad);

        float rearLoad = totalLoad - frontLoad;
        float lateralTransfer = massKg * lateralAcceleration * config.CenterOfGravityHeightMeters /
                                Math.Max(config.TrackWidthMeters, Epsilon);
        float frontTransfer = lateralTransfer * Math.Clamp(config.FrontLateralLoadTransferShare, 0f, 1f);
        float rearTransfer = lateralTransfer - frontTransfer;

        float fl = frontLoad * 0.5f - frontTransfer * 0.5f;
        float fr = frontLoad * 0.5f + frontTransfer * 0.5f;
        float rl = rearLoad * 0.5f - rearTransfer * 0.5f;
        float rr = rearLoad * 0.5f + rearTransfer * 0.5f;

        float minWheelLoad = totalLoad * 0.25f * config.MinimumWheelLoadShare;
        return new WheelLoads(
            Math.Max(fl, minWheelLoad),
            Math.Max(fr, minWheelLoad),
            Math.Max(rl, minWheelLoad),
            Math.Max(rr, minWheelLoad)
        );
    }

    private static float EffectiveTireWakeDownforceLoss(
        CarState state,
        CarConfig config
    ) => config.Aero.TireWakeDownforceLoss(AeroAir.Of(state));

    private static float EffectiveDownforcePerSpeedSquared(
        CarConfig config,
        float downforceVelocityDeficit,
        float wakeDownforceLoss,
        float activeAero
    ) => config.Aero.EffectiveDownforcePerSpeedSquared(
        new AeroAir(0f, downforceVelocityDeficit, wakeDownforceLoss, activeAero));

    private static void ApplyWheelLoads(CarState state, WheelLoads loads)
    {
        state.FrontLeft.LoadN = loads.FrontLeft;
        state.FrontRight.LoadN = loads.FrontRight;
        state.RearLeft.LoadN = loads.RearLeft;
        state.RearRight.LoadN = loads.RearRight;
    }

    private static float CalculateAxleGripAccel(
        float massKg,
        CarConfig config,
        TireSet tires,
        TireState left,
        TireState right,
        bool front
    )
    {
        // Each wheel brings its own road with it. A car straddling the
        // white line has one side of an axle on tarmac and the other on
        // grass, and the axle is worth the sum of the two rather than the
        // better of them - so half a car off the road is half an axle's
        // grip gone, and the cost of running wide scales with how far.
        float leftForce = WheelGripForce(config, tires, left, left.LoadN, front) *
                          MathF.Max(0f, left.SurfaceGrip);
        float rightForce = WheelGripForce(config, tires, right, right.LoadN, front) *
                           MathF.Max(0f, right.SurfaceGrip);
        return (leftForce + rightForce) / Math.Max(massKg, Epsilon);
    }

    /// <summary>The most force a tyre can give at this load (<see cref="ITireModel.GripForce"/>).</summary>
    private static float WheelGripForce(CarConfig config, TireSet tires, TireState tire, float loadN, bool front) =>
        tires.For(front).GripForce(tire, loadN, SizedWheelLoad(config, front));

    private static float Lerp(float from, float to, float weight)
    {
        return from + (to - from) * Math.Clamp(weight, 0f, 1f);
    }

    private static float LerpAngle(float from, float to, float weight)
    {
        float delta = MathHelper.NormalizeAngle(to - from);
        return MathHelper.NormalizeAngle(from + delta * Math.Clamp(weight, 0f, 1f));
    }

    private readonly record struct AxleScales(float Front, float Rear)
    {
        public static AxleScales Identity => new(1f, 1f);
    }

    private readonly record struct AxleLateralWorkScales(
        float FrontHeat,
        float RearHeat,
        float FrontWear,
        float RearWear
    )
    {
        public static AxleLateralWorkScales Identity => new(1f, 1f, 1f, 1f);
    }

    /// <summary>
    /// What the lateral pair did over one physics step: the axles averaged
    /// across the subdivision, the states at the end of it, and the slip
    /// angles the last piece was resolved at.
    /// </summary>
    private readonly record struct LateralIntegration(
        AxleResult Front,
        AxleResult Rear,
        float PathLateralAccel,
        float YawAcceleration,
        float Sideslip,
        float YawRate,
        float HeadingDelta,
        float FrontSlipAngle,
        float RearSlipAngle,
        int Substeps
    );

    private readonly record struct AxleResult(
        float LateralAccel,
        float LongitudinalAccel,
        float OverLimit,
        float CombinedRequest,
        float LateralUse,
        float LongitudinalUse,
        float LimiterCut
    );

    private readonly record struct WheelLoads(
        float FrontLeft,
        float FrontRight,
        float RearLeft,
        float RearRight
    );
}
