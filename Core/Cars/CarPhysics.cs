using System;
using System.Numerics;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Cars;

public static class CarPhysics
{
    private const float Gravity = 9.80665f;
    private const float Epsilon = 1e-5f;
    private const float MinimumTireHeatLoadScale = 0.2f;
    private const float MaximumThermalOverLimit = 1f;
    private const float MinimumTemperatureGripFactor = 0.55f;
    private const float MaximumTemperatureGripFactor = 1.08f;
    private const float MinimumWearGripFactor = 0.45f;
    private const float RearSlipOnsetCombinedUse = 0.82f;
    private const float RearSlipDominanceRange = 0.2f;
    private const float DynamicYawMinimumSpeed = 5f;
    private const float DynamicYawBlendRange = 5f;
    private const float SideslipEnergyLossScale = 1f;

    /// <summary>
    /// Speed below which a slip angle stops meaning anything, because the
    /// yaw-rate term divided by it stops meaning anything. The kinematic
    /// blend owns everything under here.
    /// </summary>
    private const float MinimumSlipAngleSpeed = 3f;

    /// <summary>
    /// How loaded the car has to be, as a share of what its tyres can give
    /// laterally, before the driver has wound on all the extra lock the
    /// slip angles want. Below it the extra fades away, so turning in from
    /// a straight line starts with the angle geometry asks for and nothing
    /// else - which is what hands are actually doing at that moment.
    /// </summary>
    private const float FullSlipCompensationShare = 0.35f;

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

    internal static float EffectiveDownforceAccelPerSpeedSquared(
        CarState state,
        CarConfig config
    )
    {
        return EffectiveDownforceAccelPerSpeedSquared(
            config,
            state.DownforceVelocityDeficit,
            state.WakeDownforceLoss,
            state.OvertakeAssist
        );
    }

    internal static CarPerformanceLimits EstimatePerformanceLimits(
        CarState state,
        CarConfig config,
        TireConfig tires,
        CarStrategy strategy,
        float speed,
        float curvature,
        float gripUsage = 1f,
        float assumedLongitudinalAcceleration = 0f,
        float frontBrakeBiasOffset = 0f,
        float corneringEfficiency = 1f
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
            state.OvertakeAssist
        );
        float usage = Math.Clamp(gripUsage, 0.05f, 1f);
        float frontGrip = (
            loads.FrontLeft * CalculateTireMu(tires, state.FrontLeft) +
            loads.FrontRight * CalculateTireMu(tires, state.FrontRight)
        ) / Math.Max(massKg, Epsilon) * usage;
        float rearGrip = (
            loads.RearLeft * CalculateTireMu(tires, state.RearLeft) +
            loads.RearRight * CalculateTireMu(tires, state.RearRight)
        ) / Math.Max(massKg, Epsilon) * usage;

        float extraction = Math.Clamp(corneringEfficiency, 0.05f, 1f);
        // What the car can hold, not what it can touch. See
        // TireSlipCurve.SustainablePeakShare: planning against the whole
        // circle means arriving at every apex a tenth over what the tyres
        // will give, and running wide by exactly that.
        float lateralLimit =
            AxleLateralCeiling(config, frontGrip, rearGrip) * extraction;

        // What the corner costs the tyre, which is not what the corner is
        // worth to the car. A driver who only gets part of the cornering out
        // of a tyre still spends the tyre on all of it, so the grip left over
        // for braking has to be measured against the bill, not the benefit.
        // Charging the smaller number here is what let a plan brake as though
        // it were still on the straight while the car was already at the limit.
        float chargedLateralAcceleration = lateralAcceleration / extraction;
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

        float gripDriveLimit = DistributedLongitudinalLimit(
            frontLongitudinal,
            rearLongitudinal,
            config.FrontDriveShare
        );
        float powertrainDriveLimit = config.Powertrain.DriveAccelerationLimit(
            state.Energy,
            strategy,
            speed,
            massKg,
            config.MaxDriveAcceleration
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
        float lateralUse = Math.Abs(lateralAcceleration) / Math.Max(frontGrip + rearGrip, Epsilon);
        float loss = CalculateLossAccel(
            config,
            speed,
            lateralUse,
            state.AirVelocityDeficit,
            state.OvertakeAssist
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
        TireConfig tires,
        float speed,
        float curvature,
        float gripUsage,
        float assumedLongitudinalAcceleration,
        float corneringEfficiency
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
            state.OvertakeAssist
        );
        float usage = Math.Clamp(gripUsage, 0.05f, 1f);
        float mass = Math.Max(massKg, Epsilon);
        float frontGrip = (
            loads.FrontLeft * CalculateTireMu(tires, state.FrontLeft) +
            loads.FrontRight * CalculateTireMu(tires, state.FrontRight)
        ) / mass * usage;
        float rearGrip = (
            loads.RearLeft * CalculateTireMu(tires, state.RearLeft) +
            loads.RearRight * CalculateTireMu(tires, state.RearRight)
        ) / mass * usage;

        return MathF.Max(
            0f,
            AxleLateralCeiling(config, frontGrip, rearGrip) *
            Math.Clamp(corneringEfficiency, 0.05f, 1f)
        );
    }

    public static void Step(
        CarState state,
        CarConfig config,
        TireConfig tires,
        CarPhysicsStepInput input,
        float dt
    )
    {
        if (dt <= 0f)
            return;

        state.Normalize();

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

        float frontGrip = CalculateAxleGripAccel(massKg, tires, state.FrontLeft, state.FrontRight);
        float rearGrip = CalculateAxleGripAccel(massKg, tires, state.RearLeft, state.RearRight);
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
        // This is still worth naming because it is what the driver asked
        // for and what the telemetry reports against. Nothing downstream
        // obeys it any more: the tyres deliver what their slip angles say,
        // and the difference between the two is understeer.
        float requestedLateralAccel =
            curvatureDemandScale * state.Speed * state.Speed * desiredCurvature +
            roadLateralDemand;
        float referenceYawRate = state.Speed * desiredCurvature;
        float dynamicYawBlend = CalculateDynamicYawBlend(state.Speed);
        float corneringEfficiency = Math.Clamp(input.CorneringEfficiency, 0.05f, 1f);
        float limitSettleUse = MathF.Max(input.LimitSettleUse, 0.5f);

        float steerAngle = UpdateSteerAngle(
            state,
            config,
            desiredCurvature,
            requestedLateralAccel,
            frontGrip,
            rearGrip,
            corneringEfficiency,
            limitSettleUse,
            dt
        );

        // What each axle is giving laterally right now with its whole
        // circle available. The brake allocator needs to know how much of
        // each axle is already spoken for, and that is no longer a request
        // to be granted: it is a measurement of what the rubber is doing at
        // the angle it is at.
        float frontLatRequest = frontGrip * corneringEfficiency *
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
        float rearLatRequest = rearGrip * corneringEfficiency *
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
            float driveAccel = Math.Min(
                desiredAccel,
                config.Powertrain.DriveAccelerationLimit(
                    state.Energy,
                    input.Strategy,
                    state.Speed,
                    massKg,
                    config.MaxDriveAcceleration
                )
            );
            frontLongRequest = driveAccel * config.FrontDriveShare;
            rearLongRequest = driveAccel - frontLongRequest;
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
            frontBrake = ApplyAntiLock(
                config, frontLatRequest, frontBrake, frontGrip);
            rearBrake = ApplyAntiLock(
                config, rearLatRequest, rearBrake, rearGrip);
            frontLongRequest = -frontBrake;
            rearLongRequest = -rearBrake;
            requestedLongitudinalAccel = -(frontBrake + rearBrake);
        }

        float tractionControlCutAccel = 0f;
        if (rearLongRequest > 0f)
        {
            float uncontrolledRearDrive = rearLongRequest;
            rearLongRequest = ApplyRearTractionControl(
                config,
                rearLatRequest,
                rearLongRequest,
                rearGrip
            );
            tractionControlCutAccel = Math.Max(
                0f,
                uncontrolledRearDrive - rearLongRequest
            );
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
            corneringEfficiency,
            roadLateralDemand,
            curvatureDemandScale,
            dynamicYawBlend,
            dt
        );
        AxleResult front = lateral.Front;
        AxleResult rear = lateral.Rear;

        float actualLateralAccel = front.LateralAccel + rear.LateralAccel;
        float driveAccelActual = Math.Max(0f, front.LongitudinalAccel) + Math.Max(0f, rear.LongitudinalAccel);
        float brakeAccelActual = Math.Max(0f, -front.LongitudinalAccel) + Math.Max(0f, -rear.LongitudinalAccel);
        float axleLongitudinalAccel = front.LongitudinalAccel + rear.LongitudinalAccel;

        // What the tyre was worked, not what came out of it. A driver who
        // wastes part of the cornering still spent the tyre on all of it, and
        // charging the wear on the smaller number would hand the slower driver
        // longer-lasting tyres for being slow.
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

        float sideslipLossAccel = CalculateSideslipLossAccel(
            actualLateralAccel,
            state.SideslipAngleRadians
        );
        float lossAccel = CalculateLossAccel(
            config,
            state.Speed,
            lateralUse,
            state.AirVelocityDeficit,
            state.OvertakeAssist
        ) + sideslipLossAccel;
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
        // The rear tyre's own scrub, which is what this was always trying
        // to be. It used to be body sideslip against a clamp that no longer
        // exists; it is now the angle the rear rubber is actually being
        // dragged at, against the angle it stops paying at.
        float normalizedSideslip = Math.Clamp(
            MathF.Abs(lateral.RearSlipAngle) /
            TireSlipCurve.PeakSlipAngleRadians,
            0f,
            1f
        );

        PowertrainSettlement settlement = config.Powertrain.Settle(
            state.Energy,
            driveAccelActual,
            brakeAccelActual,
            averageSpeed,
            massKg,
            dt
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
        // A driver approaching the car's combined limit creates a common
        // intensity for this instant. Each axle still receives heat according
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
            0f,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            tireWakeDownforceLoss,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.FrontRight,
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
            0f,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            tireWakeDownforceLoss,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.RearLeft,
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
            normalizedSideslip,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            tireWakeDownforceLoss,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.RearRight,
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
            normalizedSideslip,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            tireWakeDownforceLoss,
            dt,
            input.TireEnergyEfficiency
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
            frontGrip,
            rearGrip,
            frontLateralUse,
            rearLateralUse,
            frontLongitudinalUse,
            rearLongitudinalUse,
            overLimit,
            drivePowerWatts,
            regenPowerWatts,
            tractionControlCutAccel,
            sideslipLossAccel,
            state.SideslipAngleRadians,
            rearSlideSeverity,
            referenceYawRate,
            state.YawRateRadiansPerSecond,
            actualYawAcceleration
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
        TireConfig tires,
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
            massKg, tires, state.FrontLeft, state.FrontRight
        );
        float rearSpinGrip = CalculateAxleGripAccel(
            massKg, tires, state.RearLeft, state.RearRight
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
                0f,
                input.AirTempC,
                input.TrackTempC,
                averageSpeed,
                coolingAirSpeed,
                tireWakeDownforceLoss,
                dt,
                input.TireEnergyEfficiency
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
            CalculateAxleGripAccel(massKg, tires, state.FrontLeft, state.FrontRight),
            CalculateAxleGripAccel(massKg, tires, state.RearLeft, state.RearRight),
            0f,
            0f,
            0f,
            0f,
            0f,
            settlement.DrawnPowerWatts,
            0f,
            0f,
            0f,
            state.SideslipAngleRadians,
            1f,
            0f,
            yawRate,
            (yawRate - previousYawRate) / dt
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
    /// The driver's language has not changed - they ask for a curvature -
    /// but the car no longer grants it. Geometry says what angle would draw
    /// that corner if the tyres followed exactly; the tyres do not, so a
    /// small gain closes what is left, which is what a driver is doing when
    /// they wind on more lock because the car is running wide.
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
        float corneringEfficiency,
        float limitSettleUse,
        float dt
    )
    {
        float wheelBase = MathF.Max(config.WheelBaseMeters, Epsilon);
        float maximum = MathF.Max(config.MaxSteerAngleRadians, 0f);
        float frontArm = FrontMomentArm(config);
        float rearArm = RearMomentArm(config);

        // Geometry is the start of the answer and not the whole of it. The
        // wheels have to be turned further than the corner's own angle by
        // exactly the difference between the two ends' slip angles, and a
        // driver knows that difference the way they know the car: they wind
        // it on rather than discovering it. Past the peak the inverse runs
        // out, which is the point at which no amount of lock buys any more
        // corner - understeer, arrived at honestly.
        float frontShare = rearArm / wheelBase * requestedLateralAccel;
        float rearShare = frontArm / wheelBase * requestedLateralAccel;
        float frontCapacity = MathF.Max(frontGrip * corneringEfficiency, Epsilon);
        float rearCapacity = MathF.Max(rearGrip * corneringEfficiency, Epsilon);
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
        // load, which is what a driver's hands do. Applied in full from the
        // first frame it is worse than useless - a car with a worn rear
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

        // Feel for the limit, carried over from the model this replaces. A
        // driver who can feel the front let go stops adding lock past the
        // angle where it stops paying; one who cannot keeps winding it on
        // and gets nothing back for it. Infinity, which is what a learned
        // driver is handed, means no ceiling at all.
        //
        // The steady-state inversion above holds the front at its peak for
        // the same reason, and that is a professional's hands: they do not
        // wind lock on past the angle the tyre wants. A driver who does is
        // exactly what a low CarControl rating looks like, and the amount
        // by which they may is the knob - zero for the best hands, positive
        // for the worst, front tyres scrubbed accordingly. Same family as
        // the settle ceiling below, and it belongs to the ability era; the
        // blueprint is drawn here and nothing is built.
        if (float.IsFinite(limitSettleUse))
        {
            float ceiling = TireSlipCurve.PeakSlipAngleRadians * limitSettleUse;
            float carried = state.SideslipAngleRadians +
                            FrontMomentArm(config) *
                            state.YawRateRadiansPerSecond /
                            MathF.Max(state.Speed, MinimumSlipAngleSpeed);
            target = Math.Clamp(target, carried - ceiling, carried + ceiling);
        }

        float step = MathF.Max(config.SteerRateLimitRadiansPerSecond, 0f) * dt;
        float change = Math.Clamp(target - state.SteerAngleRadians, -step, step);
        return Math.Clamp(state.SteerAngleRadians + change, -maximum, maximum);
    }

    /// <summary>
    /// One axle, at the angle it is at, with whatever the brakes or the
    /// motor have left it.
    ///
    /// Longitudinal first, because that is commanded directly: a locked
    /// wheel steers nothing, and the anti-lock upstream exists precisely to
    /// stop the driver spending the whole circle on stopping. What remains
    /// of the circle is what the slip angle gets to work with.
    ///
    /// The tyre is charged for what it was worked at and the car receives
    /// what the driver managed to extract, which is the same split the
    /// model has always used: wasting part of a corner still spends the
    /// rubber on all of it.
    /// </summary>
    private static AxleResult ResolveAxleSlip(
        CarConfig config,
        float grip,
        float slipAngle,
        float peakScale,
        float longitudinalRequest,
        float corneringEfficiency
    )
    {
        if (grip <= Epsilon)
            return default;

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
        float lateralAccel = grip * circle * shape * corneringEfficiency;

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
            longitudinalUse
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
    /// Below walking pace the slip angles are meaningless, so the same
    /// kinematic blend the model has always used takes over: the body
    /// simply follows the path. Above it the car is free to be out of
    /// shape, and there is no clamp anywhere that says how far.
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
        float corneringEfficiency,
        float roadLateralDemand,
        float curvatureDemandScale,
        float dynamicYawBlend,
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
            frontSlip = FrontSlipAngle(
                config, steerAngle, sideslip, yawRate, speed
            );
            rearSlip = RearSlipAngle(config, sideslip, yawRate, speed);
            front = ResolveAxleSlip(
                config,
                frontGrip,
                frontSlip,
                frontPeakScale,
                frontLongRequest,
                corneringEfficiency
            );
            rear = ResolveAxleSlip(
                config, rearGrip, rearSlip, 1f, rearLongRequest, corneringEfficiency
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
            float dynamicYawRate = Math.Clamp(
                yawRate + yawStep * h,
                -SingleTrackDynamicsLimits.MaximumYawRateRadiansPerSecond,
                SingleTrackDynamicsLimits.MaximumYawRateRadiansPerSecond
            );
            float dynamicSideslip = sideslip + (velocityYawRate - yawRate) * h;
            float nextYawRate = Lerp(
                velocityYawRate, dynamicYawRate, dynamicYawBlend
            );
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
                front.LongitudinalUse
            ),
            new AxleResult(
                rearLateral * share,
                rear.LongitudinalAccel,
                rearOver * share,
                rearCombined * share,
                rearUse * share,
                rear.LongitudinalUse
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
    /// The force share the calibration is matched at: hard cornering, but
    /// not at the limit. This is where a racing car actually spends its
    /// cornering seconds, and matching there is what "the same below the
    /// peak" has to mean.
    /// </summary>
    private const float SlipHeatReferenceUse = 0.85f;

    /// <summary>
    /// The factor that makes force-times-sliding agree with the
    /// force-squared reading it replaces, at everyday sub-limit use.
    ///
    /// Solved, not fitted: set <c>use^2 = k x use x sin(alpha(use)) /
    /// sin(peak)</c> at the reference share and the constant falls out as
    /// <c>ref x sin(peak) / sin(alpha(ref))</c>, with the slip angle read
    /// off the tyre curve itself.
    ///
    /// Two corrections got it here, both found by measuring against the
    /// old model rather than by reading the algebra twice.
    ///
    /// The first version solved at the tangent at zero slip, on the
    /// reasoning that the curve is linear down there. It is, but the car
    /// is not: over a five-minute mixed-driving cycle that ran 1.4 C hot
    /// at 60% use and 4.8 C hot at 95%, because the curve has already left
    /// its tangent by 14% at 60%. Matching a model at an operating point
    /// nothing operates at is not matching it.
    ///
    /// The second is <see cref="TireConfig.MinimumDirectionalHeatScale"/>.
    /// The old lateral term did not stand alone — it went through the
    /// ramp that says most of the contact patch still adheres at modest
    /// use, and below <see cref="TireConfig.DirectionalHeatRampStartUse"/>
    /// that ramp is not a ramp at all but a flat multiplier of a fifth.
    /// The reference share sits below the ramp start, so the thing being
    /// matched is a fifth of what the term appeared to say, and leaving it
    /// out made this five times too hot. The lateral term does not go
    /// through that ramp any more — expressing "little of the work becomes
    /// heat at modest use" is exactly what slip power does natively — so
    /// the factor belongs here, in the constant, where it can be seen.
    ///
    /// Above the reference the two still part company, and that is the
    /// point of the change: force falls away past the peak while the
    /// dragging keeps growing, so heat goes on rising where a use-squared
    /// reading would have it fall. That is the near-limit heating that
    /// used to need its own hand-placed branch, and it arrives here for
    /// free and continuous.
    /// </summary>
    private static readonly float SlipHeatCalibration =
        SlipHeatReferenceUse *
        TireConfig.MinimumDirectionalHeatScale *
        MathF.Sin(TireSlipCurve.PeakSlipAngleRadians) /
        MathF.Max(
            MathF.Sin(TireSlipCurve.InverseEvaluate(SlipHeatReferenceUse)),
            Epsilon
        );

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
    /// oversteering, and the driver has a decision to make about it.
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
    /// Holds an axle's brakes back before the tyre gives up, the mirror of the
    /// traction control below and written the same way so the pair can be read
    /// together.
    /// </summary>
    private static float ApplyAntiLock(
        CarConfig config,
        float lateralRequest,
        float brakeRequest,
        float grip
    )
    {
        float strength = Math.Clamp(config.AntiLockStrength, 0f, 1f);
        if (brakeRequest <= 0f || grip <= Epsilon || strength <= 0f)
            return brakeRequest;

        float activationUse = Math.Clamp(
            config.AntiLockActivationUse,
            0.05f,
            1f
        );
        float available = RemainingLongitudinalGrip(
            grip * activationUse,
            lateralRequest
        );
        return Lerp(brakeRequest, Math.Min(brakeRequest, available), strength);
    }

    private static float ApplyRearTractionControl(
        CarConfig config,
        float lateralRequest,
        float driveRequest,
        float rearGrip
    )
    {
        float strength = Math.Clamp(config.TractionControlStrength, 0f, 1f);
        if (driveRequest <= 0f || rearGrip <= Epsilon || strength <= 0f)
            return driveRequest;

        float activationUse = Math.Clamp(
            config.TractionControlActivationUse,
            0.05f,
            1f
        );
        float activationGrip = rearGrip * activationUse;
        float availableAtActivation = RemainingLongitudinalGrip(
            activationGrip,
            lateralRequest
        );
        float targetDrive = Math.Min(driveRequest, availableAtActivation);
        return Lerp(driveRequest, targetDrive, strength);
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
    /// What an axle actually delivers of what it was asked for, and how much
    /// of itself it spent doing so.
    ///
    /// The same corner costs one driver more tyre than another. Where the
    /// difference goes is not modelled in detail - it is line, hands, the
    /// hundred small things - only that it is spent: a driver who extracts
    /// nine tenths of what the tyre offers reaches the same corner speed
    /// having used a ninth more of it, and runs out of tyre that much sooner.
    ///
    /// The force itself is not discounted here. What the driver can reach at
    /// all is already lower, because the speed plan is drawn against the same
    /// figure, and taking it off the delivered force as well would charge the
    /// discount twice: the car would be planned to a corner speed it then
    /// could not hold, and would understeer wide of the line all lap.
    ///
    /// Only cornering. Applying the same discount to braking and driving would
    /// say that a slower driver cannot use the brakes, which is a different
    /// claim and a wrong one, and on a straight it would say nothing at all
    /// because the car is limited by its battery there and not by its tyres.
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
    /// front, and the tyre scrub of cornering is not an air loss at all.
    /// </summary>
    private static float CalculateLossAccel(
        CarConfig config,
        float speed,
        float lateralUse,
        float airVelocityDeficit,
        float overtakeAssist
    )
    {
        if (speed <= 0.01f)
            return 0f;

        float metAir = 1f - Math.Clamp(airVelocityDeficit, 0f, 1f);
        float assist = Math.Clamp(overtakeAssist, 0f, 1f);
        return
            config.RollingDragAccel +
            config.AeroDragAccelPerSpeedSquared * speed * speed *
            metAir * metAir *
            (1f - config.OvertakeAssistDragReduction * assist) +
            config.CorneringScrubAccel * lateralUse * lateralUse;
    }

    private static float CalculateSideslipLossAccel(
        float lateralAcceleration,
        float sideslipAngle
    )
    {
        return Math.Abs(
            lateralAcceleration * MathF.Sin(sideslipAngle)
        ) * SideslipEnergyLossScale;
    }

    /// <summary>
    /// What is left of a longitudinal demand that is past what the axle
    /// has. Falls from all of it to the car's floor over the cost cap, so a
    /// driver who over-brakes by a little loses a little.
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
        CarConfig config,
        TireConfig tires,
        float lateralUse,
        float lateralSlipWork,
        float longitudinalUse,
        float brakeUse,
        float directionalHeatDemandUse,
        float lateralHeatScale,
        float lateralWearScale,
        float overLimit,
        float sideslipRatio,
        float airTempC,
        float trackTempC,
        float speed,
        float coolingAirSpeed,
        float wakeDownforceLoss,
        float dt,
        float tireEnergyEfficiency
    )
    {
        float loadScale = CalculateTireWorkLoadScale(tire, config);
        float thermalOverLimit = Math.Min(overLimit, MaximumThermalOverLimit);
        float normalizedLateralUse = Math.Clamp(Math.Abs(lateralUse), 0f, 1f);
        float normalizedLongitudinalUse = Math.Clamp(
            Math.Abs(longitudinalUse),
            0f,
            1f
        );
        float normalizedBrakeUse = Math.Clamp(brakeUse, 0f, 1f);
        float tireWorkSpeedMultiplier = Math.Clamp(
            Math.Max(0f, speed) /
            TireConfig.TireWorkReferenceSpeedMps,
            0f,
            TireConfig.MaximumTireWorkSpeedMultiplier
        );
        float longitudinalHeatUse = MathF.Pow(
            normalizedLongitudinalUse,
            TireConfig.LongitudinalHeatExponent
        );
        float combinedUse = Math.Clamp(
            MathF.Sqrt(
                normalizedLateralUse * normalizedLateralUse +
                normalizedLongitudinalUse * normalizedLongitudinalUse
            ),
            0f,
            1f
        );
        float rollingHeatSpeedFactor = Math.Max(0f, speed) /
                                       (
                                           Math.Max(0f, speed) +
                                           TireConfig.RollingHeatReferenceSpeedMps
                                       );
        float rollingSurfaceHeat = TireConfig.RollingSurfaceHeatRate *
                                   loadScale * rollingHeatSpeedFactor;

        float driverSensitiveEnergyFactor = Math.Clamp(tireEnergyEfficiency, 0.9f, 1.1f);
        // Lateral tread heat is the rubber's own dissipation: the force it
        // is carrying times how fast it is being dragged sideways. The
        // constant is solved, not chosen. Below the peak the tyre curve is
        // linear, so force share is the cornering stiffness times the slip
        // angle, and force x slip and force-squared are the same function
        // of each other up to exactly this factor. Matching there is what
        // keeps every tyre temperature ever calibrated on this car -- the
        // same discipline the wear half was done under.
        //
        // Above the peak the two part company, which is the point. Force
        // falls away while the dragging keeps growing, so heat goes on
        // rising where a use-squared reading would have it fall. That is
        // the near-limit heating that used to need its own hand-placed
        // branch, and it arrives here for free and continuous.
        float lateralSlipHeat =
            tires.LateralHeatRate * SlipHeatCalibration *
            Math.Max(0f, lateralSlipWork) * lateralHeatScale;
        float directionalHeat =
            tires.LongitudinalHeatRate * longitudinalHeatUse;
        // Force demand is not the same thing as rubber dissipation. At modest
        // utilization most of the contact patch still adheres and relatively
        // little of the work becomes tread heat; local micro-slip grows quickly
        // as the friction circle fills. Keep this continuous and based on the
        // actual combined vehicle demand, rather than on the named strategy mode.
        float directionalHeatProgress = Math.Clamp(
            (directionalHeatDemandUse - TireConfig.DirectionalHeatRampStartUse) /
            Math.Max(
                TireConfig.DirectionalHeatRampEndUse -
                TireConfig.DirectionalHeatRampStartUse,
                Epsilon
            ),
            0f,
            1f
        );
        float directionalHeatSmoothStep =
            directionalHeatProgress * directionalHeatProgress *
            (3f - 2f * directionalHeatProgress);
        directionalHeat *=
            TireConfig.MinimumDirectionalHeatScale +
            (1f - TireConfig.MinimumDirectionalHeatScale) *
            directionalHeatSmoothStep;
        float wakeCorneringHeat = TireConfig.WakeCorneringHeatRate *
                                   Math.Clamp(
                                       wakeDownforceLoss,
                                       0f,
                                       0.1f
                                   ) *
                                   normalizedLateralUse *
                                   normalizedLateralUse *
                                   lateralHeatScale *
                                   tireWorkSpeedMultiplier;
        float surfaceHeat =
            tireWorkSpeedMultiplier *
            (directionalHeat + lateralSlipHeat) *
            driverSensitiveEnergyFactor +
            TireConfig.OverLimitHeatRate * thermalOverLimit * thermalOverLimit +
            TireConfig.SideslipHeatRate * sideslipRatio * sideslipRatio +
            wakeCorneringHeat;
        surfaceHeat *= loadScale;
        surfaceHeat += rollingSurfaceHeat;

        float airCoolingMultiplier = CalculateAirCoolingMultiplier(
            coolingAirSpeed
        );
        float surfaceToAir = TireConfig.SurfaceCoolingRate * airCoolingMultiplier * (tire.SurfaceTempC - airTempC);
        float surfaceToTrack = TireConfig.TrackSurfaceTransferRate *
                               (tire.SurfaceTempC - trackTempC);
        float surfaceToCore = TireConfig.SurfaceCoreTransferRate * (tire.SurfaceTempC - tire.CoreTempC);
        float rollingCoreHeat = TireConfig.RollingCoreHeatRate *
                                loadScale * rollingHeatSpeedFactor;
        float brakeCoreHeat = TireConfig.BrakeCoreHeatRate *
                              normalizedBrakeUse *
                              tireWorkSpeedMultiplier *
                              loadScale;

        tire.SurfaceTempC += (
            surfaceHeat - surfaceToAir - surfaceToTrack - surfaceToCore
        ) * dt;
        // The carcass makes its own heat by flexing, so without somewhere to put
        // it the only way out is backwards through the tread, and it has to
        // stand hotter than the tread for that to happen - permanently, by an
        // amount nothing the driver does can change. It does have somewhere to
        // put it: the rim, which is metal and has air moving over it, and the
        // gas inside the tyre. Both are stood in for here by the outside air.
        //
        // Far slower than the tread, which is lying on the road inside its own
        // hurricane. On its own this path takes something like twenty minutes,
        // against the tread's forty seconds, which is what makes a soaked
        // carcass something a stint has to live with rather than something a
        // straight fixes.
        float coreToAir = TireConfig.CoreAirCoolingRate *
                          airCoolingMultiplier *
                          (tire.CoreTempC - airTempC);
        tire.CoreTempC += (
            rollingCoreHeat + brakeCoreHeat + surfaceToCore - coreToAir
        ) / TireConfig.CoreHeatCapacityRatio * dt;

        float tempWearFactor = CalculateTemperatureWearFactor(
            tires,
            tire.SurfaceTempC
        );
        float directionalWear =
            tires.LateralWearRate * lateralUse * lateralUse *
            lateralWearScale +
            tires.LongitudinalWearRate * longitudinalUse * longitudinalUse;
        float partialSlipWear = Math.Max(0f, tires.NearLimitWearRate) *
                                MathF.Pow(
                                    combinedUse,
                                    TireConfig.NearLimitWearExponent
                                );
        float tireWorkWear =
            (directionalWear + partialSlipWear) *
            driverSensitiveEnergyFactor +
            tires.OverLimitWearRate * thermalOverLimit * thermalOverLimit +
            tires.SideslipWearRate * sideslipRatio * sideslipRatio;
        float wearDelta = tireWorkWear * tireWorkSpeedMultiplier *
                          tempWearFactor * loadScale * dt;

        tire.Wear = Math.Clamp(tire.Wear + wearDelta, 0f, 1f);
    }

    private static AxleLateralWorkScales CalculateAxleLateralWorkScales(
        CarState state,
        CarConfig config,
        AxleResult front,
        AxleResult rear,
        float frontSlipWork,
        float rearSlipWork
    )
    {
        float frontLoadWeight =
            CalculateTireWorkLoadScale(state.FrontLeft, config) +
            CalculateTireWorkLoadScale(state.FrontRight, config);
        float rearLoadWeight =
            CalculateTireWorkLoadScale(state.RearLeft, config) +
            CalculateTireWorkLoadScale(state.RearRight, config);

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

    private static float CalculateTireWorkLoadScale(
        TireState tire,
        CarConfig config
    )
    {
        // Chassis mass on purpose, not what the car weighs at this moment.
        // This is the reference a tyre's work is counted against, and a
        // reference that fell with the fuel load would cancel the very effect
        // it is there to show: a car that has burned half its fuel presses
        // its tyres less hard and should be recorded as working them less.
        return Math.Max(
            MinimumTireHeatLoadScale,
            tire.LoadN /
            Math.Max(config.MassKg * Gravity * 0.25f, Epsilon)
        );
    }

    private static float CalculateTemperatureWearFactor(
        TireConfig tires,
        float surfaceTempC
    )
    {
        float idealLow = Math.Min(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float idealHigh = Math.Max(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float coldDistance = Math.Max(0f, idealLow - surfaceTempC);
        float hotDistance = Math.Max(0f, surfaceTempC - idealHigh);
        return 1f +
               Math.Max(0f, tires.ColdWearPerCSquared) *
               coldDistance * coldDistance +
               Math.Max(0f, tires.HotWearPerCSquared) *
               hotDistance * hotDistance;
    }

    private static float CalculateAirCoolingMultiplier(float speed)
    {
        float speedFactor = Math.Max(0f, speed) /
                            (
                                Math.Max(0f, speed) +
                                TireConfig.SpeedCoolingReferenceMps
                            );
        return 1f +
               (TireConfig.MaximumSpeedCoolingMultiplier - 1f) * speedFactor;
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
            state.OvertakeAssist,
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
    /// Downforce is shared front to rear in the same proportion as weight.
    /// Real cars are trimmed away from that, and that trim is a setup choice
    /// this model does not offer yet.
    /// </summary>
    private static WheelLoads CalculateWheelLoads(
        CarConfig config,
        float massKg,
        float longitudinalAcceleration,
        float lateralAcceleration,
        float speed,
        float downforceVelocityDeficit,
        float wakeDownforceLoss,
        float overtakeAssist,
        float normalGravity = Gravity
    )
    {
        float downforceAcceleration = EffectiveDownforceAccelPerSpeedSquared(
            config,
            downforceVelocityDeficit,
            wakeDownforceLoss,
            overtakeAssist
        ) * speed * speed;
        float totalLoad = massKg * (normalGravity + downforceAcceleration);
        float frontLoad = totalLoad * config.FrontStaticLoadShare;
        frontLoad -= massKg * longitudinalAcceleration * config.CenterOfGravityHeightMeters /
                     Math.Max(config.WheelBaseMeters, Epsilon);
        frontLoad = Math.Clamp(frontLoad, 0f, totalLoad);

        float rearLoad = totalLoad - frontLoad;
        float lateralTransfer = massKg * lateralAcceleration * config.CenterOfGravityHeightMeters /
                                Math.Max(config.TrackWidthMeters, Epsilon);
        float frontTransfer = lateralTransfer * frontLoad / Math.Max(totalLoad, Epsilon);
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

    /// <summary>
    /// The share of the wake's downforce disruption the tires actually feel.
    /// Whatever the overtake mode hands back to the load model must also stop
    /// shaking the car, or an assisted car would corner on recovered grip
    /// while being charged full dirty-air tire temperature for it.
    /// </summary>
    private static float EffectiveTireWakeDownforceLoss(
        CarState state,
        CarConfig config
    )
    {
        return state.WakeDownforceLoss *
               (1f - config.OvertakeAssistDownforceRecovery *
                Math.Clamp(state.OvertakeAssist, 0f, 1f));
    }

    private static float EffectiveDownforceAccelPerSpeedSquared(
        CarConfig config,
        float downforceVelocityDeficit,
        float wakeDownforceLoss,
        float overtakeAssist
    )
    {
        float metAir = 1f - Math.Clamp(downforceVelocityDeficit, 0f, 1f);
        float usableDownforce = 1f - Math.Clamp(wakeDownforceLoss, 0f, 1f);
        float wakeFactor = metAir * metAir * usableDownforce;
        // Hands back part of what the wake took, and nothing more: in clean air
        // the factor is already one, so this half of the mode cannot make a car
        // quicker than itself with nobody in front.
        float restored = wakeFactor +
                         config.OvertakeAssistDownforceRecovery *
                         Math.Clamp(overtakeAssist, 0f, 1f) *
                         (1f - wakeFactor);
        return MathF.Max(
            0f,
            config.DownforceAccelPerSpeedSquared * restored
        );
    }

    private static void ApplyWheelLoads(CarState state, WheelLoads loads)
    {
        state.FrontLeft.LoadN = loads.FrontLeft;
        state.FrontRight.LoadN = loads.FrontRight;
        state.RearLeft.LoadN = loads.RearLeft;
        state.RearRight.LoadN = loads.RearRight;
    }

    private static float CalculateAxleGripAccel(
        float massKg,
        TireConfig tires,
        TireState left,
        TireState right
    )
    {
        // Each wheel brings its own road with it. A car straddling the
        // white line has one side of an axle on tarmac and the other on
        // grass, and the axle is worth the sum of the two rather than the
        // better of them - so half a car off the road is half an axle's
        // grip gone, and the cost of running wide scales with how far.
        float leftForce = left.LoadN * CalculateTireMu(tires, left) *
                          MathF.Max(0f, left.SurfaceGrip);
        float rightForce = right.LoadN * CalculateTireMu(tires, right) *
                           MathF.Max(0f, right.SurfaceGrip);
        return (leftForce + rightForce) / Math.Max(massKg, Epsilon);
    }

    private static float CalculateTireMu(TireConfig tires, TireState tire)
    {
        float tempGrip = 1f;
        float idealLow = Math.Min(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float idealHigh = Math.Max(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        if (tire.SurfaceTempC < idealLow)
        {
            float coldDistance = idealLow - tire.SurfaceTempC;
            tempGrip -= coldDistance * coldDistance *
                        tires.ColdGripLossPerCSquared;
        }
        else if (tire.SurfaceTempC > idealHigh)
        {
            float hotDistance = tire.SurfaceTempC - idealHigh;
            tempGrip -= hotDistance * hotDistance *
                        tires.HotGripLossPerCSquared;
        }

        tempGrip -= Math.Max(0f, tire.CoreTempC - tires.CoreOverheatTempC) * tires.CoreOverheatGripLossPerC;

        float wear = Math.Clamp(tire.Wear, 0f, 1f);
        float cliffStart = Math.Clamp(tires.WearCliffStart, 0f, 1f);
        float cliffProgress = Math.Clamp(
            (wear - cliffStart) / Math.Max(1f - cliffStart, Epsilon),
            0f,
            1f
        );
        float smoothCliff = cliffProgress * cliffProgress *
                            (3f - 2f * cliffProgress);
        float wearGrip = 1f -
                         Math.Max(0f, tires.WearLinearGripLoss) * wear -
                         Math.Max(0f, tires.WearCliffGripLoss) * smoothCliff;
        return tires.BaseMu *
               Math.Clamp(tempGrip, MinimumTemperatureGripFactor, MaximumTemperatureGripFactor) *
               Math.Clamp(wearGrip, MinimumWearGripFactor, 1f);
    }

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
        float LongitudinalUse
    );

    private readonly record struct WheelLoads(
        float FrontLeft,
        float FrontRight,
        float RearLeft,
        float RearRight
    );
}
