using System;
using Godot;
using StintegyEVO.Core.Car.Components;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public sealed class VehicleAccelerationEnvelope(CarLogic carLogic, float safetyFactor = 0.98f) : IAccelerationEnvelope
{
    private readonly float _safetyFactor = Mathf.Clamp(safetyFactor, 0.05f, 1f);

    public void GetLateralBounds(float speed, out float minLateralAccel, out float maxLateralAccel)
    {
        float lateralLimit = LateralAccelLimit(speed) * _safetyFactor;
        minLateralAccel = -lateralLimit;
        maxLateralAccel = lateralLimit;
    }

    public void GetLongitudinalBounds(
        float lateralAccel,
        float speed,
        out float minLongitudinalAccel,
        out float maxLongitudinalAccel
    )
    {
        float safeSpeed = Math.Max(0f, speed);
        float lateralLimit = Math.Max(LateralAccelLimit(safeSpeed), 1e-4f);
        float longGripLimit = LongitudinalGripAccelLimit(safeSpeed);
        float lateralUsage = Mathf.Clamp(Math.Abs(lateralAccel) / lateralLimit, 0f, 1f);
        float remainingLongitudinalGrip = longGripLimit * Mathf.Sqrt(Math.Max(0f, 1f - lateralUsage * lateralUsage));
        float dragAccel = DragAccel(safeSpeed);
        float driveAccel = carLogic.Config.Power.CalcMaxDriveForceAtSpeed(Math.Max(safeSpeed, 0.1f)) /
                           Math.Max(carLogic.Config.Chassis.DryMass, 1f);
        float brakeAccel = carLogic.Config.Power.MaxBrakeForce / Math.Max(carLogic.Config.Chassis.DryMass, 1f);

        maxLongitudinalAccel = Math.Min(remainingLongitudinalGrip, driveAccel) * _safetyFactor - dragAccel;
        minLongitudinalAccel = -Math.Min(remainingLongitudinalGrip, brakeAccel) * _safetyFactor - dragAccel;

        if (maxLongitudinalAccel < minLongitudinalAccel)
            maxLongitudinalAccel = minLongitudinalAccel;
    }

    private float LateralAccelLimit(float speed)
    {
        return NormalForce(speed) * AverageLatMu() / Math.Max(carLogic.Config.Chassis.DryMass, 1f);
    }

    private float LongitudinalGripAccelLimit(float speed)
    {
        return NormalForce(speed) * AverageLongMu() / Math.Max(carLogic.Config.Chassis.DryMass, 1f);
    }

    private float NormalForce(float speed)
    {
        AeroOutput aero = carLogic.Aero.CalculateAero(Math.Max(0f, speed));
        return carLogic.Config.Chassis.DryMass * GeomUtil.g + aero.DownforceFront + aero.DownforceRear;
    }

    private float DragAccel(float speed)
    {
        return carLogic.Aero.CalculateAero(Math.Max(0f, speed)).DragForce /
               Math.Max(carLogic.Config.Chassis.DryMass, 1f);
    }

    private float AverageLatMu()
    {
        return AverageMu(tire => tire.CurrLatPeakFriction);
    }

    private float AverageLongMu()
    {
        return AverageMu(tire => tire.CurrLongPeakFriction);
    }

    private float AverageMu(Func<TireComponent, float> selector)
    {
        float sum = 0f;
        int count = 0;
        foreach (TireComponent tire in carLogic.Tires)
        {
            sum += Math.Max(0f, selector(tire));
            count++;
        }

        return count == 0 ? 1f : sum / count;
    }
}
