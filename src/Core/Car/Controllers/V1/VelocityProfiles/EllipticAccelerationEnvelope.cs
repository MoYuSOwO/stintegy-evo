using System;
using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public sealed class EllipticAccelerationEnvelope(
    float maxLongitudinalAccel,
    float maxLongitudinalDecel,
    float maxLateralAccel,
    float safetyFactor = 1f
) : IAccelerationEnvelope
{
    private readonly float _maxLongitudinalAccel = Math.Max(0f, maxLongitudinalAccel);
    private readonly float _maxLongitudinalDecel = Math.Max(0f, maxLongitudinalDecel);
    private readonly float _maxLateralAccel = Math.Max(0f, maxLateralAccel);
    private readonly float _safetyFactor = Mathf.Clamp(safetyFactor, 0.05f, 1f);

    public void GetLateralBounds(float speed, out float minLateralAccel, out float maxLateralAccel)
    {
        float limit = _maxLateralAccel * _safetyFactor;
        minLateralAccel = -limit;
        maxLateralAccel = limit;
    }

    public void GetLongitudinalBounds(
        float lateralAccel,
        float speed,
        out float minLongitudinalAccel,
        out float maxLongitudinalAccel
    )
    {
        float latLimit = Math.Max(_maxLateralAccel * _safetyFactor, 1e-4f);
        float latRatio = Mathf.Clamp(Math.Abs(lateralAccel) / latLimit, 0f, 1f);
        float available = Mathf.Sqrt(Math.Max(0f, 1f - latRatio * latRatio));
        maxLongitudinalAccel = _maxLongitudinalAccel * _safetyFactor * available;
        minLongitudinalAccel = -_maxLongitudinalDecel * _safetyFactor * available;
    }
}
