namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public interface IAccelerationEnvelope
{
    void GetLateralBounds(float speed, out float minLateralAccel, out float maxLateralAccel);
    void GetLongitudinalBounds(
        float lateralAccel,
        float speed,
        out float minLongitudinalAccel,
        out float maxLongitudinalAccel
    );
}
