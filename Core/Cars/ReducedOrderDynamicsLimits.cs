namespace StintegyEVO.Core.Cars;

/// <summary>
/// Shared numerical limits for the reduced-order runtime dynamics and its
/// lightweight motion predictor. Keeping them together prevents silent model
/// drift when either side is tuned.
/// </summary>
internal static class ReducedOrderDynamicsLimits
{
    public const float MaximumBodySideslipRadians = 0.174532925f;
    public const float MaximumYawAccelerationRadiansPerSecondSquared = 2f;
    public const float MaximumYawRateRadiansPerSecond = 2.5f;
}
