using System;

namespace StintegyEVO.Core.Drivers;

/// <summary>A participant's identity and ratings, paired with an externally supplied controller.</summary>
public sealed class Driver
{
    public Driver(DriverProfile profile, IDriverController controller)
    {
        Profile = profile ?? throw new ArgumentNullException(nameof(profile));
        Controller = controller ?? throw new ArgumentNullException(nameof(controller));
    }

    public DriverProfile Profile { get; }
    public IDriverController Controller { get; }
}
