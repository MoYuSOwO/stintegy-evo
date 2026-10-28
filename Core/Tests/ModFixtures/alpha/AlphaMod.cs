using System;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Mods;

namespace Alpha;

/// <summary>What alpha offers other mods' code.</summary>
public static class AlphaParts
{
    public const string DriverName = "steady";
}

public sealed class AlphaMod : IMod
{
    public void Register(ModContext context)
    {
        context.Registry.Drivers.Register(AlphaParts.DriverName, new SteadyDriver());
        context.Log.Info("steady is in");
    }
}

/// <summary>A mod that tries to register in another's namespace.</summary>
public sealed class Squatter : IMod
{
    public void Register(ModContext context) =>
        context.Registry.Drivers.Register($"alpha:{AlphaParts.DriverName}", new SteadyDriver());
}

public sealed class SteadyDriver : IDriverDefinition
{
    public CarNeeds Needs => CarNeeds.None;

    public IDriverController Seat(DriverSeat seat) => throw new NotSupportedException("A registry test's driver drives nothing.");
}
