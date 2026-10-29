using Alpha;
using StintegyEVO.Core.Mods;

namespace Beta;

/// <summary>Built on alpha's code, and reading its own files.</summary>
public sealed class BetaMod : IMod
{
    public void Register(ModContext context)
    {
        string name = context.Resources.ReadAllText("data/driver_name.txt").Trim();
        context.Registry.Drivers.Register(name, new SteadyDriver());
        context.Log.Info($"follows {AlphaParts.DriverName}");
    }
}
