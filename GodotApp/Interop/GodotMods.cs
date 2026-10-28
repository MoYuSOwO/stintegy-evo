using Godot;
using StintegyEVO.Core.Mods;

namespace StintegyEVO.GodotApp.Interop;

/// <summary>
/// The game's way into the mods: the one registry everything else uses,
/// with the one place only the engine knows added to where it looks -- the
/// player's own <c>user://mods/</c>, where a mod someone downloads goes. The
/// game's own mods, beside it, are found first, so nobody's mod can stand in
/// for them.
/// </summary>
public static class GodotMods
{
    private static readonly object Gate = new();
    private static bool _rooted;

    public static ModRegistry Registry
    {
        get
        {
            lock (Gate)
            {
                if (!_rooted)
                {
                    GameMods.Roots.Add(new ModRoot(ProjectSettings.GlobalizePath("user://mods"), ModOrigin.User));
                    _rooted = true;
                }
            }
            return GameMods.Registry;
        }
    }
}
