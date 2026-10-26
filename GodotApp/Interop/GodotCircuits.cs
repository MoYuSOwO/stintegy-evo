using Godot;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.Interop;

/// <summary>
/// Points Core's circuit library at the game's own data and the player's.
/// In an exported game the game's own circuits are inside the pack, where
/// only Godot can read them, so the library reads through
/// <see cref="FileAccess"/>: from <c>res://data</c> first, then from
/// <c>user://data</c>, where a downloaded circuit goes as a downloaded car
/// does, so nobody's pack can stand in for the game's own.
/// </summary>
public static class GodotCircuits
{
    public const string Root = "res://data";
    public const string UserRoot = "user://data";

    private static bool _used;

    public static void Use()
    {
        if (_used)
            return;
        _used = true;
        CircuitLibrary.Roots.Clear();
        CircuitLibrary.Roots.Add(Root);
        CircuitLibrary.Roots.Add(UserRoot);
        CircuitLibrary.ReadText = path =>
            FileAccess.FileExists(path) ? FileAccess.GetFileAsString(path) : null;
    }
}
