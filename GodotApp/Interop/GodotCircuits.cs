using Godot;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.Interop;

/// <summary>
/// Points Core's circuit library at the project's own files. In an exported
/// game they are inside the pack, where only Godot can read them, so the
/// library reads through <see cref="FileAccess"/> from <c>res://</c>.
/// </summary>
public static class GodotCircuits
{
    public const string Root = "res://Circuits";
    public const string PrivateRoot = "res://private/circuits";

    private static bool _used;

    public static void Use()
    {
        if (_used)
            return;
        _used = true;
        CircuitLibrary.Roots.Clear();
        CircuitLibrary.Roots.Add(Root);
        CircuitLibrary.Roots.Add(PrivateRoot);
        CircuitLibrary.ReadText = path =>
            FileAccess.FileExists(path) ? FileAccess.GetFileAsString(path) : null;
    }
}
