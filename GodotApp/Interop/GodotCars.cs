using Godot;
using StintegyEVO.Core.Cars;

namespace StintegyEVO.GodotApp.Interop;

/// <summary>
/// The game's way into <see cref="CarLibrary"/>: the same library, with the
/// one place only the engine knows added to where it looks -- the player's
/// own <c>user://data/</c>, where a pack someone downloads goes, the way a
/// scenery mod goes in <c>user://mods/</c>. Cars the game ships beside its
/// executable are found first, so nobody's pack can stand in for them.
/// </summary>
public static class GodotCars
{
    private static bool _rooted;

    public static LoadedCar Load(string id)
    {
        if (!_rooted)
        {
            string user = ProjectSettings.GlobalizePath("user://data");
            if (!CarLibrary.Roots.Contains(user))
                CarLibrary.Roots.Add(user);
            _rooted = true;
        }
        return CarLibrary.Load(id);
    }
}
