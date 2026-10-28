using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace StintegyEVO.Core.Mods;

/// <summary>
/// The game's mods: where it looks for them and the one registry they
/// load into, loaded the first time anything asks for it. The game, its
/// tools and its tests all take their cars, circuits and tyres from here,
/// so there is one way in for everything and one definition of each.
/// </summary>
public static class GameMods
{
    /// <summary>The folder the game's own mods are in, beside the game.</summary>
    public const string FolderName = "mods";

    /// <summary>Folders to add to <see cref="Roots"/>, as the game's own: a path list.</summary>
    public const string Environment = "STINTEGY_MODS";

    private static readonly object Gate = new();
    private static ModRegistry? _registry;

    /// <summary>
    /// Where to look, the game's own folders and the player's. Set before
    /// <see cref="Registry"/> is first read: a host that knows a folder the
    /// game cannot find for itself, the player's, adds it here.
    /// </summary>
    public static List<ModRoot> Roots { get; } = DefaultRoots();

    /// <summary>Every mod, loaded once; refused as a whole when any of them is wrong (<see cref="ModLoadException"/>).</summary>
    public static ModRegistry Registry
    {
        get
        {
            lock (Gate)
                return _registry ??= ModLoader.Load(Roots);
        }
    }

    // The folders named by STINTEGY_MODS, then the first mods/ folder found
    // beside the game or above it, and above where it was started from: the
    // repository's own while it is being made.
    private static List<ModRoot> DefaultRoots()
    {
        var roots = new List<ModRoot>();
        if (System.Environment.GetEnvironmentVariable(Environment) is { Length: > 0 } configured)
            roots.AddRange(configured.Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries)
                .Select(folder => new ModRoot(folder, ModOrigin.Game)));
        foreach (string start in new[] { AppContext.BaseDirectory, System.Environment.CurrentDirectory })
        {
            for (DirectoryInfo? dir = new(start); dir is not null; dir = dir.Parent)
            {
                string mods = Path.Combine(dir.FullName, FolderName);
                if (!HoldsMods(mods))
                    continue;
                if (!roots.Any(r => Path.GetFullPath(r.Folder) == Path.GetFullPath(mods)))
                    roots.Add(new ModRoot(mods, ModOrigin.Game));
                break;
            }
        }
        return roots;
    }

    // A folder called mods with a mod in it: not merely one called Mods, as
    // a source folder may be on a disk that does not tell the two apart.
    private static bool HoldsMods(string folder) =>
        Directory.Exists(folder) &&
        (Directory.EnumerateFiles(folder, "*" + ModFormat.Extension).Any() ||
         Directory.EnumerateDirectories(folder).Any(d => File.Exists(Path.Combine(d, ModFormat.ManifestFile))));
}
