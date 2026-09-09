using System;
using System.Collections.Generic;
using System.Linq;
using Godot;
using StintegyEVO.Core.Drivers.Learned;

namespace StintegyEVO.GodotApp.Race;

/// <summary>
/// Walks whatever is installed and hands the core what it found.
///
/// Two roots, in the order that decides conflicts: what ships with the
/// game, and what the player put there. Pack zero — the built-in car —
/// lives under the first and is otherwise an ordinary pack, declaring
/// itself through the same manifest and the same <c>drivers/</c>
/// convention a mod would use. Official content goes through the mod door
/// so the door stays in repair.
///
/// Nothing here decides anything. It enumerates and reads; which driver
/// wins is the catalogue's business, and keeping that split is what lets
/// the resolution rules be tested without a file system.
/// </summary>
public static class InstalledPacks
{
    private const string BuiltinRoot = "res://Assets/Packs";
    private const string UserRoot = "user://packs";

    public static DriverCatalog Scan()
    {
        List<DriverPackEntry> found = new();
        Collect(BuiltinRoot, ContentPackTier.Builtin, found);
        Collect(UserRoot, ContentPackTier.User, found);
        return DriverCatalog.Aggregate(found);
    }

    private static void Collect(
        string root,
        ContentPackTier tier,
        List<DriverPackEntry> into
    )
    {
        if (!DirAccess.DirExistsAbsolute(root))
            return;

        foreach (string name in DirAccess.GetDirectoriesAt(root).OrderBy(d => d))
        {
            string pack = $"{root}/{name}";
            string manifestPath = $"{pack}/pack.json";
            if (!Godot.FileAccess.FileExists(manifestPath))
                continue;

            ContentPackManifest manifest = DriverPackScanner.ReadManifest(
                Godot.FileAccess.GetFileAsString(manifestPath),
                manifestPath
            );

            string drivers = $"{pack}/drivers";
            if (!DirAccess.DirExistsAbsolute(drivers))
                continue;

            // Godot rewrites imported resources, so a shipped .nn may sit
            // on disk as .nn.import beside a remapped original. Taking the
            // stem of whatever ends in .nn covers both without the caller
            // having to know which.
            List<string> tracks = DirAccess.GetFilesAt(drivers)
                .Where(file => file.EndsWith(".nn", StringComparison.Ordinal))
                .Select(file => file[..^3])
                .ToList();
            if (tracks.Count == 0)
                continue;

            into.AddRange(DriverPackScanner.Entries(
                manifest,
                tier,
                tracks,
                track =>
                {
                    string sidecar = $"{drivers}/{track}.json";
                    return Godot.FileAccess.FileExists(sidecar)
                        ? Godot.FileAccess.GetFileAsString(sidecar)
                        : null;
                },
                track => Godot.FileAccess.GetFileAsBytes($"{drivers}/{track}.nn")
            ));
        }
    }
}
