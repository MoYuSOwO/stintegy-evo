using System;
using System.Collections.Generic;
using System.IO;

namespace StintegyEVO.Core.Content;

/// <summary>
/// Where the game looks for what it loads -- car packs, tyre packs -- each
/// under <c>&lt;root&gt;/&lt;namespace&gt;/&lt;kind&gt;/&lt;name&gt;/</c>.
/// The first root that has a thing is where it comes from, so the game's
/// own data, beside it, is found before anyone else's.
/// </summary>
public static class DataFolders
{
    public const string DataFolder = "data";

    /// <summary>Data folders to look in, in order.</summary>
    public static List<string> Roots { get; } = DefaultRoots();

    /// <summary>The folder holding <paramref name="id"/> of this kind, from the first root that has it; none if no root does.</summary>
    public static string? Find(ResourceId id, string kind, string marker)
    {
        foreach (string root in Roots)
        {
            string folder = Path.Combine(root, id.Namespace, kind, id.Name);
            if (File.Exists(Path.Combine(folder, marker)))
                return folder;
        }
        return null;
    }

    /// <summary>Every id of this kind the roots hold, each once, in root order.</summary>
    public static IEnumerable<ResourceId> Available(string kind, string marker)
    {
        var seen = new HashSet<ResourceId>();
        foreach (string root in Roots)
        {
            if (!Directory.Exists(root))
                continue;
            foreach (string space in Directory.EnumerateDirectories(root))
            {
                string kindFolder = Path.Combine(space, kind);
                if (!Directory.Exists(kindFolder))
                    continue;
                foreach (string folder in Directory.EnumerateDirectories(kindFolder))
                {
                    if (!File.Exists(Path.Combine(folder, marker)))
                        continue;
                    if (ResourceId.TryParse($"{Path.GetFileName(space)}:{Path.GetFileName(folder)}", out ResourceId id) &&
                        seen.Add(id))
                        yield return id;
                }
            }
        }
    }

    private static List<string> DefaultRoots()
    {
        var roots = new List<string>();
        if (Environment.GetEnvironmentVariable("STINTEGY_DATA") is { Length: > 0 } configured)
            roots.AddRange(configured.Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries));
        foreach (string start in new[] { AppContext.BaseDirectory, Environment.CurrentDirectory })
        {
            for (DirectoryInfo? dir = new(start); dir is not null; dir = dir.Parent)
            {
                string data = Path.Combine(dir.FullName, DataFolder);
                if (!Directory.Exists(data))
                    continue;
                if (!roots.Contains(data))
                    roots.Add(data);
                break;
            }
        }
        return roots;
    }
}
