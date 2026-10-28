using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Text;

namespace StintegyEVO.Core.Mods;

/// <summary>
/// One mod's files, by their path in the mod (<c>data/tyres/formula_a.json</c>,
/// forward slashes), read the same whether the mod is packed or a folder.
/// Read only: nothing is ever written into a mod.
/// </summary>
public interface IModResources
{
    /// <summary>Where the mod is: its packed file or its folder.</summary>
    string Location { get; }

    /// <summary>Whether the mod has a file at <paramref name="path"/>.</summary>
    bool Exists(string path);

    /// <summary>The file at <paramref name="path"/>, read whole into a stream of its own.</summary>
    Stream Open(string path);

    /// <summary>Every file in the mod, by path.</summary>
    IEnumerable<string> Files { get; }
}

public static class ModResources
{
    /// <summary>The files of the mod at <paramref name="location"/>: a packed mod or a folder.</summary>
    public static IModResources At(string location) =>
        Directory.Exists(location) ? new FolderModResources(location) : new ZipModResources(location);

    public static byte[] ReadAllBytes(this IModResources resources, string path)
    {
        using Stream stream = resources.Open(path);
        using var copy = new MemoryStream();
        stream.CopyTo(copy);
        return copy.ToArray();
    }

    public static string ReadAllText(this IModResources resources, string path) =>
        Encoding.UTF8.GetString(resources.ReadAllBytes(path));

    /// <summary>
    /// <paramref name="path"/> joined to <paramref name="folder"/>, both
    /// paths in a mod (<c>data/circuits/silverstone</c> and <c>track.csv</c>).
    /// </summary>
    public static string Join(string folder, string path) =>
        folder.Length == 0 ? path : $"{folder.TrimEnd('/', '\\')}/{path}";

    /// <summary>
    /// A path in a mod as the mod is read: forward slashes, no leading
    /// <c>./</c>, nothing that climbs out of the mod or starts anywhere but
    /// at its top.
    /// </summary>
    public static string Normalize(string path)
    {
        ArgumentNullException.ThrowIfNull(path);
        string slashed = path.Replace('\\', '/');
        if (slashed.StartsWith('/') || Path.IsPathRooted(path))
            throw new ArgumentException($"'{path}' is not a path in a mod: it starts outside it.", nameof(path));
        var parts = new List<string>();
        foreach (string part in slashed.Split('/'))
        {
            if (part.Length == 0 || part == ".")
                continue;
            if (part == "..")
                throw new ArgumentException($"'{path}' is not a path in a mod: it climbs out of it.", nameof(path));
            parts.Add(part);
        }
        if (parts.Count == 0)
            throw new ArgumentException("A path in a mod names something in it.", nameof(path));
        return string.Join('/', parts);
    }
}

internal sealed class FolderModResources : IModResources
{
    private readonly string _root;

    public FolderModResources(string folder)
    {
        _root = Path.GetFullPath(folder);
        Location = folder;
    }

    public string Location { get; }

    public bool Exists(string path) => File.Exists(Full(path));

    public Stream Open(string path)
    {
        string full = Full(path);
        if (!File.Exists(full))
            throw new FileNotFoundException($"The mod at {Location} has no {ModResources.Normalize(path)}.", full);
        return new MemoryStream(File.ReadAllBytes(full), writable: false);
    }

    public IEnumerable<string> Files =>
        Directory.EnumerateFiles(_root, "*", SearchOption.AllDirectories)
            .Select(f => Path.GetRelativePath(_root, f).Replace('\\', '/'))
            .Order(StringComparer.Ordinal);

    private string Full(string path) => Path.Combine(_root, ModResources.Normalize(path));
}

/// <summary>A packed mod, read in place: its entries are never unpacked to disk.</summary>
internal sealed class ZipModResources : IModResources
{
    private readonly ZipArchive _zip;
    private readonly Dictionary<string, ZipArchiveEntry> _entries = new(StringComparer.Ordinal);
    private readonly object _gate = new();

    public ZipModResources(string file)
    {
        Location = file;
        // Held open for the game's life: a mod is loaded once and never unloaded.
        _zip = ZipFile.OpenRead(file);
        foreach (ZipArchiveEntry entry in _zip.Entries)
        {
            if (entry.FullName.EndsWith('/') || entry.FullName.EndsWith('\\'))
                continue;
            string path;
            try
            {
                path = ModResources.Normalize(entry.FullName);
            }
            catch (ArgumentException error)
            {
                throw new InvalidDataException($"{file}: {error.Message}", error);
            }
            if (!_entries.TryAdd(path, entry))
                throw new InvalidDataException($"{file} holds {path} twice.");
        }
    }

    public string Location { get; }

    public bool Exists(string path) => _entries.ContainsKey(ModResources.Normalize(path));

    public Stream Open(string path)
    {
        string normalized = ModResources.Normalize(path);
        if (!_entries.TryGetValue(normalized, out ZipArchiveEntry? entry))
            throw new FileNotFoundException($"The mod at {Location} has no {normalized}.", normalized);
        // An archive reads one entry at a time; each caller gets its own copy.
        lock (_gate)
        {
            using Stream inside = entry.Open();
            var copy = new MemoryStream();
            inside.CopyTo(copy);
            copy.Position = 0;
            return copy;
        }
    }

    public IEnumerable<string> Files => _entries.Keys.Order(StringComparer.Ordinal);
}
