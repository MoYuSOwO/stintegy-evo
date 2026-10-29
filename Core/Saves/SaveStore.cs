using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using StintegyEVO.Core.Content;

namespace StintegyEVO.Core.Saves;

/// <summary>
/// Whose saved data and about what: the part that owns it, a driver by its
/// id, and what it is about -- a person, a car, a circuit, any of them or
/// none. The owner says what it wants kept; where it is kept is the game's.
/// </summary>
public readonly record struct SaveKey
{
    /// <param name="owner">The part whose data it is: a driver's id, say.</param>
    /// <param name="person">The person it is about, by the save's own name for them: lower case letters, digits and underscores.</param>
    /// <param name="car">The car it is about.</param>
    /// <param name="circuit">The circuit it is about.</param>
    public SaveKey(ResourceId owner, string? person = null, ResourceId? car = null, ResourceId? circuit = null)
    {
        if (person is not null && !ResourceId.IsPart(person))
            throw new ArgumentException($"'{person}' cannot name a person: lower case letters, digits and underscores only.", nameof(person));
        Owner = owner;
        Person = person;
        Car = car;
        Circuit = circuit;
    }

    public ResourceId Owner { get; }
    public string? Person { get; }
    public ResourceId? Car { get; }
    public ResourceId? Circuit { get; }
}

/// <summary>
/// Where what runs in the game keeps what it learns between sessions. A part
/// does not choose a path: it says whose data and about what
/// (<see cref="SaveKey"/>), and is handed a slot to read and write. Mods are
/// read only; everything written while the game runs goes here.
/// </summary>
public interface ISaveStore
{
    ISaveSlot Slot(SaveKey key);
}

/// <summary>
/// One owner's data about one subject: files by name, in whatever form the
/// owner keeps them. What is in them is the owner's; the store keeps them as
/// they were written.
/// </summary>
public interface ISaveSlot
{
    SaveKey Key { get; }

    /// <summary>The file named <paramref name="name"/>, or none when there is none.</summary>
    string? Read(string name);

    /// <summary>Writes the file named <paramref name="name"/>, whole, in place of what was there.</summary>
    void Write(string name, string text);
}

/// <summary>
/// A save as files on disk, every id split into its namespace and its name
/// and a part of the subject that is not given written as <c>-</c>, which no
/// id can be:
/// <c>{root}/{owner namespace}/{owner name}/{person}/{car namespace}/{car name}/{circuit namespace}/{circuit name}/{file}</c>.
/// A file is written beside itself first and moved into place, so a write cut
/// short leaves the old one.
/// </summary>
public sealed class SaveFolder(string root) : ISaveStore
{
    private const string Absent = "-";

    public string Root { get; } = root;

    public ISaveSlot Slot(SaveKey key) => new FolderSlot(key, Path.Combine(
        Root,
        key.Owner.Namespace, key.Owner.Name,
        key.Person ?? Absent,
        key.Car?.Namespace ?? Absent, key.Car?.Name ?? Absent,
        key.Circuit?.Namespace ?? Absent, key.Circuit?.Name ?? Absent));

    private sealed class FolderSlot(SaveKey key, string folder) : ISaveSlot
    {
        public SaveKey Key { get; } = key;

        public string? Read(string name)
        {
            string path = Path.Combine(folder, SaveFiles.Checked(name));
            return File.Exists(path) ? File.ReadAllText(path) : null;
        }

        public void Write(string name, string text)
        {
            string path = Path.Combine(folder, SaveFiles.Checked(name));
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            string written = path + ".writing";
            File.WriteAllText(written, text);
            File.Move(written, path, overwrite: true);
        }
    }
}

/// <summary>
/// A save kept in memory and lost with it: for a quick race, an offline
/// tool, a test -- wherever nothing should outlive the session.
/// </summary>
public sealed class ThrowawaySave : ISaveStore
{
    private readonly Dictionary<(SaveKey Key, string Name), string> _files = new();
    private readonly object _gate = new();

    public ISaveSlot Slot(SaveKey key) => new MemorySlot(this, key);

    private sealed class MemorySlot(ThrowawaySave save, SaveKey key) : ISaveSlot
    {
        public SaveKey Key { get; } = key;

        public string? Read(string name)
        {
            lock (save._gate)
                return save._files.TryGetValue((Key, SaveFiles.Checked(name)), out string? text) ? text : null;
        }

        public void Write(string name, string text)
        {
            ArgumentNullException.ThrowIfNull(text);
            lock (save._gate)
                save._files[(Key, SaveFiles.Checked(name))] = text;
        }
    }
}

internal static class SaveFiles
{
    // A file an owner names is inside its slot and nowhere else.
    public static string Checked(string name)
    {
        if (string.IsNullOrWhiteSpace(name) || Path.IsPathRooted(name) ||
            name.Replace('\\', '/').Split('/').Any(part => part is "" or "." or ".." || part.EndsWith(".writing", StringComparison.Ordinal)))
            throw new ArgumentException($"'{name}' is not a file in a save slot.", nameof(name));
        return name.Replace('\\', '/');
    }
}
