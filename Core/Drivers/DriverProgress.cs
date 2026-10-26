using System;
using System.IO;
using StintegyEVO.Core.Content;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Whose experience of what: a person, driving with a driver pack, in a
/// car, at a circuit. The person is the save's own name for them, held to
/// the same letters an id's parts are, so it is a folder's name everywhere.
/// </summary>
public readonly record struct DriverProgressKey
{
    public DriverProgressKey(string person, ResourceId driver, ResourceId car, ResourceId circuit)
    {
        if (!ResourceId.IsPart(person))
            throw new ArgumentException($"'{person}' cannot name a person: lower case letters, digits and underscores only.", nameof(person));
        Person = person;
        Driver = driver;
        Car = car;
        Circuit = circuit;
    }

    public string Person { get; }
    public ResourceId Driver { get; }
    public ResourceId Car { get; }
    public ResourceId Circuit { get; }
}

/// <summary>
/// Where people's experience is kept between sessions: the save's to
/// decide. What the experience is, the driver pack's: the store keeps it as
/// the driver wrote it.
/// </summary>
public interface IDriverProgressStore
{
    string? Load(DriverProgressKey key);
    void Save(DriverProgressKey key, string progress);
}

/// <summary>
/// People's experience as files, every id split into its namespace and its
/// name:
/// <c>{root}/{person}/{driver namespace}/{driver name}/{car namespace}/{car name}/{circuit namespace}/{circuit name}.json</c>.
/// </summary>
public sealed class DriverProgressFolder(string root) : IDriverProgressStore
{
    public string? Load(DriverProgressKey key)
    {
        string path = PathOf(key);
        return File.Exists(path) ? File.ReadAllText(path) : null;
    }

    public void Save(DriverProgressKey key, string progress)
    {
        string path = PathOf(key);
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path, progress);
    }

    private string PathOf(DriverProgressKey key) =>
        Path.Combine(root, key.Person,
            key.Driver.Namespace, key.Driver.Name,
            key.Car.Namespace, key.Car.Name,
            key.Circuit.Namespace, key.Circuit.Name + ".json");
}

/// <summary>One person's experience of one car at one circuit, as a seated driver reads and writes it.</summary>
public sealed class DriverProgress(IDriverProgressStore store, DriverProgressKey key)
{
    public DriverProgressKey Key { get; } = key;

    public string? Read() => store.Load(Key);

    public void Write(string progress) => store.Save(Key, progress);
}
