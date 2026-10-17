using System;

namespace StintegyEVO.Core.Content;

/// <summary>
/// The name of a thing the game loads -- a car, a circuit -- with whose it
/// is: <c>official:formula_a_mixed</c>, <c>someone:their_car</c>. A bare
/// name is the game's own (<see cref="Official"/>).
///
/// Lower case letters, digits and underscores only, on both sides, so the
/// same id names the same folder on every file system.
/// </summary>
public readonly record struct ResourceId
{
    public const string Official = "official";
    private const char Separator = ':';

    public ResourceId(string space, string name)
    {
        Namespace = Check(space, nameof(space));
        Name = Check(name, nameof(name));
    }

    public string Namespace { get; }
    public string Name { get; }

    public static ResourceId Parse(string text)
    {
        ArgumentNullException.ThrowIfNull(text);
        int at = text.IndexOf(Separator);
        return at < 0
            ? new ResourceId(Official, text)
            : new ResourceId(text[..at], text[(at + 1)..]);
    }

    public static bool TryParse(string? text, out ResourceId id)
    {
        try
        {
            id = Parse(text!);
            return true;
        }
        catch (ArgumentException)
        {
            id = default;
            return false;
        }
    }

    public override string ToString() => $"{Namespace}{Separator}{Name}";

    private static string Check(string part, string what)
    {
        if (string.IsNullOrEmpty(part))
            throw new ArgumentException("An id needs a namespace and a name.", what);
        foreach (char c in part)
            if (!(c is >= 'a' and <= 'z' or >= '0' and <= '9' or '_'))
                throw new ArgumentException(
                    $"'{part}' is not an id: lower case letters, digits and underscores only.", what);
        return part;
    }
}
