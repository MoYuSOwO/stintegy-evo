using System.Collections.Generic;
using System.Text.Json;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// What the game's own prop numbers mean.
///
/// A circuit's plan may ask for prop 2 instead of naming a file, and this
/// is where 2 is answered. The indirection is the point: the numbers are
/// what a plan is written against, so the props that ship with the game
/// can be renamed, re-modelled or replaced outright without touching a
/// single circuit. A plan that brings its own asset skips all of this and
/// names its file.
/// </summary>
public static class SceneryCatalogue
{
    /// <summary>Number to prop name, as the catalogue file lists them.</summary>
    public static IEnumerable<KeyValuePair<string, string>> Parse(string json)
    {
        JsonDocumentOptions options = new()
        {
            CommentHandling = JsonCommentHandling.Skip,
            AllowTrailingCommas = true
        };
        using JsonDocument document = JsonDocument.Parse(json, options);
        if (!document.RootElement.TryGetProperty("props", out JsonElement props))
            yield break;
        foreach (JsonProperty entry in props.EnumerateObject())
        {
            string? name = entry.Value.GetString();
            if (!string.IsNullOrWhiteSpace(name))
                yield return new KeyValuePair<string, string>(entry.Name, name);
        }
    }
}
