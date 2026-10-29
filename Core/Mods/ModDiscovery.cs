using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace StintegyEVO.Core.Mods;

/// <summary>Whose a folder of mods is: the game's own, or the player's.</summary>
public enum ModOrigin
{
    Game,
    User
}

/// <summary>A folder holding mods, packed (<see cref="ModFormat.Extension"/>) or as folders.</summary>
public sealed record ModRoot(string Folder, ModOrigin Origin);

/// <summary>A mod found, read and not yet loaded.</summary>
public sealed record FoundMod(ModManifest Manifest, ModVersion Version, ModOrigin Origin, IModResources Resources)
{
    public string Id => Manifest.Id;
    public string Location => Resources.Location;
}

/// <summary>
/// Everything wrong with a set of mods, found before the game starts rather
/// than in a race. Each line names the mod it is about.
/// </summary>
public sealed class ModLoadException : Exception
{
    public ModLoadException(IReadOnlyList<string> problems)
        : base("The mods cannot be loaded:" + Environment.NewLine + string.Join(Environment.NewLine, problems.Select(p => "  " + p)))
    {
        Problems = problems;
    }

    public IReadOnlyList<string> Problems { get; }
}

public static class ModDiscovery
{
    /// <summary>
    /// Every mod in <paramref name="roots"/>, one per id. The game's own
    /// come first and win: a player's mod with the same id as one of the
    /// game's is not loaded, and <paramref name="notices"/> says so. A mod
    /// is a packed file or a folder named by its id, with its
    /// <c>mod.json</c> at its top.
    /// </summary>
    public static IReadOnlyList<FoundMod> Find(IEnumerable<ModRoot> roots, ICollection<string> notices)
    {
        var problems = new List<string>();
        var found = new Dictionary<string, FoundMod>(StringComparer.Ordinal);
        // The game's roots before the player's, each kind in the order given.
        foreach (ModRoot root in roots.OrderBy(r => r.Origin == ModOrigin.Game ? 0 : 1))
        {
            if (!Directory.Exists(root.Folder))
                continue;
            var inRoot = new Dictionary<string, string>(StringComparer.Ordinal);
            foreach (string location in Candidates(root.Folder))
            {
                FoundMod? mod = Read(location, root.Origin, problems);
                if (mod is null)
                    continue;
                if (inRoot.TryGetValue(mod.Id, out string? other))
                {
                    problems.Add($"{mod.Id}: found twice in {root.Folder}, at {other} and {location}; keep one.");
                    continue;
                }
                inRoot[mod.Id] = location;
                if (found.TryGetValue(mod.Id, out FoundMod? first))
                {
                    notices.Add($"{mod.Id}: {location} is not loaded; {first.Location} has the same id and comes first.");
                    continue;
                }
                found[mod.Id] = mod;
            }
        }
        if (problems.Count > 0)
            throw new ModLoadException(problems);
        return found.Values.ToList();
    }

    private static IEnumerable<string> Candidates(string root)
    {
        IEnumerable<string> packed = Directory.EnumerateFiles(root, "*" + ModFormat.Extension);
        IEnumerable<string> folders = Directory.EnumerateDirectories(root)
            .Where(d => File.Exists(Path.Combine(d, ModFormat.ManifestFile)));
        return packed.Concat(folders).Order(StringComparer.Ordinal);
    }

    private static FoundMod? Read(string location, ModOrigin origin, List<string> problems)
    {
        string named = Path.GetFileName(location.TrimEnd('/', '\\'));
        if (named.EndsWith(ModFormat.Extension, StringComparison.Ordinal))
            named = named[..^ModFormat.Extension.Length];
        IModResources resources;
        ModManifest manifest;
        try
        {
            resources = ModResources.At(location);
            if (!resources.Exists(ModFormat.ManifestFile))
            {
                problems.Add($"{named}: {location} has no {ModFormat.ManifestFile} at its top.");
                return null;
            }
            manifest = ModManifest.Parse(resources.ReadAllText(ModFormat.ManifestFile));
        }
        catch (Exception error) when (error is IOException or InvalidDataException or JsonException)
        {
            problems.Add($"{named}: {location} cannot be read: {error.Message}");
            return null;
        }
        string[] wrong = manifest.Problems().ToArray();
        if (manifest.Id != named)
            wrong = [.. wrong, $"calls itself '{manifest.Id}' in {location}; a mod's file or folder is named by its id"];
        if (wrong.Length > 0)
        {
            problems.AddRange(wrong.Select(w => $"{named}: {w}."));
            return null;
        }
        return new FoundMod(manifest, ModVersion.Parse(manifest.Version), origin, resources);
    }

    /// <summary>
    /// <paramref name="mods"/> with every mod after the ones it depends on,
    /// and otherwise by id. A dependency that is not there, is the wrong
    /// version or leads back round to the mod is refused here.
    /// </summary>
    public static IReadOnlyList<FoundMod> Order(IReadOnlyList<FoundMod> mods)
    {
        var byId = mods.ToDictionary(m => m.Id, StringComparer.Ordinal);
        var problems = new List<string>();
        foreach (FoundMod mod in mods)
            foreach ((string needed, string versions) in mod.Manifest.Depends)
            {
                ModVersionRequirement wanted = ModVersionRequirement.TryParse(versions, out ModVersionRequirement parsed)
                    ? parsed : throw new InvalidOperationException("Manifests are checked when they are found.");
                if (needed == mod.Id)
                    problems.Add($"{mod.Id}: depends on itself.");
                else if (!byId.TryGetValue(needed, out FoundMod? there))
                    problems.Add($"{mod.Id}: needs {needed} {wanted}, which is not installed.");
                else if (!wanted.IsMetBy(there.Version))
                    problems.Add($"{mod.Id}: needs {needed} {wanted}; {there.Version} is installed.");
            }
        if (problems.Count > 0)
            throw new ModLoadException(problems);

        var ordered = new List<FoundMod>(mods.Count);
        var waiting = mods.ToDictionary(m => m.Id, m => m.Manifest.Depends.Count, StringComparer.Ordinal);
        var ready = new SortedSet<string>(waiting.Where(w => w.Value == 0).Select(w => w.Key), StringComparer.Ordinal);
        while (ready.Count > 0)
        {
            string next = ready.Min!;
            ready.Remove(next);
            ordered.Add(byId[next]);
            foreach (FoundMod mod in mods.Where(m => m.Manifest.Depends.ContainsKey(next)))
                if (--waiting[mod.Id] == 0)
                    ready.Add(mod.Id);
        }
        if (ordered.Count < mods.Count)
        {
            HashSet<string> placed = ordered.Select(m => m.Id).ToHashSet(StringComparer.Ordinal);
            throw new ModLoadException([.. Cycles(mods.Where(m => !placed.Contains(m.Id)).ToDictionary(m => m.Id, StringComparer.Ordinal))]);
        }
        return ordered;
    }

    // One line per cycle among the mods that could not be placed: a -> b -> a.
    private static IEnumerable<string> Cycles(Dictionary<string, FoundMod> stuck)
    {
        var reported = new HashSet<string>(StringComparer.Ordinal);
        foreach (string start in stuck.Keys.Order(StringComparer.Ordinal))
        {
            if (reported.Contains(start))
                continue;
            var path = new List<string> { start };
            string at = start;
            while (true)
            {
                at = stuck[at].Manifest.Depends.Keys.Where(stuck.ContainsKey).Order(StringComparer.Ordinal).First();
                int seen = path.IndexOf(at);
                if (seen >= 0)
                {
                    List<string> cycle = path[seen..];
                    if (cycle.Any(reported.Contains))
                        break;
                    reported.UnionWith(cycle);
                    yield return $"{cycle[0]}: its dependencies go round in a circle: {string.Join(" -> ", cycle.Append(cycle[0]))}.";
                    break;
                }
                path.Add(at);
            }
            reported.Add(start);
        }
    }
}
