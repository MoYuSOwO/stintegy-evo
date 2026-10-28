using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Reflection.Metadata;
using System.Reflection.PortableExecutable;
using System.Runtime.Loader;
using StintegyEVO.Core.Cars;

namespace StintegyEVO.Core.Mods;

/// <summary>
/// Mods' code, read from their bytes into the context Core lives in, so a
/// mod's <see cref="ICar"/> is the game's. Loaded once and never unloaded:
/// changing mods means starting the game again. One mod's assembly that
/// refers to another's finds it here, whichever mod brought it.
/// </summary>
internal static class ModAssemblies
{
    private static readonly Dictionary<string, (Assembly Assembly, Guid Build, string Mod)> Loaded = new(StringComparer.Ordinal);
    private static readonly object Gate = new();
    private static AssemblyLoadContext? _context;

    private static AssemblyLoadContext Context
    {
        get
        {
            if (_context is null)
            {
                _context = AssemblyLoadContext.GetLoadContext(typeof(ICar).Assembly) ?? AssemblyLoadContext.Default;
                _context.Resolving += Resolve;
            }
            return _context;
        }
    }

    /// <summary>
    /// The assembly at <paramref name="path"/> in <paramref name="mod"/>,
    /// with its debugging symbols beside it when it has them. The same
    /// build loaded before, by this mod or another, is that one.
    /// </summary>
    public static Assembly Load(FoundMod mod, string path)
    {
        byte[] image = mod.Resources.ReadAllBytes(path);
        (string name, Guid build) = Identity(image, mod, path);
        lock (Gate)
        {
            if (Loaded.TryGetValue(name, out var loaded))
                return loaded.Build == build ? loaded.Assembly
                    : throw new InvalidDataException(
                        $"its {path} is called {name}, as is the one {loaded.Mod} brought; two mods' code cannot share a name.");
            AssemblyLoadContext context = Context;
            if (context.Assemblies.FirstOrDefault(a => a.GetName().Name == name) is { } present)
                return present.ManifestModule.ModuleVersionId == build ? Remember(name, present, build, mod.Id)
                    : throw new InvalidDataException($"its {path} is called {name}, as is code the game already has.");

            string symbols = Path.ChangeExtension(path, ".pdb");
            using var assemblyStream = new MemoryStream(image, writable: false);
            using Stream? symbolStream = mod.Resources.Exists(symbols) ? mod.Resources.Open(symbols) : null;
            return Remember(name, context.LoadFromStream(assemblyStream, symbolStream), build, mod.Id);
        }
    }

    private static Assembly Remember(string name, Assembly assembly, Guid build, string mod)
    {
        Loaded[name] = (assembly, build, mod);
        return assembly;
    }

    private static Assembly? Resolve(AssemblyLoadContext context, AssemblyName name)
    {
        lock (Gate)
            return name.Name is { } wanted && Loaded.TryGetValue(wanted, out var loaded) ? loaded.Assembly : null;
    }

    // Its name and build, read without loading it.
    private static (string Name, Guid Build) Identity(byte[] image, FoundMod mod, string path)
    {
        try
        {
            using var stream = new MemoryStream(image, writable: false);
            using var reader = new PEReader(stream);
            MetadataReader metadata = reader.GetMetadataReader();
            string name = metadata.GetString(metadata.GetAssemblyDefinition().Name);
            Guid build = metadata.GetGuid(metadata.GetModuleDefinition().Mvid);
            return (name, build);
        }
        catch (Exception error) when (error is BadImageFormatException or InvalidOperationException)
        {
            throw new InvalidDataException($"its {path} is not a .NET assembly.", error);
        }
    }
}
