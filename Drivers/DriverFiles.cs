using System.Reflection;

namespace StintegyEVO.Drivers;

/// <summary>Where the driver's files for a car on a circuit are found: beside the build, or in the assembly.</summary>
public static class DriverFiles
{
    /// <summary>The car the shipped driver files were made for.</summary>
    public const string StockCar = "formula_a_mixed";

    /// <summary>
    /// A driver file under <c>Cars/</c>, by its path there
    /// (<c>formula_a_mixed/silverstone/lanes.csv</c>): on disk beside the build, or else
    /// unpacked from the assembly. What a driver learns is about a car on a
    /// circuit, so it is kept under the car, never with the circuit.
    /// </summary>
    public static string? FindFile(string file)
    {
        foreach (string candidate in CandidatePaths(file))
        {
            if (File.Exists(candidate))
                return candidate;
        }

        return ExtractEmbedded(file);
    }

    private static IEnumerable<string> CandidatePaths(string file)
    {
        string baseDir = AppContext.BaseDirectory;
        yield return Path.Combine(baseDir, "Cars", file);
        yield return Path.Combine(baseDir, file);
        string? dir = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);
        if (dir is not null)
            yield return Path.Combine(dir, "Cars", file);
        yield return Path.GetFullPath(Path.Combine(baseDir, "..", "..", "..", "..", "Cars", file));
    }

    private static string? ExtractEmbedded(string file)
    {
        Assembly assembly = typeof(DriverFiles).Assembly;
        string name = $"StintegyEVO.Drivers.Cars.{file.Replace('/', '.')}";
        using Stream? stream = assembly.GetManifestResourceStream(name);
        if (stream is null)
            return null;
        string temp = Path.Combine(Path.GetTempPath(), "stintegy-driver-files", file);
        Directory.CreateDirectory(Path.GetDirectoryName(temp)!);
        using FileStream output = File.Create(temp);
        stream.CopyTo(output);
        return temp;
    }
}
