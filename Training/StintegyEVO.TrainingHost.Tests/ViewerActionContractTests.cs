using StintegyEVO.Core.Cars;
using StintegyEVO.GodotApp.Drivers;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// The viewer reads the action contract off the network's card.
///
/// A policy baked on increments and driven as if its action were the
/// command — or the other way round — is not a slightly different driver,
/// it is a car that looks broken. The card is written by the exporter from
/// what the checkpoint itself recorded, so nobody has to remember.
/// </summary>
public sealed class ViewerActionContractTests
{
    private static string Assets => Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "..",
        "Assets", "Drivers"
    ));

    private static string? AnyNetwork()
    {
        string[] found = Directory.Exists(Assets)
            ? Directory.GetFiles(Assets, "*.onnx")
            : [];
        return found.Length > 0 ? found[0] : null;
    }

    private static NeuralDriverController Open(string model) => new(
        model,
        new CarConfig { CombinedGripLimiterStrength = 1f },
        new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f },
        new CarStrategy(TireUsageMode.Normal, 3)
    );

    [Theory]
    [InlineData("delta", true)]
    [InlineData("absolute", false)]
    public void TheCardDecidesWhatTheFirstActionMeans(string written, bool delta)
    {
        if (AnyNetwork() is not string source)
        {
            Console.WriteLine("no exported network to read; skipped.");
            return;
        }

        string directory = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());
        Directory.CreateDirectory(directory);
        try
        {
            string model = Path.Combine(directory, "driver.onnx");
            File.Copy(source, model);
            File.WriteAllText(
                Path.ChangeExtension(model, ".json"),
                $"{{\n  \"action_semantics\": \"{written}\",\n  \"head\": \"tanh(mean)\"\n}}"
            );
            using NeuralDriverController controller = Open(model);
            Assert.Equal(delta, controller.DeltaActions);
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public void ANetworkWithNoCardIsTheContractEveryGenerationSoFarWasBakedOn()
    {
        if (AnyNetwork() is not string source)
        {
            Console.WriteLine("no exported network to read; skipped.");
            return;
        }

        string directory = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());
        Directory.CreateDirectory(directory);
        try
        {
            string model = Path.Combine(directory, "driver.onnx");
            File.Copy(source, model);
            using NeuralDriverController controller = Open(model);
            Assert.False(controller.DeltaActions);
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }
}
