using System;
using System.IO;
using System.Linq;
using Official.SingleTrack;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Tyres come in packs of compounds, every number written out and nothing
/// else; a car runs what it declares and nothing it does not.
/// </summary>
public sealed class TireLibraryTests
{
    private static readonly ResourceId Pack = new("someone", "their_tyres");

    private const string Numbers = """
        "baseMu": 2.2, "loadSensitivity": 0.25,
        "idealSurfaceTempLowC": 80, "idealSurfaceTempHighC": 105,
        "coldGripLossPerCSquared": 0.0001, "hotGripLossPerCSquared": 0.0002,
        "coreOverheatTempC": 108, "heatScale": 1.2, "wearScale": 1.5, "wearCliffGripLoss": 0.2
        """;

    private static string PackJson(string compounds) =>
        $$"""{ "format": 1, "id": "their_tyres", "name": "Their tyres", "compounds": [ {{compounds}} ] }""";

    [Fact]
    public void FormulaAIsTheGamesOwnPackAndTheOfficialCarRunsIt()
    {
        LoadedTires formulaA = TireLibrary.Load("official:formula_a");
        Assert.Matches("^[0-9a-f]{12}$", formulaA.Fingerprint);
        Assert.Contains("official:formula_a/medium", formulaA.Compounds.Select(c => c.Id));

        ICar car = CarLibrary.Load("official:formula_a_mixed").Car;
        Assert.Equal(formulaA.Compounds.Select(c => c.Id), car.As<ICarTires>().Compounds.Select(c => c.Id));
    }

    [Fact]
    public void ACompoundIsItsPackAndItsOwnName()
    {
        LoadedTires pack = TireLibrary.Read(Pack, PackJson(
            $$"""{ "id": "soft", "name": "Soft", "tyre": { {{Numbers}} } }"""));
        TireCompound soft = Assert.Single(pack.Compounds);
        Assert.Equal("someone:their_tyres/soft", soft.Id);
        Assert.Equal("Soft", soft.Name);
        Assert.Same(soft.Front, soft.Rear);
        Assert.Equal(2.2f, soft.Front.BaseMu);
        Assert.Equal(1.5f, soft.Front.WearScale);
    }

    [Fact]
    public void ACompoundMayPutItsOwnNumbersOnEachAxle()
    {
        string rear = Numbers.Replace("\"baseMu\": 2.2", "\"baseMu\": 2.3");
        TireCompound compound = Assert.Single(TireLibrary.Read(Pack, PackJson(
            $$"""{ "id": "split", "front": { {{Numbers}} }, "rear": { {{rear}} } }""")).Compounds);
        Assert.Equal(2.2f, compound.Front.BaseMu);
        Assert.Equal(2.3f, compound.Rear.BaseMu);
    }

    /// <summary>
    /// Every number written out: a number left out would be the game's
    /// standing in a pack that never chose it, and a number the model does
    /// not take is a typo or a reach into the model.
    /// </summary>
    [Theory]
    [InlineData("\"wearScale\": 1.5, ", "")]
    [InlineData("\"wearScale\": 1.5", "\"wearScale\": 1.5, \"lateralHeatRate\": 3")]
    [InlineData("\"idealSurfaceTempHighC\": 105", "\"idealSurfaceTempHighC\": 70")]
    [InlineData("\"heatScale\": 1.2", "\"heatScale\": 0")]
    public void APackThatLeavesOutReachesPastOrContradictsANumberIsRefused(string from, string to)
    {
        string numbers = Numbers.Replace(from, to);
        Assert.NotEqual(Numbers, numbers);
        Assert.Throws<InvalidDataException>(() => TireLibrary.Read(Pack, PackJson(
            $$"""{ "id": "soft", "tyre": { {{numbers}} } }""")));
    }

    [Theory]
    [InlineData("""{ "id": "Soft!", "tyre": { NUMBERS } }""")]
    [InlineData("""{ "id": "soft", "front": { NUMBERS } }""")]
    [InlineData("""{ "id": "soft", "tyre": { NUMBERS }, "rear": { NUMBERS } }""")]
    [InlineData("""{ "id": "soft", "tyre": { NUMBERS } }, { "id": "soft", "tyre": { NUMBERS } }""")]
    public void ACompoundMustBeNamedOnceAndSayWhatEachAxleRuns(string compounds)
    {
        Assert.Throws<InvalidDataException>(() =>
            TireLibrary.Read(Pack, PackJson(compounds.Replace("NUMBERS", Numbers))));
    }

    [Fact]
    public void ACarThatDeclaresNoTyresDoesNotRun()
    {
        var manifest = new CarManifest { Id = "bare", Assembly = "bare.dll" };
        Assert.Throws<InvalidDataException>(() => CarLibrary.TiresFor(manifest, new ResourceId("someone", "bare")));
    }

    [Fact]
    public void ACarDeclaresAWholePackOrOneCompoundOfIt()
    {
        CarManifest whole = new() { Tyres = ["official:formula_a"] };
        CarManifest one = new() { Tyres = ["official:formula_a/medium"] };
        CarManifest none = new() { Tyres = ["official:formula_a/ice"] };
        ResourceId id = new("someone", "car");
        Assert.Equal(TireLibrary.Load("official:formula_a").Compounds, CarLibrary.TiresFor(whole, id));
        Assert.Equal("official:formula_a/medium", Assert.Single(CarLibrary.TiresFor(one, id)).Id);
        Assert.Throws<InvalidDataException>(() => CarLibrary.TiresFor(none, id));
    }

    [Fact]
    public void ACarRunsOnlyTheCompoundsItLists()
    {
        TireCompound hard = TireCompound.Of(new TireConfig(), "someone:their_tyres/hard");
        TireCompound soft = TireCompound.Of(new TireConfig { BaseMu = 2.6f }, "someone:their_tyres/soft");
        var car = new SingleTrackCar(new CarConfig(), [hard, soft]);
        var state = new CarState();

        car.InstallFreshTires(state);
        Assert.Equal(hard.Id, state.Chassis().Compound);

        car.FitTires(state, soft.Id);
        Assert.Equal(soft.Id, state.Chassis().Compound);
        Assert.Equal(soft.Id, car.TireWindow(state, WheelId.RearLeft).Compound);
        Assert.Equal(Tire.BlanketTempC, state.Chassis().FrontLeft.SurfaceTempC);

        Assert.Throws<ArgumentException>(() => car.FitTires(state, "official:formula_a/medium"));
        Assert.Equal(soft.Id, state.Chassis().Compound);
    }

    [Fact]
    public void TheCompoundOnIsTheOneThatGrips()
    {
        TireCompound hard = TireCompound.Of(new TireConfig(), "someone:their_tyres/hard");
        TireCompound soft = TireCompound.Of(new TireConfig { BaseMu = 2.6f }, "someone:their_tyres/soft");
        var car = new SingleTrackCar(new CarConfig(), [hard, soft]);
        float Limit(TireCompound compound)
        {
            var state = new CarState { Speed = 40f }.WithEnergy(PowertrainState.Filled(0.8f));
            car.FitTires(state, compound.Id);
            state.Chassis().WarmTires(95f, 95f);
            return car.EstimateCorneringLimit(state, CarStrategy.Default, 40f);
        }
        Assert.True(Limit(soft) > Limit(hard) * 1.03f, $"soft {Limit(soft):F2} against hard {Limit(hard):F2}");
    }

    /// <summary>A compound that heats faster or wears faster does, through every cause at once.</summary>
    [Fact]
    public void HeatAndWearScaleWithTheCompound()
    {
        (float Heat, float Wear) Corner(TireConfig tyre)
        {
            var car = new SingleTrackCar(new CarConfig(), tyre);
            var state = new CarState { Speed = 45f }.WithEnergy(PowertrainState.Filled(0.8f));
            car.InstallFreshTires(state);
            state.Chassis().WarmTires(95f, 95f);
            for (int i = 0; i < 600; i++)
                car.Step(state, new CarStepInput(new DriverInput(car.EstimateSteering(state, RoadAttitude.Flat).AngleFor(0.012f), 0.5f), CarStrategy.Default, 25f), 1f / 60f);
            return (state.Chassis().FrontLeft.SurfaceTempC + state.Chassis().RearRight.SurfaceTempC, state.Chassis().FrontLeft.Wear + state.Chassis().RearRight.Wear);
        }
        var plain = Corner(new TireConfig());
        var hot = Corner(new TireConfig { HeatScale = 1.5f });
        var worn = Corner(new TireConfig { WearScale = 2f });
        Assert.True(hot.Heat > plain.Heat + 2f, $"heat {hot.Heat:F1} against {plain.Heat:F1}");
        Assert.True(worn.Wear > plain.Wear * 1.6f, $"wear {worn.Wear:E2} against {plain.Wear:E2}");
    }

    /// <summary>
    /// The development compound: the same grip as the medium, always in its
    /// window and wearing a hundred times slower, so work on the driver is
    /// not work on the tyres. Ten minutes of hard cornering leave it gripping
    /// as it started; the medium, run the same way, loses about one per cent.
    /// </summary>
    [Fact]
    public void TheSuperTyreGripsAfterTenMinutesAsItDidAtTheStart()
    {
        ICar car = CarLibrary.Load("official:formula_a_mixed").Car;
        Assert.Equal("official:formula_a/medium", car.As<ICarTires>().Compounds[0].Id);
        float Kept(string compound)
        {
            var state = new CarState { Speed = 45f }.WithEnergy(PowertrainState.Filled(0.8f));
            car.As<ICarTires>().FitTires(state, compound);
            // Measure from the same hot state: the medium would otherwise earn
            // back part of its wear by warming out of the blankets, and the
            // test would be measuring warm-up rather than degradation.
            state.Chassis().WarmTires(95f, 95f);
            float before = car.As<ICarEnvelope>().EstimateCorneringLimit(state, CarStrategy.Default, 45f);
            for (int i = 0; i < 60 * 600; i++)
            {
                state.Speed = 45f;
                car.Step(state, new CarStepInput(new DriverInput(0.016f, 0.5f), CarStrategy.Default, 25f), 1f / 60f);
            }
            return car.As<ICarEnvelope>().EstimateCorneringLimit(state, CarStrategy.Default, 45f) / before;
        }
        float super = Kept("official:formula_a/super");
        float medium = Kept("official:formula_a/medium");
        Assert.InRange(super, 0.99f, 1.01f);
        // The medium loses about one per cent over the ten minutes on this
        // calibration, which is enough to tell it from the development
        // compound; its absolute wear is a separate calibration.
        Assert.True(medium < super - 0.005f, $"medium kept {medium:F3} against the super's {super:F3}");
    }
}
