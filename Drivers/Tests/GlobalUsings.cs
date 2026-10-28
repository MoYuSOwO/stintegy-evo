global using Official.SingleTrack;
global using StintegyEVO.Core.Mods;
global using StintegyEVO.Core.Tests;

namespace StintegyEVO.Drivers.Tests
{
    using StintegyEVO.Core.Cars;

    /// <summary>
    /// The stock car's chassis, for the probes that build variants of it:
    /// tests may know the official chassis; the driver knows only features.
    /// </summary>
    internal static class TestCar
    {
        public static CarConfig Config => ((SingleTrackCar)StockCar.Model).Config;

        /// <summary>The stock car built to <paramref name="config"/>, on every compound it runs.</summary>
        public static SingleTrackCar OnEveryCompound(CarConfig config) => new(config, StockCar.Loaded.Compounds);

        /// <summary>The stock car built to <paramref name="config"/>, on the development compound alone, as the lap runner builds it.</summary>
        public static SingleTrackCar With(CarConfig config) =>
            new(config, [StockCar.Loaded.Compounds.Single(c => c.Id == Lanes.DevelopmentTyres.SuperId)]);
    }
}
