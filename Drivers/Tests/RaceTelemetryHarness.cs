using System.Globalization;
using System.Text;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Dev harness, not an assertion: with STINTEGY_TELEM set to a path it runs the
/// Silverstone grid and writes one CSV row per car per 0.1 s, so a change can
/// be measured instead of argued about. Without the variable it returns at once
/// and costs nothing.
///
/// STINTEGY_TELEM          output csv path
/// STINTEGY_TELEM_CARS     number of cars on grid (default 6)
/// STINTEGY_TELEM_SECONDS  race seconds to run (default 60)
/// STINTEGY_TELEM_EVERY    sample interval seconds (default 0.1)
/// STINTEGY_TELEM_BOOST_FROM / _TO / _CAR   held button 0 window for one car
/// </summary>
public sealed class RaceTelemetryHarness
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void WritesGridCsvWhenAsked()
    {
        string? path = Environment.GetEnvironmentVariable("STINTEGY_TELEM");
        if (string.IsNullOrWhiteSpace(path))
            return;

        float seconds = Env("STINTEGY_TELEM_SECONDS", 60f);
        float every = Env("STINTEGY_TELEM_EVERY", 0.1f);
        int? carCount = int.TryParse(Environment.GetEnvironmentVariable("STINTEGY_TELEM_CARS"), out int requested)
            ? requested : null;

        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        var simulation = new RaceSimulation(track);
        List<RaceCar> cars = SilverstoneGrid.Seat(simulation, file, track, DevelopmentTyres.SuperId, carCount);

        float boostFrom = Env("STINTEGY_TELEM_BOOST_FROM", float.PositiveInfinity);
        float boostTo = Env("STINTEGY_TELEM_BOOST_TO", float.NegativeInfinity);
        int boostCar = (int)Env("STINTEGY_TELEM_BOOST_CAR", 1f);

        var csv = new StringBuilder();
        csv.AppendLine("t,car,s,raceDist,d,freeD,offFree,v,ax,demand,off,wall,touch,spin");
        var prevV = new float[cars.Count];
        for (int i = 0; i < cars.Count; i++)
            prevV[i] = cars[i].State.Speed;

        int steps = (int)MathF.Round(seconds / Dt);
        int everySteps = Math.Max(1, (int)MathF.Round(every / Dt));
        var axAccum = new float[cars.Count];
        var axCount = new int[cars.Count];
        int offTicks = 0, wallTicks = 0, touchTicks = 0, spinTicks = 0;
        for (int step = 1; step <= steps; step++)
        {
            float now = step * Dt;
            if (boostCar >= 0 && boostCar < cars.Count)
            {
                bool held = now >= boostFrom && now < boostTo;
                if (held != cars[boostCar].Strategy.Holding(0))
                    cars[boostCar].Strategy = cars[boostCar].Strategy.WithButton(0, held);
            }
            simulation.Step(Dt);
            for (int i = 0; i < cars.Count; i++)
            {
                RaceCar car = cars[i];
                axAccum[i] += (car.State.Speed - prevV[i]) / Dt;
                axCount[i]++;
                prevV[i] = car.State.Speed;
                if (car.Progress.Region != TrackRegion.RacingSurface) offTicks++;
                if (car.LastBoundaryContact.HasValue) wallTicks++;
                if (car.HitCarThisStep) touchTicks++;
                if ((car.Car as ICarMotion)?.Motion(car.State).Spinning == true) spinTicks++;
            }
            if (step % everySteps != 0)
                continue;

            float t = step * Dt;
            for (int i = 0; i < cars.Count; i++)
            {
                RaceCar car = cars[i];
                RaceProgress p = car.Progress;
                int station = lanes.Wrap((int)MathF.Round(lanes.StationOf(p.CurrentS)));
                float free = lanes.Offset(LaneId.Free, station);
                bool off = p.Region != TrackRegion.RacingSurface;
                bool wall = car.LastBoundaryContact.HasValue;
                bool touch = car.HitCarThisStep;
                bool spin = (car.Car as ICarMotion)?.Motion(car.State).Spinning == true;
                float ax = axCount[i] > 0 ? axAccum[i] / axCount[i] : 0f;
                axAccum[i] = 0f;
                axCount[i] = 0;
                csv.Append(F(t)).Append(',')
                   .Append(car.Id).Append(',')
                   .Append(F(p.CurrentS)).Append(',')
                   .Append(F(p.RaceDistanceMeters)).Append(',')
                   .Append(F(p.CurrentD)).Append(',')
                   .Append(F(free)).Append(',')
                   .Append(F(p.CurrentD - free)).Append(',')
                   .Append(F(car.State.Speed)).Append(',')
                   .Append(F(ax)).Append(',')
                   .Append(F(car.LastInput.DesiredAccel)).Append(',')
                   .Append(off ? '1' : '0').Append(',')
                   .Append(wall ? '1' : '0').Append(',')
                   .Append(touch ? '1' : '0').Append(',')
                   .Append(spin ? '1' : '0')
                   .AppendLine();
            }
        }

        File.WriteAllText(path, csv.ToString());
        Console.WriteLine($"[Telem] wrote {path} ({cars.Count} cars, {seconds:F0} s) " +
            $"off={offTicks} wall={wallTicks} touch={touchTicks} spin={spinTicks}");
    }

    private static float Env(string name, float fallback) =>
        Environment.GetEnvironmentVariable(name) is { Length: > 0 } raw
        && float.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out float parsed)
            ? parsed
            : fallback;

    private static string F(float value) => value.ToString("0.###", CultureInfo.InvariantCulture);
}
