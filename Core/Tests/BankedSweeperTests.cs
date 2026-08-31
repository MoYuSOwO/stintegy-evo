using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// This circuit exists to fill one measured hole: every banked corner in
/// the training set was a hairpin, so the policy had seen banking and it
/// had seen speed and never the two together. If these pins break, the
/// hole is back and nothing else will say so.
/// </summary>
public sealed class BankedSweeperTests
{
    [Fact]
    public void TheFastCornersLeanAndTheHairpinsDoNot()
    {
        TrackData track = TrackFactory.BankedSweeperTestTrack();
        float tightBank = 0f, fastBank = 0f;
        int fastBanked = 0;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            float radius = Radius(track, s);
            float bank = MathF.Abs(track.Sample(s).BankSlope);
            if (radius < 80f)
                tightBank = MathF.Max(tightBank, bank);
            if (radius is > 200f and < 600f)
            {
                fastBank = MathF.Max(fastBank, bank);
                if (Degrees(bank) > 15f)
                    fastBanked++;
            }
        }

        // A hairpin here keeps the drainage crossfall and nothing else.
        Assert.InRange(Degrees(tightBank), 0f, 5f);
        // A fast corner gets the full lean, and inside what has already been
        // trained on — the circuit is here to add speed to the banking, not
        // to add degrees.
        Assert.InRange(Degrees(fastBank), 16f, 20f);
        Assert.True(
            fastBanked > 400,
            $"only {fastBanked} m of fast corner is properly banked"
        );
    }

    [Fact]
    public void TheFlatVariantSharesTheLayoutAndOnlyTheLayout()
    {
        TrackData banked = TrackFactory.BankedSweeperTestTrack();
        TrackData flat = TrackFactory.FlatSweeperTestTrack();
        // Same geometry to the metre, or the pair no longer differs in
        // exactly one variable and the comparison it exists for is dead.
        Assert.Equal(banked.LengthMeters, flat.LengthMeters, 1);

        float flatBank = 0f, bankedBank = 0f;
        for (int s = 0; s < (int)flat.LengthMeters; s++)
        {
            flatBank = MathF.Max(
                flatBank, MathF.Abs(flat.Sample(s).BankSlope));
            bankedBank = MathF.Max(
                bankedBank, MathF.Abs(banked.Sample(s).BankSlope));
        }
        // Crossfall only on the flat one; the full lean on the original.
        Assert.True(flatBank < 0.05f, $"flat sweeper leans {flatBank:0.000}");
        Assert.True(bankedBank > 0.3f, "the banked sweeper lost its banking");
    }

    [Fact]
    public void TheBankingWindsOnRatherThanArriving()
    {
        TrackData track = TrackFactory.BankedSweeperTestTrack();
        float worst = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            worst = MathF.Max(
                worst,
                MathF.Abs(track.Sample(s + 1).BankSlope -
                          track.Sample(s).BankSlope)
            );
        }
        Assert.True(
            worst < 0.02f,
            $"banking changes by {worst:0.0000} per metre at its sharpest"
        );
    }

    [Fact]
    public void ItIsAClosedCircuitAnAnalyticDriverCanLap()
    {
        TrackData track = TrackFactory.BankedSweeperTestTrack();
        float height = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
            height += track.Sample(s + 0.5f).Grade;
        Assert.InRange(height, -0.05f, 0.05f);

        TrackSample start = track.Sample(0f);
        RaceCar car = new(
            "banked-sweeper",
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            new ReferenceLineDriver(),
            new CarState
            {
                Position = start.RefPosition,
                Heading = start.RefHeading,
                Speed = 0f,
                Energy = PowertrainState.Filled(0.8f)
            }
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        bool leftTheRoad = false;
        for (int i = 0; i < 120 * 120; i++)
        {
            simulation.Step(1f / 120f);
            if (car.LastBoundaryContact.HasValue)
                leftTheRoad = true;
        }

        Assert.False(leftTheRoad, "the analytic driver left the surface");
        // Two minutes should be a lap and most of another on a five
        // kilometre circuit; well under that means it is not drivable.
        Assert.True(
            car.Progress.TotalDistance > track.LengthMeters * 1.2f,
            $"only covered {car.Progress.TotalDistance:0} m in two minutes"
        );
    }

    private static float Radius(TrackData track, float s)
    {
        Vector2 a = track.Sample(s - 8f).Center;
        Vector2 b = track.Sample(s).Center;
        Vector2 c = track.Sample(s + 8f).Center;
        Vector2 ab = b - a, bc = c - b;
        float cross = ab.X * bc.Y - ab.Y * bc.X;
        float denominator = ab.Length() * bc.Length() * (c - a).Length();
        if (denominator < 1e-6f)
            return float.PositiveInfinity;
        float curvature = MathF.Abs(2f * cross / denominator);
        return curvature < 1e-5f ? float.PositiveInfinity : 1f / curvature;
    }

    private static float Degrees(float tangent) =>
        MathF.Atan(tangent) * 180f / MathF.PI;
}
