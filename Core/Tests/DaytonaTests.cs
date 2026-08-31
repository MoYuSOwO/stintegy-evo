using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Track.RefLines;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Daytona is in this project for one measurement: whether a policy taught
/// on twenty degrees of banking can drive on thirty-one. Every other
/// circuit here is either flat or inside the training range, so if these
/// pins break the banking dimension has stopped being tested and nothing
/// else will say so.
/// </summary>
public sealed class DaytonaTests
{
    [Fact]
    public void TheTurnsAreBankedAndTheStraightsAreNot()
    {
        TrackData track = TrackFactory.DaytonaStyleTestTrack();
        int metres = (int)track.LengthMeters;
        float steepest = 0f, shallowest = 1f;
        int banked = 0;
        for (int s = 0; s < metres; s++)
        {
            float bank = MathF.Abs(track.Sample(s).BankSlope);
            steepest = MathF.Max(steepest, bank);
            shallowest = MathF.Min(shallowest, bank);
            if (Degrees(bank) > 25f)
                banked++;
        }

        Assert.InRange(Degrees(steepest), 30f, 32f);
        Assert.InRange(Degrees(shallowest), 2f, 4f);
        // Two turns of a thousand feet is 1,916 metres of arc, and the
        // part of each transition already past twenty-five degrees adds
        // about another 130. If this drifts far either way the banking has
        // leaked onto a straight or stopped reaching the middle of a turn.
        Assert.InRange(banked, (int)(metres * 0.44f), (int)(metres * 0.56f));
    }

    [Fact]
    public void TheBankingWindsOnRatherThanArriving()
    {
        // Reading the banking off the curvature instead puts it on in a
        // metre, and a car meets a thirty-one degree corner's lateral
        // gravity in one wheel rotation.
        TrackData track = TrackFactory.DaytonaStyleTestTrack();
        int metres = (int)track.LengthMeters;
        float worst = 0f;
        for (int s = 0; s < metres; s++)
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
    public void TheBankingCarriesHalfTheCorner()
    {
        // The point of the circuit, stated as physics rather than as lap
        // time: at the same speed through the same radius, thirty-one
        // degrees takes about half the lateral off the tyres and puts about
        // a third more load on them.
        (float use, float load, float speed) flat = TurnLoading(FlatOval());
        (float use, float load, float speed) banked =
            TurnLoading(TrackFactory.DaytonaStyleTestTrack());

        Assert.InRange(banked.speed - flat.speed, -3f, 3f);
        Assert.True(
            banked.use < flat.use * 0.7f,
            $"banked lateral use {banked.use:0.00} against flat {flat.use:0.00}"
        );
        Assert.True(
            banked.load > flat.load * 1.15f,
            $"banked load {banked.load:0} N against flat {flat.load:0} N"
        );
    }

    /// <summary>
    /// Peak speed reached in the middle of the first turn, and what the
    /// tyres were carrying there.
    /// </summary>
    private static (float Use, float Load, float Speed) TurnLoading(
        TrackData track
    )
    {
        TrackSample start = track.Sample(0f);
        RaceCar car = new(
            "daytona-probe",
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

        float use = 0f, load = 0f, speed = 0f;
        for (int i = 0; i < 90 * 120; i++)
        {
            simulation.Step(1f / 120f);
            float s = track.Project(car.State.Position).S;
            if (s < 1_400f || s > 1_700f || car.State.Speed <= speed)
                continue;
            speed = car.State.Speed;
            CarTelemetry telemetry = car.State.Telemetry;
            use = MathF.Max(
                telemetry.FrontLateralUse,
                telemetry.RearLateralUse
            );
            load = car.State.FrontLeft.LoadN + car.State.FrontRight.LoadN +
                   car.State.RearLeft.LoadN + car.State.RearRight.LoadN;
        }
        return (use, load, speed);
    }

    private static TrackData FlatOval() =>
        new TrackBuilder(
                Vector2.Zero,
                startWidth: 18f,
                startLeftBuffer: 5f,
                startRightBuffer: 5f,
                refLineSolver: CenterLineRefLineSolver.Instance
            )
            .AddStraight(1_050f)
            .AddTurn(180f, 305f)
            .AddStraight(1_050f)
            .AddTurn(180f, 305f)
            .CloseLoop()
            .Build(new TrackGridConfig());

    private static float Degrees(float tangent) =>
        MathF.Atan(tangent) * 180f / MathF.PI;
}
