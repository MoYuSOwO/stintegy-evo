using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// What the world hands a car and a driver beyond the road and the air: its
/// randomness, seeded by the race, and the blows the walls and other cars
/// gave it.
/// </summary>
public sealed class RaceRandomnessAndImpactTests
{
    /// <summary>
    /// A race run again on the same seed is the same race, to the bit, with
    /// a car and a driver that both act on chance; on another seed it is
    /// another race; and two cars side by side do not draw the same chances.
    /// </summary>
    [Fact]
    public void TheSameSeedGivesTheSameRace()
    {
        (Vector2 A, Vector2 B) first = Race(seed: 7);
        (Vector2 A, Vector2 B) again = Race(seed: 7);
        (Vector2 A, Vector2 B) other = Race(seed: 8);

        Assert.Equal(first, again);
        Assert.NotEqual(first.A, other.A);
        Assert.NotEqual(first.A - Lane(0), first.B - Lane(1));
    }

    /// <summary>
    /// A car run into the back of a stopped one: each is told, at its next
    /// step, that a car hit it -- the stopped car at its tail and pushed on,
    /// the other at its nose and held back -- and how hard, a blow the size
    /// of the speed it changed.
    /// </summary>
    [Fact]
    public void ACarIsToldWhereAnotherHitIt()
    {
        TrackData track = Straight();
        TrackSample start = track.Sample(20f);
        var rear = new Box();
        var front = new Box();
        var simulation = new RaceSimulation(track);
        simulation.AddCar(new RaceCar("rear", rear, state: At(start, 0f, 10f)));
        simulation.AddCar(new RaceCar("front", front, state: At(start, 6f, 0f)));

        for (int i = 0; i < 60 && front.Felt.Count == 0; i++)
            simulation.Step(1f / 60f);

        CarImpact onFront = Assert.Single(front.Felt.Take(1));
        CarImpact onRear = rear.Felt.First();
        Assert.Equal(ImpactWith.Car, onFront.With);
        Assert.True(onFront.Where.X < 0f && onFront.Direction.X > 0.9f, $"front hit at {onFront.Where} towards {onFront.Direction}");
        Assert.True(onRear.Where.X > 0f && onRear.Direction.X < -0.9f, $"rear hit at {onRear.Where} towards {onRear.Direction}");
        Assert.InRange(onFront.ImpulseNewtonSeconds / Box.Mass, 1f, 10f);
    }

    /// <summary>
    /// A car run nose first into the left-hand barrier is told the wall hit
    /// it, at the front left of the body, and pushed back to the right.
    /// </summary>
    [Fact]
    public void ACarIsToldWhereTheWallHitIt()
    {
        TrackData track = Straight();
        TrackSample start = track.Sample(20f);
        var box = new Box();
        var simulation = new RaceSimulation(track);
        CarState state = At(start, 0f, 15f);
        state.Heading += 0.5f;
        simulation.AddCar(new RaceCar("box", box, state: state));

        for (int i = 0; i < 120 && box.Felt.Count == 0; i++)
            simulation.Step(1f / 60f);

        CarImpact hit = Assert.Single(box.Felt.Take(1));
        Assert.Equal(ImpactWith.Wall, hit.With);
        Assert.True(hit.Where.X > 0f && hit.Where.Y > 0f, $"hit at {hit.Where}");
        Assert.True(hit.Direction.Y < 0f, $"pushed towards {hit.Direction}");
        Assert.True(hit.ImpulseNewtonSeconds > 0f);
    }

    private static readonly TrackData Road = Straight();

    private static (Vector2 A, Vector2 B) Race(ulong seed)
    {
        TrackSample start = Road.Sample(20f);
        var simulation = new RaceSimulation(Road, seed: seed);
        RaceCar a = new("a", new Box(), new Driver(new DriverProfile("a", new()), new ShakyHands()), At(start, 0f, 20f, Lane(0)));
        RaceCar b = new("b", new Box(), new Driver(new DriverProfile("b", new()), new ShakyHands()), At(start, 0f, 20f, Lane(1)));
        simulation.AddCar(a);
        simulation.AddCar(b);
        for (int i = 0; i < 120; i++)
            simulation.Step(1f / 60f);
        return (a.State.Position, b.State.Position);
    }

    private static Vector2 Lane(int lane) => Road.Sample(20f).Normal * (lane == 0 ? 3f : -3f);

    private static TrackData Straight() =>
        new TrackBuilder(Vector2.Zero, 12f).AddStraight(600f).AddTurn(180f, 60f)
            .AddStraight(600f).AddTurn(180f, 60f).CloseLoop().Build(new TrackGridConfig());

    private static CarState At(TrackSample start, float ahead, float speed, Vector2 aside = default) => new()
    {
        Position = start.Center + start.Tangent * ahead + aside,
        Heading = start.Heading,
        Speed = speed
    };

    /// <summary>A driver who steers a little at random about straight ahead.</summary>
    private sealed class ShakyHands : IDriverController
    {
        public DriverInput GetControl(in DriverContext context, float dt) =>
            new(context.Random.Draw().NextNormal() * 0.05f, 0f);
    }

    /// <summary>
    /// A box that rolls on at its speed, turns as its wheels point, wanders
    /// a little at random of its own, and keeps every blow it is told of.
    /// </summary>
    private sealed class Box : ICar
    {
        public const float Mass = 800f;

        public List<CarImpact> Felt { get; } = [];

        public CarSpec Spec { get; } = new()
        {
            WheelBaseMeters = 2.5f,
            FrontAxleOffsetMeters = 1.2f,
            RearAxleOffsetMeters = 1.3f,
            TrackWidthMeters = 1.5f,
            YawInertiaKgM2 = 1200f,
            MaximumBrakeDeceleration = 9f,
            KinematicBelowSpeed = 1f,
            Body = new CarBody
            {
                LengthMeters = 4f,
                WidthMeters = 1.8f,
                Restitution = 0.1f,
                Friction = 0.3f,
                WallRestitution = 0.1f,
                WallFriction = 0.5f,
            },
            Wake = new CarWake
            {
                VelocityDeficit = 0f,
                TowHalfDistanceMeters = 10f,
                DownforceDecayLengthMeters = 10f,
                DownforceDisruption = 0f,
                DirtyAirDecayLengthMeters = 10f,
            },
            DirtyAirSensitivity = 0f,
        };

        public IReadOnlyList<ModeLadder> Ladders => [];
        public float TotalMassKg(CarState state) => Mass;

        public void Step(CarState state, CarStepInput input, float dt)
        {
            Felt.AddRange(input.Impacts);
            RandomStream chance = input.Random.Draw();
            state.Heading += (input.DriverInput.SteerAngleRadians + chance.NextNormal() * 0.01f) * state.Speed * dt / Spec.WheelBaseMeters;
            state.SideslipAngleRadians = 0f;
            state.YawRateRadiansPerSecond = 0f;
            state.Position += new Vector2(MathF.Cos(state.Heading), MathF.Sin(state.Heading)) * state.Speed * dt;
        }

        public void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy) => dashboard.SetMass(Mass);
        public IReadOnlyList<TelemetryChannel> TelemetryChannels => [];
        public void ReadTelemetry(CarState state, Span<float> values) { }
    }
}
