using System;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The road model against the closed forms it has to agree with.
///
/// Everything here is written in the plan view, because that is what the
/// track is: the centreline is a two-dimensional curve, s is arc length
/// measured on it, the car's position moves in it, and the elevation is a
/// height laid over the top. So speed is horizontal speed and curvature is
/// the curvature of the horizontal path, and the balance to satisfy is the
/// horizontal-and-vertical one out of any textbook:
///
///     N sin(phi) + f cos(phi) = m v^2 / R
///     N cos(phi) - f sin(phi) = m g
///
/// which solve to N/m = g cos(phi) + v^2 k sin(phi) and
/// f/m = v^2 k cos(phi) - g sin(phi). A model that gets the first right and
/// the second wrong will look correct in every test of grip and still take
/// a banked corner too slowly, which is what happened here.
/// </summary>
public sealed class BankedCurveTextbookTests
{
    private const float G = 9.80665f;

    [Theory]
    [InlineData(2.5f)]
    [InlineData(12f)]
    [InlineData(18f)]
    [InlineData(24f)]
    [InlineData(31f)]
    [InlineData(45f)]
    public void ABankedCornerMatchesTheTextbookForceBalance(float degrees)
    {
        float phi = degrees * MathF.PI / 180f;
        RoadAttitude road = new(0f, MathF.Tan(phi));

        const float radius = 90f;
        const float curvature = 1f / radius;
        float speed = MathF.Sqrt(G * radius * MathF.Tan(phi) * 1.4f);
        float speedSquared = speed * speed;

        Assert.Equal(
            G * MathF.Cos(phi) + speedSquared * curvature * MathF.Sin(phi),
            road.NormalGravity(G, speed, curvature),
            3
        );

        // What the tyres are left holding, in the sense curvature is written
        // in: the corner's own demand scaled onto the surface, less what the
        // bank carries for free.
        float tyre = road.CurvatureDemandScale * speedSquared * curvature +
                     road.LateralGravityDemand(G, speed);
        Assert.Equal(
            speedSquared * curvature * MathF.Cos(phi) - G * MathF.Sin(phi),
            tyre,
            3
        );
    }

    [Theory]
    [InlineData(2.5f)]
    [InlineData(12f)]
    [InlineData(24f)]
    [InlineData(31f)]
    [InlineData(45f)]
    public void TheSpeedThatNeedsNoTyresAtAllIsTheTextbookOne(float degrees)
    {
        // v^2 = g R tan(phi) is where the bank alone holds the car round.
        float phi = degrees * MathF.PI / 180f;
        RoadAttitude road = new(0f, MathF.Tan(phi));
        const float radius = 90f;
        const float curvature = 1f / radius;
        float speed = MathF.Sqrt(G * radius * MathF.Tan(phi));

        float tyre = road.CurvatureDemandScale * speed * speed * curvature +
                     road.LateralGravityDemand(G, speed);
        Assert.Equal(0f, tyre, 3);
    }

    [Theory]
    [InlineData(0.02f)]
    [InlineData(0.08f)]
    [InlineData(0.15f)]
    public void AClimbTakesTheComponentOfGravityAlongIt(float grade)
    {
        // Written in the plan view, so it is the horizontal component of the
        // pull along the slope: g sin(theta) cos(theta).
        RoadAttitude road = new(grade, 0f);
        float theta = MathF.Atan(grade);
        Assert.Equal(
            -G * MathF.Sin(theta) * MathF.Cos(theta),
            road.AlongTrackGravity(G, speedMetersPerSecond: 50f),
            4
        );
    }

    [Fact]
    public void ACrestUnloadsTheCarByTheCentripetalTermAndACompressionLoadsIt()
    {
        const float radius = 500f;
        RoadAttitude crest = new(0f, 0f, -1f / radius);
        RoadAttitude compression = new(0f, 0f, 1f / radius);

        Assert.Equal(G - 3600f / radius, crest.NormalGravity(G, 60f, 0f), 3);
        Assert.Equal(G + 3600f / radius, compression.NormalGravity(G, 60f, 0f), 3);
        Assert.Equal(G, crest.NormalGravity(G, 0f, 0f), 4);
    }

    [Fact]
    public void TheCarIsNeverAllowedToBeLiftedCleanOff()
    {
        RoadAttitude brow = new(0f, 0f, -1f / 40f);
        Assert.Equal(
            G * RoadAttitude.MinimumNormalShare,
            brow.NormalGravity(G, 60f, 0f),
            4
        );
    }

    [Fact]
    public void AFlatRoadAsksForNothingAndChangesNothing()
    {
        Assert.Equal(1f, RoadAttitude.Flat.CurvatureDemandScale, 6);
        Assert.Equal(0f, RoadAttitude.Flat.LateralGravityDemand(G, 60f), 6);
        Assert.Equal(0f, RoadAttitude.Flat.AlongTrackGravity(G, 60f), 6);
        Assert.Equal(G, RoadAttitude.Flat.NormalGravity(G, 60f, 0.01f), 6);
    }
}
