using System.Numerics;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Track.RefLines;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class TrackFactoryTests
{
    [Fact]
    public void TrackBuilderUsesInjectedReferenceLineSolver()
    {
        RecordingRefLineSolver solver = new();
        TrackData track = new TrackBuilder(
                Vector2.Zero,
                startWidth: 10f,
                refLineSolver: solver
            )
            .AddStraight(80f)
            .AddTurn(180f, 24f)
            .AddStraight(80f)
            .AddTurn(180f, 24f)
            .CloseLoop()
            .Build(default);

        Assert.True(solver.WasCalled);
        Assert.True(track.LengthMeters > 0f);
    }

    [Fact]
    public void CloseLoopKeepsTheSeamAtTheTrackSamplingInterval()
    {
        TrackData track = new TrackBuilder(
                Vector2.Zero,
                startWidth: 18f,
                startLeftBuffer: 5f,
                startRightBuffer: 5f,
                refLineSolver: CenterLineRefLineSolver.Instance
            )
            .AddStraight(550f)
            .AddTurn(-180f, 20f, 25f)
            .AddStraight(120f, 18f)
            .AddTurn(45f, 30f)
            .AddTurn(-45f, 30f)
            .AddStraight(5f)
            .AddTurn(-45f, 30f)
            .AddTurn(45f, 30f)
            .AddStraight(500f, 12f)
            .AddTurn(-180f, 80f, 12f)
            .AddStraight(20f)
            .AddTurn(-90f, 15f)
            .CloseLoop()
            .Build(default);

        for (float s = 0f; s < track.LengthMeters; s += TrackData.StepLength)
        {
            float segmentLength = Vector2.Distance(
                track.Sample(s).Center,
                track.Sample(s + TrackData.StepLength).Center
            );
            Assert.InRange(segmentLength, 0.95f, 1.05f);
        }
    }

    [Fact]
    public void SilverstoneUsesPublishedArenaGrandPrixGeometry()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();

        Assert.Equal(5_891f, track.LengthMeters);
        Assert.Equal(0f, track.StartingLineS);
        Assert.Equal(5_881f, track.Grids[1].S);
        Assert.InRange(MinimumWidth(track), 11.2f, 11.4f);
        Assert.InRange(MaximumWidth(track), 17.7f, 18.0f);
        Assert.Equal(0, CountCoarseCenterlineIntersections(track, 10f));
        AssertSmoothAndNonIntersecting(track);
    }

    [Fact]
    public void MonacoUsesPublishedGrandPrixGeometry()
    {
        TrackData track = TrackFactory.MonacoStyleTestTrack();

        Assert.Equal(3_337f, track.LengthMeters);
        Assert.Equal(0f, track.StartingLineS);
        Assert.Equal(3_327f, track.Grids[1].S);
        Assert.InRange(MinimumWidth(track), 10.5f, 10.7f);
        Assert.InRange(MaximumWidth(track), 10.5f, 10.7f);
        Assert.Equal(0, CountCoarseCenterlineIntersections(track, 5f));
        AssertSmoothAndNonIntersecting(track);
    }

    [Fact]
    public void ShanghaiUsesPublishedGrandPrixGeometry()
    {
        TrackData track = TrackFactory.ShanghaiStyleTestTrack();

        Assert.Equal(5_451f, track.LengthMeters);
        Assert.Equal(0f, track.StartingLineS);
        Assert.Equal(5_441f, track.Grids[1].S);
        Assert.InRange(MinimumWidth(track), 10f, 12f);
        Assert.InRange(MaximumWidth(track), 16f, 19f);
        Assert.True(MaximumWidth(track) - MinimumWidth(track) > 6f);
        Assert.Equal(0, CountCoarseCenterlineIntersections(track, 10f));
        AssertSmoothAndNonIntersecting(track);
    }

    [Fact]
    public void SepangUsesPublishedGrandPrixGeometry()
    {
        TrackData track = TrackFactory.SepangStyleTestTrack();

        Assert.Equal(5_543f, track.LengthMeters);
        Assert.Equal(0f, track.StartingLineS);
        Assert.Equal(5_533f, track.Grids[1].S);
        Assert.InRange(MinimumWidth(track), 16f, 17f);
        Assert.InRange(MaximumWidth(track), 21f, 22.1f);
        Assert.Equal(0, CountCoarseCenterlineIntersections(track, 10f));
        AssertSmoothAndNonIntersecting(track);
    }

    [Fact]
    public void GrandPrixStyleBenchmarksHaveDistinctExpectedLengths()
    {
        TrackData silverstone = TrackFactory.SilverstoneStyleTestTrack();
        TrackData monaco = TrackFactory.MonacoStyleTestTrack();
        TrackData shanghai = TrackFactory.ShanghaiStyleTestTrack();
        TrackData sepang = TrackFactory.SepangStyleTestTrack();

        Assert.InRange(silverstone.LengthMeters, 5840f, 5920f);
        Assert.InRange(monaco.LengthMeters, 3290f, 3390f);
        Assert.InRange(shanghai.LengthMeters, 5410f, 5500f);
        Assert.InRange(sepang.LengthMeters, 5500f, 5580f);

        AssertClosedAndGridded(silverstone);
        AssertClosedAndGridded(monaco);
        AssertClosedAndGridded(shanghai);
        AssertClosedAndGridded(sepang);
    }

    private static void AssertClosedAndGridded(TrackData track)
    {
        Vector2 start = track.Sample(0f).Center;
        Vector2 end = track.Sample(track.LengthMeters - TrackData.StepLength).Center;
        Assert.InRange(Vector2.Distance(start, end), 0f, 2.5f);
        Assert.Equal(30, track.StartingGridCount);
    }

    private static float MinimumWidth(TrackData track)
    {
        float minimum = float.MaxValue;
        for (float s = 0f; s < track.LengthMeters; s += 10f)
            minimum = MathF.Min(minimum, track.Sample(s).Width);
        return minimum;
    }

    private static float MaximumWidth(TrackData track)
    {
        float maximum = float.MinValue;
        for (float s = 0f; s < track.LengthMeters; s += 10f)
            maximum = MathF.Max(maximum, track.Sample(s).Width);
        return maximum;
    }

    private static int CountCoarseCenterlineIntersections(
        TrackData track,
        float sampleStepMeters
    )
    {
        List<Vector2> points = [];
        for (float s = 0f; s < track.LengthMeters; s += sampleStepMeters)
            points.Add(track.Sample(s).Center);

        int intersections = 0;
        for (int i = 0; i < points.Count; i++)
        {
            Vector2 a = points[i];
            Vector2 b = points[(i + 1) % points.Count];
            for (int j = i + 2; j < points.Count; j++)
            {
                if (i == 0 && j == points.Count - 1)
                    continue;
                Vector2 c = points[j];
                Vector2 d = points[(j + 1) % points.Count];
                if (ProperlyIntersects(a, b, c, d))
                    intersections++;
            }
        }
        return intersections;
    }

    private static void AssertSmoothAndNonIntersecting(TrackData track)
    {
        const float wallAuditStepMeters = 4f;
        List<Vector2> leftWall = [];
        List<Vector2> rightWall = [];
        float maximumCenterCurvatureJump = 0f;
        float maximumRefCurvature = 0f;
        float maximumRefCurvatureJump = 0f;
        float maximumRefSafetyExcess = 0f;
        float maximumRefLongitudinalResidual = 0f;
        float previousCenterCurvature = CenterCurvatureAt(track, -1f);
        float previousRefCurvature = track.Sample(-1f).RefCurvature;

        for (float s = 0f; s < track.LengthMeters; s += TrackData.StepLength)
        {
            TrackSample sample = track.Sample(s);
            float centerCurvature = CenterCurvatureAt(track, s);
            maximumCenterCurvatureJump = MathF.Max(
                maximumCenterCurvatureJump,
                MathF.Abs(centerCurvature - previousCenterCurvature)
            );
            maximumRefCurvature = MathF.Max(
                maximumRefCurvature,
                MathF.Abs(sample.RefCurvature)
            );
            maximumRefCurvatureJump = MathF.Max(
                maximumRefCurvatureJump,
                MathF.Abs(sample.RefCurvature - previousRefCurvature)
            );
            Vector2 refDelta = sample.RefPosition - sample.Center;
            maximumRefSafetyExcess = MathF.Max(
                maximumRefSafetyExcess,
                MathF.Abs(sample.RefOffset) - (
                    sample.HalfWidth - TrackPlanningBounds.VehicleHalfWidthMeters
                )
            );
            maximumRefLongitudinalResidual = MathF.Max(
                maximumRefLongitudinalResidual,
                MathF.Abs(Vector2.Dot(refDelta, sample.Tangent))
            );
            previousCenterCurvature = centerCurvature;
            previousRefCurvature = sample.RefCurvature;

            if ((int)s % (int)wallAuditStepMeters == 0)
            {
                leftWall.Add(sample.LeftSpace);
                rightWall.Add(sample.RightSpace);
            }
        }

        Assert.InRange(maximumCenterCurvatureJump, 0f, 0.025f);
        Assert.InRange(maximumRefCurvature, 0f, 0.25f);
        Assert.InRange(maximumRefCurvatureJump, 0f, 0.05f);
        Assert.InRange(maximumRefSafetyExcess, 0f, 0.05f);
        Assert.InRange(maximumRefLongitudinalResidual, 0f, 0.25f);
        Assert.Equal(0, CountBoundaryIntersections(leftWall, leftWall, true));
        Assert.Equal(0, CountBoundaryIntersections(rightWall, rightWall, true));
        Assert.Equal(0, CountBoundaryIntersections(leftWall, rightWall, false));
    }

    private static float CenterCurvatureAt(TrackData track, float s)
    {
        Vector2 tangent = track.Sample(s).Tangent;
        Vector2 nextTangent = track.Sample(s + TrackData.StepLength).Tangent;
        float heading = MathF.Atan2(tangent.Y, tangent.X);
        float nextHeading = MathF.Atan2(nextTangent.Y, nextTangent.X);
        float delta = nextHeading - heading;
        return MathF.Atan2(MathF.Sin(delta), MathF.Cos(delta)) /
               TrackData.StepLength;
    }

    private static int CountBoundaryIntersections(
        IReadOnlyList<Vector2> first,
        IReadOnlyList<Vector2> second,
        bool sameBoundary
    )
    {
        int intersections = 0;
        for (int i = 0; i < first.Count; i++)
        {
            Vector2 a = first[i];
            Vector2 b = first[(i + 1) % first.Count];
            int firstCandidate = sameBoundary ? i + 2 : 0;
            for (int j = firstCandidate; j < second.Count; j++)
            {
                if (sameBoundary && i == 0 && j == second.Count - 1)
                    continue;
                Vector2 c = second[j];
                Vector2 d = second[(j + 1) % second.Count];
                if (ProperlyIntersects(a, b, c, d))
                    intersections++;
            }
        }
        return intersections;
    }

    private static bool ProperlyIntersects(
        Vector2 a,
        Vector2 b,
        Vector2 c,
        Vector2 d
    )
    {
        float abC = Cross(a, b, c);
        float abD = Cross(a, b, d);
        float cdA = Cross(c, d, a);
        float cdB = Cross(c, d, b);
        return abC * abD < 0f && cdA * cdB < 0f;
    }

    private static float Cross(Vector2 a, Vector2 b, Vector2 c) =>
        (b.X - a.X) * (c.Y - a.Y) -
        (b.Y - a.Y) * (c.X - a.X);

    private sealed class RecordingRefLineSolver : IRefLineSolver
    {
        public bool WasCalled { get; private set; }

        public RefLine Generate(IReadOnlyList<RefLineTrackPoint> track)
        {
            WasCalled = true;
            return CenterLineRefLineSolver.Instance.Generate(track);
        }
    }
}
