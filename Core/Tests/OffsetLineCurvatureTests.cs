using System;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Whether the target line handed to the controller beside the racing line is
/// the line it is supposed to be.
///
/// A line held a constant distance d from the racing line has curvature
/// k / (1 - k*d) in closed form. That is the whole of the geometry, and it is
/// why running three metres wide costs a fraction of a per cent of a lap. If
/// the curvature the controller is actually shown does not match it, the speed
/// planner will price a line nobody asked for, and every layer above will be
/// negotiating with a number that has nothing to do with the track.
/// </summary>
public class OffsetLineCurvatureTests(ITestOutputHelper output)
{
    private const float VehicleHalfWidthMeters = 0.95f;
    private const float SampleStepMeters = 1f;

    [Theory]
    [InlineData(1f)]
    [InlineData(3f)]
    public void TheOffsetLineHasTheCurvatureGeometryGivesIt(float offsetMeters)
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);

        double worst = 0d;
        double total = 0d;
        int samples = 0;
        float worstAt = 0f;
        float worstShown = 0f;
        float worstExpected = 0f;
        for (float s = 0f; s < track.LengthMeters; s += SampleStepMeters)
        {
            TrackLateralTargetSample shown = profile.SampleGeometry(
                track,
                s,
                offsetMeters,
                executionOffsetMeters: 0f,
                VehicleHalfWidthMeters
            );
            // Compare against the closed form at the offset the profile
            // actually settled on, so that width clamping is not counted as
            // an error: the question is whether the line it produced has the
            // curvature that line should have.
            TrackSample reference = track.Sample(s);
            float held = shown.OffsetMeters;
            float scale = 1f - reference.RefCurvature * held;
            if (MathF.Abs(scale) < 0.05f)
                continue;

            float expected = reference.RefCurvature / scale;
            double error = Math.Abs(shown.Curvature - expected);
            total += error;
            samples++;
            if (error <= worst)
                continue;

            worst = error;
            worstAt = s;
            worstShown = shown.Curvature;
            worstExpected = expected;
        }

        double mean = total / Math.Max(samples, 1);
        output.WriteLine($"offset {offsetMeters:F1} m over {samples} samples");
        output.WriteLine($"  mean curvature error  {mean:F5} 1/m");
        output.WriteLine($"  worst                 {worst:F5} 1/m at s={worstAt:F0}");
        output.WriteLine(
            $"    shown {worstShown:F5}, geometry says {worstExpected:F5}");

        // Corner curvature on this circuit runs to about 0.02 1/m, and the
        // speed a corner allows goes as the square root of it, so an error of
        // that size is not a detail: it is the difference between a line the
        // car can carry speed through and one it cannot.
        Assert.True(
            mean < 0.002d,
            $"the offset line is shown with a mean curvature error of {mean:F5} 1/m, " +
            "so the speed planner is not pricing the line the geometry describes"
        );
    }
}
