using System;
using System.Globalization;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// What the road is shaped like, measured rather than asserted.
///
/// The circuits come from surveyed points that were never meant to be
/// differentiated twice. Reading a heading off them is fine; reading the
/// rate the heading changes at is where the survey noise lives, and every
/// consumer downstream of that rate — the banking written from curvature,
/// the observation's geometry block, the 3D road mesh — inherits it.
/// </summary>
public sealed class CentrelineQualityTests
{
    private readonly ITestOutputHelper output;

    public CentrelineQualityTests(ITestOutputHelper output) =>
        this.output = output;

    /// <summary>A straight, for the purpose of counting dither on one:
    /// anywhere the road's own long-window curvature is under this. A
    /// radius of two kilometres is not a corner in any car.</summary>
    private const float StraightCurvature = 0.0005f;

    /// <summary>The window the long-window curvature is read over. Long
    /// enough that a real corner cannot hide inside it.</summary>
    private const int StraightWindowMetres = 60;

    private static float[] Curvature(TrackData track, int window)
    {
        int count = (int)track.LengthMeters;
        float[] heading = new float[count];
        for (int s = 0; s < count; s++)
        {
            Vector2 tangent = track.Sample(s).Tangent;
            heading[s] = MathF.Atan2(tangent.Y, tangent.X);
        }

        float[] curvature = new float[count];
        int half = Math.Max(1, window / 2);
        for (int s = 0; s < count; s++)
        {
            float turn = 0f;
            for (int step = -half; step < half; step++)
            {
                float a = heading[((s + step) % count + count) % count];
                float b = heading[((s + step + 1) % count + count) % count];
                turn += MathHelper.NormalizeAngle(b - a);
            }
            curvature[s] = turn / (2f * half);
        }
        return curvature;
    }

    private static int StraightSignChanges(TrackData track, int window)
    {
        float[] fine = Curvature(track, window);
        float[] coarse = Curvature(track, StraightWindowMetres);
        int count = fine.Length;
        int changes = 0;
        int previousSign = 0;
        for (int s = 0; s < count; s++)
        {
            if (MathF.Abs(coarse[s]) >= StraightCurvature)
            {
                previousSign = 0;
                continue;
            }
            int sign = fine[s] > 0f ? 1 : (fine[s] < 0f ? -1 : 0);
            if (sign == 0)
                continue;
            if (previousSign != 0 && sign != previousSign)
                changes++;
            previousSign = sign;
        }
        return changes;
    }

    /// <summary>How many times the road tips the other way while going
    /// straight. Cross slope is written from the curvature the surface
    /// layer reads, so dither in that curvature becomes a road that leans
    /// left, then right, then left again down a straight -- which no
    /// circuit does and every car feels.</summary>
    private static int BankSignChanges(TrackData track)
    {
        float[] coarse = Curvature(track, StraightWindowMetres);
        int count = coarse.Length;
        int changes = 0, previous = 0;
        for (int s = 0; s < count; s++)
        {
            if (MathF.Abs(coarse[s]) >= StraightCurvature)
            {
                previous = 0;
                continue;
            }
            float bank = track.Sample(s).BankSlope;
            // Anything under a hundredth of a degree is not a lean.
            if (MathF.Abs(bank) < 1.7e-4f)
                continue;
            int sign = bank > 0f ? 1 : -1;
            if (previous != 0 && sign != previous)
                changes++;
            previous = sign;
        }
        return changes;
    }

    /// <summary>How fast the cross slope changes, per five metres, in
    /// degrees. <paramref name="straightOnly"/> separates the two things
    /// this number mixes: on a corner entry a changing bank is the road
    /// being built correctly, and superelevation run-in is supposed to
    /// take tens of metres; on a straight it is noise, because a straight
    /// has nothing to bank for.</summary>
    private static float BankStepP95(TrackData track, bool straightOnly)
    {
        float[] coarse = Curvature(track, StraightWindowMetres);
        int count = coarse.Length;
        List<float> steps = [];
        for (int s = 0; s < count; s += 5)
        {
            if (straightOnly && MathF.Abs(coarse[s]) >= StraightCurvature)
                continue;
            float here = MathF.Atan(track.Sample(s).BankSlope);
            float next = MathF.Atan(track.Sample((s + 5) % count).BankSlope);
            steps.Add(MathF.Abs(next - here) * 180f / MathF.PI);
        }
        if (steps.Count == 0)
            return 0f;
        steps.Sort();
        return steps[Math.Min(steps.Count - 1, (int)(steps.Count * 0.95f))];
    }

    /// <summary>The apex of every corner, as the peak of the curvature read
    /// over a window the length of a corner entry. Reported as a table so a
    /// change to the centreline can be checked against it corner by corner
    /// rather than in aggregate — a smoothing that rounds one hairpin off
    /// and leaves everything else alone is invisible in a mean.</summary>
    private static List<(float Station, float Curvature)> Apexes(
        TrackData track, int window = 20
    )
    {
        float[] curvature = Curvature(track, window);
        int count = curvature.Length;
        const float cornering = 0.004f;
        List<(float, float)> apexes = [];
        int start = 0;
        while (start < count && MathF.Abs(curvature[start]) >= cornering)
            start++;

        int runStart = -1;
        for (int offset = 0; offset <= count; offset++)
        {
            int i = (start + offset) % count;
            bool inside =
                offset < count && MathF.Abs(curvature[i]) >= cornering;
            if (inside && runStart < 0)
                runStart = offset;
            else if (!inside && runStart >= 0)
            {
                if ((offset - runStart) >= 30)
                {
                    float peak = 0f;
                    int peakAt = runStart;
                    for (int k = runStart; k < offset; k++)
                    {
                        float value = curvature[(start + k) % count];
                        if (MathF.Abs(value) > MathF.Abs(peak))
                        {
                            peak = value;
                            peakAt = k;
                        }
                    }
                    apexes.Add(((start + peakAt) % count, peak));
                }
                runStart = -1;
            }
        }
        return apexes;
    }

    public static IEnumerable<object[]> Circuits() =>
    [
        ["silverstone"], ["shanghai"], ["monaco"], ["zandvoort"],
        ["sepang"], ["baku"], ["spa"], ["monza"], ["interlagos"],
        ["singapore"], ["portimao"],
    ];

    private static TrackData Build(string name) => name switch
    {
        "silverstone" => TrackFactory.SilverstoneStyleTestTrack(),
        "shanghai" => TrackFactory.ShanghaiStyleTestTrack(),
        "monaco" => TrackFactory.MonacoStyleTestTrack(),
        "zandvoort" => TrackFactory.ZandvoortStyleTestTrack(),
        "sepang" => TrackFactory.SepangStyleTestTrack(),
        "baku" => TrackFactory.BakuStyleTestTrack(),
        "spa" => TrackFactory.SpaStyleTestTrack(),
        "monza" => TrackFactory.MonzaStyleTestTrack(),
        "interlagos" => TrackFactory.InterlagosStyleTestTrack(),
        "singapore" => TrackFactory.SingaporeStyleTestTrack(),
        "portimao" => TrackFactory.PortimaoStyleTestTrack(),
        _ => throw new ArgumentOutOfRangeException(nameof(name)),
    };

    [Theory]
    [MemberData(nameof(Circuits))]
    public void ReportCentrelineQuality(string name)
    {
        TrackData track = Build(name);
        List<(float Station, float Curvature)> apexes = Apexes(track);
        output.WriteLine(
            $"{name,-12} 长 {track.LengthMeters,6:0} m  " +
            $"直道倾角变号 {BankSignChanges(track),4}  " +
            $"曲率变号 16m {StraightSignChanges(track, 16),4}  " +
            $"倾角台阶 p95 直道 {BankStepP95(track, true),6:0.000} " +
            $"全场 {BankStepP95(track, false),6:0.000}°/5m  " +
            $"弯 {apexes.Count,3}"
        );
        foreach ((float station, float curvature) in apexes)
            output.WriteLine(
                $"    T@{station,6:0} m  κ {curvature,8:0.00000}  " +
                $"R {(curvature == 0f ? 0f : 1f / MathF.Abs(curvature)),7:0}"
            );
        Assert.True(track.LengthMeters > 0f);
    }

    /// <summary>
    /// Every corner still bends as hard as the survey says it does.
    ///
    /// The fixture is the apex table measured from the centreline before
    /// the heading profile was cleaned — station and signed curvature for
    /// all 189 corners in the set. Cleaning is allowed to take ripple out
    /// of the straights; it is not allowed to round a corner off, and this
    /// is where that line is drawn. The comparison searches a short way
    /// either side of the recorded station because smoothing
    /// reparameterises the lap very slightly, so the same apex can sit a
    /// metre or two along from where it was.
    /// </summary>
    [Theory]
    [MemberData(nameof(Circuits))]
    public void CleaningDoesNotRoundTheCornersOff(string name)
    {
        TrackData track = Build(name);
        float[] curvature = Curvature(track, 20);
        int count = curvature.Length;
        List<string> broken = [];
        int checkedCorners = 0;

        foreach ((float station, float expected) in ApexFixture(name))
        {
            // The apex may have shifted; take the strongest curvature of
            // the same hand within ten metres.
            float best = 0f;
            for (int offset = -10; offset <= 10; offset++)
            {
                int at = (((int)station + offset) % count + count) % count;
                float value = curvature[at];
                if (value * expected > 0f && MathF.Abs(value) > MathF.Abs(best))
                    best = value;
            }

            checkedCorners++;
            float error = MathF.Abs(best - expected) / MathF.Abs(expected);
            if (error > 0.05f)
                broken.Add(
                    $"{station:0} m: κ {expected:0.00000} -> {best:0.00000} " +
                    $"({error * 100f:0.0}%)"
                );
        }

        Assert.True(checkedCorners > 0, $"{name} has no corners on file");
        Assert.True(
            broken.Count == 0,
            $"{name}: {broken.Count}/{checkedCorners} corners moved more " +
            $"than 5%: {string.Join("; ", broken)}"
        );
    }

    /// <summary>
    /// The road does not tip one way and then the other while going
    /// straight.
    ///
    /// Cross slope is written from the curvature the surface layer reads,
    /// so ripple in that curvature comes out as a road that leans left,
    /// then right, then left down a straight. No circuit does that, and
    /// unlike the ripple itself it is something a car can feel — which is
    /// why this is the quantity pinned rather than the curvature ripple
    /// that causes it.
    ///
    /// The curvature's own sign changes are deliberately not asserted on,
    /// and that is a finding rather than an omission. On genuinely
    /// straight road the true curvature is zero, node positions are held
    /// as thirty-two-bit floats with coordinates in the thousands of
    /// metres, and a heading differenced across a metre of that carries
    /// about a ten-thousandth of a radian of quantisation. The sign of a
    /// quantity that small flips on rounding, roughly every other metre,
    /// on every straight in the set — and no amount of smoothing the
    /// input removes it, because it is created after the smoothing, when
    /// the answer is written down. It is also harmless: the curvature it
    /// dithers by is a hundred-kilometre radius. Monza reports over nine
    /// hundred sign changes and is a correct road.
    /// </summary>
    [Theory]
    [MemberData(nameof(Circuits))]
    public void TheRoadDoesNotWobbleDownTheStraights(string name)
    {
        TrackData track = Build(name);
        // Measured worst case across the set is 0.060 degrees per five
        // metres, on Monaco, which has barely enough straight to measure.
        Assert.InRange(BankStepP95(track, straightOnly: true), 0f, 0.1f);
        // A ceiling against regression rather than a target: the measured
        // spread is 1 to 29, and the top of it is Shanghai, whose long
        // constant-radius curves put a lot of road just under the
        // threshold that counts as straight here.
        Assert.InRange(BankSignChanges(track), 0, 40);
    }

    private static IEnumerable<(float Station, float Curvature)> ApexFixture(
        string circuit
    )
    {
        DirectoryInfo? at = new(Directory.GetCurrentDirectory());
        while (at != null && !File.Exists(Path.Combine(at.FullName, "StintegyEVO.sln")))
            at = at.Parent;
        Assert.NotNull(at);
        string path = Path.Combine(
            at!.FullName, "Core", "Tests", "Fixtures", "centreline-apexes.csv"
        );
        foreach (string line in File.ReadAllLines(path))
        {
            if (line.Length == 0 || line[0] == '#')
                continue;
            string[] fields = line.Split(',');
            if (fields[0] != circuit)
                continue;
            yield return (
                float.Parse(fields[1], CultureInfo.InvariantCulture),
                float.Parse(fields[2], CultureInfo.InvariantCulture)
            );
        }
    }
}
