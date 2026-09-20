using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// The solo path, pinned to the bit.
///
/// Reviving wheel-to-wheel means moving the code every solo bake runs
/// through: the opponent is built in the same reset, sampled by the same
/// observation builder, stepped in the same simulation. Nothing in a solo
/// episode may move as a result -- not an observation channel, not a reward
/// component, not the order the episode's randomness is drawn in -- because
/// the parent chain is mid-bake against this world and a refactor that
/// shifts it by a float invalidates every checkpoint in the chain.
///
/// So: a fingerprint, taken on the tree before the duel work began and
/// committed beside this test. It replays fixed seeds on three circuits
/// with a fixed action script and hashes every observation channel and
/// every reward component of every step. The file is provenance -- if it is
/// missing the test writes it and fails, and the only honest way to produce
/// one is to run it on the pristine tree.
///
/// Re-taken on 2026-09-20 at the merge of master's cornering drag (#66):
/// the physics changed on purpose, the parent3 chain was retired at this
/// same merge (its heir continues by transplant onto this tree), so the
/// pin moves with the world it guards. From here it guards THIS tree's
/// solo path against the next refactor.
/// </summary>
public sealed class SoloFingerprintTests
{
    private static readonly string[] Circuits =
        ["silverstone", "monaco", "banked-sweeper"];

    private const int StepsPerEpisode = 240;

    private static string FingerprintPath =>
        Path.Combine(
            AppContext.BaseDirectory,
            "..", "..", "..", "..",
            "experiments", "2026-09-20-duel-arena", "solo-fingerprint.txt"
        );

    [Fact]
    public void ASoloEpisodeIsBitForBitWhatItWasBeforeTheDuelWork()
    {
        string measured = Fingerprint();
        string path = Path.GetFullPath(FingerprintPath);
        if (!File.Exists(path))
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            File.WriteAllText(path, measured + System.Environment.NewLine);
            Assert.Fail(
                $"No fingerprint was on record, so this run's was written to " +
                $"{path}. Check it was taken on the pristine tree, commit it, " +
                "and run again."
            );
        }

        string recorded = File.ReadAllText(path).Trim();
        Assert.Equal(recorded, measured);
    }

    /// <summary>
    /// Every channel and every component of a scripted solo run, hashed.
    /// The script is deliberately not a straight line: it turns, lifts and
    /// brakes, so the episode visits the grip limiter, the barrier and the
    /// off-course region rather than coasting down a straight.
    /// </summary>
    private static string Fingerprint()
    {
        using IncrementalHash hash = IncrementalHash.CreateHash(
            HashAlgorithmName.SHA256
        );
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        Span<float> action = stackalloc float[DirectDriveObservation.ActionSize];

        foreach (string circuit in Circuits)
        {
            foreach (bool curriculum in new[] { false, true })
            {
                DirectDriveDuelEnvironment environment = new(
                    solo: true,
                    randomiseEpisodeStart: curriculum,
                    hiddenCurriculum: curriculum
                );
                environment.ResetTrack(circuit, 20260920, observation);
                Absorb(hash, observation);
                for (int step = 0; step < StepsPerEpisode; step++)
                {
                    // A lap's worth of variety from a formula rather than a
                    // table: a slow weave with a faster throttle cycle, so
                    // the same script exercises a different part of every
                    // circuit it is run on.
                    action[0] = MathF.Sin(step * 0.11f) * 0.8f;
                    action[1] = MathF.Cos(step * 0.07f);
                    TrainingStepResult result = environment.Step(
                        action, observation
                    );
                    Absorb(hash, observation);
                    for (int i = 0; i < TrainingStepResult.ComponentCount; i++)
                        Absorb(hash, result.GetComponent(i));
                    Absorb(hash, (int)result.TerminalReason);
                    Absorb(hash, environment.SpinEventsThisStep);
                    Absorb(hash, environment.Ego.Progress.RaceDistanceMeters);
                    Absorb(hash, environment.FourWheelsOffSecondsThisStep);
                    if (environment.IsTerminal)
                    {
                        environment.ResetTrack(circuit, 20260920 + step, observation);
                        Absorb(hash, observation);
                    }
                }
            }
        }

        return Convert.ToHexString(hash.GetHashAndReset()).ToLowerInvariant();
    }

    private static void Absorb(IncrementalHash hash, ReadOnlySpan<float> values)
    {
        foreach (float value in values)
            Absorb(hash, value);
    }

    private static void Absorb(IncrementalHash hash, float value) =>
        hash.AppendData(BitConverter.GetBytes(BitConverter.SingleToInt32Bits(value)));

    private static void Absorb(IncrementalHash hash, int value) =>
        hash.AppendData(BitConverter.GetBytes(value));
}
