using System;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Drivers.Learned;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// What the observation promises about its own shape.
///
/// The blocks added here — generic resource slots and the reserved vehicle
/// descriptors — are mostly empty on the car that exists today, and empty
/// things are exactly what rots without being noticed. A dormant channel
/// that quietly starts carrying something, or a slot that stops saying it
/// is absent, would not fail any driving test; it would just teach the
/// next policy something false. So the emptiness is asserted.
/// </summary>
public sealed class ObservationContractTests
{
    private const float Dt = 1f / 60f;

    /// <summary>
    /// Drives straight and slow, and keeps a copy of the first observation
    /// it is handed. The driving is irrelevant; the vector is the point.
    /// </summary>
    private sealed class CapturingPolicy : IDrivingPolicy
    {
        public float[]? Captured { get; private set; }

        public void Act(
            ReadOnlySpan<float> observation,
            Span<float> action
        )
        {
            Captured ??= observation.ToArray();
            action[0] = 0f;
            action[1] = 0f;
        }
    }

    private static float[] CaptureObservation(float charge)
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        CapturingPolicy policy = new();
        TrackSample sample = track.Sample(100f);
        RaceCar car = new(
            "observation-contract",
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            new DirectDriveRaceDriver(policy),
            new CarState
            {
                Position = sample.RefPosition,
                Heading = sample.RefHeading,
                Speed = 40f,
                Energy = PowertrainState.Filled(charge)
            }
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        for (int i = 0; i < 30 && policy.Captured is null; i++)
            simulation.Step(Dt);

        Assert.NotNull(policy.Captured);
        Assert.Equal(
            DirectDriveObservation.ObservationSize,
            policy.Captured!.Length
        );
        return policy.Captured!;
    }

    /// <summary>
    /// The car's one store fills slot zero and says what kind of store it
    /// is; the strategist's two channels are silent.
    /// </summary>
    [Fact]
    public void TheBatteryFillsTheFirstResourceSlotAndTheBudgetChannelsAreSilent()
    {
        float[] observation = CaptureObservation(charge: 0.6f);
        int at = DirectDriveObservation.ResourceSlotsOffset;

        Assert.Equal(1f, observation[at], 5);
        Assert.Equal(
            (float)PowertrainResourceClass.Battery /
            DirectDriveObservation.ResourceClassScale,
            observation[at + 1],
            5
        );
        Assert.Equal(0.6f, observation[at + 2], 3);
        // Dormant until the budget re-core lands. If either of these ever
        // reads non-zero before that, something is writing to a channel the
        // network has been told means nothing.
        Assert.Equal(0f, observation[at + 3], 6);
        Assert.Equal(0f, observation[at + 4], 6);
    }

    /// <summary>
    /// A slot with nothing in it says so on every channel, not just the
    /// presence flag.
    ///
    /// This is the rule that absence and zero never share an encoding,
    /// applied where it is easiest to break: the flag is what carries
    /// absence, and the remaining channels have to stay quiet so that the
    /// flag is the only thing the network has to learn to read.
    /// </summary>
    [Fact]
    public void UnusedResourceSlotsAreEmptyOnEveryChannel()
    {
        float[] observation = CaptureObservation(charge: 0.6f);
        for (int slot = 1; slot < DirectDriveObservation.ResourceSlotCount; slot++)
        {
            int at = DirectDriveObservation.ResourceSlotsOffset +
                     slot * DirectDriveObservation.ResourceSlotChannels;
            for (int c = 0; c < DirectDriveObservation.ResourceSlotChannels; c++)
            {
                Assert.Equal(0f, observation[at + c], 6);
            }
        }
    }

    /// <summary>
    /// The descriptor block is ground held for the setup vector and holds
    /// nothing yet. When it starts carrying values this test is the thing
    /// that should be changed to say so, deliberately.
    /// </summary>
    [Fact]
    public void TheVehicleDescriptorBlockIsReservedAndEmpty()
    {
        float[] observation = CaptureObservation(charge: 0.6f);
        for (int i = 0; i < DirectDriveObservation.VehicleDescriptorSize; i++)
        {
            Assert.Equal(
                0f,
                observation[DirectDriveObservation.VehicleDescriptorOffset + i],
                6
            );
        }
    }

    /// <summary>
    /// The absence audit the batch owed: with nobody else on track, every
    /// opponent slot is empty on every channel.
    ///
    /// The hazard this guards is specific. An absent opponent and one
    /// sitting exactly level with the car both read zero on the gap
    /// channels; only the presence flag separates them. That is allowed —
    /// the flag is a dedicated, static channel and the rule asks for
    /// exactly that — but it means the flag has to be right, and a slot
    /// that leaked stale values from a car that has since been lapped
    /// would be indistinguishable from a real one.
    /// </summary>
    [Fact]
    public void WithNobodyElseOnTrackEveryOpponentSlotIsEmpty()
    {
        float[] observation = CaptureObservation(charge: 0.6f);
        for (int slot = 0; slot < DirectDriveObservation.OpponentCount; slot++)
        {
            int at = DirectDriveObservation.OpponentOffset +
                     slot * DirectDriveObservation.OpponentSize;
            for (int c = 0; c < DirectDriveObservation.OpponentSize; c++)
            {
                Assert.Equal(0f, observation[at + c], 6);
            }
        }
    }

    /// <summary>
    /// The blocks are laid out end to end with no gap and no overlap, and
    /// the dynamic slice that gets copied for the previous frame still
    /// covers exactly ego plus opponents.
    ///
    /// The second half of that is not decoration. When the ego and opponent
    /// blocks were once separated by other blocks, the previous-frame copy
    /// ran off the end of the opponents and the last car and a bit was
    /// silently missing from every previous frame. Adding two blocks is
    /// exactly the operation that could do it again.
    /// </summary>
    [Fact]
    public void TheBlocksTileTheVectorAndTheDynamicSliceStillFits()
    {
        Assert.Equal(
            DirectDriveObservation.ResourceSlotsOffset,
            DirectDriveObservation.RoadAndLimitsOffset +
            DirectDriveObservation.RoadAndLimitsSize
        );
        Assert.Equal(
            DirectDriveObservation.VehicleDescriptorOffset,
            DirectDriveObservation.ResourceSlotsOffset +
            DirectDriveObservation.ResourceSlotsSize
        );
        Assert.Equal(
            DirectDriveObservation.EgoOffset,
            DirectDriveObservation.VehicleDescriptorOffset +
            DirectDriveObservation.VehicleDescriptorSize
        );
        Assert.Equal(
            DirectDriveObservation.OpponentOffset,
            DirectDriveObservation.EgoOffset + DirectDriveObservation.EgoSize
        );

        Assert.Equal(
            DirectDriveObservation.EgoOffset,
            DirectDriveObservation.DynamicBlockOffset
        );
        Assert.Equal(
            DirectDriveObservation.DynamicBlockSize,
            DirectDriveObservation.EgoSize +
            DirectDriveObservation.OpponentCount *
            DirectDriveObservation.OpponentSize
        );
        Assert.Equal(
            DirectDriveObservation.ObservationSize,
            DirectDriveObservation.PreviousDynamicOffset +
            DirectDriveObservation.DynamicBlockSize
        );
    }
}
