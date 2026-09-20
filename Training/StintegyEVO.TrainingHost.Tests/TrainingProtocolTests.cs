using System.Buffers.Binary;
using System.Text;
using StintegyEVO.Core.Drivers;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using StintegyEVO.TrainingHost.Protocol;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

public sealed class TrainingProtocolTests
{
    [Fact]
    public void ProtocolRoundTripsFramedPayload()
    {
        using MemoryStream stream = new();
        byte[] payload = [1, 2, 3, 4, 5];

        TrainingProtocol.WriteMessage(
            stream,
            TrainingMessageKind.Reset,
            payload
        );
        stream.Position = 0;

        TrainingMessage message = TrainingProtocol.ReadMessage(stream);

        Assert.Equal(TrainingMessageKind.Reset, message.Kind);
        Assert.Equal(payload, message.Payload);
    }

    [Fact]
    public void HelloReportsTacticalTensorShapes()
    {
        using MemoryStream input = new();
        using MemoryStream output = new();
        TrainingProtocol.WriteMessage(
            input,
            TrainingMessageKind.Hello,
            ReadOnlySpan<byte>.Empty
        );
        TrainingProtocol.WriteMessage(
            input,
            TrainingMessageKind.Close,
            ReadOnlySpan<byte>.Empty
        );
        input.Position = 0;
        BatchedTrainingHost host = new(batchSize: 2, seedBase: 100);

        host.Run(input, output, TextWriter.Null);

        output.Position = 0;
        TrainingMessage hello = TrainingProtocol.ReadMessage(output);
        Assert.Equal(TrainingMessageKind.HelloResponse, hello.Kind);
        Assert.Equal(20, hello.Payload.Length);
        Assert.Equal(
            DirectDriveObservation.ObservationSize,
            BinaryPrimitives.ReadInt32LittleEndian(hello.Payload)
        );
        Assert.Equal(
            DirectDriveObservation.ActionSize,
            BinaryPrimitives.ReadInt32LittleEndian(hello.Payload.AsSpan(4))
        );
        Assert.Equal(
            2,
            BinaryPrimitives.ReadInt32LittleEndian(hello.Payload.AsSpan(8))
        );
        Assert.Equal(
            TrainingProtocol.Version,
            BinaryPrimitives.ReadInt32LittleEndian(hello.Payload.AsSpan(12))
        );
        // Seats per lane (protocol 6). One here: the host defaults to solo,
        // and a client that asked for nothing gets one car.
        Assert.Equal(
            1,
            BinaryPrimitives.ReadInt32LittleEndian(hello.Payload.AsSpan(16))
        );
    }

    /// <summary>
    /// A duel host says two, and its step messages are two seats wide in
    /// both directions: the client sizes its buffers from this number, so
    /// getting it wrong is a protocol desynchronisation rather than a bad
    /// observation.
    /// </summary>
    [Fact]
    public void ADuelHostAnnouncesTwoSeatsAndSpeaksInThem()
    {
        const int batchSize = 2;
        using MemoryStream input = new();
        using MemoryStream output = new();
        TrainingProtocol.WriteMessage(
            input, TrainingMessageKind.Hello, ReadOnlySpan<byte>.Empty
        );
        byte[] actions = new byte[
            batchSize * 2 * DirectDriveObservation.ActionSize * sizeof(float)
        ];
        TrainingProtocol.WriteMessage(input, TrainingMessageKind.Step, actions);
        TrainingProtocol.WriteMessage(
            input, TrainingMessageKind.Close, ReadOnlySpan<byte>.Empty
        );
        input.Position = 0;
        BatchedTrainingHost host = new(
            batchSize: batchSize, seedBase: 100, solo: false
        );

        host.Run(input, output, TextWriter.Null);

        output.Position = 0;
        TrainingMessage hello = TrainingProtocol.ReadMessage(output);
        Assert.Equal(
            2,
            BinaryPrimitives.ReadInt32LittleEndian(hello.Payload.AsSpan(16))
        );
        TrainingMessage step = TrainingProtocol.ReadMessage(output);
        Assert.Equal(TrainingMessageKind.StepResponse, step.Kind);
        int observationBytes =
            batchSize * 2 * DirectDriveObservation.ObservationSize * sizeof(float);
        int scoreboardBytes =
            batchSize * sizeof(float) +          // reward
            batchSize * sizeof(byte) * 2 +       // done, terminal reason
            batchSize * TrainingStepResult.ComponentCount * sizeof(float) +
            batchSize * sizeof(float) +          // race distance
            batchSize * sizeof(byte) +           // spins
            batchSize * sizeof(float) +          // four wheels off
            batchSize * sizeof(float);           // signed lead, duel only
        Assert.Equal(observationBytes + scoreboardBytes, step.Payload.Length);
        // The second seat is a real observation and not zero padding.
        int second = DirectDriveObservation.ObservationSize * sizeof(float);
        Assert.NotEqual(
            0f,
            BitConverter.ToSingle(step.Payload, second)
        );
    }

    [Fact]
    public void HostResetStepAndMaskedResetPreserveBatchOrdering()
    {
        const int batchSize = 2;
        using MemoryStream input = new();
        using MemoryStream output = new();
        byte[] resetPayload = new byte[batchSize * sizeof(long)];
        BinaryPrimitives.WriteInt64LittleEndian(resetPayload, 10L);
        BinaryPrimitives.WriteInt64LittleEndian(resetPayload.AsSpan(8), 20L);
        TrainingProtocol.WriteMessage(
            input,
            TrainingMessageKind.Reset,
            resetPayload
        );
        TrainingProtocol.WriteMessage(
            input,
            TrainingMessageKind.Step,
            new byte[
                batchSize * DirectDriveObservation.ActionSize * sizeof(float)
            ]
        );
        byte[] maskedResetPayload = new byte[
            batchSize + batchSize * sizeof(long)
        ];
        maskedResetPayload[0] = 1;
        maskedResetPayload[1] = 0;
        BinaryPrimitives.WriteInt64LittleEndian(
            maskedResetPayload.AsSpan(batchSize),
            30L
        );
        BinaryPrimitives.WriteInt64LittleEndian(
            maskedResetPayload.AsSpan(batchSize + sizeof(long)),
            40L
        );
        TrainingProtocol.WriteMessage(
            input,
            TrainingMessageKind.MaskedReset,
            maskedResetPayload
        );
        TrainingProtocol.WriteMessage(
            input,
            TrainingMessageKind.Close,
            ReadOnlySpan<byte>.Empty
        );
        input.Position = 0;
        BatchedTrainingHost host = new(batchSize);

        host.Run(input, output, TextWriter.Null);

        output.Position = 0;
        TrainingMessage reset = TrainingProtocol.ReadMessage(output);
        TrainingMessage step = TrainingProtocol.ReadMessage(output);
        TrainingMessage maskedReset = TrainingProtocol.ReadMessage(output);
        TrainingMessage close = TrainingProtocol.ReadMessage(output);
        int observationBytes =
            batchSize * DirectDriveObservation.ObservationSize * sizeof(float);
        int stepBytes = observationBytes +
                        batchSize * sizeof(float) +
                        batchSize * sizeof(byte) * 2 +
                        batchSize * TrainingStepResult.ComponentCount *
                        sizeof(float) +
                        // Race distance per lane, added with protocol two so
                        // a lap could be timed at the line rather than
                        // guessed from an average pace.
                        batchSize * sizeof(float) +
                        // Spins begun this step, added with protocol three so
                        // a lap that was survived could be told from one that
                        // was driven.
                        batchSize * sizeof(byte) +
                        // Seconds with all four wheels over the white line,
                        // added with protocol four for the race's
                        // track-limits ruler; scoreboard only.
                        batchSize * sizeof(float);
        Assert.Equal(TrainingMessageKind.ResetResponse, reset.Kind);
        Assert.Equal(observationBytes, reset.Payload.Length);
        Assert.Equal(TrainingMessageKind.StepResponse, step.Kind);
        Assert.Equal(stepBytes, step.Payload.Length);
        Assert.Equal(
            TrainingMessageKind.MaskedResetResponse,
            maskedReset.Kind
        );
        Assert.Equal(observationBytes, maskedReset.Payload.Length);
        Assert.Equal(TrainingMessageKind.CloseResponse, close.Kind);

        ReadOnlySpan<byte> secondAfterStep = step.Payload.AsSpan(
            DirectDriveObservation.ObservationSize * sizeof(float),
            DirectDriveObservation.ObservationSize * sizeof(float)
        );
        ReadOnlySpan<byte> secondAfterMaskedReset = maskedReset.Payload.AsSpan(
            DirectDriveObservation.ObservationSize * sizeof(float),
            DirectDriveObservation.ObservationSize * sizeof(float)
        );
        Assert.True(secondAfterStep.SequenceEqual(secondAfterMaskedReset));
        Assert.False(
            reset.Payload.AsSpan(
                0,
                DirectDriveObservation.ObservationSize * sizeof(float)
            ).SequenceEqual(
                maskedReset.Payload.AsSpan(
                    0,
                    DirectDriveObservation.ObservationSize * sizeof(float)
                )
            )
        );
    }

    [Fact]
    public void HostRejectsWrongStepPayloadLength()
    {
        using MemoryStream input = new();
        using MemoryStream output = new();
        TrainingProtocol.WriteMessage(
            input,
            TrainingMessageKind.Step,
            new byte[sizeof(float)]
        );
        input.Position = 0;
        BatchedTrainingHost host = new(batchSize: 2);

        host.Run(input, output, TextWriter.Null);

        output.Position = 0;
        TrainingMessage error = TrainingProtocol.ReadMessage(output);
        Assert.Equal(TrainingMessageKind.Error, error.Kind);
        Assert.Contains(
            "payload",
            Encoding.UTF8.GetString(error.Payload),
            StringComparison.OrdinalIgnoreCase
        );
    }
}
