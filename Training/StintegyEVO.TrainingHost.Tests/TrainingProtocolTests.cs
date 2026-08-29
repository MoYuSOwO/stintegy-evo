using System.Buffers.Binary;
using System.Text;
using StintegyEVO.Core.Drivers;
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
        Assert.Equal(16, hello.Payload.Length);
        Assert.Equal(
            TacticalPolicyShape.ObservationSize,
            BinaryPrimitives.ReadInt32LittleEndian(hello.Payload)
        );
        Assert.Equal(
            TacticalPolicyShape.ActionSize,
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
                batchSize * TacticalPolicyShape.ActionSize * sizeof(float)
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
            batchSize * TacticalPolicyShape.ObservationSize * sizeof(float);
        int stepBytes = observationBytes +
                        batchSize * sizeof(float) +
                        batchSize * sizeof(byte) * 2 +
                        batchSize * TrainingStepResult.ComponentCount *
                        sizeof(float);
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
            TacticalPolicyShape.ObservationSize * sizeof(float),
            TacticalPolicyShape.ObservationSize * sizeof(float)
        );
        ReadOnlySpan<byte> secondAfterMaskedReset = maskedReset.Payload.AsSpan(
            TacticalPolicyShape.ObservationSize * sizeof(float),
            TacticalPolicyShape.ObservationSize * sizeof(float)
        );
        Assert.True(secondAfterStep.SequenceEqual(secondAfterMaskedReset));
        Assert.False(
            reset.Payload.AsSpan(
                0,
                TacticalPolicyShape.ObservationSize * sizeof(float)
            ).SequenceEqual(
                maskedReset.Payload.AsSpan(
                    0,
                    TacticalPolicyShape.ObservationSize * sizeof(float)
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
