using System.Buffers.Binary;

namespace StintegyEVO.TrainingHost.Protocol;

public sealed class TrainingMessage
{
    internal TrainingMessage(TrainingMessageKind kind, byte[] payload)
    {
        Kind = kind;
        Payload = payload;
    }

    public TrainingMessageKind Kind { get; }
    public byte[] Payload { get; }
}

public readonly record struct TrainingMessageHeader(
    TrainingMessageKind Kind,
    int PayloadLength
);

public static class TrainingProtocol
{
    public const uint Magic = 0x53544556;
    public const int Version = 1;
    public const int HeaderSize = 12;
    public const int MaxPayloadLength = 64 * 1024 * 1024;

    public static TrainingMessage ReadMessage(Stream stream)
    {
        if (!TryReadMessageHeader(stream, out TrainingMessageHeader header))
            throw new EndOfStreamException("No protocol header was available.");

        byte[] payload = GC.AllocateUninitializedArray<byte>(
            header.PayloadLength
        );
        ReadPayload(stream, payload);
        return new TrainingMessage(header.Kind, payload);
    }

    public static bool TryReadMessageHeader(
        Stream stream,
        out TrainingMessageHeader messageHeader
    )
    {
        ArgumentNullException.ThrowIfNull(stream);
        Span<byte> header = stackalloc byte[HeaderSize];
        int bytesRead = stream.Read(header);
        if (bytesRead == 0)
        {
            messageHeader = default;
            return false;
        }
        while (bytesRead < header.Length)
        {
            int read = stream.Read(header[bytesRead..]);
            if (read == 0)
            {
                throw new EndOfStreamException(
                    $"Protocol header ended after {bytesRead} of " +
                    $"{HeaderSize} bytes."
                );
            }
            bytesRead += read;
        }

        uint magic = BinaryPrimitives.ReadUInt32LittleEndian(header);
        ushort version = BinaryPrimitives.ReadUInt16LittleEndian(header[4..]);
        ushort rawKind = BinaryPrimitives.ReadUInt16LittleEndian(header[6..]);
        int payloadLength = BinaryPrimitives.ReadInt32LittleEndian(header[8..]);
        if (magic != Magic)
            throw new InvalidDataException($"Invalid protocol magic 0x{magic:X8}.");
        if (version != Version)
            throw new InvalidDataException($"Unsupported protocol version {version}.");
        if (!Enum.IsDefined(typeof(TrainingMessageKind), rawKind))
            throw new InvalidDataException($"Unknown message kind {rawKind}.");
        if (payloadLength < 0 || payloadLength > MaxPayloadLength)
        {
            throw new InvalidDataException(
                $"Payload length {payloadLength} is outside " +
                $"0..{MaxPayloadLength}."
            );
        }

        messageHeader = new TrainingMessageHeader(
            (TrainingMessageKind)rawKind,
            payloadLength
        );
        return true;
    }

    public static void ReadPayload(Stream stream, Span<byte> destination)
    {
        ArgumentNullException.ThrowIfNull(stream);
        stream.ReadExactly(destination);
    }

    public static void WriteMessage(
        Stream stream,
        TrainingMessageKind kind,
        ReadOnlySpan<byte> payload
    )
    {
        ArgumentNullException.ThrowIfNull(stream);
        if (!Enum.IsDefined(kind))
            throw new ArgumentOutOfRangeException(nameof(kind));
        if (payload.Length > MaxPayloadLength)
            throw new ArgumentOutOfRangeException(nameof(payload));

        Span<byte> header = stackalloc byte[HeaderSize];
        BinaryPrimitives.WriteUInt32LittleEndian(header, Magic);
        BinaryPrimitives.WriteUInt16LittleEndian(header[4..], Version);
        BinaryPrimitives.WriteUInt16LittleEndian(header[6..], (ushort)kind);
        BinaryPrimitives.WriteInt32LittleEndian(header[8..], payload.Length);
        stream.Write(header);
        stream.Write(payload);
        stream.Flush();
    }
}
