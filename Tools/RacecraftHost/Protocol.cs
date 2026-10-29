using System.Buffers.Binary;

namespace StintegyEVO.RacecraftHost;

public enum MessageKind : ushort
{
    Hello = 1,
    HelloResponse = 2,
    Reset = 3,
    ResetResponse = 4,
    Step = 5,
    StepResponse = 6,
    Close = 7,
    CloseResponse = 8,
    Error = ushort.MaxValue
}

/// <summary>
/// Framed binary messages over a stream: a twelve-byte header (magic,
/// version, kind, payload length, little-endian) and the payload.
/// </summary>
public static class Protocol
{
    public const uint Magic = 0x52435654;
    public const ushort Version = 1;
    public const int HeaderSize = 12;
    public const int MaxPayloadLength = 64 * 1024 * 1024;

    /// <summary>The next message, or null at the end of the stream.</summary>
    public static (MessageKind Kind, byte[] Payload)? Read(Stream stream)
    {
        Span<byte> header = stackalloc byte[HeaderSize];
        int read = stream.ReadAtLeast(header, HeaderSize, throwOnEndOfStream: false);
        if (read == 0)
            return null;
        if (read < HeaderSize)
            throw new EndOfStreamException($"The header ended after {read} of {HeaderSize} bytes.");
        uint magic = BinaryPrimitives.ReadUInt32LittleEndian(header);
        ushort version = BinaryPrimitives.ReadUInt16LittleEndian(header[4..]);
        ushort kind = BinaryPrimitives.ReadUInt16LittleEndian(header[6..]);
        int length = BinaryPrimitives.ReadInt32LittleEndian(header[8..]);
        if (magic != Magic)
            throw new InvalidDataException($"Not a racecraft message: magic 0x{magic:X8}.");
        if (version != Version)
            throw new InvalidDataException($"Protocol version {version}, the host speaks {Version}.");
        if (!Enum.IsDefined(typeof(MessageKind), kind))
            throw new InvalidDataException($"Unknown message kind {kind}.");
        if (length < 0 || length > MaxPayloadLength)
            throw new InvalidDataException($"Payload length {length} is outside 0..{MaxPayloadLength}.");
        byte[] payload = new byte[length];
        stream.ReadExactly(payload);
        return ((MessageKind)kind, payload);
    }

    public static void Write(Stream stream, MessageKind kind, ReadOnlySpan<byte> payload)
    {
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
