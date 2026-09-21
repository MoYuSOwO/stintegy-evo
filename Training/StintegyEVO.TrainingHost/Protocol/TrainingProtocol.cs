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
    /// <summary>
    /// Version 2 appends each lane's along-track race distance to the step
    /// response. It is not an observation and never reaches the policy — the
    /// harness needs it to time a lap, which nothing else in the protocol
    /// could do: the progress reward is masked off course and the
    /// observation is forbidden absolute position by design.
    ///
    /// Version 3 appends the number of spin events each lane began during
    /// the step, for the same reason and on the same terms. A lap that was
    /// survived rather than driven has to be tellable from one that was not,
    /// and the policy is no more entitled to be told it spun than it is to be
    /// told where on Earth it is.
    ///
    /// Version 4 appends how many seconds of the step each lane spent with
    /// all four wheels over the white line -- the race steward's track-limits
    /// ruler, read for certification. It is a scoreboard field on the same
    /// terms as the two before it, and it is never a reward: training keeps
    /// the stricter centreline ruler, which is where the car's margin comes
    /// from.
    ///
    /// Version 5 is the world-v3 freeze batch: a 457-channel observation, a
    /// twelfth reward component (the budget shaping), and a sixth terminal
    /// reason (finished: the race's flag).
    ///
    /// Version 6 carries wheel-to-wheel. The handshake gains a fifth field,
    /// the number of cars a lane carries — one solo, two in a duel — and
    /// every lane's observations and actions are that many in a row, the
    /// ego's first. Rewards, terminals and the scoreboard fields stay one
    /// per lane: only the ego is being trained, and the partner's race is
    /// nobody's reward. A duel appends one more scoreboard field, the
    /// signed lead in metres (positive when the partner is ahead), because
    /// the alternative is asking the policy's own observation who won.
    /// </summary>
    ///
    /// Version 7 carries two more reward components, the steering costs.
    /// A component count is a payload length on both sides of the pipe, so
    /// a client and a host that disagree about it disagree about where
    /// every field after the components begins: the version is what stops
    /// that being discovered as a plausible-looking number.
    /// </summary>
    public const int Version = 7;
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
