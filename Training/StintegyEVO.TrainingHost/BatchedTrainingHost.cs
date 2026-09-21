using System.Buffers.Binary;
using System.Text;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using StintegyEVO.TrainingHost.Protocol;

namespace StintegyEVO.TrainingHost;

public sealed class BatchedTrainingHost
{
    private readonly int _batchSize;
    private readonly int _carsPerLane;
    private readonly string? _trackFamily;
    private readonly DirectDriveDuelEnvironment[] _environments;
    private readonly float[] _observations;
    private readonly float[] _actions;
    private readonly TrainingStepResult[] _results;
    private readonly Exception?[] _workerExceptions;
    private readonly byte[] _requestBuffer;
    private readonly byte[] _responseBuffer;
    private readonly int _resetRequestBytes;
    private readonly int _maskedResetRequestBytes;
    private readonly int _stepRequestBytes;
    private readonly Action<int> _resetOneAction;
    private readonly Action<int> _stepOneAction;
    private readonly Action<int> _maskedResetOneAction;

    public BatchedTrainingHost(
        int batchSize,
        long seedBase = 0,
        string? trackFamily = null,
        float minimumForwardGapMeters =
            DirectDriveDuelEnvironment.DefaultMinimumForwardGapMeters,
        float maximumForwardGapMeters =
            DirectDriveDuelEnvironment.DefaultMaximumForwardGapMeters,
        float episodeDurationSeconds =
            DirectDriveDuelEnvironment.DefaultEpisodeDurationSeconds,
        CarStrategy? opponentStrategy = null,
        float opponentPace = 70f,
        bool solo = true,
        CarStrategy? egoStrategy = null,
        float decisionHz = DirectDriveController.DefaultDecisionHz,
        bool randomiseEpisodeStart = false,
        EpisodeStartDistribution? episodeStarts = null,
        bool hiddenCurriculum = false,
        float raceKilometres = EnergyBudget.DefaultRaceKilometres,
        float budgetLambda = EnergyBudget.DefaultLambda,
        float budgetGamma = EnergyBudget.DefaultGamma,
        bool deltaActions = false
    )
    {
        if (batchSize <= 0)
            throw new ArgumentOutOfRangeException(nameof(batchSize));

        _batchSize = batchSize;
        // One seat solo, two in a duel. Every buffer below is sized in
        // seats rather than lanes, and the ego is always the first seat of
        // its lane, so the solo layout is the duel layout with the second
        // seat absent rather than a different shape.
        _carsPerLane = solo ? 1 : 2;
        _trackFamily = string.IsNullOrWhiteSpace(trackFamily)
            ? null
            : trackFamily;
        int observationCount = checked(
            batchSize * _carsPerLane * DirectDriveObservation.ObservationSize
        );
        int actionCount = checked(
            batchSize * _carsPerLane * DirectDriveObservation.ActionSize
        );
        _resetRequestBytes = checked(batchSize * sizeof(long));
        _maskedResetRequestBytes = checked(
            batchSize * (sizeof(byte) + sizeof(long))
        );
        _stepRequestBytes = checked(actionCount * sizeof(float));
        int stepResponseBytes = checked(
            observationCount * sizeof(float) +
            batchSize * sizeof(float) +
            batchSize * sizeof(byte) * 2 +
            batchSize * TrainingStepResult.ComponentCount * sizeof(float) +
            // The along-track race distance appended for the lap timer.
            batchSize * sizeof(float) +
            // And the spin events begun this step, for the scoreboard.
            batchSize * sizeof(byte) +
            // And the seconds all four wheels spent over the white line.
            batchSize * sizeof(float) +
            // And, in a duel, the signed lead in metres: who is in front.
            (solo ? 0 : batchSize * sizeof(float))
        );
        if (stepResponseBytes > TrainingProtocol.MaxPayloadLength)
        {
            throw new ArgumentOutOfRangeException(
                nameof(batchSize),
                "Batch response exceeds the protocol payload limit."
            );
        }

        _environments = new DirectDriveDuelEnvironment[batchSize];
        _observations = new float[observationCount];
        _actions = new float[actionCount];
        _results = new TrainingStepResult[batchSize];
        _workerExceptions = new Exception?[batchSize];
        _requestBuffer = new byte[
            Math.Max(
                _maskedResetRequestBytes,
                Math.Max(_resetRequestBytes, _stepRequestBytes)
            )
        ];
        _responseBuffer = new byte[Math.Max(stepResponseBytes, 20)];
        _resetOneAction = ResetOne;
        _stepOneAction = StepOne;
        _maskedResetOneAction = MaskedResetOne;

        for (int i = 0; i < batchSize; i++)
        {
            _environments[i] = new DirectDriveDuelEnvironment(
                minimumForwardGapMeters,
                maximumForwardGapMeters,
                episodeDurationSeconds,
                opponentStrategy,
                opponentPace,
                solo,
                egoStrategy,
                decisionHz,
                randomiseEpisodeStart,
                episodeStarts,
                hiddenCurriculum,
                raceKilometres,
                budgetLambda,
                budgetGamma,
                deltaActions
            );
            ResetEnvironment(i, unchecked(seedBase + i));
        }
    }

    public void Run(Stream input, Stream output, TextWriter diagnostics)
    {
        ArgumentNullException.ThrowIfNull(input);
        ArgumentNullException.ThrowIfNull(output);
        ArgumentNullException.ThrowIfNull(diagnostics);

        while (true)
        {
            TrainingMessageHeader header;
            try
            {
                if (!TrainingProtocol.TryReadMessageHeader(input, out header))
                    return;
            }
            catch (Exception exception) when (
                exception is InvalidDataException or IOException
            )
            {
                WriteError(output, diagnostics, exception.Message);
                return;
            }

            if (!TryExpectedPayloadLength(header.Kind, out int expectedLength))
            {
                DiscardPayload(input, header.PayloadLength);
                WriteError(
                    output,
                    diagnostics,
                    $"Invalid request kind {header.Kind}."
                );
                continue;
            }
            if (header.PayloadLength != expectedLength)
            {
                DiscardPayload(input, header.PayloadLength);
                WriteError(
                    output,
                    diagnostics,
                    $"{header.Kind} payload must be {expectedLength} bytes; " +
                    $"received {header.PayloadLength}."
                );
                continue;
            }

            TrainingProtocol.ReadPayload(
                input,
                _requestBuffer.AsSpan(0, header.PayloadLength)
            );
            try
            {
                switch (header.Kind)
                {
                    case TrainingMessageKind.Hello:
                        WriteHello(output);
                        break;
                    case TrainingMessageKind.Reset:
                        ProcessReset(output);
                        break;
                    case TrainingMessageKind.MaskedReset:
                        ProcessMaskedReset(output);
                        break;
                    case TrainingMessageKind.Step:
                        ProcessStep(output);
                        break;
                    case TrainingMessageKind.Close:
                        TrainingProtocol.WriteMessage(
                            output,
                            TrainingMessageKind.CloseResponse,
                            ReadOnlySpan<byte>.Empty
                        );
                        return;
                }
            }
            catch (Exception exception) when (
                exception is InvalidDataException or ArgumentException or
                    InvalidOperationException
            )
            {
                WriteError(output, diagnostics, exception.Message);
            }
        }
    }

    private void WriteHello(Stream output)
    {
        Span<byte> payload = _responseBuffer.AsSpan(0, 20);
        BinaryPrimitives.WriteInt32LittleEndian(
            payload,
            DirectDriveObservation.ObservationSize
        );
        BinaryPrimitives.WriteInt32LittleEndian(
            payload[4..],
            DirectDriveObservation.ActionSize
        );
        BinaryPrimitives.WriteInt32LittleEndian(payload[8..], _batchSize);
        BinaryPrimitives.WriteInt32LittleEndian(
            payload[12..],
            TrainingProtocol.Version
        );
        // Seats per lane: what the client has to send actions for and will
        // be sent observations for.
        BinaryPrimitives.WriteInt32LittleEndian(payload[16..], _carsPerLane);
        TrainingProtocol.WriteMessage(
            output,
            TrainingMessageKind.HelloResponse,
            payload
        );
    }

    private void ProcessReset(Stream output)
    {
        ExecuteBatch(_resetOneAction);
        WriteObservationResponse(output, TrainingMessageKind.ResetResponse);
    }

    private void ProcessMaskedReset(Stream output)
    {
        ExecuteBatch(_maskedResetOneAction);
        WriteObservationResponse(
            output,
            TrainingMessageKind.MaskedResetResponse
        );
    }

    private void ProcessStep(Stream output)
    {
        for (int i = 0; i < _batchSize; i++)
        {
            if (_environments[i].IsTerminal)
            {
                throw new InvalidOperationException(
                    $"Environment {i} is terminal and must be reset before Step."
                );
            }
        }
        ReadFloats(
            _requestBuffer.AsSpan(0, _stepRequestBytes),
            _actions
        );
        ExecuteBatch(_stepOneAction);

        int offset = WriteFloats(_observations, _responseBuffer, 0);
        for (int i = 0; i < _batchSize; i++)
            offset = WriteFloat(_responseBuffer, offset, _results[i].Reward);
        for (int i = 0; i < _batchSize; i++)
            _responseBuffer[offset++] = _results[i].Done ? (byte)1 : (byte)0;
        for (int i = 0; i < _batchSize; i++)
            _responseBuffer[offset++] = (byte)_results[i].TerminalReason;
        for (int component = 0;
             component < TrainingStepResult.ComponentCount;
             component++)
        {
            for (int i = 0; i < _batchSize; i++)
            {
                offset = WriteFloat(
                    _responseBuffer,
                    offset,
                    _results[i].GetComponent(component)
                );
            }
        }

        // Where each car is round the lap, which is the one thing a lap
        // timer needs and nothing else here carries. Continuous across the
        // start line and unaffected by leaving the road, unlike both the
        // progress reward and the observation.
        for (int i = 0; i < _batchSize; i++)
        {
            offset = WriteFloat(
                _responseBuffer,
                offset,
                _environments[i].Ego.Progress.RaceDistanceMeters
            );
        }

        // How many times each lane was declared lost during this step. A
        // byte because a fifteenth of a second cannot hold two spins, and the
        // clamp is there so that a bug upstream shows up as a saturated count
        // rather than as a wrapped one.
        for (int i = 0; i < _batchSize; i++)
        {
            _responseBuffer[offset++] = (byte)Math.Clamp(
                _environments[i].SpinEventsThisStep,
                0,
                255
            );
        }

        // How long all four of each lane's wheels were over the white line
        // this step: the race's track-limits ruler, for certification. The
        // reward keeps charging the stricter centreline ruler; this is read
        // by the scoreboard only.
        for (int i = 0; i < _batchSize; i++)
        {
            offset = WriteFloat(
                _responseBuffer,
                offset,
                _environments[i].FourWheelsOffSecondsThisStep
            );
        }

        // Who is in front, in metres, positive while the sparring partner
        // leads. The scoreboard's own reading of the duel: the alternative
        // is reconstructing it from the ego's opponent block, which is
        // scaled, clipped and noised for the policy's benefit.
        if (_carsPerLane > 1)
        {
            for (int i = 0; i < _batchSize; i++)
            {
                offset = WriteFloat(
                    _responseBuffer,
                    offset,
                    _environments[i].SignedLeadDistanceMeters
                );
            }
        }

        TrainingProtocol.WriteMessage(
            output,
            TrainingMessageKind.StepResponse,
            _responseBuffer.AsSpan(0, offset)
        );
    }

    private void ResetOne(int index)
    {
        long seed = BinaryPrimitives.ReadInt64LittleEndian(
            _requestBuffer.AsSpan(index * sizeof(long), sizeof(long))
        );
        ResetEnvironment(index, seed);
    }

    private void MaskedResetOne(int index)
    {
        if (_requestBuffer[index] == 0)
            return;
        int seedsOffset = _batchSize + index * sizeof(long);
        long seed = BinaryPrimitives.ReadInt64LittleEndian(
            _requestBuffer.AsSpan(seedsOffset, sizeof(long))
        );
        ResetEnvironment(index, seed);
    }

    private void ResetEnvironment(int index, long seed)
    {
        if (_trackFamily is null)
        {
            _environments[index].Reset(
                seed, ObservationFor(index), OpponentObservationFor(index)
            );
        }
        else
        {
            _environments[index].ResetTrack(
                _trackFamily,
                seed,
                ObservationFor(index),
                OpponentObservationFor(index)
            );
        }
    }

    private void StepOne(int index)
    {
        int actionOffset =
            index * _carsPerLane * DirectDriveObservation.ActionSize;
        ReadOnlySpan<float> ego = _actions.AsSpan(
            actionOffset, DirectDriveObservation.ActionSize
        );
        ReadOnlySpan<float> opponent = _carsPerLane == 1
            ? ReadOnlySpan<float>.Empty
            : _actions.AsSpan(
                actionOffset + DirectDriveObservation.ActionSize,
                DirectDriveObservation.ActionSize
            );
        _results[index] = _environments[index].Step(
            ego,
            opponent,
            ObservationFor(index),
            OpponentObservationFor(index)
        );
    }

    private void ExecuteBatch(Action<int> action)
    {
        Array.Clear(_workerExceptions);
        if (_batchSize == 1)
            ExecuteOne(action, 0);
        else
            Parallel.For(0, _batchSize, index => ExecuteOne(action, index));

        for (int i = 0; i < _workerExceptions.Length; i++)
        {
            if (_workerExceptions[i] is Exception exception)
            {
                throw new InvalidOperationException(
                    $"Environment {i} failed: {exception.Message}",
                    exception
                );
            }
        }
    }

    private void ExecuteOne(Action<int> action, int index)
    {
        try
        {
            action(index);
        }
        catch (Exception exception)
        {
            _workerExceptions[index] = exception;
        }
    }

    /// <summary>The ego's seat in a lane: the first of that lane's seats.</summary>
    private Span<float> ObservationFor(int index) => SeatFor(index, 0);

    /// <summary>
    /// The sparring partner's seat, or nothing at all when the lane has one
    /// car. An empty span is how the environment is told this is solo.
    /// </summary>
    private Span<float> OpponentObservationFor(int index) =>
        _carsPerLane == 1 ? Span<float>.Empty : SeatFor(index, 1);

    private Span<float> SeatFor(int index, int seat) =>
        _observations.AsSpan(
            (index * _carsPerLane + seat) *
                DirectDriveObservation.ObservationSize,
            DirectDriveObservation.ObservationSize
        );

    private void WriteObservationResponse(
        Stream output,
        TrainingMessageKind kind
    )
    {
        int bytes = WriteFloats(_observations, _responseBuffer, 0);
        TrainingProtocol.WriteMessage(
            output,
            kind,
            _responseBuffer.AsSpan(0, bytes)
        );
    }

    private bool TryExpectedPayloadLength(
        TrainingMessageKind kind,
        out int length
    )
    {
        switch (kind)
        {
            case TrainingMessageKind.Hello:
            case TrainingMessageKind.Close:
                length = 0;
                return true;
            case TrainingMessageKind.Reset:
                length = _resetRequestBytes;
                return true;
            case TrainingMessageKind.MaskedReset:
                length = _maskedResetRequestBytes;
                return true;
            case TrainingMessageKind.Step:
                length = _stepRequestBytes;
                return true;
            default:
                length = 0;
                return false;
        }
    }

    private static void ReadFloats(
        ReadOnlySpan<byte> source,
        Span<float> destination
    )
    {
        if (source.Length != destination.Length * sizeof(float))
            throw new InvalidDataException("Float payload length mismatch.");
        for (int i = 0; i < destination.Length; i++)
        {
            int bits = BinaryPrimitives.ReadInt32LittleEndian(
                source.Slice(i * sizeof(float), sizeof(float))
            );
            float value = BitConverter.Int32BitsToSingle(bits);
            if (!float.IsFinite(value))
            {
                throw new InvalidDataException(
                    "Action payload contains a non-finite value."
                );
            }
            destination[i] = value;
        }
    }

    private static int WriteFloats(
        ReadOnlySpan<float> source,
        Span<byte> destination,
        int offset
    )
    {
        foreach (float value in source)
            offset = WriteFloat(destination, offset, value);
        return offset;
    }

    private static int WriteFloat(
        Span<byte> destination,
        int offset,
        float value
    )
    {
        BinaryPrimitives.WriteInt32LittleEndian(
            destination.Slice(offset, sizeof(float)),
            BitConverter.SingleToInt32Bits(value)
        );
        return offset + sizeof(float);
    }

    private static void DiscardPayload(Stream input, int length)
    {
        Span<byte> buffer = stackalloc byte[4096];
        int remaining = length;
        while (remaining > 0)
        {
            int count = Math.Min(remaining, buffer.Length);
            input.ReadExactly(buffer[..count]);
            remaining -= count;
        }
    }

    private static void WriteError(
        Stream output,
        TextWriter diagnostics,
        string message
    )
    {
        diagnostics.WriteLine(message);
        byte[] payload = Encoding.UTF8.GetBytes(message);
        TrainingProtocol.WriteMessage(
            output,
            TrainingMessageKind.Error,
            payload
        );
    }
}
