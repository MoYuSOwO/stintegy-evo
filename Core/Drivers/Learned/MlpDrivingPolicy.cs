using System;
using System.Buffers.Binary;
using System.IO;
using System.Numerics;

namespace StintegyEVO.Core.Drivers.Learned;

/// <summary>
/// A trained actor, run forward. Weights come out of PyTorch through
/// Training/python/export_policy.py, which strips everything the game does
/// not need — the critics, the entropy coefficient, the optimizer moments,
/// and the head that predicts a standard deviation, because the game never
/// samples. What is left is a stack of dense layers with rectified
/// activations and a hyperbolic tangent on the output, which is the whole
/// of what "the policy" means once training has stopped.
///
/// The file says its own shape, so this class knows nothing about how wide
/// the network happens to be. It checks that shape against the observation
/// it is handed, which is the difference between a mismatched policy
/// failing loudly at load and driving strangely for a lap and a half.
///
/// A port of a forward pass is several chances to be quietly wrong: a
/// transposed matrix, a bias on the wrong axis, a missing activation, the
/// two output heads swapped. None of those crash and all of them drive.
/// The guard is <c>MlpDrivingPolicyTests</c>, which replays sixty-four
/// observations the trained network answered in Python and requires the
/// same answers here.
/// </summary>
public sealed class MlpDrivingPolicy : IDrivingPolicy
{
    private readonly MlpNetwork _network;
    private readonly float[] _front;
    private readonly float[] _back;

    /// <summary>
    /// Shares one set of weights. Several cars can run the same network;
    /// each needs its own policy, because the scratch space between layers
    /// belongs to a call and not to the weights.
    /// </summary>
    public MlpDrivingPolicy(MlpNetwork network)
    {
        _network = network ?? throw new ArgumentNullException(nameof(network));
        _front = new float[network.WidestLayer];
        _back = new float[network.WidestLayer];
    }

    public static MlpDrivingPolicy FromFile(string path)
    {
        return new MlpDrivingPolicy(MlpNetwork.LoadFile(path));
    }

    public static MlpDrivingPolicy FromBytes(ReadOnlySpan<byte> bytes)
    {
        return new MlpDrivingPolicy(MlpNetwork.Load(bytes));
    }

    public MlpNetwork Network => _network;

    public void Act(ReadOnlySpan<float> observation, Span<float> action)
    {
        _network.Evaluate(observation, action, _front, _back);
    }
}

/// <summary>
/// The weights themselves: immutable once loaded, and shareable across
/// every car running the same policy.
/// </summary>
public sealed class MlpNetwork
{
    private const uint Magic = 0x4E4E5453;   // "STNN", little-endian
    private const int SupportedVersion = 1;
    private const int ActivationNone = 0;
    private const int ActivationRelu = 1;
    private const int SquashNone = 0;
    private const int SquashTanh = 1;

    private readonly Layer[] _layers;
    private readonly int _squash;

    private readonly record struct Layer(
        float[] Weights,   // row-major, one output's whole row at a time
        float[] Bias,
        int Inputs,
        int Outputs,
        int Activation
    );

    private MlpNetwork(Layer[] layers, int inputSize, int outputSize, int squash)
    {
        _layers = layers;
        _squash = squash;
        InputSize = inputSize;
        OutputSize = outputSize;
        int widest = inputSize;
        foreach (Layer layer in layers)
            widest = Math.Max(widest, layer.Outputs);
        WidestLayer = widest;
    }

    public int InputSize { get; }
    public int OutputSize { get; }
    public int LayerCount => _layers.Length;

    /// <summary>How much scratch one forward pass needs per buffer.</summary>
    public int WidestLayer { get; }

    public static MlpNetwork LoadFile(string path)
    {
        return Load(File.ReadAllBytes(path));
    }

    public static MlpNetwork Load(Stream stream)
    {
        ArgumentNullException.ThrowIfNull(stream);
        using MemoryStream buffer = new();
        stream.CopyTo(buffer);
        return Load(buffer.GetBuffer().AsSpan(0, (int)buffer.Length));
    }

    public static MlpNetwork Load(ReadOnlySpan<byte> bytes)
    {
        if (bytes.Length < 24)
            throw new InvalidDataException("Too short to be a network file.");
        if (BinaryPrimitives.ReadUInt32LittleEndian(bytes) != Magic)
            throw new InvalidDataException("Not a network file (bad magic).");

        int at = 4;
        int version = ReadInt(bytes, ref at);
        if (version != SupportedVersion)
        {
            throw new InvalidDataException(
                $"Network format {version}; this reads {SupportedVersion}."
            );
        }
        int inputSize = ReadInt(bytes, ref at);
        int outputSize = ReadInt(bytes, ref at);
        int squash = ReadInt(bytes, ref at);
        int layerCount = ReadInt(bytes, ref at);
        if (inputSize <= 0 || outputSize <= 0 || layerCount <= 0)
            throw new InvalidDataException("Network file declares no shape.");
        if (squash is not (SquashNone or SquashTanh))
            throw new InvalidDataException($"Unknown output squash {squash}.");

        var shapes = new (int Inputs, int Outputs, int Activation)[layerCount];
        for (int i = 0; i < layerCount; i++)
        {
            int inputs = ReadInt(bytes, ref at);
            int outputs = ReadInt(bytes, ref at);
            int activation = ReadInt(bytes, ref at);
            if (inputs <= 0 || outputs <= 0)
                throw new InvalidDataException($"Layer {i} has no shape.");
            if (activation is not (ActivationNone or ActivationRelu))
            {
                throw new InvalidDataException(
                    $"Layer {i} wants activation {activation}, which is not one."
                );
            }
            shapes[i] = (inputs, outputs, activation);
        }

        if (shapes[0].Inputs != inputSize)
        {
            throw new InvalidDataException(
                $"The file takes {inputSize} inputs but its first layer " +
                $"takes {shapes[0].Inputs}."
            );
        }
        if (shapes[^1].Outputs != outputSize)
        {
            throw new InvalidDataException(
                $"The file gives {outputSize} outputs but its last layer " +
                $"gives {shapes[^1].Outputs}."
            );
        }
        for (int i = 1; i < layerCount; i++)
        {
            if (shapes[i].Inputs != shapes[i - 1].Outputs)
            {
                throw new InvalidDataException(
                    $"Layer {i} takes {shapes[i].Inputs} where layer " +
                    $"{i - 1} gives {shapes[i - 1].Outputs}."
                );
            }
        }

        var layers = new Layer[layerCount];
        for (int i = 0; i < layerCount; i++)
        {
            (int inputs, int outputs, int activation) = shapes[i];
            layers[i] = new Layer(
                ReadFloats(bytes, ref at, inputs * outputs),
                ReadFloats(bytes, ref at, outputs),
                inputs,
                outputs,
                activation
            );
        }
        if (at != bytes.Length)
        {
            throw new InvalidDataException(
                $"{bytes.Length - at} bytes past the end of the last layer."
            );
        }
        return new MlpNetwork(layers, inputSize, outputSize, squash);
    }

    /// <summary>
    /// One forward pass. <paramref name="front"/> and
    /// <paramref name="back"/> are scratch, each at least
    /// <see cref="WidestLayer"/> long; they are handed in rather than
    /// allocated so that a decision costs no garbage.
    /// </summary>
    public void Evaluate(
        ReadOnlySpan<float> input,
        Span<float> output,
        Span<float> front,
        Span<float> back
    )
    {
        if (input.Length < InputSize)
        {
            throw new ArgumentException(
                $"This network reads {InputSize} values and was given " +
                $"{input.Length}.",
                nameof(input)
            );
        }
        if (output.Length < OutputSize)
        {
            throw new ArgumentException(
                $"This network writes {OutputSize} values and was given " +
                $"room for {output.Length}.",
                nameof(output)
            );
        }
        if (front.Length < WidestLayer || back.Length < WidestLayer)
        {
            throw new ArgumentException(
                $"A forward pass needs {WidestLayer} values of scratch."
            );
        }

        input[..InputSize].CopyTo(front);
        int width = InputSize;
        foreach (Layer layer in _layers)
        {
            ReadOnlySpan<float> source = front[..width];
            Span<float> destination = back[..layer.Outputs];
            for (int row = 0; row < layer.Outputs; row++)
            {
                float sum = layer.Bias[row] + Dot(
                    layer.Weights.AsSpan(row * layer.Inputs, layer.Inputs),
                    source
                );
                destination[row] =
                    layer.Activation == ActivationRelu
                        ? MathF.Max(0f, sum)
                        : sum;
            }
            width = layer.Outputs;
            Span<float> swap = front;
            front = back;
            back = swap;
        }

        ReadOnlySpan<float> result = front[..OutputSize];
        for (int i = 0; i < OutputSize; i++)
        {
            output[i] = _squash == SquashTanh
                ? MathF.Tanh(result[i])
                : result[i];
        }
    }

    private static float Dot(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
    {
        int width = Vector<float>.Count;
        int i = 0;
        Vector<float> accumulator = Vector<float>.Zero;
        for (; i + width <= a.Length; i += width)
        {
            accumulator += new Vector<float>(a.Slice(i, width)) *
                           new Vector<float>(b.Slice(i, width));
        }
        float sum = Vector.Dot(accumulator, Vector<float>.One);
        for (; i < a.Length; i++)
            sum += a[i] * b[i];
        return sum;
    }

    private static int ReadInt(ReadOnlySpan<byte> bytes, ref int at)
    {
        if (at + 4 > bytes.Length)
            throw new InvalidDataException("The file ends mid-header.");
        int value = BinaryPrimitives.ReadInt32LittleEndian(bytes[at..]);
        at += 4;
        return value;
    }

    private static float[] ReadFloats(
        ReadOnlySpan<byte> bytes, ref int at, int count
    )
    {
        if (at + count * 4 > bytes.Length)
            throw new InvalidDataException("The file ends mid-weights.");
        float[] values = new float[count];
        for (int i = 0; i < count; i++)
        {
            values[i] = BitConverter.Int32BitsToSingle(
                BinaryPrimitives.ReadInt32LittleEndian(bytes[(at + i * 4)..])
            );
        }
        at += count * 4;
        return values;
    }
}
