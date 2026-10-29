using System.Runtime.InteropServices;
using StintegyEVO.Core.Mods;
using StintegyEVO.Drivers.Racecraft;
using StintegyEVO.RacecraftHost;

// A batch of racecraft duels on one circuit, stepped together.
//
//   Hello                      -> obs size, action size, batch, version, component count (int32 each)
//   Reset  (seed int64)        -> observations           batch x obs float32
//   Step   (actions)           -> observations           batch x obs float32
//                                 final observations     batch x obs float32 (the row a duel ended on)
//                                 rewards                batch float32
//                                 ended, out of time     batch byte each
//                                 components             batch x components float32
//   Close                      -> (nothing)
//
// A duel that ends is drawn afresh at once: the observation sent for it is
// the new duel's first, and the one it ended on goes in the final rows.

string circuit = "silverstone";
int batch = 16;
for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--circuit": circuit = args[++i]; break;
        case "--batch": batch = int.Parse(args[++i]); break;
        default: throw new ArgumentException($"Unknown argument {args[i]}.");
    }
}

var track = GameMods.Registry.Circuits.Get(circuit).Track;
var duels = new RacecraftDuel[batch];
var randoms = new Random[batch];
Parallel.For(0, batch, i => duels[i] = new RacecraftDuel(circuit, track));
int obsSize = RacecraftDuel.ObservationSize, actSize = RacecraftDuel.ActionSize;
const int components = RacecraftStep.ComponentCount;
var obs = new float[batch * obsSize];
var final = new float[batch * obsSize];
var rewards = new float[batch];
var ended = new byte[batch];
var outOfTime = new byte[batch];
var parts = new float[batch * components];
var actions = new float[batch * actSize];

using Stream input = Console.OpenStandardInput();
using Stream output = Console.OpenStandardOutput();
Console.SetOut(TextWriter.Null);

while (Protocol.Read(input) is { } message)
{
    try
    {
        switch (message.Kind)
        {
            case MessageKind.Hello:
                Protocol.Write(output, MessageKind.HelloResponse,
                    MemoryMarshal.AsBytes<int>([obsSize, actSize, batch, Protocol.Version, components]));
                break;
            case MessageKind.Reset:
                long seed = BitConverter.ToInt64(message.Payload);
                Parallel.For(0, batch, i =>
                {
                    randoms[i] = new Random(unchecked((int)(seed * 1_000_003 + i)));
                    duels[i].Reset(randoms[i], obs.AsSpan(i * obsSize, obsSize));
                });
                Protocol.Write(output, MessageKind.ResetResponse, MemoryMarshal.AsBytes(obs.AsSpan()));
                break;
            case MessageKind.Step:
                if (message.Payload.Length != actions.Length * sizeof(float))
                    throw new InvalidDataException($"A step carries {batch} x {actSize} actions.");
                MemoryMarshal.Cast<byte, float>(message.Payload).CopyTo(actions);
                Parallel.For(0, batch, i =>
                {
                    Span<float> row = obs.AsSpan(i * obsSize, obsSize);
                    RacecraftStep step = duels[i].Step(actions.AsSpan(i * actSize, actSize), row);
                    rewards[i] = step.Reward;
                    ended[i] = step.Ended ? (byte)1 : (byte)0;
                    outOfTime[i] = step.OutOfTime ? (byte)1 : (byte)0;
                    Span<float> p = parts.AsSpan(i * components, components);
                    p[0] = step.Gain; p[1] = step.Passes; p[2] = step.Contact; p[3] = step.Off; p[4] = step.Wall; p[5] = step.Spin;
                    row.CopyTo(final.AsSpan(i * obsSize, obsSize));
                    if (step.Ended)
                        duels[i].Reset(randoms[i], row);
                });
                using (var buffer = new MemoryStream())
                {
                    buffer.Write(MemoryMarshal.AsBytes(obs.AsSpan()));
                    buffer.Write(MemoryMarshal.AsBytes(final.AsSpan()));
                    buffer.Write(MemoryMarshal.AsBytes(rewards.AsSpan()));
                    buffer.Write(ended);
                    buffer.Write(outOfTime);
                    buffer.Write(MemoryMarshal.AsBytes(parts.AsSpan()));
                    Protocol.Write(output, MessageKind.StepResponse, buffer.GetBuffer().AsSpan(0, (int)buffer.Length));
                }
                break;
            case MessageKind.Close:
                Protocol.Write(output, MessageKind.CloseResponse, []);
                return;
            default:
                throw new InvalidDataException($"The host does not take {message.Kind}.");
        }
    }
    catch (Exception e)
    {
        Protocol.Write(output, MessageKind.Error, System.Text.Encoding.UTF8.GetBytes(e.ToString()));
    }
}
