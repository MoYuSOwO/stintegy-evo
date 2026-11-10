using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Security.Cryptography;

namespace StintegyEVO.Drivers.Lanes;

internal sealed partial class LineSearch
{
    internal sealed class FixedGeometry : IDisposable
    {
        private const int Version = 2;
        private const int Places = 5;
        private const int Headings = 9;
        private const int SpanCount = 4;
        private const int SlotsPerRow = Places * Headings * SpanCount * Places * Headings;
        private readonly MemoryMappedViewAccessor _view;
        private readonly long[] _offsets;
        private readonly byte[] _signature;
        private readonly bool _hasMatchingJoins;

        private FixedGeometry(MemoryMappedViewAccessor view, long[] offsets, byte[] signature, bool hasMatchingJoins)
        {
            _view = view;
            _offsets = offsets;
            _signature = signature;
            _hasMatchingJoins = hasMatchingJoins;
        }

        private static int Slot(int row, int place, int heading, int span, int to, int toHeading) =>
            ((((row * Places + place) * Headings + heading) * SpanCount + span) * Places + to) * Headings + toHeading;

        internal static FixedGeometry Load(string path, Table table)
        {
            using var stream = File.OpenRead(path);
            using var reader = new BinaryReader(stream);
            int count = checked(table.Rows * SlotsPerRow);
            if (reader.ReadInt32() != Version || reader.ReadInt32() != count)
                throw new InvalidDataException("The geometry file does not match the five-place table.");
            byte[] signature = reader.ReadBytes(32);
            if (signature.Length != 32)
                throw new EndOfStreamException();
            byte[] joinSignature = reader.ReadBytes(32);
            if (joinSignature.Length != 32)
                throw new EndOfStreamException();
            bool hasMatchingJoins = joinSignature.AsSpan().SequenceEqual(JoinSignature(table));
            var offsets = new long[count];
            stream.ReadExactly(MemoryMarshal.AsBytes(offsets.AsSpan()));
            using var mapping = MemoryMappedFile.CreateFromFile(stream, null, 0, MemoryMappedFileAccess.Read,
                HandleInheritability.None, leaveOpen: true);
            return new FixedGeometry(mapping.CreateViewAccessor(0, 0, MemoryMappedFileAccess.Read), offsets, signature,
                hasMatchingJoins);
        }

        internal void Validate(LaneSet lanes, Table table)
        {
            if (!_signature.AsSpan().SequenceEqual(Signature(lanes, table)))
                throw new InvalidDataException("The geometry was baked for different lanes or lattice rows.");
        }

        private static byte[] Signature(LaneSet lanes, Table table)
        {
            using var data = new MemoryStream();
            using var writer = new BinaryWriter(data);
            writer.Write(lanes.Count);
            writer.Write(lanes.Spacing);
            for (int i = 0; i < lanes.Count; i++)
            {
                var center = lanes.Centre(i);
                var normal = lanes.Normal(i);
                writer.Write(center.X);
                writer.Write(center.Y);
                writer.Write(normal.X);
                writer.Write(normal.Y);
                writer.Write(lanes.HalfWidth(i));
                writer.Write(lanes.Offset(LaneId.Free, i));
            }
            foreach (int station in table.Starts)
                writer.Write(station);
            foreach (var row in table.Places)
            {
                writer.Write(row.Count);
                foreach (float place in row)
                    writer.Write(place);
            }
            return SHA256.HashData(data.GetBuffer().AsSpan(0, (int)data.Length));
        }

        private static byte[] JoinSignature(Table table)
        {
            using var data = new MemoryStream();
            using var writer = new BinaryWriter(data);
            for (int row = 0; row <= table.Rows; row++)
            {
                foreach (var groups in table.OnlineArrivals![row])
                {
                    writer.Write(groups.Length);
                    foreach (var group in groups)
                        writer.Write(group.Tail);
                }
                writer.Write(table.Tails[row].Count);
                foreach (float[] line in table.Tails[row])
                {
                    int count = Math.Min(line.Length, Chord + 1);
                    writer.Write(count);
                    writer.Write(MemoryMarshal.AsBytes(line.AsSpan(0, count)));
                }
            }
            return SHA256.HashData(data.GetBuffer().AsSpan(0, (int)data.Length));
        }

        internal static long Bake(string path, LaneSet lanes, Table table, Action<int, long>? progress)
        {
            if (table.Starts[0] != 0 || table.Starts[^1] != lanes.Count ||
                table.Places.Any(row => row.Count > Places))
                throw new InvalidDataException("Expected a complete five-place lap table.");

            // The bake uses only road bounds and the lattice. No tyre or speed
            // information determines whether an edge is included.
            var search = new LineSearch(lanes, default, new(), _ => 1f, (_, _) => default) { ExactSpeeds = true };
            table.IndexOnlineArrivals();
            var offsets = new long[checked(table.Rows * SlotsPerRow)];
            string temporary = path + ".part";
            long edges = 0;
            using (var stream = File.Create(temporary))
            using (var writer = new BinaryWriter(stream))
            {
                writer.Write(Version);
                writer.Write(offsets.Length);
                writer.Write(Signature(lanes, table));
                writer.Write(JoinSignature(table));
                long index = stream.Position;
                stream.Position += (long)offsets.Length * sizeof(long);
                for (int row = 0; row < table.Rows; row++)
                {
                    for (int span = 0; span < SpanCount; span++)
                    {
                        int end = row + search.Spans[span];
                        if (end > table.Rows)
                            continue;
                        int st0 = table.Starts[row];
                        int n = table.Starts[end] - st0;
                        var edge = new Edge(n);
                        for (int place = 0; place < table.Places[row].Count; place++)
                        for (int to = 0; to < table.Places[end].Count; to++)
                        {
                            float y = table.Places[row][place], endY = table.Places[end][to];
                            if (MathF.Abs(endY - y) > search.MaxLateralJumps[span] + 1e-3f)
                                continue;
                            for (int h = 0; h < Headings; h++)
                            for (int endH = 0; endH < Headings; endH++)
                            {
                                if (!search.ShapeGeometry(edge, st0, row, y, search.Slopes[h], endY, search.Slopes[endH], null))
                                    continue;
                                offsets[Slot(row, place, h, span, to, endH)] = stream.Position;
                                writer.Write(edge.Length);
                                writer.Write(MemoryMarshal.AsBytes(edge.Offset.AsSpan()));
                                writer.Write(MemoryMarshal.AsBytes(edge.Curvature.AsSpan(0, n)));
                                writer.Write(MemoryMarshal.AsBytes(edge.Step.AsSpan(0, n)));
                                (edge.Y0, edge.H0, edge.Y1, edge.H1) = (y, search.Slopes[h], endY, search.Slopes[endH]);
                                int split = Math.Max(0, n - Chord);
                                foreach (var group in table.OnlineArrivals![end][to * Headings + endH])
                                {
                                    ushort tail = group.Tail;
                                    search.JoinGeometry(edge, st0, tail == NoTail ? null : table.Tails[end][tail], split);
                                    for (int k = split; k <= n; k++)
                                        writer.Write(BitConverter.HalfToUInt16Bits((Half)edge.Corner[k]));
                                }
                                edges++;
                            }
                        }
                    }
                    progress?.Invoke(row + 1, edges);
                }
                stream.Position = index;
                writer.Write(MemoryMarshal.AsBytes(offsets.AsSpan()));
            }
            File.Move(temporary, path, overwrite: true);
            return edges;
        }

        internal bool Read(Edge edge, int row, int place, int heading, int span, int to, int toHeading)
        {
            long offset = _offsets[Slot(row, place, heading, span, to, toHeading)];
            if (offset == 0)
                return false;
            int n = edge.N;
            edge.Length = _view.ReadSingle(offset);
            offset += sizeof(float);
            _view.ReadArray(offset, edge.Offset, 0, n + 1);
            offset += (n + 1) * sizeof(float);
            _view.ReadArray(offset, edge.Curvature, 0, n);
            offset += n * sizeof(float);
            _view.ReadArray(offset, edge.Step, 0, n);
            edge.GeometryJoins = _hasMatchingJoins ? offset + n * sizeof(float) : 0;
            edge.Step[n] = 0f;
            return true;
        }

        internal bool ReadJoin(Edge edge, int group, int from)
        {
            if (edge.GeometryJoins == 0 || from != edge.N - Chord)
                return false;
            ulong packed = _view.ReadUInt64(edge.GeometryJoins + group * sizeof(ulong));
            for (int k = from; k <= edge.N; k++)
            {
                edge.Corner[k] = (float)BitConverter.UInt16BitsToHalf((ushort)packed);
                packed >>= 16;
            }
            return true;
        }

        public void Dispose() => _view.Dispose();
    }

    private bool ReadFixedShape(Edge edge, int st0, int row, int place, int heading, int span, int to, int toHeading,
        float y, float slope, float endY, float endSlope)
    {
        if (!Geometry!.Read(edge, row, place, heading, span, to, toHeading))
            return false;
        GeometryShapeHits++;
        FitShapeToCar(edge, st0, y, slope, endY, endSlope);
        return true;
    }
}
