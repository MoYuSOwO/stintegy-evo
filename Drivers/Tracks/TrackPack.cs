using System.Globalization;
using System.Numerics;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Tracks;

/// <summary>
/// Reads a circuit kept outside Core: centreline x, y, right and left width,
/// and optionally the bank in degrees where the road is a banked oval (into
/// the turn, the circuit's own way round). Rows without a bank are an
/// ordinary road circuit. The data files themselves are not part of the
/// repository.
/// </summary>
public static class TrackPack
{
    public static TrackData Load(string path, float lengthMeters, TrackGridConfig? grid = null)
    {
        var centre = new List<Vector2>();
        var right = new List<float>();
        var left = new List<float>();
        var bank = new List<float>();
        int bankCol = -1;
        foreach (string line in File.ReadLines(path))
        {
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith('#'))
                continue;
            string[] cols = line.Split(',');
            if (cols[0] == "x_m")
            {
                bankCol = Array.IndexOf(cols, "bank_deg");
                continue;
            }
            centre.Add(new Vector2(F(cols[0]), F(cols[1])));
            right.Add(F(cols[2]));
            left.Add(F(cols[3]));
            bank.Add(bankCol >= 0 && bankCol < cols.Length && cols[bankCol].Length > 0 ? F(cols[bankCol]) : float.NaN);
        }

        // Which way round the circuit runs decides which way "into the turn"
        // leans: counter-clockwise, the bank rises to the right of travel,
        // which is the track's positive offset.
        float area = 0f;
        for (int i = 0; i < centre.Count; i++)
        {
            Vector2 a = centre[i], b = centre[(i + 1) % centre.Count];
            area += a.X * b.Y - b.X * a.Y;
        }
        float sign = area >= 0f ? 1f : -1f;
        float[] bankTable = bank.ToArray();

        TrackSurface Surface(TrackSurfaceContext context)
        {
            float t = context.DistanceMeters / MathF.Max(context.LapLengthMeters, 1f) * bankTable.Length;
            int i0 = ((int)MathF.Floor(t) % bankTable.Length + bankTable.Length) % bankTable.Length;
            int i1 = (i0 + 1) % bankTable.Length;
            float u = t - MathF.Floor(t);
            float b0 = bankTable[i0], b1 = bankTable[i1];
            if (float.IsNaN(b0) && float.IsNaN(b1))
                return TrackSurfaces.RoadCircuit(context);
            float degrees = float.IsNaN(b0) ? b1 : float.IsNaN(b1) ? b0 : b0 + (b1 - b0) * u;
            return new TrackSurface(BankSlope: sign * MathF.Tan(degrees * MathF.PI / 180f));
        }

        return TrackFactory.FromCenterline(centre, right, left, lengthMeters, Surface, grid);
    }

    private static float F(string s) => float.Parse(s, CultureInfo.InvariantCulture);
}
