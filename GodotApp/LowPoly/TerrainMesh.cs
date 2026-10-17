using System;
using System.Collections.Generic;
using System.Numerics;
using Clipper2Lib;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>
/// The ground round a circuit as one sheet: everything off the road, out to
/// the edge of the map, triangulated so that no two faces lie over each
/// other and none lies over the road.
///
/// The road with its run-off, barrier to barrier, is first made a single
/// clean region: a quad per stretch of road, unioned, so where the inside
/// of a hairpin folds back on itself or two parts of the lap touch, the
/// region simply has the shape the road does. The map is cut into a grid
/// of square cells; each cell has the road taken out of it, and what is
/// left is triangulated (constrained Delaunay) along the barrier exactly.
/// A cell the road does not reach is two triangles. Neighbouring cells cut
/// the same barrier line at the same points, so they share their edges.
///
/// Heights come from <see cref="TrackSurfaceGeometry.GroundHeight"/>, one
/// per vertex: at the barrier the road's edge, easing to the meadow.
/// Pure geometry, no engine types: it hands back triangles in the view's
/// axes (x, height, z), grouped by chunk.
/// </summary>
public static class TerrainMesh
{
    /// <summary>The integer grid the clipping runs on: centimetres.</summary>
    private const double Scale = 100.0;

    public static Dictionary<(int X, int Z), List<Vector3>> Build(
        TrackSurfaceGeometry surface, float cell, float margin, int chunkCells)
    {
        Paths64 road = RoadRegion(surface);
        Vector3 min = surface.Minimum, max = surface.Maximum;
        float x0 = min.X - margin, z0 = min.Z - margin;
        int nx = (int)MathF.Ceiling((max.X - min.X + 2f * margin) / cell);
        int nz = (int)MathF.Ceiling((max.Z - min.Z + 2f * margin) / cell);

        var heights = new Dictionary<(long, long), float>();
        Vector3 At(long x, long z)
        {
            if (!heights.TryGetValue((x, z), out float h))
                heights[(x, z)] = h = surface.GroundHeight((float)(x / Scale), (float)(z / Scale));
            return new Vector3((float)(x / Scale), h, (float)(z / Scale));
        }

        var chunks = new Dictionary<(int, int), List<Vector3>>();
        for (int ix = 0; ix < nx; ix++)
        for (int iz = 0; iz < nz; iz++)
        {
            long left = Cm(x0 + ix * cell), right = Cm(x0 + (ix + 1) * cell);
            long top = Cm(z0 + iz * cell), bottom = Cm(z0 + (iz + 1) * cell);
            var key = (ix / chunkCells, iz / chunkCells);
            if (!chunks.TryGetValue(key, out List<Vector3>? triangles))
                chunks[key] = triangles = [];

            Paths64 near = Clipper.RectClip(new Rect64(left, top, right, bottom), road);
            if (near.Count == 0)
            {
                Vector3 a = At(left, top), b = At(right, top), c = At(right, bottom), d = At(left, bottom);
                triangles.AddRange([a, b, c, a, c, d]);
                continue;
            }
            var square = new Paths64 { new Path64 { new(left, top), new(right, top), new(right, bottom), new(left, bottom) } };
            Paths64 ground = Clipper.Difference(square, near, FillRule.NonZero);
            if (ground.Count == 0)
                continue;
            // A cut can leave a sliver Delaunay refuses; the plain
            // triangulation of the same outline is still a correct cover.
            if (Clipper.Triangulate(ground, out Paths64 pieces, true) != TriangulateResult.success &&
                Clipper.Triangulate(ground, out pieces, false) != TriangulateResult.success)
                continue;
            foreach (Path64 piece in pieces)
                if (piece.Count == 3)
                    triangles.AddRange([At(piece[0].X, piece[0].Y), At(piece[1].X, piece[1].Y), At(piece[2].X, piece[2].Y)]);
        }
        return chunks;
    }

    /// <summary>
    /// Barrier to barrier, as one region: a quad for each stretch the road
    /// is built in (the same stretches, so the outline runs through the
    /// very points the run-off's outer edge does), and the pit lane's the
    /// same way, unioned.
    /// </summary>
    public static Paths64 RoadRegion(TrackSurfaceGeometry surface)
    {
        var track = surface.Track;
        int segments = (int)MathF.Ceiling(track.LengthMeters / 3f);
        float step = track.LengthMeters / segments;
        var quads = new Paths64(segments);
        for (int i = 0; i < segments; i++)
        {
            float a = i * step, b = (i + 1) * step;
            var quad = new Path64
            {
                Plan(surface.Point(a, -surface.BarrierOffset(a, -1))),
                Plan(surface.Point(b, -surface.BarrierOffset(b, -1))),
                Plan(surface.Point(b, surface.BarrierOffset(b, 1))),
                Plan(surface.Point(a, surface.BarrierOffset(a, 1))),
            };
            if (Clipper.Area(quad) < 0)
                quad.Reverse();
            quads.Add(quad);
        }
        // The pit lane is road too: the ground stops at its edges.
        if (surface.Pit is { } pit)
        {
            int pieces = (int)MathF.Ceiling(pit.LengthMeters / 3f);
            float length = pit.LengthMeters / pieces;
            for (int i = 0; i < pieces; i++)
            {
                float a = i * length, b = (i + 1) * length;
                float ha = pit.Sample(a).HalfWidth, hb = pit.Sample(b).HalfWidth;
                var quad = new Path64
                {
                    Plan(surface.PitPoint(a, -ha)), Plan(surface.PitPoint(b, -hb)),
                    Plan(surface.PitPoint(b, hb)), Plan(surface.PitPoint(a, ha)),
                };
                if (Clipper.Area(quad) < 0)
                    quad.Reverse();
                quads.Add(quad);
            }
        }
        return Clipper.Union(quads, FillRule.NonZero);
    }

    private static Point64 Plan(Vector3 point) => new(Cm(point.X), Cm(point.Z));
    private static long Cm(float meters) => (long)Math.Round(meters * Scale);
}
