using System.Globalization;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>Offline export for the five-place joint line and speed search.</summary>
public static class JointLineBake
{
    public static string TableKey(string circuitId) => $"joint-five-v1:{circuitId}";

    public static long ExportGeometry(string circuitId, LaneSet lanes, string tablePath, string path,
        Action<int, long>? progress = null)
    {
        LineSearch.Table table = LineSearch.Table.Load(tablePath, TableKey(circuitId), readSteps: false)
            ?? throw new InvalidDataException($"No matching five-place table in '{tablePath}'.");
        return LineSearch.FixedGeometry.Bake(path, lanes, table, progress);
    }

    public static float Export(string circuitId, TrackData track, LaneSet lanes, ICar car, string folder,
        bool exact, float seamSpeed, Action<int>? progress = null)
    {
        var state = new CarState();
        var tyres = (ICarTires)car;
        tyres.InstallFreshTires(state);
        tyres.WarmTires(state, 95f, 95f);
        (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
        var envelope = LaneEnvelope.From(state, (ICarEnvelope)car, (ICarGripFeel)car, CarStrategy.Default)
            .WithRoadGrip(track.AsphaltGrip);
        var search = new LineSearch(lanes, envelope, new(), _ => 1f, (station, offset) =>
        {
            TrackSample sample = track.Sample(lanes.Wrap(station) * lanes.Spacing);
            return new LaneRoad(sample.Grade, sample.BankSlopeAt(offset), sample.VerticalRate);
        }) { ExactSpeeds = exact, RetainBakedSteps = false, Baked = progress };

        // A fixed seam state makes the exported lap repeatable.
        LineSearch.Table table = search.BakeLap(0, null, seamSpeed: seamSpeed);
        LineSearch.Way way = search.Follow(table, 0, 0f, 0f, seamSpeed, maxStations: lanes.Count)
            ?? throw new InvalidOperationException($"No closed lap at seam speed {seamSpeed} m/s.");
        if (way.Offset[0] != way.Offset[^1] || MathF.Abs(way.Speed[0] - way.Speed[^1]) > 0.01f)
            throw new InvalidOperationException("The exported lap does not close at the seam.");

        Directory.CreateDirectory(folder);
        string stem = Path.Combine(folder, "five");
        float seconds = 0f;
        using (var writer = new StreamWriter(stem + ".csv"))
        {
            writer.WriteLine("station,s_m,x_m,y_m,offset_m,speed_mps,time_s");
            var previous = lanes.Position(0, lanes.Offset(LaneId.Free, 0) + way.Offset[0]);
            for (int k = 0; k < way.Offset.Length; k++)
            {
                float offset = lanes.Offset(LaneId.Free, k) + way.Offset[k];
                var point = lanes.Position(k, offset);
                if (k > 0)
                    seconds += (point - previous).Length() / MathF.Max(1f, 0.5f * (way.Speed[k - 1] + way.Speed[k]));
                writer.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"{k},{k * lanes.Spacing:R},{point.X:R},{point.Y:R},{offset:R},{way.Speed[k]:R},{seconds:R}"));
                previous = point;
            }
        }
        table.Save(stem + ".table", TableKey(circuitId));
        return seconds;
    }
}
