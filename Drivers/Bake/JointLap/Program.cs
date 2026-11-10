using System.Globalization;
using StintegyEVO.Core.Mods;
using StintegyEVO.Drivers;
using StintegyEVO.Drivers.Lanes;

if (args is ["geometry", var geometryCircuit, var tablePath, var geometryPath])
{
    var geometryTrack = GameMods.Registry.Circuits.Get(geometryCircuit).Track;
    var geometryLanes = LaneTrackFile.ForCircuit(geometryCircuit, geometryTrack).LanesFor(geometryTrack);
    long edges = JointLineBake.ExportGeometry(geometryCircuit, geometryLanes, tablePath, geometryPath, (row, written) =>
    {
        if (row % 50 == 0)
            Console.WriteLine($"geometry: row {row}, {written:N0} edges");
    });
    Console.WriteLine($"{edges:N0} legal edges; {new FileInfo(geometryPath).Length:N0} bytes");
    return 0;
}

if (args.Length is < 4 or > 5 || args[2] != "five" ||
    !float.TryParse(args[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float seamSpeed) ||
    !float.IsFinite(seamSpeed) || seamSpeed is < 10f or > 95f ||
    (args.Length == 5 && args[4] != "--exact"))
{
    Console.Error.WriteLine("Usage: JointLap <official:circuit> <output folder> five <seam speed m/s> [--exact]");
    Console.Error.WriteLine("       JointLap geometry <official:circuit> <input table> <output geometry>");
    return 2;
}

string id = args[0];
var track = GameMods.Registry.Circuits.Get(id).Track;
var lanes = LaneTrackFile.ForCircuit(id, track).LanesFor(track);
var car = StockCar.Build(DevelopmentTyres.SuperId);
bool exact = args.Length == 5;
var clock = System.Diagnostics.Stopwatch.StartNew();
Console.WriteLine($"five: {lanes.Count / 5} rows; exact={exact}; seam={seamSpeed} m/s");
float seconds = JointLineBake.Export(id, track, lanes, car, args[1], exact, seamSpeed, row =>
{
    if (row % 25 == 0)
        Console.WriteLine($"five: row {row} at {clock.Elapsed.TotalSeconds:F1}s");
});
Console.WriteLine($"five: {seconds:F3}s predicted lap, {clock.Elapsed.TotalSeconds:F1}s bake/export");
return 0;
