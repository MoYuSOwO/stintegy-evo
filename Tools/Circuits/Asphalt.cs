using StintegyEVO.Core.Track;

namespace StintegyEVO.Circuits;

/// <summary>
/// Each circuit's asphalt, from Pirelli's pre-race ratings (1 to 5, three
/// the average): grip is 1.5 per cent a step from three, abrasion
/// multiplies wear by 1.25 a step. Only the asphalt's own two ratings are
/// used; the rest (lateral, traction, braking, tyre stress) come out of the
/// circuit's shape in the physics and would be counted twice.
/// </summary>
internal static class Asphalt
{
    private static readonly Dictionary<string, (int Grip, int Abrasion, string Source, string Others)> Ratings = new()
    {
        ["baku"] = (2, 1, "Pirelli 2025 Azerbaijan GP preview", "traction 5, braking 4, lateral 1, tyre stress 3, track evolution 5, downforce 2"),
        ["monaco"] = (2, 1, "Pirelli 2025 Monaco GP preview", "traction 5, braking 2, lateral 1, tyre stress 1, track evolution 5, downforce 5"),
        ["interlagos"] = (3, 2, "Pirelli 2025 Sao Paulo GP preview", "traction 3, braking 3, lateral 3, tyre stress 3, track evolution 5, downforce 4"),
        ["monza"] = (3, 2, "Pirelli 2025 Italian GP preview", "traction 3, braking 4, lateral 2, tyre stress 3, track evolution 4, downforce 1"),
        ["silverstone"] = (4, 2, "Pirelli 2025 British GP preview", "traction 3, braking 2, lateral 5, tyre stress 5, track evolution 2, downforce 4"),
        ["zandvoort"] = (3, 3, "Pirelli 2025 Dutch GP preview", "traction 3, braking 3, lateral 4, tyre stress 5, track evolution 4, downforce 4"),
        ["spa"] = (3, 3, "Pirelli 2025 Belgian GP preview", "traction 4, braking 4, lateral 5, tyre stress 5, track evolution 3, downforce 2"),
        ["singapore"] = (3, 3, "Pirelli 2025 Singapore GP preview", "traction 4, braking 5, lateral 2, tyre stress 2, track evolution 5, downforce 5"),
        ["shanghai"] = (3, 3, "Pirelli 2025 Chinese GP preview", "traction 3, braking 4, lateral 4, tyre stress 4, track evolution 5, downforce 3"),
        ["sepang"] = (2, 4, "Pirelli 2026 Bahrain GP in Malaysia preview (the 2026 infographic)", "traction 3, braking 4, lateral 4, tyre stress 4, track evolution 4"),
        ["portimao"] = (3, 4, "No Pirelli rating found (last raced 2021); estimated from Pirelli's Isola in 2021, that the layout and the tarmac are abrasive", ""),
    };

    /// <summary>The circuit's asphalt, or the average circuit where there is no rating.</summary>
    public static CircuitSurface For(string id)
    {
        if (!Ratings.TryGetValue(id, out var r))
            return new CircuitSurface();
        string others = r.Others.Length > 0 ? $" Other ratings, not used here: {r.Others}." : "";
        return new CircuitSurface
        {
            Grip = MathF.Round(1f + 0.015f * (r.Grip - 3), 3),
            Abrasion = MathF.Round(MathF.Pow(1.25f, r.Abrasion - 3), 2),
            Basis = $"{r.Source}: asphalt grip {r.Grip}/5, asphalt abrasion {r.Abrasion}/5.{others}"
        };
    }
}
