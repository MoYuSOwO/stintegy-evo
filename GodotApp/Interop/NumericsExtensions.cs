namespace StintegyEVO.GodotApp.Interop;

public static class NumericsExtensions
{
    public static global::Godot.Vector2 ToGodot(this System.Numerics.Vector2 value)
    {
        return new global::Godot.Vector2(value.X, value.Y);
    }
}
