using System;
using Godot;
using StintegyEVO.Presentation.Interop;
using CoreRaceCar = TheStint.Core.Racing.RaceCar;

namespace StintegyEVO.Presentation.Car;

public partial class CarView : Node2D
{
    private CoreRaceCar? _car;

    public CoreRaceCar Car => _car ?? throw new InvalidOperationException("CarView is not bound.");

    public void Bind(CoreRaceCar car, Color bodyColor)
    {
        _car = car ?? throw new ArgumentNullException(nameof(car));
        Name = $"CarView_{car.Id}";
        BuildVisuals(bodyColor);
        SyncFromCore();
    }

    public void SyncFromCore()
    {
        if (_car == null)
            return;
        Position = _car.State.Position.ToGodot();
        Rotation = _car.State.Heading;
    }

    private void BuildVisuals(Color bodyColor)
    {
        foreach (Node child in GetChildren())
            child.QueueFree();

        float length = Car.Collision.LengthMeters;
        float width = Car.Collision.WidthMeters;
        float halfLength = length * 0.5f;
        float halfWidth = width * 0.5f;

        DrawTire(+length * 0.28f, -halfWidth, length, width);
        DrawTire(+length * 0.28f, +halfWidth, length, width);
        DrawTire(-length * 0.28f, -halfWidth, length, width);
        DrawTire(-length * 0.28f, +halfWidth, length, width);

        Polygon2D body = new()
        {
            Polygon =
            [
                new Vector2(halfLength, 0f),
                new Vector2(halfLength * 0.62f, halfWidth * 0.72f),
                new Vector2(-halfLength * 0.72f, halfWidth * 0.72f),
                new Vector2(-halfLength, halfWidth * 0.42f),
                new Vector2(-halfLength, -halfWidth * 0.42f),
                new Vector2(-halfLength * 0.72f, -halfWidth * 0.72f),
                new Vector2(halfLength * 0.62f, -halfWidth * 0.72f)
            ],
            Color = bodyColor,
            Antialiased = true,
            ZIndex = 12
        };
        AddChild(body);

        AddChild(Rectangle(
            minX: -length * 0.02f,
            minY: -width * 0.22f,
            maxX: length * 0.32f,
            maxY: width * 0.22f,
            Color.FromHtml("#17212b"),
            zIndex: 13
        ));
        AddChild(Rectangle(
            minX: halfLength - length * 0.12f,
            minY: -width * 0.62f,
            maxX: halfLength,
            maxY: width * 0.62f,
            bodyColor.Darkened(0.25f),
            zIndex: 14
        ));
        AddChild(Rectangle(
            minX: -halfLength,
            minY: -width * 0.58f,
            maxX: -halfLength + length * 0.10f,
            maxY: width * 0.58f,
            bodyColor.Darkened(0.25f),
            zIndex: 14
        ));
        ZIndex = 20;
    }

    private void DrawTire(float x, float y, float length, float width)
    {
        float tireLength = length * 0.18f;
        float tireWidth = width * 0.22f;
        AddChild(Rectangle(
            x - tireLength * 0.5f,
            y - tireWidth * 0.5f,
            x + tireLength * 0.5f,
            y + tireWidth * 0.5f,
            Color.FromHtml("#111418"),
            10
        ));
    }

    private static Polygon2D Rectangle(
        float minX,
        float minY,
        float maxX,
        float maxY,
        Color color,
        int zIndex
    )
    {
        return new Polygon2D
        {
            Polygon =
            [
                new Vector2(minX, minY),
                new Vector2(maxX, minY),
                new Vector2(maxX, maxY),
                new Vector2(minX, maxY)
            ],
            Color = color,
            Antialiased = true,
            ZIndex = zIndex
        };
    }
}
