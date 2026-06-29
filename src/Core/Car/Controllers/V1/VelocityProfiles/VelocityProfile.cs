using System;

namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public sealed class VelocityProfile(VelocityProfilePoint[] points)
{
    public static readonly VelocityProfile Empty = new([]);

    public readonly VelocityProfilePoint[] Points = points;
    public int Count => Points.Length;

    public VelocityProfilePoint this[int index]
    {
        get
        {
            if (Points.Length == 0)
                return default;

            int safeIndex = Math.Clamp(index, 0, Points.Length - 1);
            return Points[safeIndex];
        }
    }
}
