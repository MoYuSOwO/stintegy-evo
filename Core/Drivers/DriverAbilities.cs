using System;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Stable, manager-facing driver ratings. Ratings describe the driver's mean
/// ability. Controllers interpret how each rating affects their commands;
/// this domain model does not prescribe a sampling or driving algorithm.
/// </summary>
/// <remarks>
/// Every rating is a finite number in the closed range 0..100; a profile with
/// anything else is rejected. A rating's meaning belongs to the driver, never to
/// the car: mapping it onto behaviour happens in a controller, upstream of
/// <see cref="StintegyEVO.Core.Cars.DriverInput"/>, and must not come back into
/// <see cref="StintegyEVO.Core.Cars.CarPhysics"/> as a second driver model.
/// </remarks>
public sealed record DriverAbilities
{
    public float Pace { get; init; } = 100f;
    public float Consistency { get; init; } = 100f;
    public float CarControl { get; init; } = 100f;
    public float TireManagement { get; init; } = 80f;
    public float Adaptability { get; init; } = 100f;
    public float Reactions { get; init; } = 100f;
    public float Awareness { get; init; } = 100f;
    public float Overtaking { get; init; } = 100f;
    public float Defending { get; init; } = 100f;

    internal void Validate()
    {
        ValidateRating(Pace, nameof(Pace));
        ValidateRating(Consistency, nameof(Consistency));
        ValidateRating(CarControl, nameof(CarControl));
        ValidateRating(TireManagement, nameof(TireManagement));
        ValidateRating(Adaptability, nameof(Adaptability));
        ValidateRating(Reactions, nameof(Reactions));
        ValidateRating(Awareness, nameof(Awareness));
        ValidateRating(Overtaking, nameof(Overtaking));
        ValidateRating(Defending, nameof(Defending));
    }

    private static void ValidateRating(float rating, string name)
    {
        if (!float.IsFinite(rating) || rating < 0f || rating > 100f)
            throw new ArgumentOutOfRangeException(name, "Driver ratings must be finite and in [0, 100].");
    }
}
