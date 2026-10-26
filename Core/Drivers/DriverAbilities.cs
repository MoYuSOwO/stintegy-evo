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
    // Each rating is one thing the driver does; none of them overlaps another.
    // How hard to drive is not one of them: in a car alone that is the rung
    // the pit wall asks for.

    // Driving.

    /// <summary>How much of the car's limit the driver can find: where they believe the edge is (a slow driver reads it short, every lap the same way), how late the brake point, how soon and how surely the throttle out of a corner. Skill, not daring.</summary>
    public float Pace { get; init; } = 100f;

    /// <summary>How alike one corner and one lap are to the next: the wander of the brake marker, the line and the speed, and the swing of form from lap to lap and day to day.</summary>
    public float Consistency { get; init; } = 100f;

    /// <summary>How seldom a corner is got wrong outright: in too fast, the marker missed.</summary>
    public float Focus { get; init; } = 100f;

    /// <summary>The hands: how close to what they aim for the lock and the edge of the tyre land, one corner to the next.</summary>
    public float Precision { get; init; } = 100f;

    /// <summary>Once the car slides, how cleanly it is caught: the opposite lock, the throttle kept, the power given back.</summary>
    public float Recovery { get; init; } = 100f;

    /// <summary>The one delay from something happening to the hands moving: the rear going, the front letting go, the car ahead braking.</summary>
    public float Reactions { get; init; } = 100f;

    /// <summary>How soon a change in grip (tyres going off, the track coming in) is noticed, and how well it is read.</summary>
    public float Adaptability { get; init; } = 100f;

    /// <summary>How much wear the driver's hands add or save, and how closely the tyre rung the pit wall asks for is the one driven.</summary>
    public float TireManagement { get; init; } = 80f;

    /// <summary>How closely the energy rung the pit wall asks for is the one driven.</summary>
    public float EnergyManagement { get; init; } = 80f;

    // Off the road.

    /// <summary>Knowing the car: how near the right setting the dials in the cockpit are set (the brake balance).</summary>
    public float Technical { get; init; } = 100f;

    // Wheel to wheel.

    /// <summary>The cars around: how soon one alongside or in the mirrors is noticed, and how often one in the blind spot is missed. Every driver sees the road ahead alike.</summary>
    public float Awareness { get; init; } = 100f;

    /// <summary>The craft of a pass: choosing where and when, and making the move stick once committed.</summary>
    public float Overtaking { get; init; } = 100f;

    /// <summary>The craft of keeping a place: when to cover, which line, and seldom too late.</summary>
    public float Defending { get; init; } = 100f;

    /// <summary>How much risk the driver takes wheel to wheel: how narrow a gap they go for, how late they leave the door shut. Craft is Overtaking and Defending; this is nerve.</summary>
    public float Boldness { get; init; } = 50f;

    internal void Validate()
    {
        ValidateRating(Pace, nameof(Pace));
        ValidateRating(Consistency, nameof(Consistency));
        ValidateRating(Focus, nameof(Focus));
        ValidateRating(Precision, nameof(Precision));
        ValidateRating(Recovery, nameof(Recovery));
        ValidateRating(Reactions, nameof(Reactions));
        ValidateRating(Adaptability, nameof(Adaptability));
        ValidateRating(TireManagement, nameof(TireManagement));
        ValidateRating(EnergyManagement, nameof(EnergyManagement));
        ValidateRating(Technical, nameof(Technical));
        ValidateRating(Awareness, nameof(Awareness));
        ValidateRating(Overtaking, nameof(Overtaking));
        ValidateRating(Defending, nameof(Defending));
        ValidateRating(Boldness, nameof(Boldness));
    }

    private static void ValidateRating(float rating, string name)
    {
        if (!float.IsFinite(rating) || rating < 0f || rating > 100f)
            throw new ArgumentOutOfRangeException(name, "Driver ratings must be finite and in [0, 100].");
    }
}
