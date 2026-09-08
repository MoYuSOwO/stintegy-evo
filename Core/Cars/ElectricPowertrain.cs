using System;
using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// A battery and a motor: one store that does not weigh anything as it
/// empties, five output settings, and brakes that put some of it back.
///
/// This is the car the game was built around and the car every lap time here
/// was calibrated against. It was the whole of the vehicle model until the
/// powertrain became something a car could choose, and the numbers below are
/// the ones it always had.
/// </summary>
public sealed class ElectricPowertrain : IPowertrain
{
    private const float Epsilon = 1e-5f;

    public static readonly ElectricPowertrain Default = new();

    private static readonly PowertrainResourceInfo[] ResourceList =
    {
        // A pack weighs what it weighs whether it is full or flat, so none of
        // this car's mass burns off over a race.
        new("battery", "Charge", 0f)
    };

    private static readonly ModeLadder Ladder = new(
        "Power",
        "Save",
        "Eco",
        "Normal",
        "Push",
        "Attack"
    );

    public IReadOnlyList<PowertrainResourceInfo> Resources => ResourceList;

    public ModeLadder OutputLadder => Ladder;

    /// <summary>
    /// Enough for the middle setting to finish a race on full power, and only
    /// just.
    ///
    /// Sized against the distance the car is built to race - two hundred and
    /// forty kilometres, between a Formula 2 feature race and a Formula 1 one,
    /// forty four laps of a five and a half kilometre circuit. Normal comes
    /// home with eight point eight per cent, which is eight tenths clear of
    /// where the low charge limiter starts, so it runs the whole distance at
    /// the power it was asked for and finishes with nothing to spare. Push and
    /// Attack both end up inside the limiter, which is what a setting above the
    /// middle one is supposed to mean: not something to leave on.
    ///
    /// It was set to a round number before and never checked against a race.
    /// Run over one every setting ran itself flat, including the setting whose
    /// whole job is to save, and finished the last laps crawling - Attack lost
    /// half a minute on its final lap that way. Over the distance the five came
    /// out within four tenths of each other, because whatever any of them
    /// gained early it handed back at the end. That is not five settings.
    ///
    /// Worth being honest about what this still does not buy. Attack finishes
    /// the distance twenty six seconds up on Normal even after the limiter has
    /// taken its share, so leaving it on remains the right thing to do and the
    /// choice is not yet a real one. The settings differ by less than three per
    /// cent in what they consume, because a lap's energy goes mostly into
    /// pushing air aside and that hardly cares where the power cap sits, and
    /// running the last laps on the limiter costs about four seconds. What
    /// would make these five a decision is a way of saving charge that costs a
    /// little lap time and a lot of energy - off the throttle before the
    /// braking zone, which is what energy management actually is - and there is
    /// none of that here.
    /// </summary>
    public float BatteryCapacityJoules { get; init; } = 1100000000f;

    /// <summary>
    /// Why this number and not the 1470 MJ it was.
    ///
    /// Measured rather than chosen: driven flat out on both ladders round
    /// Silverstone, this car spends <b>27.4 MJ a lap</b> (12 lanes, 600
    /// simulated seconds, 68 laps; the scan is
    /// <c>Training/python/scarcity_scan.py</c>). At the old capacity that
    /// bought 53.6 laps against a race distance of 52 — the car could go
    /// flat out for the whole race and finish with charge to spare, which
    /// is why the five power settings differed by under three per cent in
    /// consumption and Attack was always right. A choice with one right
    /// answer is not a choice.
    ///
    /// The band is anchored outside the project rather than to taste.
    /// Formula 2 manages no energy at all; Formula 1's 2026 rules make
    /// deployment something to budget; Formula E uses about twice its
    /// store at full effort, so half the race has to be driven for rather
    /// than raced. Under 1.0 there is no decision. Near 2.0 saving beats
    /// pace and it stops being racing. This project wants the middle:
    /// finishing flat out should cost 1.2 to 1.4 times what is on board.
    ///
    /// 52 laps x 27.4 MJ / 1100 MJ = <b>1.30</b>.
    ///
    /// Nothing else moves with it. <see cref="ConsumableMassKg"/> is zero
    /// for a battery — a pack does not get lighter as it empties — so the
    /// dial changes what there is to allocate and changes no physics.
    ///
    /// What it does <i>not</i> do on its own is reach training. A 240
    /// second episode is about two and a half laps and spends six per cent
    /// of this pack; a policy trained only on that never meets a car that
    /// is running out. Scarcity becomes visible to the learner through
    /// randomised starting store, not through this constant.
    /// </summary>
    public float BatteryDriveEfficiency { get; init; } = 0.92f;

    /// <summary>
    /// The charge below which the pack stops being able to give what it is
    /// asked for, and how sharply it falls away underneath.
    ///
    /// A racing battery does not empty like a bucket. Its voltage sags as the
    /// charge goes, so the power it can deliver goes with it, gently at first
    /// and then not gently at all - and the last of it is never released,
    /// because taking a cell that low ruins it. So the useful bottom of the
    /// pack is well above zero, and a car that plans to use everything is
    /// planning on charge that was never there.
    ///
    /// Which makes this the price of overspending, and it has to be a real
    /// price. Taken away as a straight line from a low starting point, it was
    /// not one: a car that finished a race on a quarter of what it should have
    /// had still enjoyed most of its power the whole way, and running the tank
    /// dry cost about four seconds against the twenty six that spending it
    /// bought. Squared, and starting where a real pack starts to fade, missing
    /// by a little costs a little and missing by a lot costs the race.
    /// </summary>
    public float LowSocPowerLimitStart { get; init; } = 0.20f;

    public float LowSocPowerFalloffExponent { get; init; } = 2f;

    /// <summary>
    /// The limp. See <see cref="OutputAvailability"/> for why it exists and
    /// how the number was picked; in short, eight per cent of nominal is
    /// about 31 kW, which this car's drag turns into roughly 118 km/h
    /// sustained and a lap about twice as slow as a racing one.
    /// </summary>
    public float LowSocPowerFloor { get; init; } = 0.08f;
    public float RegenEfficiency { get; init; } = 0.56f;
    public float RegenPowerCapWatts { get; init; } = 260000f;

    /// <summary>
    /// Below this speed, power is not what limits the car - dividing by it
    /// would hand a standing car an infinite shove - so the force ceiling is
    /// left to do the limiting on its own.
    /// </summary>
    public float MinPowerSpeed { get; init; } = 8f;

    public float SaveDrivePowerLimitWatts { get; init; } = 372000f;
    public float EcoDrivePowerLimitWatts { get; init; } = 381000f;
    public float NormalDrivePowerLimitWatts { get; init; } = 390000f;
    public float PushDrivePowerLimitWatts { get; init; } = 400000f;
    public float AttackDrivePowerLimitWatts { get; init; } = 409000f;

    public float GetDrivePowerLimitWatts(PowerOutputMode mode)
    {
        return GetDrivePowerLimitWatts((int)mode);
    }

    /// <summary>
    /// A rung of this pack's own ladder, clamped to the five it has. A
    /// strategy written for an engine with more of them lands on the top
    /// setting rather than on nothing.
    /// </summary>
    public float GetDrivePowerLimitWatts(int rung)
    {
        return Ladder.Clamp(rung) switch
        {
            1 => SaveDrivePowerLimitWatts,
            2 => EcoDrivePowerLimitWatts,
            3 => NormalDrivePowerLimitWatts,
            4 => PushDrivePowerLimitWatts,
            5 => AttackDrivePowerLimitWatts,
            _ => NormalDrivePowerLimitWatts
        };
    }

    public float GetDrivePowerLimitWatts(CarStrategy strategy)
    {
        if (!strategy.DrivePowerLimitWattsOverride.HasValue)
            return GetDrivePowerLimitWatts(strategy.PowerRung);

        float minimum = Math.Min(
            SaveDrivePowerLimitWatts,
            AttackDrivePowerLimitWatts
        );
        float maximum = Math.Max(
            SaveDrivePowerLimitWatts,
            AttackDrivePowerLimitWatts
        );
        return Math.Clamp(
            strategy.DrivePowerLimitWattsOverride.Value,
            minimum,
            maximum
        );
    }

    public float GetDrivePowerLimitWatts(float sliderPosition)
    {
        float scaled = Math.Clamp(sliderPosition, 0f, 1f) * 4f;
        int segment = Math.Min((int)scaled, 3);
        float t = scaled - segment;
        return segment switch
        {
            0 => Lerp(SaveDrivePowerLimitWatts, EcoDrivePowerLimitWatts, t),
            1 => Lerp(EcoDrivePowerLimitWatts, NormalDrivePowerLimitWatts, t),
            2 => Lerp(NormalDrivePowerLimitWatts, PushDrivePowerLimitWatts, t),
            _ => Lerp(PushDrivePowerLimitWatts, AttackDrivePowerLimitWatts, t)
        };
    }

    /// <summary>
    /// What a nearly empty pack can still deliver, as a fraction of
    /// nominal — and the floor it never falls through.
    ///
    /// The fade itself is unchanged: below <see cref="LowSocPowerLimitStart"/>
    /// the available power falls away as the square of what is left, so
    /// missing the target by a little costs a little and missing it by a
    /// lot costs the race. What changed is the bottom. It used to reach
    /// zero, and a car at zero is not a car being punished — it is a car
    /// that has stopped, taking a retirement and becoming an obstacle for
    /// everyone still racing. Running out of energy should be ruinous and
    /// survivable, the way it is on a real grid, where a car that has
    /// nothing left still crawls home under its own power.
    ///
    /// So the floor. It is not a taste: at
    /// <see cref="LowSocPowerFloor"/> of nominal this car can hold about
    /// 118 km/h against its own drag, which puts a drained lap at roughly
    /// twice a racing lap. That is the price — a lap lost per lap — and it
    /// is far enough above the harness's one metre a second stall line
    /// that a drained car keeps moving even up the steepest gradient in
    /// the set. Recovery is untouched: braking still charges the pack at
    /// full efficiency, so a driver who overspent can nurse it back rather
    /// than being written off.
    /// </summary>
    public float OutputAvailability(in PowertrainState energy)
    {
        float charge = energy.Primary;
        if (charge >= LowSocPowerLimitStart)
            return 1f;

        float faded = MathF.Pow(
            Math.Clamp(
                charge / Math.Max(LowSocPowerLimitStart, Epsilon),
                0f,
                1f
            ),
            MathF.Max(LowSocPowerFalloffExponent, 1f)
        );
        // Blended rather than clamped, so the curve stays continuous and
        // monotone through the whole fade instead of running into a shelf.
        return LowSocPowerFloor + (1f - LowSocPowerFloor) * faded;
    }

    /// <summary>
    /// How hard the car may be driven right now, given what is left.
    ///
    /// An empty pack used to answer zero here, which is not a punishment
    /// but a full stop: the car parks itself, the harness calls it a
    /// retirement, and everyone still racing inherits an obstacle. The
    /// early return is gone. A pack at nothing goes through the same fade
    /// as a pack at anything else and lands on the floor
    /// <see cref="OutputAvailability"/> holds it at, so overspending is
    /// ruinous and survivable rather than terminal.
    /// </summary>
    public float DriveAccelerationLimit(
        in PowertrainState energy,
        in CarStrategy strategy,
        float speed,
        float massKg,
        float chassisForceCeiling
    )
    {
        float powerLimitedAccel =
            GetDrivePowerLimitWatts(strategy) /
            (massKg * Math.Max(speed, MinPowerSpeed));

        return Math.Min(chassisForceCeiling, powerLimitedAccel) *
               OutputAvailability(energy);
    }

    public PowertrainSettlement Settle(
        in PowertrainState energy,
        float driveAccel,
        float brakeAccel,
        float speed,
        float massKg,
        float dt
    )
    {
        float drivePower = driveAccel <= 0f
            ? 0f
            : massKg * driveAccel * speed /
              Math.Max(BatteryDriveEfficiency, Epsilon);
        float regenPower = RecoveredPowerWatts(brakeAccel, speed, massKg);

        float netEnergy = (drivePower - regenPower) * dt;

        PowertrainState spent = energy;
        spent.Primary = Math.Clamp(
            spent.Primary - netEnergy / Math.Max(BatteryCapacityJoules, Epsilon),
            0f,
            1f
        );
        return new PowertrainSettlement(spent, drivePower);
    }

    public float RecoveredPowerWatts(float brakeAccel, float speed, float massKg)
    {
        if (brakeAccel <= 0f || speed <= 0f)
            return 0f;

        float brakePower = massKg * brakeAccel * speed;
        return Math.Min(brakePower * RegenEfficiency, RegenPowerCapWatts);
    }

    public float ConsumableMassKg(in PowertrainState energy)
    {
        return 0f;
    }

    private static float Lerp(float from, float to, float t) =>
        from + (to - from) * t;
}
