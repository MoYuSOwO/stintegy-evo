using System.Numerics;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>A short forecast in track coordinates, shared by the route and pace generators.</summary>
internal readonly struct JointTraffic
{
    private const float ForecastSeconds = 1.5f;
    private const float NearbyDistance = 25f;
    private readonly Rival[] _rivals;
    private readonly float _ownOffset;
    private readonly DriverTrackView _track;
    private readonly RaceCarSnapshot _own;

    private readonly record struct SpeedSegment(float Speed, float Acceleration, float Duration)
    {
        public float Distance(float time)
        {
            float accelerating = MathF.Min(time, Duration);
            if (Acceleration < 0f)
                accelerating = MathF.Min(accelerating, Speed / -Acceleration);
            return (Speed + 0.5f * Acceleration * accelerating) * accelerating +
                Pace(accelerating) * (time - accelerating);
        }

        public float Pace(float time) => MathF.Max(0f, Speed + Acceleration * MathF.Min(time, Duration));
    }

    private readonly record struct Rival(string Id, float Gap, float Offset, SpeedSegment Measured,
        float HalfLength, float HalfWidth, bool Alongside, bool ClosingLeader,
        float Length, float Width, float BodyAngle, float[] SideForecast,
        float BrakeAt = float.PositiveInfinity, SpeedSegment Braking = default)
    {
        public float Speed => Measured.Speed;
        public float Position(float time) => Gap + (time <= BrakeAt ? Measured.Distance(time) :
            Measured.Distance(BrakeAt) + Braking.Distance(time - BrakeAt));

        public float Side(float time)
        {
            float sample = Math.Clamp(time * 20f, 0f, SideForecast.Length - 1);
            int k = Math.Min((int)sample, SideForecast.Length - 2);
            return SideForecast[k] + (sample - k) * (SideForecast[k + 1] - SideForecast[k]);
        }
        public float Pace(float time) => time <= BrakeAt ? Measured.Pace(time) : Braking.Pace(time - BrakeAt);
    }

    private JointTraffic(Rival[] rivals, RaceCarSnapshot own, DriverTrackView track) =>
        (_rivals, _ownOffset, _own, _track) = (rivals, own.TrackD, own, track);

    public bool HasRivals => _rivals.Length != 0;
    public bool HasAlongside => _rivals.Any(rival => rival.Alongside);
    public bool HasClosingLeader => _rivals.Any(rival => rival.ClosingLeader);
    public bool HasNearbyRival => _rivals.Any(rival => MathF.Abs(rival.Gap) < NearbyDistance);

    public static JointTraffic Capture(in DriverContext context, LaneSet lanes)
    {
        RaceCarSnapshot car = context.Car;
        if (context.Frame.Count <= 1)
            return new JointTraffic([], car, context.Track);
        var rivals = new List<Rival>();
        float length = context.Track.LengthMeters;
        var (_, ownScale, ownTrackScale) = TrackMotion(context.Track, car);
        float ownSpeed = car.SpeedMetersPerSecond * ownScale;
        float maximumSpeed = LaneSpeedPlanner.TopSpeed * ownTrackScale;
        float drive = (MathF.Max(8f, car.Capabilities.MaxDriveAcceleration) + 5f) * ownTrackScale;
        float brakeLimit = (MathF.Max(12f, car.MaximumBrakeDecelerationMetersPerSecondSquared) + 5f) * ownTrackScale;
        for (int i = 0; i < context.Frame.Count; i++)
        {
            if (i == context.CarSnapshotIndex)
                continue;
            RaceCarSnapshot other = context.Frame[i];
            float gap = MathF.IEEERemainder(other.TrackS - car.TrackS, length);
            float halfLength = 0.5f * (car.LengthMeters + other.LengthMeters);
            float buffer = MathF.Max(NearbyDistance, halfLength + 8f);
            // The projected speed can at most double. Discard the clearly
            // unreachable cars before querying track shape or grip.
            if (gap > buffer + 2f * LaneSpeedPlanner.TopSpeed * ForecastSeconds ||
                gap < -buffer - (2f * other.SpeedMetersPerSecond + 4f) * ForecastSeconds)
                continue;
            var (roadHeading, alongScale, _) = TrackMotion(context.Track, other);
            float angle = other.VelocityHeadingRadians - roadHeading;
            float speed = other.SpeedMetersPerSecond * alongScale;
            float lateral = -other.SpeedMetersPerSecond * MathF.Sin(angle);
            float halfWidth = 0.5f * (car.WidthMeters + other.WidthMeters);
            float acceleration = (other.Motion?.StepLongitudinalAccel ?? 0f) * alongScale;
            if (!float.IsFinite(acceleration))
                acceleration = 0f;
            // Preserve the observed braking force over the next two plans.
            // Beyond that, the bend hypothesis decides whether braking continues.
            var measured = new SpeedSegment(speed, MathF.Min(acceleration, 8f), acceleration < 0f ? 0.2f : 0.5f);
            float closing = car.SpeedMetersPerSecond - speed;
            bool alongside = MathF.Abs(gap) < halfLength + 1f &&
                MathF.Abs(car.TrackD - other.TrackD) >= halfWidth * 0.8f;
            bool aheadInReach = gap >= 0f && gap < halfLength + MathF.Max(30f, 2f * closing);
            var brake = aheadInReach ? AnticipatedBrake(other, measured, alongScale) : default;
            bool challenger = gap < 0f && gap > -35f && speed > car.SpeedMetersPerSecond + 0.5f;
            if (MathF.Abs(gap) > buffer && !challenger && !CouldMeet(gap, measured, brake, halfLength + 8f))
                continue;
            bool closingLeader = aheadInReach && (brake.Segment.Duration > 0f || acceleration < -1f ||
                MathF.Abs(car.TrackD - other.TrackD) < halfWidth + 0.7f && closing > -1f);
            var rival = new Rival(other.Id, gap, other.TrackD, measured,
                halfLength, halfWidth, alongside, closingLeader, other.LengthMeters, other.WidthMeters,
                other.HeadingRadians - roadHeading, new float[31]);
            float reference = FreeOffset(other.TrackS);
            float referenceRate = (FreeOffset(other.TrackS + 0.1f * speed) - reference) * 10f;
            bool followsReference = MathF.Abs(other.TrackD - reference) < 0.9f &&
                MathF.Abs(lateral - referenceRate) < 4f;
            FillSide(rival);
            rivals.Add(rival);
            if (brake.Segment.Duration > 0f)
            {
                var alternative = rival with { BrakeAt = brake.At, Braking = brake.Segment, SideForecast = new float[31] };
                FillSide(alternative);
                rivals.Add(alternative);
            }

            void FillSide(Rival forecast)
            {
                for (int k = 0; k < forecast.SideForecast.Length; k++)
                {
                    float time = k * 0.05f;
                    float drift = other.TrackD + lateral * MathF.Min(time, 0.2f) +
                        Math.Clamp(lateral, -4f, 4f) * Math.Clamp(time - 0.2f, 0f, 0.4f);
                    float expected = other.TrackD + FreeOffset(car.TrackS + forecast.Position(time)) - reference;
                    float blend = followsReference ? Math.Clamp((time - 0.1f) * 5f, 0f, 1f) : 0f;
                    forecast.SideForecast[k] = drift + blend * (expected - drift);
                }
            }
        }
        return new JointTraffic([.. rivals], car, context.Track);

        bool CouldMeet(float gap, SpeedSegment measured, (float At, SpeedSegment Segment) braking, float buffer)
        {
            // Integrate a bound on closing speed. At each interval take the
            // fastest follower and slowest leader allowed by these forecasts.
            const float interval = 0.25f;
            float remaining = MathF.Abs(gap) - buffer;
            float previous = measured.Speed;
            for (int i = 1; i <= (int)(ForecastSeconds / interval); i++)
            {
                float time = i * interval;
                float nominal = measured.Pace(time);
                float slower = braking.Segment.Duration > 0f && time > braking.At
                    ? braking.Segment.Pace(time - braking.At)
                    : nominal;
                float closing = gap >= 0f
                    ? MathF.Min(MathF.Max(maximumSpeed, ownSpeed), ownSpeed + drive * time) - MathF.Min(previous, slower)
                    : MathF.Max(previous, nominal) - MathF.Max(0f, ownSpeed - brakeLimit * time);
                remaining -= interval * MathF.Max(0f, closing);
                if (remaining <= 0f)
                    return true;
                previous = gap >= 0f ? slower : nominal;
            }
            return false;
        }

        (float At, SpeedSegment Segment) AnticipatedBrake(RaceCarSnapshot other, SpeedSegment measured, float alongScale)
        {
            if (measured.Speed < 20f || alongScale < 0.5f || other.GripUse is not { } tyres)
                return default;
            float grip = tyres.FrontGripAccel + tyres.RearGripAccel;
            float deceleration = MathF.Min(other.MaximumBrakeDecelerationMetersPerSecondSquared, 0.85f * grip) * alongScale;
            if (deceleration < 3f)
                return default;

            int start = (int)lanes.StationOf(other.TrackS);
            int stride = Math.Max(1, (int)MathF.Round(8f / lanes.Spacing));
            float earliest = float.PositiveInfinity;
            float target = 0f;
            for (int k = 2 * stride; k * lanes.Spacing <= 95f; k += stride)
            {
                int at = start + k;
                float curvature = MathF.Abs(LaneSet.Curvature(Reference(at - stride), Reference(at), Reference(at + stride)));
                if (curvature < 1e-4f)
                    continue;
                float cornerSpeed = MathF.Sqrt(0.9f * grip / curvature) * alongScale;
                if (cornerSpeed > measured.Speed - 4f)
                    continue;
                float brakingDistance = (measured.Speed * measured.Speed - cornerSpeed * cornerSpeed) / (2f * deceleration);
                float brakeAt = MathF.Max(0.05f, (k * lanes.Spacing - brakingDistance) / measured.Speed - 0.15f);
                if (brakeAt > 0.75f || brakeAt >= earliest)
                    continue;
                earliest = brakeAt;
                target = cornerSpeed;
            }
            if (!float.IsFinite(earliest))
                return default;
            earliest = MathF.Max(earliest, measured.Acceleration < 0f ? measured.Duration : 0f);
            float startSpeed = measured.Pace(earliest);
            float duration = MathF.Min(0.65f, (startSpeed - target) / deceleration);
            return duration > 0.05f ? (earliest, new SpeedSegment(startSpeed, -deceleration, duration)) : default;
        }

        Vector2 Reference(int station) => lanes.Position(station, lanes.Offset(LaneId.Free, station));

        float FreeOffset(float s)
        {
            float station = lanes.StationOf(s);
            int k = (int)station;
            float a = lanes.Offset(LaneId.Free, k), b = lanes.Offset(LaneId.Free, k + 1);
            return a + (station - k) * (b - a);
        }
    }

    private static (float Heading, float AlongScale, float TrackScale) TrackMotion(DriverTrackView track, RaceCarSnapshot car)
    {
        var road = track.Sample(car.TrackS);
        var ahead = track.Sample(car.TrackS + 1f);
        var behind = track.Sample(car.TrackS - 1f);
        float metresPerTrackMetre = 0.5f * Vector2.Dot(
            ahead.Center - behind.Center + (ahead.Normal - behind.Normal) * car.TrackD, road.Tangent);
        float trackScale = 1f / MathF.Max(0.5f, metresPerTrackMetre);
        float alignment = MathF.Max(0f, MathF.Cos(car.VelocityHeadingRadians - road.Heading));
        return (road.Heading, alignment / MathF.Max(0.5f, metresPerTrackMetre), trackScale);
    }

    public string? Challenger(float ownSpeed)
    {
        string? id = null;
        float nearest = -35f;
        foreach (Rival rival in _rivals)
            if (rival.Gap < -rival.HalfLength && rival.Gap > nearest && rival.Speed > ownSpeed + 0.5f)
                (id, nearest) = (rival.Id, rival.Gap);
        return id;
    }

    // Score the quickest lateral response available to the challenger at a
    // common time. The penalty is bounded, so giving away more time to defend
    // than it can save is never worthwhile.
    public float DefencePenalty(string challenger, float distance, float offset, float time, float halfRoad)
    {
        foreach (Rival rival in _rivals)
        {
            if (rival.Id != challenger || rival.Position(time) < distance - rival.HalfLength - 2f)
                continue;
            float expected = rival.Side(time);
            float room = MathF.Max(0f, halfRoad - rival.HalfWidth * 0.5f);
            float clearance = rival.HalfWidth + 0.2f;
            float left = offset - clearance, right = offset + clearance;
            float shift = float.PositiveInfinity;
            if (left >= -room)
                shift = MathF.Max(0f, expected - left);
            if (right <= room)
                shift = MathF.Min(shift, MathF.Max(0f, right - expected));
            return 0.25f * Math.Clamp(1f - shift / 2.5f, 0f, 1f);
        }
        return 0f;
    }

    public bool Overlaps(float distance, float offset, float time)
    {
        foreach (Rival rival in _rivals)
            if (MathF.Abs(rival.Position(time) - distance) < rival.HalfLength + 0.35f &&
                MathF.Abs(rival.Side(time) - offset) < rival.HalfWidth + 0.2f)
                return true;
        return false;
    }

    public bool OverlapsBetween(float from, float fromOffset, float fromTime, float to, float toOffset, float toTime,
        ref float proximity)
    {
        foreach (Rival rival in _rivals)
        {
            float low = 0f, high = 1f;
            float alongMargin = rival.HalfLength + 1.25f;
            float lateralMargin = rival.HalfWidth + 0.85f;
            float along = rival.Position(toTime) - to;
            float side = rival.Side(toTime) - toOffset;
            float fromAlong = rival.Position(fromTime) - from;
            float fromSide = rival.Side(fromTime) - fromOffset;
            if (Intersects(fromAlong, along, alongMargin + 2f, ref low, ref high) &&
                Intersects(fromSide, side, lateralMargin + 2f, ref low, ref high) &&
                (BodiesOverlap(rival, low, from, fromOffset, fromTime, to, toOffset, toTime) ||
                 BodiesOverlap(rival, 0.5f * (low + high), from, fromOffset, fromTime, to, toOffset, toTime) ||
                 BodiesOverlap(rival, high, from, fromOffset, fromTime, to, toOffset, toTime)))
                return true;
            // Charge the closest approach along the plan without making
            // the preferred clearance a second hard constraint.
            float alongClearance = MathF.Abs(along) - rival.HalfLength;
            float sideClearance = MathF.Abs(side) - rival.HalfWidth;
            float near = Math.Clamp(1f - alongClearance / 7f, 0f, 1f) *
                Math.Clamp(1f - sideClearance / 2.5f, 0f, 1f);
            proximity = MathF.Max(proximity, near);
        }
        return false;

        static bool Intersects(float a, float b, float margin, ref float low, ref float high)
        {
            float delta = b - a;
            if (MathF.Abs(delta) < 1e-5f)
                return MathF.Abs(a) < margin;
            float x = (-margin - a) / delta, y = (margin - a) / delta;
            low = MathF.Max(low, MathF.Min(x, y));
            high = MathF.Min(high, MathF.Max(x, y));
            return low < high;
        }
    }

    private bool BodiesOverlap(Rival rival, float u, float from, float fromOffset, float fromTime,
        float to, float toOffset, float toTime)
    {
        float time = fromTime + u * (toTime - fromTime);
        var road = _track.Sample(_own.TrackS + from + u * (to - from));
        var otherRoad = _track.Sample(_own.TrackS + rival.Position(time));
        Vector2 center = road.Center + road.Normal * (fromOffset + u * (toOffset - fromOffset));
        Vector2 otherCenter = otherRoad.Center + otherRoad.Normal * rival.Side(time);
        float heading = road.Heading - MathF.Atan2(toOffset - fromOffset, MathF.Max(0.1f, to - from));
        float ownYawBlend = Math.Clamp(time * 8f, 0f, 1f);
        heading = _own.HeadingRadians +
            ownYawBlend * MathF.IEEERemainder(heading - _own.HeadingRadians, 2f * MathF.PI);
        float expectedSideSpeed = (rival.Side(time + 0.05f) - rival.Side(time)) * 20f;
        float otherHeading = otherRoad.Heading - MathF.Atan2(expectedSideSpeed, MathF.Max(1f, rival.Pace(time)));
        float yawBlend = Math.Clamp(time * 5f, 0f, 1f);
        otherHeading = otherRoad.Heading + rival.BodyAngle +
            yawBlend * MathF.IEEERemainder(otherHeading - otherRoad.Heading - rival.BodyAngle, 2f * MathF.PI);
        var ownBody = CarBodyGeometry.FromPose(center, heading, _own.LengthMeters + 1f, _own.WidthMeters + 0.7f);
        var otherBody = CarBodyGeometry.FromPose(otherCenter, otherHeading, rival.Length + 1f, rival.Width + 0.7f);
        return ownBody.Overlaps(otherBody);
    }

    public bool KeepsAlongsideRoom(float distance, float offset, float time)
    {
        foreach (Rival rival in _rivals)
        {
            if (!rival.Alongside || MathF.Abs(rival.Position(time) - distance) >= rival.HalfLength + 1f)
                continue;
            float side = MathF.Sign(_ownOffset - rival.Offset);
            float room = MathF.Min(rival.HalfWidth + 0.35f, MathF.Abs(_ownOffset - rival.Offset));
            if (side * (offset - rival.Side(time)) < room)
                return false;
        }
        return true;
    }

    public float Pace(float distance, float offset, float time, float speed)
    {
        float target = float.PositiveInfinity;
        foreach (Rival rival in _rivals)
        {
            if (rival.Gap < 0f || MathF.Abs(rival.Side(time) - offset) > rival.HalfWidth + 0.85f)
                continue;
            float gap = rival.Position(time) - distance;
            if (gap <= -rival.HalfLength)
                continue;
            float room = gap - rival.HalfLength - 2f - 0.1f * speed;
            target = MathF.Min(target, MathF.Max(0f, rival.Pace(time) + room / 0.4f));
        }
        return target;
    }
}
