using System;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.Core.Drivers;

internal readonly record struct RacingRoomPairSnapshot(
    string LeftCarId,
    string RightCarId,
    string EntryTrailingCarId,
    float SeparatorFraction,
    int Generation
)
{
    public bool Contains(string carId)
    {
        return string.Equals(LeftCarId, carId, StringComparison.Ordinal) ||
               string.Equals(RightCarId, carId, StringComparison.Ordinal);
    }

    public bool Matches(string firstCarId, string secondCarId)
    {
        return Contains(firstCarId) && Contains(secondCarId);
    }
}

internal readonly record struct RacingRoomCorridor(
    float MinimumTrackD,
    float MaximumTrackD,
    bool Feasible,
    bool MustYield,
    int Generation
);

internal readonly struct RacingRoomSnapshot
{
    private const float TrackEdgeMarginMeters = 0.3f;
    private const float CarToCarGapMeters = 0.2f;

    private readonly RacingRoomPairSnapshot[]? _pairs;

    internal RacingRoomSnapshot(RacingRoomPairSnapshot[] pairs, int count)
    {
        _pairs = pairs;
        Count = count;
    }

    public int Count { get; }
    public ReadOnlySpan<RacingRoomPairSnapshot> Pairs =>
        _pairs is null ? [] : _pairs.AsSpan(0, Count);

    public bool TryGetPair(
        string firstCarId,
        string secondCarId,
        out RacingRoomPairSnapshot pair
    )
    {
        if (_pairs is not null)
        {
            for (int i = 0; i < Count; i++)
            {
                if (_pairs[i].Matches(firstCarId, secondCarId))
                {
                    pair = _pairs[i];
                    return true;
                }
            }
        }

        pair = default;
        return false;
    }

    public bool TryGetCorridor(
        string carId,
        float trackHalfWidthMeters,
        float carWidthMeters,
        out RacingRoomCorridor corridor
    )
    {
        float halfCarWidth = MathF.Max(0f, carWidthMeters * 0.5f);
        float usableHalfWidth = MathF.Max(0f, trackHalfWidthMeters) -
                                TrackEdgeMarginMeters - halfCarWidth;
        bool carFitsTrack = usableHalfWidth >= 0f;
        usableHalfWidth = MathF.Max(0f, usableHalfWidth);
        float minimumD = -usableHalfWidth;
        float maximumD = usableHalfWidth;
        bool active = false;
        bool entryTrailing = false;
        int generation = 0;

        if (_pairs is not null)
        {
            for (int i = 0; i < Count; i++)
            {
                RacingRoomPairSnapshot pair = _pairs[i];
                bool isLeft = string.Equals(
                    pair.LeftCarId,
                    carId,
                    StringComparison.Ordinal
                );
                bool isRight = string.Equals(
                    pair.RightCarId,
                    carId,
                    StringComparison.Ordinal
                );
                if (!isLeft && !isRight)
                    continue;

                active = true;
                generation = Math.Max(generation, pair.Generation);
                entryTrailing |= string.Equals(
                    pair.EntryTrailingCarId,
                    carId,
                    StringComparison.Ordinal
                );
                float separatorD = pair.SeparatorFraction *
                                   MathF.Max(0f, trackHalfWidthMeters);
                float sideClearance = halfCarWidth +
                                      CarToCarGapMeters * 0.5f;
                if (isLeft)
                    minimumD = MathF.Max(minimumD, separatorD + sideClearance);
                else
                    maximumD = MathF.Min(maximumD, separatorD - sideClearance);
            }
        }

        bool feasible = carFitsTrack && minimumD <= maximumD;
        corridor = new RacingRoomCorridor(
            minimumD,
            maximumD,
            feasible,
            active && !feasible && entryTrailing,
            generation
        );
        return active;
    }
}

/// <summary>
/// Simulation-owned referee for persistent side-by-side room. It consumes one
/// frozen physical frame and produces one order-independent result before any
/// driver is allowed to plan.
/// </summary>
internal sealed class RacingRoomCoordinator
{
    private const float SameDirectionDotThreshold = 0.5f;

    private RacingRoomPairSnapshot[] _pairs = [];
    private RacingRoomCandidate[] _candidates = [];
    private int _pairCount;
    private int _nextGeneration = 1;

    public RacingRoomSnapshot Update(ReadOnlySpan<RaceCarSnapshot> cars)
    {
        int maximumPairCount = cars.Length * Math.Max(0, cars.Length - 1) / 2;
        EnsureCapacity(maximumPairCount);
        RetainValidReservations(cars);

        int candidateCount = 0;
        for (int firstIndex = 0; firstIndex < cars.Length; firstIndex++)
        {
            for (int secondIndex = firstIndex + 1;
                 secondIndex < cars.Length;
                 secondIndex++)
            {
                RaceCarSnapshot first = cars[firstIndex];
                RaceCarSnapshot second = cars[secondIndex];
                if (FindPair(first.Id, second.Id) >= 0 ||
                    !AreLaterallyAdjacent(
                        cars,
                        firstIndex,
                        secondIndex
                    ) ||
                    !RacingRoomGeometry.TryCreateCandidate(
                        in first,
                        in second,
                        out RacingRoomCandidate candidate
                    ))
                {
                    continue;
                }

                _candidates[candidateCount++] = candidate;
            }
        }

        if (candidateCount > 1)
        {
            Array.Sort(
                _candidates,
                0,
                candidateCount,
                RacingRoomCandidateComparer.Instance
            );
        }
        bool addedPair = false;
        for (int i = 0; i < candidateCount; i++)
        {
            RacingRoomCandidate candidate = _candidates[i];
            if (FindPair(candidate.LeftCarId, candidate.RightCarId) >= 0)
                continue;

            RacingRoomPairSnapshot pair = new(
                candidate.LeftCarId,
                candidate.RightCarId,
                candidate.EntryTrailingCarId,
                candidate.SeparatorFraction,
                _nextGeneration
            );
            if (!CanAcceptPair(in pair, cars))
                continue;

            _pairs[_pairCount++] = pair;
            _nextGeneration++;
            addedPair = true;
        }

        // Compaction retains the already sorted order. Sorting is therefore a
        // topology-change cost, not a per-frame cost while a pack is stable.
        if (addedPair && _pairCount > 1)
        {
            Array.Sort(
                _pairs,
                0,
                _pairCount,
                RacingRoomPairComparer.Instance
            );
        }
        return new RacingRoomSnapshot(_pairs, _pairCount);
    }

    private void RetainValidReservations(ReadOnlySpan<RaceCarSnapshot> cars)
    {
        int writeIndex = 0;
        for (int pairIndex = 0; pairIndex < _pairCount; pairIndex++)
        {
            RacingRoomPairSnapshot pair = _pairs[pairIndex];
            int leftIndex = FindCar(cars, pair.LeftCarId);
            int rightIndex = FindCar(cars, pair.RightCarId);
            if (leftIndex < 0 || rightIndex < 0)
                continue;

            RaceCarSnapshot left = cars[leftIndex];
            RaceCarSnapshot right = cars[rightIndex];
            if (!CanMaintain(in left, in right) ||
                RacingRoomGeometry.HasFullBodyClearance(in left, in right))
            {
                continue;
            }

            _pairs[writeIndex++] = pair;
        }

        _pairCount = writeIndex;
    }

    private int FindPair(string firstCarId, string secondCarId)
    {
        for (int i = 0; i < _pairCount; i++)
        {
            if (_pairs[i].Matches(firstCarId, secondCarId))
                return i;
        }
        return -1;
    }

    private bool CanAcceptPair(
        in RacingRoomPairSnapshot pair,
        ReadOnlySpan<RaceCarSnapshot> cars
    )
    {
        _pairs[_pairCount] = pair;
        RacingRoomSnapshot tentative = new(_pairs, _pairCount + 1);
        for (int i = 0; i < cars.Length; i++)
        {
            RaceCarSnapshot car = cars[i];
            if (tentative.TryGetCorridor(
                    car.Id,
                    car.TrackWidthMeters * 0.5f,
                    car.WidthMeters,
                    out RacingRoomCorridor corridor
                ) &&
                !corridor.Feasible)
            {
                return false;
            }
        }
        return true;
    }

    private static bool AreLaterallyAdjacent(
        ReadOnlySpan<RaceCarSnapshot> cars,
        int firstIndex,
        int secondIndex
    )
    {
        RaceCarSnapshot first = cars[firstIndex];
        RaceCarSnapshot second = cars[secondIndex];
        float lowerD = MathF.Min(first.TrackD, second.TrackD);
        float upperD = MathF.Max(first.TrackD, second.TrackD);
        for (int i = 0; i < cars.Length; i++)
        {
            if (i == firstIndex || i == secondIndex)
                continue;

            RaceCarSnapshot between = cars[i];
            if (between.TrackD <= lowerD || between.TrackD >= upperD)
                continue;

            bool overlapsFirst = LongitudinalBodiesOverlap(
                in between,
                in first
            );
            bool overlapsSecond = LongitudinalBodiesOverlap(
                in between,
                in second
            );
            if (overlapsFirst && overlapsSecond)
                return false;
        }
        return true;
    }

    private static bool LongitudinalBodiesOverlap(
        in RaceCarSnapshot first,
        in RaceCarSnapshot second
    )
    {
        float separation = MathF.Abs(
            RacingRoomGeometry.SignedLongitudinalDelta(in first, in second)
        );
        float extent = 0.5f * (
            MathF.Max(0f, first.LengthMeters) +
            MathF.Max(0f, second.LengthMeters)
        );
        return separation <= extent;
    }

    private static int FindCar(
        ReadOnlySpan<RaceCarSnapshot> cars,
        string carId
    )
    {
        for (int i = 0; i < cars.Length; i++)
        {
            if (string.Equals(cars[i].Id, carId, StringComparison.Ordinal))
                return i;
        }
        return -1;
    }

    private static bool CanMaintain(
        in RaceCarSnapshot first,
        in RaceCarSnapshot second
    )
    {
        return first.Region == TrackRegion.RacingSurface &&
               second.Region == TrackRegion.RacingSurface &&
               MathF.Cos(first.HeadingRadians - second.HeadingRadians) >
               SameDirectionDotThreshold;
    }

    private void EnsureCapacity(int required)
    {
        if (_pairs.Length >= required)
            return;

        int capacity = Math.Max(required, Math.Max(4, _pairs.Length * 2));
        Array.Resize(ref _pairs, capacity);
        Array.Resize(ref _candidates, capacity);
    }

    private sealed class RacingRoomCandidateComparer :
        System.Collections.Generic.IComparer<RacingRoomCandidate>
    {
        public static readonly RacingRoomCandidateComparer Instance = new();

        public int Compare(RacingRoomCandidate x, RacingRoomCandidate y)
        {
            int overlap = y.LongitudinalOverlapMeters.CompareTo(
                x.LongitudinalOverlapMeters
            );
            if (overlap != 0)
                return overlap;

            string xFirst = CanonicalFirst(x.LeftCarId, x.RightCarId);
            string yFirst = CanonicalFirst(y.LeftCarId, y.RightCarId);
            int first = string.CompareOrdinal(xFirst, yFirst);
            if (first != 0)
                return first;
            return string.CompareOrdinal(
                CanonicalSecond(x.LeftCarId, x.RightCarId),
                CanonicalSecond(y.LeftCarId, y.RightCarId)
            );
        }
    }

    private sealed class RacingRoomPairComparer :
        System.Collections.Generic.IComparer<RacingRoomPairSnapshot>
    {
        public static readonly RacingRoomPairComparer Instance = new();

        public int Compare(RacingRoomPairSnapshot x, RacingRoomPairSnapshot y)
        {
            string xFirst = CanonicalFirst(x.LeftCarId, x.RightCarId);
            string yFirst = CanonicalFirst(y.LeftCarId, y.RightCarId);
            int first = string.CompareOrdinal(xFirst, yFirst);
            if (first != 0)
                return first;
            return string.CompareOrdinal(
                CanonicalSecond(x.LeftCarId, x.RightCarId),
                CanonicalSecond(y.LeftCarId, y.RightCarId)
            );
        }
    }

    private static string CanonicalFirst(string first, string second)
    {
        return string.CompareOrdinal(first, second) <= 0 ? first : second;
    }

    private static string CanonicalSecond(string first, string second)
    {
        return string.CompareOrdinal(first, second) <= 0 ? second : first;
    }
}
