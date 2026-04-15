using System.Collections.Generic;

namespace PloyRacing.Util;

public static class IListExtensions
{
    public static T GetCircular<T>(this IList<T> list, int index)
    {
        int size = list.Count;
        int wrappedIndex = (index % size + size) % size;
        return list[wrappedIndex];
    }
}