/*
 * Author: ShiQin
 * Date: 2017-08-31
 */

#ifndef COMMON_HELPER_H_
#define COMMON_HELPER_H_

#include <string>
#include <map>
#include <algorithm>

#include <opencv2/core.hpp>

namespace lslam {
    
template<typename A, typename B>
std::pair<B,A> FlipPair(const std::pair<A,B> &p)
{
    return std::pair<B,A>(p.second, p.first);
}

// Sorting std::map using value
template<typename A, typename B>
std::multimap<B,A> FlipMap(const std::map<A,B> &src)
{
    std::multimap<B,A> dst;
    std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()), 
                    FlipPair<A,B>);
    return dst;
}

/// Performs a binary search for an element
///
/// The range `[first, last)` must be ordered via `comparer`.  If `value` is
/// found in the range, an iterator to the first element comparing equal to
/// `value` will be returned; if `value` is not found in the range, `last` is
/// returned.
template <typename RandomAccessIterator, typename Value, typename Comparer=std::less<Value>>
auto binary_search(RandomAccessIterator const  first,
                   RandomAccessIterator const  last,
                   Value                const& value,
                   Comparer                    comparer={}) -> RandomAccessIterator
{
    RandomAccessIterator it(std::lower_bound(first, last, value, comparer));
    if (it == last || comparer(*it, value) || comparer(value, *it))
        return last;

    return it;
}

} // namespace LSLAM

#endif  //COMMON_HELPER_H_
