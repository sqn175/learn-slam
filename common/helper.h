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

} // namespace LSLAM

#endif  //COMMON_HELPER_H_
