/*
 * Author: ShiQin
 * Date: 2017-08-31
 */

#ifndef COMMON_HELPER_H_
#define COMMON_HELPER_H_

#include <string>
#include <map>
#include <algorithm>
#include <chrono>

#include <opencv2/core.hpp>

namespace lslam {
    
template<typename TimeT = std::chrono::nanoseconds, 
        class ClockT = std::chrono::steady_clock>
struct Measure
{
    template<typename F, typename ...Args>
    static typename TimeT::rep Execution(F func, Args&&... args)
    {
        auto start = ClockT::now();

        // Now call the function with all the parameters you need.
        func(std::forward<Args>(args)...);

        auto duration = std::chrono::duration_cast<TimeT>(ClockT::now() - start);

        return duration.count();
    }
};

template<typename A, typename B>
std::pair<B,A> FlipPair(const std::pair<A,B> &p)
{
    return std::pair<B,A>(p.second, p.first);
}

// Sorting std::map using value B
// If B is the same, key A which is inserted in src first is inserted later in dst
template<typename A, typename B>
std::multimap<B,A> FlipMap(const std::map<A,B> &src)
{
    std::multimap<B,A> dst;
    std::transform(src.rbegin(), src.rend(), std::inserter(dst, dst.begin()), 
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

// http://roth.cs.kuleuven.be/w-ess/index.php/Accurate_variance_and_mean_calculations_in_C%2B%2B11
// example:
// int main()
// {
//     Accumulator<float_t> a;
// std::for_each(std::istream_iterator<double>(std::cin),
//               std::istream_iterator<double>(), 
//               [&a](const double& x) { // lambda function which has access
//                                       // to 'a' by reference
//                   a(x); // add x to the accumulator; then print statistics:
//                   std::cout << a;
//               } );
// }
template <typename T, typename T2=T>
struct Accumulator
{
    T2 sum; // we could plug in a more accurate type for the sum
    T S;
    T M;
    size_t N;
    
    bool is_empty;
    T min;
    T max;

    // default constructor initializes all values
    Accumulator() : sum(0), S(0), M(0), N(0), is_empty(true) { }
 
    // add another number
    T2 operator()(const T& x)
    {
        ++N;
        sum += x;
        T Mprev = M;
        M += (x - Mprev) / N;
        S += (x - Mprev) * (x - M);
        if (is_empty) {
            min = x;
			max = x;
			is_empty = false;
        } else {
			if (x < min) {
				min = x;
			} else if (x > max) {
				max = x;
			}
		}
        return sum;
    }
 
    T mean() const
    {
        return sum / N;
    }
 
    T variance() const
    {
        return S / (N - 1);
    }
 
    // operator<< to print the statistics to screen:
    // denoted friend just to be able to write this inside
    // the class definition and thus not to need to write
    // the template specification of accumulator...
    friend std::ostream& operator<<(std::ostream& out,
            const Accumulator& a)
    {
        out << " N         = " << a.N << std::endl
            << " sum       = " << a.sum << std::endl
            << " mean      = " << a.mean() << std::endl
            << " variance  = " << a.variance() << std::endl
			<< " min       = " << a.min << std::endl
			<< " max       = " << a.max << std::endl;
        return out;
    }
 
};



} // namespace LSLAM

#endif  //COMMON_HELPER_H_
