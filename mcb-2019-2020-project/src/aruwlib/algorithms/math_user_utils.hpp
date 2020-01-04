#ifndef __USER_MATH_UTILS_HPP__
#define __USER_MATH_UTILS_HPP__

namespace aruwlib
{

namespace algorithms
{

const float PI = 3.14159265359f;

template< typename T >
T limitVal(T val, T min, T max)
{
    if (min >= max)
    {
        return val;
    }
    if (val < min)
    {
        return min;
    }
    else if (val > max)
    {
        return max;
    }
    else
    {
        return val;
    }
}

}  // namespace algorithms

}  // namespace aruwlib

#endif
