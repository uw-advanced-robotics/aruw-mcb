#ifndef __USER_MATH_UTILS_HPP__
#define __USER_MATH_UTILS_HPP__

namespace aruwlib
{

namespace algorithms
{

#define PI (3.1415926f)
#define RADIANS_TO_DEGREES(val) (static_cast<float>(val) * 180.0f / PI)
#define DEGREES_TO_RADIANS(val) (static_cast<float>(val) * PI / 180.0f)

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
