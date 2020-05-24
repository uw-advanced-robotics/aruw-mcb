#include <cstdint>
#include "math_user_utils.hpp"

constexpr uint32_t FNV_PRIME = 16777619u;
constexpr uint32_t OFFSET_BASIS = 2166136261u;

aruwlib::algorithms::fnvhash_t fnvHash(const char* str)
{
    const size_t length = strlen(str) + 1;
    uint32_t hash = OFFSET_BASIS;
    for (size_t i = 0; i < length; ++i)
    {
        hash ^= *str++;
        hash *= FNV_PRIME;
    }
    return hash;
}

float aruwlib::algorithms::fastInvSqrt(float x)
{
    static_assert(sizeof(float) == 4, "fast inverse sqrt requires 32-bit float");
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = reinterpretCopy<float, int32_t>(y);
    i = 0x5f3759df - (i >> 1);
    y = reinterpretCopy<int32_t, float>(i);
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void aruwlib::algorithms::rotateVector(float* x, float* y, float radians) {
    float x_temp = *x;
    *x = (*x) * cosf(radians) - *y * sinf(radians);
    *y = x_temp * sinf(radians) + *y * cosf(radians);
}
