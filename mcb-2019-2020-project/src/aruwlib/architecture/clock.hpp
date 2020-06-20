#ifndef CLOCK_HPP_
#define CLOCK_HPP_

#include <stdint.h>

#ifndef ENV_SIMULATOR
#include <modm/platform.hpp>
#endif

namespace aruwlib
{
namespace arch
{
namespace clock
{
inline uint32_t getTimeMilliseconds()
{
#ifdef ENV_SIMULATOR
    return 0;
#else
    return modm::Clock::now().getTime();
#endif
}

inline uint32_t getTimeMicroseconds()
{
#ifdef ENV_SIMULATOR
    return 0;
#else
    return DWT->CYCCNT / static_cast<uint32_t>(modm::clock::fcpu_MHz);
#endif
}
}  // namespace clock

}  // namespace arch

}  // namespace aruwlib

#endif  // CLOCK_HPP_
