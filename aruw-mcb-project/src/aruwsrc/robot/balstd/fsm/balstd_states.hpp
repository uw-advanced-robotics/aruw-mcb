
#ifndef BALSTD_STATES_HPP_
#define BALSTD_STATES_HPP_

namespace aruwsrc::control::balstd
{

enum class BalstdState
{
    UNKNOWN,
    FALLEN_FORWARD,
    FALLEN_BACKWARD,
    GETTING_UP_FORWARD,
    GETTING_UP_BACKWARD,
    BALANCING,
    NUM_STATES
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_STATES_HPP_