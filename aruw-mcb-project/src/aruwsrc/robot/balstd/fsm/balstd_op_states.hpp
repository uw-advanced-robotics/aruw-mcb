
#ifndef BALSTD_OP_STATES_HPP_
#define BALSTD_OP_STATES_HPP_

namespace aruwsrc::balstd
{

enum class BalstdOpState
{
    UNKNOWN,
    FALLEN_FORWARD,
    FALLEN_BACKWARD,
    GETTING_UP_FORWARD,
    GETTING_UP_BACKWARD,
    BALANCING,
    NUM_STATES
};

}  // namespace aruwsrc::balstd

#endif  // BALSTD_OP_STATES_HPP_