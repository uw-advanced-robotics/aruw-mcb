
#ifndef BALSTD_OP_STATES_HPP_
#define BALSTD_OP_STATES_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::control::balstd
{

typedef struct BalstdChassisState
{
    tap::algorithms::transforms::Transform& worldToChassis;
    BalstdLegState leftLegState, rightLegState;
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_OP_STATES_HPP_