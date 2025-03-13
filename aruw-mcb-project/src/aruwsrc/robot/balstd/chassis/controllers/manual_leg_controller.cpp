#include "manual_leg_controller.hpp"

namespace aruwsrc::control::balstd
{

BalstdChassisOutput ManualLegController::runController(const BalstdChassisState&) const
{
    return BalstdChassisOutput(
        controlOperatorInterface.getLegXForce(),
        controlOperatorInterface.getLegYForce(),
        controlOperatorInterface.getLegXForce(),
        controlOperatorInterface.getLegYForce(),
        controlOperatorInterface.getWheelTorque(),
        controlOperatorInterface.getWheelTorque());
}

}  // namespace aruwsrc::control::balstd
