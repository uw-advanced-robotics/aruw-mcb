#include "manual_leg_controller.hpp"

namespace aruwsrc::control::balstd
{

BalstdChassisOutput ManualLegController::runController(const BalstdChassisState&) const
{
    return BalstdChassisOutput(
        controlOperatorInterface.getManualLegXForce(),
        controlOperatorInterface.getManualLegYForce(),
        controlOperatorInterface.getManualLegXForce(),
        controlOperatorInterface.getManualLegYForce(),
        controlOperatorInterface.getManualWheelTorque(),
        controlOperatorInterface.getManualWheelTorque());
}

}  // namespace aruwsrc::control::balstd
