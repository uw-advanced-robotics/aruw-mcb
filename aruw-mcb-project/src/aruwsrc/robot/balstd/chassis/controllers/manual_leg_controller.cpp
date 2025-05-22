#include "manual_leg_controller.hpp"

namespace aruwsrc::control::balstd
{
float mlx, mly;
BalstdChassisOutput ManualLegController::runController(const BalstdChassisState&, float)
{
    mlx = controlOperatorInterface.getManualLegXForce();
    mly = controlOperatorInterface.getManualLegYForce();
    return BalstdChassisOutput(
        controlOperatorInterface.getManualLegXForce(),
        controlOperatorInterface.getManualLegYForce(),
        controlOperatorInterface.getManualLegXForce(),
        controlOperatorInterface.getManualLegYForce(),
        controlOperatorInterface.getManualWheelTorque(),
        controlOperatorInterface.getManualWheelTorque());
}

}  // namespace aruwsrc::control::balstd
