#include "balance_controller.hpp"

#include "tap/algorithms/cmsis_mat.hpp"
using tap::algorithms::CMSISMat;

namespace aruwsrc::control::balstd
{

BalstdChassisOutput BalanceController::runController(const BalstdChassisState& currState)
{
    // LQR
    this->vmState.data = {0, 0, vmX, 0, currState.pitch, currState.pitchVel};
    CMSISMat<6, 1> vmRef = CMSISMat<6, 1>({0, 0, controlOperatorInterface.getXVel(), 0, 0, 0});

    CMSISMat<2, 1> vmOuts = getLQRGains(currState.virtualLegState.L) * (vmRef - vmState);
    float hipTorque = vmOuts.data[0];
    float wheelTorque = vmOuts.data[1];

    // VMC

    return BalstdChassisOutput(
        controlOperatorInterface.getManualLegXForce(),
        controlOperatorInterface.getManualLegYForce(),
        controlOperatorInterface.getManualLegXForce(),
        controlOperatorInterface.getManualLegYForce(),
        controlOperatorInterface.getManualWheelTorque(),
        controlOperatorInterface.getManualWheelTorque());
}

CMSISMat<2, 6> BalanceController::getLQRGains(const float legLength) const
{
    // clang-format off
    return CMSISMat<2, 6>({
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
    });
    // clang-format on
}

}  // namespace aruwsrc::control::balstd
