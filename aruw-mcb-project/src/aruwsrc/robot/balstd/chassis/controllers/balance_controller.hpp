#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

#include "tap/algorithms/smooth_pid.hpp"

#include "chassis_controller_interface.hpp"

namespace aruwsrc::control::balstd
{
class BalanceController : public BalstdChassisControllerInterface
{
public:
    BalanceController(
        const BalstdControlOperatorInterface& controlOperatorInterface,
        const tap::algorithms::SmoothPidConfig heightControllerConfig,
        const tap::algorithms::SmoothPidConfig splitControllerConfig,
        const tap::algorithms::SmoothPidConfig yawControllerConfig)
        : BalstdChassisControllerInterface(controlOperatorInterface),
          heightController(heightControllerConfig),
          splitController(splitControllerConfig),
          yawController(yawControllerConfig)
    {
    }

    BalstdChassisOutput runController(const BalstdChassisState& state) override;

private:
    tap::algorithms::SmoothPid heightController, splitController, yawController;

    tap::algorithms::CMSISMat<6, 1> vmState;
    float vmX{0};  // x position of the 2d robot model's wheel

    tap::algorithms::CMSISMat<2, 6> getLQRGains(const float legLength) const;
};
}  // namespace aruwsrc::control::balstd

#endif  // BALANCE_CONTROLLER_HPP_