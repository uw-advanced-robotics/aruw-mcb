#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

#include "chassis_controller_interface.hpp"

namespace aruwsrc::control::balstd
{
class BalanceController : public BalstdChassisControllerInterface
{
public:
    BalanceController(const BalstdControlOperatorInterface& controlOperatorInterface)
        : BalstdChassisControllerInterface(controlOperatorInterface)
    {
    }

    BalstdChassisOutput runController(const BalstdChassisState& state) override;

private:
    tap::algorithms::CMSISMat<6, 1> vmState;
    float vmX{0};  // x position of the 2d robot model's wheel

    tap::algorithms::CMSISMat<2, 6> getLQRGains(const float legLength) const;
};
}  // namespace aruwsrc::control::balstd

#endif  // BALANCE_CONTROLLER_HPP_