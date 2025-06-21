#ifndef MANUAL_LEG_CONTROLLER_HPP_
#define MANUAL_LEG_CONTROLLER_HPP_

#include "chassis_controller_interface.hpp"

namespace aruwsrc::balstd::chassis::controllers
{
class ManualLegController : public BalstdChassisControllerInterface
{
public:
    ManualLegController(const BalstdControlOperatorInterface& controlOperatorInterface)
        : BalstdChassisControllerInterface(controlOperatorInterface)
    {
    }

    BalstdChassisOutput runController(const BalstdChassisState& state, float dt) override;
};
}  // namespace aruwsrc::balstd::chassis::controllers

#endif  // MANUAL_LEG_CONTROLLER_HPP_