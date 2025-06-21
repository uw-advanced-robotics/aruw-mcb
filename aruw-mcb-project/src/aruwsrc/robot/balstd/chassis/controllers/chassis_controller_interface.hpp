#ifndef CHASSIS_CONTROLLER_INTERFACE_HPP_
#define CHASSIS_CONTROLLER_INTERFACE_HPP_

#include "aruwsrc/robot/balstd/balstd_control_operator_interface.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_chassis_output.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_chassis_state.hpp"

namespace aruwsrc::balstd::chassis::controllers
{
class BalstdChassisControllerInterface
{
public:
    BalstdChassisControllerInterface(const BalstdControlOperatorInterface& controlOperatorInterface)
        : controlOperatorInterface(controlOperatorInterface)
    {
    }

    virtual void initialize(const BalstdChassisState&) {}

    virtual BalstdChassisOutput runController(const BalstdChassisState& state, float dt);

protected:
    const BalstdControlOperatorInterface& controlOperatorInterface;
};
}  // namespace aruwsrc::balstd::chassis::controllers

#endif  // CHASSIS_CONTROLLER_INTERFACE_HPP_