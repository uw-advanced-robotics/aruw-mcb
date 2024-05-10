#ifndef CHASSIS_AUTO_NAV_CONTROLLER_HPP_
#define CHASSIS_AUTO_NAV_CONTROLLER_HPP_

#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/algorithms/transforms/position.hpp"
#include "aruwsrc/algorithms/auto_nav_path.hpp"

namespace aruwsrc::chassis
{
class ChassisAutoNavController {
public:
    ChassisAutoNavController(tap::control::chassis::ChassisSubsystemInterface& chassis,
                             aruwsrc::algorithms::AutoNavPath& path) : 
        chassis(chassis),
        path(path) 
        {}
    void runController(const uint32_t dt, Position currentPos);

private:
    tap::control::chassis::ChassisSubsystemInterface& chassis;
    aruwsrc::algorithms::AutoNavPath& path;
    
};
} // namespace aruwsrc::chassis

#endif // CHASSIS_AUTO_NAV_CONTROLLER_HPP_