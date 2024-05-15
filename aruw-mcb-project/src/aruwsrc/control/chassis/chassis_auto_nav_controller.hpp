#ifndef CHASSIS_AUTO_NAV_CONTROLLER_HPP_
#define CHASSIS_AUTO_NAV_CONTROLLER_HPP_

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "tap/algorithms/transforms/position.hpp"
#include "aruwsrc/algorithms/auto_nav_path.hpp"

namespace aruwsrc::chassis
{
class ChassisAutoNavController {
public:
    ChassisAutoNavController(aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
                             aruwsrc::algorithms::AutoNavPath& path) : 
        chassis(chassis),
        path(path) 
        {}
    void runController(const uint32_t dt, Position currentPos);

private:
    aruwsrc::chassis::HolonomicChassisSubsystem& chassis;
    aruwsrc::algorithms::AutoNavPath& path;
};
} // namespace aruwsrc::chassis

#endif // CHASSIS_AUTO_NAV_CONTROLLER_HPP_