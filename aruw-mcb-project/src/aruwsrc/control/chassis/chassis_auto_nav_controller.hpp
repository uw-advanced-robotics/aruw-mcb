#ifndef CHASSIS_AUTO_NAV_CONTROLLER_HPP_
#define CHASSIS_AUTO_NAV_CONTROLLER_HPP_

#include "tap/drivers.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/algorithms/transforms/position.hpp"
#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"


namespace aruwsrc::chassis
{
class ChassisAutoNavController {
public:
    ChassisAutoNavController(tap::Drivers& drivers,
                             tap::control::chassis::ChassisSubsystemInterface& chassis,
                             const aruwsrc::control::turret::TurretMotor& yawMotor,
                             const aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
                             const tap::algorithms::odometry::Odometry2DInterface& odometryInterface,
                             aruwsrc::algorithms::AutoNavPath& path
                             ) : 
        drivers(drivers),
        chassis(chassis),
        yawMotor(yawMotor),
        visionCoprocessor(visionCoprocessor),
        odometryInterface(odometryInterface),
        path(path)
        {
            // // TODO: sucks that we have to pull the address out of the reference bc everything else uses
            // // pointers
            // addSubsystemRequirement(&chassis);
        }

    void runController(const uint32_t dt, Position currentPos);

private:
    aruwsrc::chassis::HolonomicChassisSubsystem& chassis;
    aruwsrc::algorithms::AutoNavPath& path;
    tap::Drivers& drivers;
    const aruwsrc::control::turret::TurretMotor& yawMotor;
    const aruwsrc::serial::VisionCoprocessor& visionCoprocessor;
    const tap::algorithms::odometry::Odometry2DInterface& odometryInterface;
};
} // namespace aruwsrc::chassis

#endif // CHASSIS_AUTO_NAV_CONTROLLER_HPP_