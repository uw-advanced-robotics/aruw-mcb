#include <aruwlib/control/controller_mapper.hpp>
#include "robot_type.hpp"
#include "turret/drone_turret_subsystem.hpp"
#include "turret/drone_turret_init_command.hpp"
#include "turret/drone_turret_manual_command.hpp"
#include "turret/drone_turret_world_relative_position_command.hpp"
#include "drone/pwm_friction_wheel_subsystem.hpp"
#include "drone/init_pwm_friction_wheel_command.hpp"
#include "drone/manual_pwm_friction_wheel_command.hpp"

#if defined(TARGET_DRONE)

using namespace aruwsrc::turret;
using namespace aruwsrc::drone;

namespace aruwsrc
{

namespace control
{

/* define subsystems --------------------------------------------------------*/
DroneTurretSubsystem turret;
PWMFrictionWheelSubsystem frictionWheels;

/* define commands ----------------------------------------------------------*/

DroneTurretInitCommand turretInitCommand(&turret);

DroneTurretManualCommand turretManualCommand(&turret);

DroneTurretWorldRelativePositionCommand turretPositionCommand(&turret);
drone::InitPWMFrictionWheelCommand initFrictionWheelCommand(&frictionWheels);
drone::PWMFrictionWheelManualCommand manualFrictionWheelCommand(&frictionWheels);

/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems()
{
    CommandScheduler::getMainScheduler().registerSubsystem(&turret);
    CommandScheduler::getMainScheduler().registerSubsystem(&frictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands()
{
    turret.setDefaultCommand(&turretPositionCommand);
    frictionWheels.setDefaultCommand(&manualFrictionWheelCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands()
{
    CommandScheduler::getMainScheduler().addCommand(&turretInitCommand);
    CommandScheduler::getMainScheduler().addCommand(&initFrictionWheelCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerDroneIoMappings()
{
    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID),
            &turretManualCommand);
    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
            &turretPositionCommand);
    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
            &turretInitCommand);
}

void initSubsystemCommands()
{
    registerDroneSubsystems();
    setDefaultDroneCommands();
    startDroneCommands();
    registerDroneIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
