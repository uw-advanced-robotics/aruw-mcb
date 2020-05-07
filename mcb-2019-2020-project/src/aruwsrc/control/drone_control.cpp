#include "robot-type/robot_type.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"
#include "turret/drone_turret_subsystem.hpp"
#include "turret/drone_turret_init_command.hpp"
#include "turret/drone_turret_manual_command.hpp"
#include "turret/drone_turret_world_relative_position_command.hpp"
#if defined(TARGET_DRONE)

using namespace aruwsrc::turret;

namespace aruwsrc
{

namespace control
{

/* define subsystems --------------------------------------------------------*/
DroneTurretSubsystem turret;
/* define commands ----------------------------------------------------------*/

DroneTurretInitCommand turretInitCommand(&turret);

DroneTurretManualCommand turretManualCommand(&turret);

DroneTurretWorldRelativePositionCommand turretPositionCommand(&turret);

/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems()
{
    CommandScheduler::getMainScheduler().registerSubsystem(&turret);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands()
{
    turret.setDefaultCommand(&turretManualCommand);
    //turret.setDefaultCommand(&turretPositionCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands()
{
    CommandScheduler::getMainScheduler().addCommand(&turretInitCommand);
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
