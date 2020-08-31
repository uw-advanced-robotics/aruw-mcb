#include <aruwlib/HALDrivers.hpp>
#include <aruwlib/control/command_mapper.hpp>

#include "agitator/agitator_calibrate_command.hpp"
#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "hopper-cover/hopper_subsystem.hpp"
#include "hopper-cover/open_hopper_command.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

#if defined(TARGET_OLD_SOLDIER)

using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::turret;
using namespace aruwlib::remote;
using aruwlib::HALDrivers;
using aruwlib::control::CommandMapper;

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/
TurretSubsystem<HALDrivers> turret;

ChassisSubsystem<HALDrivers> chassis;

AgitatorSubsystem<HALDrivers> agitator(
    AgitatorSubsystem<HALDrivers>::PID_17MM_P,
    AgitatorSubsystem<HALDrivers>::PID_17MM_I,
    AgitatorSubsystem<HALDrivers>::PID_17MM_D,
    AgitatorSubsystem<HALDrivers>::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem<HALDrivers>::PID_17MM_MAX_OUT,
    AgitatorSubsystem<HALDrivers>::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem<HALDrivers>::AGITATOR_MOTOR_ID,
    AgitatorSubsystem<HALDrivers>::AGITATOR_MOTOR_CAN_BUS,
    AgitatorSubsystem<HALDrivers>::isAgitatorInverted);

HopperSubsystem<HALDrivers> hopperCover(
    aruwlib::gpio::Pwm::W,
    HopperSubsystem<HALDrivers>::OLD_SOLDIER_HOPPER_OPEN_PWM,
    HopperSubsystem<HALDrivers>::OLD_SOLDIER_HOPPER_CLOSE_PWM,
    HopperSubsystem<HALDrivers>::OLD_SOLDIER_PWM_RAMP_SPEED);

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand<HALDrivers> chassisDriveCommand(&chassis);

ChassisAutorotateCommand<HALDrivers> chassisAutorotateCommand(&chassis, &turret);

WiggleDriveCommand<HALDrivers> wiggleDriveCommand(&chassis, &turret);

TurretWorldRelativePositionCommand<HALDrivers> turretWorldRelativeCommand(&turret, &chassis);

AgitatorCalibrateCommand<HALDrivers> agitatorCalibrateCommand(&agitator);

ShootFastComprisedCommand<HALDrivers> agitatorShootFastCommand(&agitator);

OpenHopperCommand<HALDrivers> openHopperCommand(&hopperCover);

/* register subsystems here -------------------------------------------------*/
void registerOldSoldierSubsystems()
{
    HALDrivers::commandScheduler.registerSubsystem(&agitator);
    HALDrivers::commandScheduler.registerSubsystem(&chassis);
    HALDrivers::commandScheduler.registerSubsystem(&turret);
    HALDrivers::commandScheduler.registerSubsystem(&hopperCover);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultOldSoldierCommands()
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startOldSoldierCommands()
{
    HALDrivers::commandScheduler.addCommand(&agitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerOldSoldierIoMappings()
{
    HALDrivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::RIGHT_SWITCH, SwitchState::UP),
        &agitatorShootFastCommand);

    HALDrivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::MID),
        &chassisAutorotateCommand);

    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::DOWN),
        &chassisDriveCommand);

    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::DOWN),
        &openHopperCommand);

    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::UP),
        &wiggleDriveCommand);
}

void initSubsystemCommands()
{
    registerOldSoldierSubsystems();
    setDefaultOldSoldierCommands();
    startOldSoldierCommands();
    registerOldSoldierIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
