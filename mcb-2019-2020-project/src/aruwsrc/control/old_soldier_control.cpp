#include <aruwlib/Drivers.hpp>
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
using aruwlib::Drivers;
using aruwlib::control::CommandMapper;

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/
TurretSubsystem<Drivers> turret;

ChassisSubsystem<Drivers> chassis;

AgitatorSubsystem<Drivers> agitator(
    AgitatorSubsystem<Drivers>::PID_17MM_P,
    AgitatorSubsystem<Drivers>::PID_17MM_I,
    AgitatorSubsystem<Drivers>::PID_17MM_D,
    AgitatorSubsystem<Drivers>::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem<Drivers>::PID_17MM_MAX_OUT,
    AgitatorSubsystem<Drivers>::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem<Drivers>::AGITATOR_MOTOR_ID,
    AgitatorSubsystem<Drivers>::AGITATOR_MOTOR_CAN_BUS,
    AgitatorSubsystem<Drivers>::isAgitatorInverted);

HopperSubsystem<Drivers> hopperCover(
    aruwlib::gpio::Pwm::W,
    HopperSubsystem<Drivers>::OLD_SOLDIER_HOPPER_OPEN_PWM,
    HopperSubsystem<Drivers>::OLD_SOLDIER_HOPPER_CLOSE_PWM,
    HopperSubsystem<Drivers>::OLD_SOLDIER_PWM_RAMP_SPEED);

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand<Drivers> chassisDriveCommand(&chassis);

ChassisAutorotateCommand<Drivers> chassisAutorotateCommand(&chassis, &turret);

WiggleDriveCommand<Drivers> wiggleDriveCommand(&chassis, &turret);

TurretWorldRelativePositionCommand<Drivers> turretWorldRelativeCommand(&turret, &chassis);

AgitatorCalibrateCommand<Drivers> agitatorCalibrateCommand(&agitator);

ShootFastComprisedCommand<Drivers> agitatorShootFastCommand(&agitator);

OpenHopperCommand<Drivers> openHopperCommand(&hopperCover);

/* register subsystems here -------------------------------------------------*/
void registerOldSoldierSubsystems()
{
    Drivers::commandScheduler.registerSubsystem(&agitator);
    Drivers::commandScheduler.registerSubsystem(&chassis);
    Drivers::commandScheduler.registerSubsystem(&turret);
    Drivers::commandScheduler.registerSubsystem(&hopperCover);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultOldSoldierCommands()
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startOldSoldierCommands() { Drivers::commandScheduler.addCommand(&agitatorCalibrateCommand); }

/* register io mappings here ------------------------------------------------*/
void registerOldSoldierIoMappings()
{
    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<Drivers>::newKeyMap(Switch::RIGHT_SWITCH, SwitchState::UP),
        &agitatorShootFastCommand);

    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<Drivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::MID),
        &chassisAutorotateCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper<Drivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::DOWN),
        &chassisDriveCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper<Drivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::DOWN),
        &openHopperCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper<Drivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::UP),
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
