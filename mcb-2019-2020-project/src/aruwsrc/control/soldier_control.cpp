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
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

#if defined(TARGET_SOLDIER)

using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::launcher;
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
    HopperSubsystem<HALDrivers>::SOLDIER_HOPPER_OPEN_PWM,
    HopperSubsystem<HALDrivers>::SOLDIER_HOPPER_CLOSE_PWM,
    HopperSubsystem<HALDrivers>::SOLDIER_PWM_RAMP_SPEED);

FrictionWheelSubsystem<HALDrivers> frictionWheels;

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand<HALDrivers> chassisDriveCommand(&chassis);

ChassisAutorotateCommand<HALDrivers> chassisAutorotateCommand(&chassis, &turret);

WiggleDriveCommand<HALDrivers> wiggleDriveCommand(&chassis, &turret);

TurretWorldRelativePositionCommand<HALDrivers> turretWorldRelativeCommand(&turret, &chassis);

AgitatorCalibrateCommand<HALDrivers> agitatorCalibrateCommand(&agitator);

ShootFastComprisedCommand<HALDrivers> agitatorShootFastCommand(&agitator);

OpenHopperCommand<HALDrivers> openHopperCommand(&hopperCover);

FrictionWheelRotateCommand<HALDrivers> spinFrictionWheels(
    &frictionWheels,
    FrictionWheelRotateCommand<HALDrivers>::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand<HALDrivers> stopFrictionWheels(&frictionWheels, 0);

/// \todo add cv turret

/* register subsystems here -------------------------------------------------*/
void registerSoldierSubsystems()
{
    HALDrivers::commandScheduler.registerSubsystem(&agitator);
    HALDrivers::commandScheduler.registerSubsystem(&chassis);
    HALDrivers::commandScheduler.registerSubsystem(&turret);
    HALDrivers::commandScheduler.registerSubsystem(&hopperCover);
    HALDrivers::commandScheduler.registerSubsystem(&frictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSoldierCommands()
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSoldierCommands() { HALDrivers::commandScheduler.addCommand(&agitatorCalibrateCommand); }

/* register io mappings here ------------------------------------------------*/
void registerSoldierIoMappings()
{
    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(SwitchState::DOWN, SwitchState::DOWN),
        &stopFrictionWheels);

    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(SwitchState::DOWN, SwitchState::DOWN),
        &openHopperCommand);

    HALDrivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::MID),
        &chassisAutorotateCommand);

    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::UP),
        &wiggleDriveCommand);

    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::DOWN),
        &chassisDriveCommand);

    HALDrivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::RIGHT_SWITCH, SwitchState::UP),
        &agitatorShootFastCommand);

    /// \todo left switch up is cv command
}

void initSubsystemCommands()
{
    registerSoldierSubsystems();
    setDefaultSoldierCommands();
    startSoldierCommands();
    registerSoldierIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
