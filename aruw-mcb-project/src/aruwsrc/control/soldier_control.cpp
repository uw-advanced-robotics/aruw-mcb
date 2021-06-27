/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */
#if defined(TARGET_SOLDIER)

#include "aruwlib/control/command_mapper.hpp"
#include "aruwlib/control/hold_command_mapping.hpp"
#include "aruwlib/control/hold_repeat_command_mapping.hpp"
#include "aruwlib/control/press_command_mapping.hpp"
#include "aruwlib/control/setpoint/commands/calibrate_command.hpp"
#include "aruwlib/control/toggle_command_mapping.hpp"
#include "aruwlib/drivers_singleton.hpp"

#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "chassis/beyblade_command.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "client-display/client_display_command.hpp"
#include "client-display/client_display_subsystem.hpp"
#include "constants/robot_constants.hpp"
#include "hopper-cover/hopper_commands.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

#ifdef PLATFORM_HOSTED
#include "aruwlib/communication/can/can.hpp"
#include "aruwlib/motor/motorsim/motor_sim.hpp"
#include "aruwlib/motor/motorsim/sim_handler.hpp"
#endif

using namespace aruwlib::control::setpoint;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::chassis;
using namespace aruwsrc::launcher;
using namespace aruwlib::control;
using namespace aruwsrc::display;
using namespace aruwsrc::control;
using aruwlib::DoNotUse_getDrivers;
using aruwlib::Remote;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwlib::driversFunc drivers = aruwlib::DoNotUse_getDrivers;

namespace soldier_control
{
/* define subsystems --------------------------------------------------------*/
TurretSubsystem turret(
    drivers(),
    constants::turret::TURRET_START_ANGLE,
    constants::turret::TURRET_YAW_MIN_ANGLE,
    constants::turret::TURRET_YAW_MAX_ANGLE,
    constants::turret::TURRET_PITCH_MIN_ANGLE,
    constants::turret::TURRET_PITCH_MAX_ANGLE,
    constants::turret::YAW_START_ENCODER_POSITION,
    constants::turret::PITCH_START_ENCODER_POSITION,
    constants::turret::FEED_FORWARD_KP,
    constants::turret::FEED_FORWARD_MAX_OUTPUT,
    constants::can::TURRET_CAN_BUS,
    constants::motor::PITCH_MOTOR_ID,
    constants::motor::YAW_MOTOR_ID,
    false);

ChassisSubsystem chassis(
    drivers(),
    constants::chassis::CHASSIS_GEARBOX_RATIO,
    constants::chassis::WIDTH_BETWEEN_WHEELS_X,
    constants::chassis::WIDTH_BETWEEN_WHEELS_Y,
    constants::chassis::WHEEL_RADIUS,
    constants::chassis::MAX_WHEEL_SPEED_SINGLE_MOTOR,
    constants::chassis::GIMBAL_X_OFFSET,
    constants::chassis::GIMBAL_Y_OFFSET,
    constants::chassis::CHASSIS_REVOLVE_PID_MAX_P,
    constants::chassis::CHASSIS_REVOLVE_PID_MAX_D,
    constants::chassis::CHASSIS_REVOLVE_PID_KD,
    constants::chassis::CHASSIS_REVOLVE_PID_MAX_OUTPUT,
    constants::chassis::CHASSIS_REVOLVE_PID_MIN_ERROR_ROTATION_D,
    constants::chassis::MIN_ROTATION_THRESHOLD,
    constants::chassis::VELOCITY_PID_KP,
    constants::chassis::VELOCITY_PID_KI,
    constants::chassis::VELOCITY_PID_KD,
    constants::chassis::VELOCITY_PID_MAX_ERROR_SUM,
    constants::chassis::VELOCITY_PID_MAX_OUTPUT,
    constants::chassis::MAX_ENERGY_BUFFER,
    constants::chassis::ENERGY_BUFFER_LIMIT_THRESHOLD,
    constants::chassis::ENERGY_BUFFER_CRIT_THRESHOLD,
    constants::chassis::POWER_CONSUMPTION_THRESHOLD,
    constants::chassis::CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING,
    constants::can::CHASSIS_CAN_BUS,
    constants::motor::RIGHT_FRONT_MOTOR_ID,
    constants::motor::LEFT_FRONT_MOTOR_ID,
    constants::motor::LEFT_BACK_MOTOR_ID,
    constants::motor::RIGHT_BACK_MOTOR_ID,
    constants::gpio::CURRENT_SENSOR_PIN);

AgitatorSubsystem agitator(
    drivers(),
    constants::agitator::PID_17MM_P,
    constants::agitator::PID_17MM_I,
    constants::agitator::PID_17MM_D,
    constants::agitator::PID_17MM_MAX_ERR_SUM,
    constants::agitator::PID_17MM_MAX_OUT,
    constants::agitator::AGITATOR_GEARBOX_RATIO,
    constants::motor::AGITATOR_MOTOR_ID,
    constants::can::AGITATOR_MOTOR_CAN_BUS,
    constants::agitator::AGITATOR_INVERTED,
    true,
    constants::agitator::AGITATOR_JAMMING_DISTANCE,
    constants::agitator::AGITATOR_JAMMING_TIME);

// TODO: validate and tune these constexpr parameters for hopper lid motor
// also find out what kind of motor hopper lid uses lol
AgitatorSubsystem hopperCover(
    drivers(),
    constants::agitator::PID_17MM_P,
    constants::agitator::PID_17MM_I,
    constants::agitator::PID_17MM_D,
    constants::agitator::PID_17MM_MAX_ERR_SUM,
    constants::agitator::PID_17MM_MAX_OUT,
    constants::agitator::AGITATOR_GEARBOX_RATIO,
    constants::motor::HOPPER_COVER_MOTOR_ID,
    constants::can::HOPPER_COVER_MOTOR_CAN_BUS,
    constants::agitator::IS_HOPPER_COVER_INVERTED,
    true,
    constants::agitator::HOPPER_COVER_JAMMING_DISTANCE,
    constants::agitator::HOPPER_COVER_JAMMING_TIME);

FrictionWheelSubsystem frictionWheels(
    drivers(),
    constants::launcher::LAUNCHER_PID_P,
    constants::launcher::LAUNCHER_PID_I,
    constants::launcher::LAUNCHER_PID_D,
    constants::launcher::LAUNCHER_PID_MAX_ERROR_SUM,
    constants::launcher::LAUNCHER_PID_MAX_OUTPUT,
    constants::motor::LAUNCHER_LEFT_MOTOR_ID,
    constants::motor::LAUNCHER_RIGHT_MOTOR_ID,
    constants::can::LAUNCHER_CAN_BUS);

ClientDisplaySubsystem clientDisplay(drivers());

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(drivers(), &chassis, &turret);

BeybladeCommand beybladeCommand(drivers(), &chassis, &turret);

TurretWorldRelativePositionCommand turretWorldRelativeCommand(
    drivers(),
    &turret,
    &chassis,
    constants::turret::TURRET_START_ANGLE,
    constants::turret::YAW_WR_P,
    constants::turret::YAW_WR_I,
    constants::turret::YAW_WR_TURRET_IMU_D,
    constants::turret::YAW_WR_CHASSIS_IMU_D,
    constants::turret::YAW_WR_MAX_ERROR_SUM,
    constants::turret::YAW_WR_MAX_OUTPUT,
    constants::turret::YAW_WR_Q_DERIVATIVE_KALMAN,
    constants::turret::YAW_WR_R_DERIVATIVE_KALMAN,
    constants::turret::YAW_WR_Q_PROPORTIONAL_KALMAN,
    constants::turret::YAW_WR_R_PROPORTIONAL_KALMAN,
    constants::turret::PITCH_WR_P,
    constants::turret::PITCH_WR_I,
    constants::turret::PITCH_WR_D,
    constants::turret::PITCH_WR_MAX_ERROR_SUM,
    constants::turret::PITCH_WR_MAX_OUTPUT,
    constants::turret::PITCH_WR_Q_DERIVATIVE_KALMAN,
    constants::turret::PITCH_WR_R_DERIVATIVE_KALMAN,
    constants::turret::PITCH_WR_Q_PROPORTIONAL_KALMAN,
    constants::turret::PITCH_WR_R_PROPORTIONAL_KALMAN,
    constants::turret::USER_YAW_INPUT_SCALAR,
    constants::turret::USER_PITCH_INPUT_SCALAR,
    constants::turret::PITCH_GRAVITY_COMPENSATION_KP,
    true);

TurretCVCommand turretCVCommand(
    drivers(),
    &turret,
    constants::turret::TURRET_START_ANGLE,
    constants::turret::YAW_CV_P,
    constants::turret::YAW_CV_I,
    constants::turret::YAW_CV_D,
    constants::turret::YAW_CV_MAX_ERROR_SUM,
    constants::turret::YAW_CV_MAX_OUTPUT,
    constants::turret::YAW_CV_Q_DERIVATIVE_KALMAN,
    constants::turret::YAW_CV_R_DERIVATIVE_KALMAN,
    constants::turret::YAW_CV_Q_PROPORTIONAL_KALMAN,
    constants::turret::YAW_CV_R_PROPORTIONAL_KALMAN,
    constants::turret::PITCH_CV_P,
    constants::turret::PITCH_CV_I,
    constants::turret::PITCH_CV_D,
    constants::turret::PITCH_CV_MAX_ERROR_SUM,
    constants::turret::PITCH_CV_MAX_OUTPUT,
    constants::turret::PITCH_CV_Q_DERIVATIVE_KALMAN,
    constants::turret::PITCH_CV_R_DERIVATIVE_KALMAN,
    constants::turret::PITCH_CV_Q_PROPORTIONAL_KALMAN,
    constants::turret::PITCH_CV_R_PROPORTIONAL_KALMAN);

CalibrateCommand agitatorCalibrateCommand(&agitator);

ShootFastComprisedCommand17MM agitatorShootFastLimited(drivers(), &agitator);

ShootFastComprisedCommand17MM agitatorShootFastNotLimited(drivers(), &agitator, false);

SoldierOpenHopperCommand openHopperCommand(&hopperCover);

SoldierCloseHopperCommand closeHopperCommand(&hopperCover);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(drivers(), &frictionWheels);

FrictionWheelRotateCommand stopFrictionWheels(&frictionWheels, 0);

ClientDisplayCommand clientDisplayCommand(
    drivers(),
    &clientDisplay,
    &beybladeCommand,
    &chassisAutorotateCommand,
    nullptr,
    &chassisDriveCommand);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&openHopperCommand, &stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&agitatorShootFastLimited},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&beybladeCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Keyboard/Mouse related mappings
ToggleCommandMapping rToggled(drivers(), {&openHopperCommand}, RemoteMapState({Remote::Key::R}));
ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));
HoldRepeatCommandMapping leftMousePressedShiftNotPressed(
    drivers(),
    {&agitatorShootFastLimited},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::SHIFT}));
HoldRepeatCommandMapping leftMousePressedShiftPressed(
    drivers(),
    {&agitatorShootFastNotLimited},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::SHIFT}));
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

/* register subsystems here -------------------------------------------------*/
void registerSoldierSubsystems(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&hopperCover);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);

#ifdef PLATFORM_HOSTED
    // Register the motor sims for the Agitator subsystem
    // TODO: Create simulator for correct motor
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::agitator::AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
        aruwsrc::agitator::AgitatorSubsystem::AGITATOR_MOTOR_ID);

    // Register the motor sims for the Chassis subsystem
    aruwlib::motorsim::MotorSim::MotorType CHASSIS_MOTOR_TYPE =
        aruwlib::motorsim::MotorSim::MotorType::M3508;
    aruwlib::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        aruwsrc::chassis::ChassisSubsystem::LEFT_FRONT_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        aruwsrc::chassis::ChassisSubsystem::LEFT_BACK_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        aruwsrc::chassis::ChassisSubsystem::RIGHT_FRONT_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        aruwsrc::chassis::ChassisSubsystem::RIGHT_BACK_MOTOR_ID);

    // Register the motor sims for the turret subsystem
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::GM6020,
        aruwsrc::control::turret::TurretSubsystem::CAN_BUS_MOTORS,
        aruwsrc::control::turret::TurretSubsystem::PITCH_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::GM6020,
        aruwsrc::control::turret::TurretSubsystem::CAN_BUS_MOTORS,
        aruwsrc::control::turret::TurretSubsystem::YAW_MOTOR_ID);

    // Register the motor sims for the Hopper Cover (There aren't any)
    // Register the motor sims for the Friction Wheels
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::launcher::FrictionWheelSubsystem::CAN_BUS_MOTORS,
        aruwsrc::launcher::FrictionWheelSubsystem::LEFT_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::launcher::FrictionWheelSubsystem::CAN_BUS_MOTORS,
        aruwsrc::launcher::FrictionWheelSubsystem::RIGHT_MOTOR_ID);
#endif  // PLATFORM_HOSTED
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turret.initialize();
    chassis.initialize();
    agitator.initialize();
    frictionWheels.initialize();
    hopperCover.initialize();
    clientDisplay.initialize();
    drivers()->xavierSerial.attachChassis(&chassis);
    drivers()->xavierSerial.attachTurret(&turret);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSoldierCommands(aruwlib::Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
    hopperCover.setDefaultCommand(&closeHopperCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSoldierCommands(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&agitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSoldierIoMappings(aruwlib::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rToggled);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressedShiftNotPressed);
    drivers->commandMapper.addMap(&leftMousePressedShiftPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
}
}  // namespace soldier_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwlib::Drivers *drivers)
{
    soldier_control::initializeSubsystems();
    soldier_control::registerSoldierSubsystems(drivers);
    soldier_control::setDefaultSoldierCommands(drivers);
    soldier_control::startSoldierCommands(drivers);
    soldier_control::registerSoldierIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif
