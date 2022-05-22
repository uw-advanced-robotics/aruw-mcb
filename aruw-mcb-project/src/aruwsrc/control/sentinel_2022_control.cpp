/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_SENTINEL_2022)

#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "agitator/agitator_subsystem.hpp"
#include "agitator/constants/agitator_constants.hpp"
#include "agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/communication/serial/sentinel_request_handler.hpp"
#include "aruwsrc/communication/serial/sentinel_request_message_types.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "governor/friction_wheels_on_governor.hpp"
#include "governor/heat_limit_governor.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "sentinel/drive/sentinel_auto_drive_comprised_command.hpp"
#include "sentinel/drive/sentinel_drive_manual_command.hpp"
#include "sentinel/drive/sentinel_drive_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/constants/turret_constants.hpp"
#include "turret/cv/sentinel_turret_cv_command.hpp"
#include "turret/sentinel_turret_subsystem.hpp"
#include "turret/user/turret_user_control_command.hpp"

using namespace tap::control::governor;
using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::sentinel::drive;
using namespace tap::gpio;
using namespace aruwsrc::control;
using namespace tap::control;
using namespace tap::communication::serial;
using namespace tap::motor;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::algorithms::odometry;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace sentinel_control
{
static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::InputPin::B;
static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::InputPin::C;

aruwsrc::communication::serial::SentinelRequestHandler sentinelRequestHandler(drivers());

// forward declare before sentinel turret to be used in turret CV command
extern OttoVelocityOdometry2DSubsystem odometrySubsystem;

class SentinelTurret
{
public:
    struct Config
    {
        aruwsrc::agitator::VelocityAgitatorSubsystemConfig agitatorConfig;
        TurretMotorConfig pitchMotorConfig;
        TurretMotorConfig yawMotorConfig;
        tap::can::CanBus turretCanBus;
        bool pitchMotorInverted;
        uint8_t turretID;
        RefSerialData::Rx::MechanismID turretBarrelMechanismId;
        tap::algorithms::SmoothPidConfig pitchPidConfig;
        tap::algorithms::SmoothPidConfig yawPidConfig;
    };

    SentinelTurret(aruwsrc::Drivers &drivers, const Config &config)
        : agitator(&drivers, constants::AGITATOR_PID_CONFIG, config.agitatorConfig),
          frictionWheels(
              &drivers,
              LEFT_MOTOR_ID,
              RIGHT_MOTOR_ID,
              config.turretCanBus,
              config.turretBarrelMechanismId),
          pitchMotor(
              &drivers,
              aruwsrc::control::turret::PITCH_MOTOR_ID,
              config.turretCanBus,
              config.pitchMotorInverted,
              "Pitch Turret"),
          yawMotor(
              &drivers,
              aruwsrc::control::turret::YAW_MOTOR_ID,
              config.turretCanBus,
              true,
              "Yaw Turret"),
          turretSubsystem(
              &drivers,
              &pitchMotor,
              &yawMotor,
              config.pitchMotorConfig,
              config.yawMotorConfig),
          rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG),
          unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG),
          rotateAndUnjamAgitator(drivers, agitator, rotateAgitator, unjamAgitator),
          frictionWheelsOnGovernor(frictionWheels),
          heatLimitGovernor(drivers, config.turretBarrelMechanismId, constants::HEAT_LIMIT_BUFFER),
          rotateAndUnjamAgitatorWithHeatLimiting(
              {&agitator},
              rotateAndUnjamAgitator,
              {&heatLimitGovernor, &frictionWheelsOnGovernor}),
          spinFrictionWheels(
              &drivers,
              &frictionWheels,
              30.0f,
              true,
              config.turretBarrelMechanismId),
          stopFrictionWheels(&drivers, &frictionWheels, 0.0f, true, config.turretBarrelMechanismId),
          chassisFramePitchTurretController(&turretSubsystem.pitchMotor, config.pitchPidConfig),
          chassisFrameYawTurretController(&turretSubsystem.yawMotor, config.yawPidConfig),
          turretManual(
              &drivers,
              &turretSubsystem,
              &chassisFrameYawTurretController,
              &chassisFramePitchTurretController,
              USER_YAW_INPUT_SCALAR,
              USER_PITCH_INPUT_SCALAR,
              config.turretID),
          turretCVCommand(
              &drivers,
              &turretSubsystem,
              &chassisFrameYawTurretController,
              &chassisFramePitchTurretController,
              agitator,
              &rotateAndUnjamAgitatorWithHeatLimiting,
              odometrySubsystem,
              frictionWheels,
              29.5f,
              config.turretID)
    {
    }

    // subsystems
    VelocityAgitatorSubsystem agitator;
    RefereeFeedbackFrictionWheelSubsystem<LAUNCH_SPEED_AVERAGING_DEQUE_SIZE> frictionWheels;
    DjiMotor pitchMotor;
    DjiMotor yawMotor;
    SentinelTurretSubsystem turretSubsystem;

    // unjam commands
    MoveIntegralCommand rotateAgitator;
    UnjamIntegralCommand unjamAgitator;
    MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator;

    // rotates agitator if friction wheels are spinning fast
    FrictionWheelsOnGovernor frictionWheelsOnGovernor;

    // rotates agitator with heat limiting applied
    HeatLimitGovernor heatLimitGovernor;
    GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatLimiting;

    // friction wheel commands
    FrictionWheelSpinRefLimitedCommand spinFrictionWheels;
    FrictionWheelSpinRefLimitedCommand stopFrictionWheels;

    // turret controllers
    algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController;
    algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController;

    // turret commands
    user::TurretUserControlCommand turretManual;
    cv::SentinelTurretCVCommand turretCVCommand;
};

SentinelTurret turretZero(
    *drivers(),
    {
        .agitatorConfig = aruwsrc::control::agitator::constants::turret0::AGITATOR_CONFIG,
        .pitchMotorConfig = aruwsrc::control::turret::turret0::PITCH_MOTOR_CONFIG,
        .yawMotorConfig = aruwsrc::control::turret::turret0::YAW_MOTOR_CONFIG,
        .turretCanBus = aruwsrc::control::turret::turret0::CAN_BUS_MOTORS,
        .pitchMotorInverted = false,
        .turretID = 0,
        .turretBarrelMechanismId = RefSerialData::Rx::MechanismID::TURRET_17MM_2,
        .pitchPidConfig = aruwsrc::control::turret::chassis_rel::turret0::PITCH_PID_CONFIG,
        .yawPidConfig = aruwsrc::control::turret::chassis_rel::turret0::YAW_PID_CONFIG,
    });

SentinelTurret turretOne(
    *drivers(),
    {
        .agitatorConfig = aruwsrc::control::agitator::constants::turret1::AGITATOR_CONFIG,
        .pitchMotorConfig = aruwsrc::control::turret::turret1::PITCH_MOTOR_CONFIG,
        .yawMotorConfig = aruwsrc::control::turret::turret1::YAW_MOTOR_CONFIG,
        .turretCanBus = aruwsrc::control::turret::turret1::CAN_BUS_MOTORS,
        .pitchMotorInverted = true,
        .turretID = 1,
        .turretBarrelMechanismId = RefSerialData::Rx::MechanismID::TURRET_17MM_1,
        .pitchPidConfig = aruwsrc::control::turret::chassis_rel::turret1::PITCH_PID_CONFIG,
        .yawPidConfig = aruwsrc::control::turret::chassis_rel::turret1::YAW_PID_CONFIG,
    });

/* define subsystems --------------------------------------------------------*/
SentinelDriveSubsystem sentinelDrive(drivers(), LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

OttoVelocityOdometry2DSubsystem odometrySubsystem(
    drivers(),
    &turretOne.turretSubsystem.yawMotor,
    &sentinelDrive);

/* define commands ----------------------------------------------------------*/
// Two identical drive commands since you can't map an identical command to two different mappings
SentinelDriveManualCommand sentinelDriveManual1(drivers(), &sentinelDrive);
SentinelDriveManualCommand sentinelDriveManual2(drivers(), &sentinelDrive);

SentinelAutoDriveComprisedCommand sentinelAutoDrive(drivers(), &sentinelDrive);

void selectNewRobotMessageHandler()
{
    turretZero.turretCVCommand.requestNewTarget();
    turretOne.turretCVCommand.requestNewTarget();
}
void targetNewQuadrantMessageHandler()
{
    turretZero.turretCVCommand.changeScanningQuadrant();
    turretOne.turretCVCommand.changeScanningQuadrant();
}

/* define command mappings --------------------------------------------------*/

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&turretZero.stopFrictionWheels, &turretOne.stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&turretZero.rotateAndUnjamAgitatorWithHeatLimiting,
     &turretOne.rotateAndUnjamAgitatorWithHeatLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
HoldRepeatCommandMapping leftSwitchDown(
    drivers(),
    {&sentinelDriveManual1, &turretZero.turretManual, &turretOne.turretManual},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
    true);
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&sentinelDriveManual2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    sentinelDrive.initialize();
    turretZero.agitator.initialize();
    turretZero.frictionWheels.initialize();
    turretZero.turretSubsystem.initialize();
    turretOne.agitator.initialize();
    turretOne.frictionWheels.initialize();
    turretOne.turretSubsystem.initialize();
    odometrySubsystem.initialize();
}

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&sentinelDrive);
    drivers->commandScheduler.registerSubsystem(&turretZero.agitator);
    drivers->commandScheduler.registerSubsystem(&turretZero.frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretZero.turretSubsystem);
    drivers->commandScheduler.registerSubsystem(&turretOne.agitator);
    drivers->commandScheduler.registerSubsystem(&turretOne.frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretOne.turretSubsystem);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turretZero.turretSubsystem, 1);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turretOne.turretSubsystem, 0);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands(aruwsrc::Drivers *)
{
    sentinelDrive.setDefaultCommand(&sentinelAutoDrive);
    turretZero.frictionWheels.setDefaultCommand(&turretZero.spinFrictionWheels);
    turretOne.frictionWheels.setDefaultCommand(&turretOne.spinFrictionWheels);
    turretZero.turretSubsystem.setDefaultCommand(&turretZero.turretCVCommand);
    turretOne.turretSubsystem.setDefaultCommand(&turretOne.turretCVCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentinelCommands(aruwsrc::Drivers *drivers)
{
    sentinelRequestHandler.attachSelectNewRobotMessageHandler(selectNewRobotMessageHandler);
    sentinelRequestHandler.attachTargetNewQuadrantMessageHandler(targetNewQuadrantMessageHandler);
    drivers->refSerial.attachRobotToRobotMessageHandler(
        aruwsrc::communication::serial::SENTINEL_REQUEST_ROBOT_ID,
        &sentinelRequestHandler);
}

/* register io mappings here ------------------------------------------------*/
void registerSentinelIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchMid);
}
}  // namespace sentinel_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &sentinel_control::remoteSafeDisconnectFunction);
    sentinel_control::initializeSubsystems();
    sentinel_control::registerSentinelSubsystems(drivers);
    sentinel_control::setDefaultSentinelCommands(drivers);
    sentinel_control::startSentinelCommands(drivers);
    sentinel_control::registerSentinelIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif
