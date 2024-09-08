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

#if defined(TARGET_HERO_PERSEUS)

#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transform_adapter.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transformer.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transformer_subsystem.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/communication/serial/sentry_request_commands.hpp"
#include "aruwsrc/communication/serial/sentry_request_subsystem.hpp"
#include "aruwsrc/communication/serial/sentry_response_handler.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_sprint_command.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_subsystem.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_toggle_command.hpp"
#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/wiggle_drive_command.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/limit_switch_depressed_governor.hpp"
#include "aruwsrc/control/governor/yellow_carded_governor.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/hero/hero_turret_subsystem.hpp"

#include "aruwsrc/control/test_subsystem.hpp"
#include "aruwsrc/robot/engineer/tester_subsystem.hpp"

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::launcher;
using namespace tap::communication::serial;
using tap::control::CommandMapper;
using tap::control::RemoteMapState;
using namespace aruwsrc::algorithms::transforms;
using namespace aruwsrc::hero;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace hero_control
{
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

MecanumChassisSubsystem chassis(drivers(), &currentSensor, &drivers()->capacitorBank);

RefereeFeedbackFrictionWheelSubsystem<aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

ClientDisplaySubsystem clientDisplay(drivers());

aruwsrc::engineer::arm::TestSubsystem testSubsystem(drivers());
aruwsrc::TesterSubsystem testerSubsystem(drivers());

VelocityAgitatorSubsystem kickerAgitator(
    drivers(),
    constants::KICKER_PID_CONFIG,
    constants::KICKER_AGITATOR_CONFIG);

VelocityAgitatorSubsystem waterwheelAgitator(
    drivers(),
    constants::WATERWHEEL_PID_CONFIG,
    constants::WATERWHEEL_AGITATOR_CONFIG);

tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_PITCH_MOTOR,
    true,
    "Pitch Turret");
tap::motor::DjiMotor yawMotor(drivers(), YAW_MOTOR_ID, CAN_BUS_YAW_MOTOR, true, "Yaw Turret");
HeroTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

OttoKFOdometry2DSubsystem odometrySubsystem(*drivers(), turret, chassis, modm::Vector2f(0, 0));

// transforms
StandardAndHeroTransformer transformer(odometrySubsystem, turret);
StandardAnderHeroTransformerSubsystem transformSubsystem(*drivers(), transformer);

StandardAndHeroTransformAdapter transformAdapter(transformer);

OttoBallisticsSolver ballisticsSolver(
    drivers()->visionCoprocessor,
    odometrySubsystem,
    turret,
    frictionWheels,
    9.0f,  // defaultLaunchSpeed
    0      // turretID
);
AutoAimLaunchTimer autoAimLaunchTimer(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &ballisticsSolver);

aruwsrc::control::capbank::CapBankSubsystem capBankSubsystem(drivers(), drivers()->capacitorBank);

/* define commands ----------------------------------------------------------*/

ChassisImuDriveCommand chassisImuDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor);

ChassisDriveCommand chassisDriveCommand(drivers(), &drivers()->controlOperatorInterface, &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

BeybladeCommand beybladeCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface));

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    10.0f,
    false,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

// Turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPid,
    worldFrameYawTurretImuVelPid);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    getTurretMCBCanComm(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPid,
    worldFramePitchTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidCv(
    world_rel_turret_imu::YAW_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidCv(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidCv(
    world_rel_turret_imu::PITCH_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidCv(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerCv(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPidCv,
    worldFrameYawTurretImuVelPidCv);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerCv(
    getTurretMCBCanComm(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPidCv,
    worldFramePitchTurretImuVelPidCv);

// turret commands
// @todo: chassis MCB is mounted vertically so world frame chassis IMU controller cannot be used for
// this
user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawChassisImuController,
    &chassisFramePitchTurretController,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

cv::TurretCVCommand turretCVCommand(
    &drivers()->visionCoprocessor,
    &drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuControllerCv,
    &worldFramePitchTurretImuControllerCv,
    &ballisticsSolver,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &getTurretMCBCanComm(),
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &chassis);

// hero agitator commands

LimitSwitchDepressedGovernor limitSwitchDepressedGovernor(
    getTurretMCBCanComm(),
    LimitSwitchDepressedGovernor::LimitSwitchGovernorBehavior::READY_WHEN_DEPRESSED);
LimitSwitchDepressedGovernor limitSwitchNotDepressedGovernor(
    getTurretMCBCanComm(),
    LimitSwitchDepressedGovernor::LimitSwitchGovernorBehavior::READY_WHEN_RELEASED);

// rotates agitator if friction wheels are spinning fast
FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

namespace waterwheel
{
MoveIntegralCommand rotateWaterwheel(
    waterwheelAgitator,
    constants::WATERWHEEL_AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamWaterwheel(
    waterwheelAgitator,
    constants::WATERWHEEL_AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamWaterwheel(
    *drivers(),
    waterwheelAgitator,
    rotateWaterwheel,
    unjamWaterwheel);

GovernorLimitedCommand<2> feedWaterwheelWhenBallNotReady(
    {&waterwheelAgitator},
    rotateAndUnjamWaterwheel,
    {&limitSwitchNotDepressedGovernor, &frictionWheelsOnGovernor});
}  // namespace waterwheel

namespace kicker
{
MoveIntegralCommand loadKicker(kickerAgitator, constants::KICKER_LOAD_AGITATOR_ROTATE_CONFIG);

GovernorLimitedCommand<2> feedKickerWhenBallNotReady(
    {&kickerAgitator},
    loadKicker,
    {&limitSwitchNotDepressedGovernor, &frictionWheelsOnGovernor});

// rotates kickerAgitator when aiming at target and within heat limit
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM,
    constants::HEAT_LIMIT_BUFFER);
CvOnTargetGovernor cvOnTargetGovernor(
    drivers(),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimer,
    CvOnTargetGovernorMode::ON_TARGET_AND_GATED);
MoveIntegralCommand launchKicker(kickerAgitator, constants::KICKER_SHOOT_AGITATOR_ROTATE_CONFIG);
GovernorLimitedCommand<1> launchKickerNoHeatLimiting(
    {&kickerAgitator},
    launchKicker,
    {&frictionWheelsOnGovernor});
GovernorLimitedCommand<3> launchKickerHeatAndCVLimited(
    {&kickerAgitator},
    launchKicker,
    {&heatLimitGovernor, &frictionWheelsOnGovernor, &cvOnTargetGovernor});
}  // namespace kicker

// @todo remove
aruwsrc::communication::serial::SentryResponseHandler sentryResponseHandler(*drivers());

ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    drivers()->commandScheduler,
    drivers()->visionCoprocessor,
    clientDisplay,
    nullptr,
    frictionWheels,
    waterwheelAgitator,
    turret,
    {&beybladeCommand},
    imuCalibrateCommand,
    nullptr,
    &kicker::cvOnTargetGovernor,
    &beybladeCommand,
    &chassisAutorotateCommand,
    nullptr,
    &drivers()->capacitorBank);

aruwsrc::control::buzzer::BuzzerSubsystem buzzer(drivers());

// Cap Bank
aruwsrc::control::capbank::CapBankToggleCommand capBankToggleCommand(drivers(), capBankSubsystem);
aruwsrc::control::capbank::CapBankSprintCommand capBankSprintCommand(
    drivers(),
    capBankSubsystem,
    aruwsrc::can::capbank::SprintMode::SPRINT);
aruwsrc::control::capbank::CapBankSprintCommand capBankHalfSprintCommand(
    drivers(),
    capBankSubsystem,
    aruwsrc::can::capbank::SprintMode::HALF_SPRINT);

/* define command mappings --------------------------------------------------*/
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&kicker::launchKickerHeatAndCVLimited},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    false);
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&beybladeCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

MultiShotCvCommandMapping leftMousePressedBNotPressedVNotPressed(
    *drivers(),
    kicker::launchKickerHeatAndCVLimited,
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::B, Remote::Key::V}),
    std::nullopt,
    kicker::cvOnTargetGovernor);
HoldRepeatCommandMapping leftMousePressedBPressed(
    drivers(),
    {&kicker::launchKickerNoHeatLimiting},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::B}),
    false);
// Same thing as leftMousePressedBPressed; used for ease of access.
HoldRepeatCommandMapping leftMousePressedVPressed(
    drivers(),
    {&kicker::launchKickerNoHeatLimiting},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::V}),
    false);
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));
ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));
PressCommandMapping zPressed(drivers(), {&turretUTurnCommand}, RemoteMapState({Remote::Key::Z}));
// The "right switch down" portion is to avoid accidentally recalibrating in the middle of a match.
PressCommandMapping bNotCtrlPressedRightSwitchDown(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(
        Remote::SwitchState::UNKNOWN,
        Remote::SwitchState::DOWN,
        {Remote::Key::B},
        {Remote::Key::CTRL},
        false,
        false));
// The user can press b+ctrl when the remote right switch is in the down position to restart the
// client display command. This is necessary since we don't know when the robot is connected to the
// server and thus don't know when to start sending the initial HUD graphics.
PressCommandMapping bCtrlPressed(
    drivers(),
    {&clientDisplayCommand},
    RemoteMapState({Remote::Key::CTRL, Remote::Key::B}));

PressCommandMapping xPressed(
    drivers(),
    {&chassisAutorotateCommand},
    RemoteMapState({Remote::Key::X}));
CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rPressed(
    drivers(),
    RemoteMapState({Remote::Key::R}),
    true,
    &kicker::cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

// cap bank
PressCommandMapping cShiftPressed(
    drivers(),
    {&capBankToggleCommand},
    RemoteMapState({Remote::Key::SHIFT, Remote::Key::C}));
HoldCommandMapping shiftPressed(
    drivers(),
    {&capBankSprintCommand},
    RemoteMapState({Remote::Key::SHIFT}));

HoldCommandMapping ctrlPressed(
    drivers(),
    {&capBankHalfSprintCommand},
    RemoteMapState({Remote::Key::CTRL}));

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    frictionWheels.initialize();
    odometrySubsystem.initialize();
    clientDisplay.initialize();
    kickerAgitator.initialize();
    waterwheelAgitator.initialize();
    turret.initialize();
    buzzer.initialize();
    transformSubsystem.initialize();
    capBankSubsystem.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&kickerAgitator);
    drivers->commandScheduler.registerSubsystem(&waterwheelAgitator);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&buzzer);
    drivers->commandScheduler.registerSubsystem(&transformSubsystem);
    drivers->commandScheduler.registerSubsystem(&capBankSubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands()
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    waterwheelAgitator.setDefaultCommand(&waterwheel::feedWaterwheelWhenBallNotReady);
    kickerAgitator.setDefaultCommand(&kicker::feedKickerWhenBallNotReady);
}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachTransformer(&transformAdapter);
}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftMousePressedBNotPressedVNotPressed);
    drivers->commandMapper.addMap(&leftMousePressedBPressed);
    drivers->commandMapper.addMap(&leftMousePressedVPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&zPressed);
    drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&bCtrlPressed);
    drivers->commandMapper.addMap(&xPressed);
    drivers->commandMapper.addMap(&rPressed);
    drivers->commandMapper.addMap(&cShiftPressed);
    drivers->commandMapper.addMap(&shiftPressed);
    drivers->commandMapper.addMap(&ctrlPressed);
}
}  // namespace hero_control

namespace aruwsrc::hero
{
void initSubsystemCommands(aruwsrc::hero::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &hero_control::remoteSafeDisconnectFunction);
    hero_control::initializeSubsystems();
    hero_control::registerHeroSubsystems(drivers);
    hero_control::setDefaultHeroCommands();
    hero_control::startHeroCommands(drivers);
    hero_control::registerHeroIoMappings(drivers);
}
}  // namespace aruwsrc::hero

#ifndef PLATFORM_HOSTED
imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &hero_control::imuCalibrateCommand; }
#endif

#endif
