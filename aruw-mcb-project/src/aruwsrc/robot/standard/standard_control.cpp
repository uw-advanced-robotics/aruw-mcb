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

#include "aruwsrc/util_macros.hpp"

#ifdef ALL_STANDARDS

#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"

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
#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/agitator/multi_shot_cv_command_mapping.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/wiggle_drive_command.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/ref_system_projectile_launched_governor.hpp"
#include "aruwsrc/control/hopper-cover/open_turret_mcb_hopper_cover_command.hpp"
#include "aruwsrc/control/hopper-cover/turret_mcb_hopper_cover_subsystem.hpp"
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
#include "aruwsrc/display/imu_calibrate_menu.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/standard/standard_drivers.hpp"
#include "aruwsrc/robot/standard/standard_turret_subsystem.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace aruwsrc::control::auto_aim;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms;
using namespace aruwsrc::standard;
using namespace tap::control;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control;
using namespace tap::communication::serial;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::algorithms::transforms;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace standard_control
{
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
aruwsrc::communication::serial::SentryRequestSubsystem sentryRequestSubsystem(drivers());

tap::motor::DjiMotor pitchMotor(drivers(), PITCH_MOTOR_ID, CAN_BUS_MOTORS, false, "Pitch Turret");

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
#if defined(TARGET_STANDARD_ELSA)
    true,
#elif defined(TARGET_STANDARD_SPIDER) || defined(TARGET_STANDARD_ORION) || \
    defined(TARGET_STANDARD_CYGNUS)
    false,
#else
#error "did not define standard!"
#endif
    "Yaw Turret");
StandardTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

aruwsrc::chassis::MecanumChassisSubsystem chassis(drivers(), &currentSensor);

OttoKFOdometry2DSubsystem odometrySubsystem(*drivers(), turret, chassis, modm::Vector2f(0, 0));

// transforms
StandardAndHeroTransformer transformer(odometrySubsystem, turret);
StandardAnderHeroTransformerSubsystem transformSubsystem(*drivers(), transformer);

StandardAndHeroTransformAdapter transformAdapter(transformer);

VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

ClientDisplaySubsystem clientDisplay(drivers());

TurretMCBHopperSubsystem hopperCover(drivers(), getTurretMCBCanComm());

OttoBallisticsSolver ballisticsSolver(
    drivers()->visionCoprocessor,
    transformAdapter,
    frictionWheels,
    14.0f,                            // defaultLaunchSpeed
    transformer.getWorldToChassis(),  // world to turretbase transform
    turret.yawMotor,
    0.0,  // turret offset from where the base rotates around
    0     // turretID

);
AutoAimLaunchTimer autoAimLaunchTimer(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &ballisticsSolver);

/* define commands ----------------------------------------------------------*/
aruwsrc::communication::serial::NoMotionStrategyCommand sendSentryNoMotionStrategy(
    sentryRequestSubsystem);
aruwsrc::communication::serial::GoToFriendlyBaseCommand sendSentryGoToFriendlyBase(
    sentryRequestSubsystem);
aruwsrc::communication::serial::GoToEnemyBaseCommand sendSentryGoToEnemyBase(
    sentryRequestSubsystem);
aruwsrc::communication::serial::GoToSupplierZoneCommand sendSentryGoToSupplierZone(
    sentryRequestSubsystem);
aruwsrc::communication::serial::GoToEnemySupplierZoneCommand sendSentryGoToEnemySupplierZone(
    sentryRequestSubsystem);
aruwsrc::communication::serial::GoToCenterPointCommand sendSentryGoToCenterPoint(
    sentryRequestSubsystem);
aruwsrc::communication::serial::HoldFireCommand sendSentryHoldFire(sentryRequestSubsystem);
aruwsrc::communication::serial::ToggleMovementCommand sendSentryToggleMovement(
    sentryRequestSubsystem);
aruwsrc::communication::serial::ToggleBeybladeCommand sendSentryToggleBeyblade(
    sentryRequestSubsystem);

aruwsrc::chassis::ChassisImuDriveCommand chassisImuDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor);

aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis);

aruwsrc::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor,
    aruwsrc::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

aruwsrc::chassis::WiggleDriveCommand wiggleCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface));
aruwsrc::chassis::BeybladeCommand beybladeCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface));

// Turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidCv(
    world_rel_turret_imu::PITCH_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    getTurretMCBCanComm(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPid,
    worldFramePitchTurretImuVelPid);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerCv(
    getTurretMCBCanComm(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPidCv,
    worldFramePitchTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPid,
    worldFrameYawTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidCv(
    world_rel_turret_imu::YAW_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidCv(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerCv(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPidCv,
    worldFrameYawTurretImuVelPidCv);

// turret commands
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

// base rotate/unjam commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

ManualFireRateReselectionManager manualFireRateReselectionManager;
FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &frictionWheelsOnGovernor, &fireRateLimitGovernor});

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);
GovernorLimitedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor});

// rotates agitator when aiming at target and within heat limit
CvOnTargetGovernor cvOnTargetGovernor(
    ((tap::Drivers *)(drivers())),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimer,
    CvOnTargetGovernorMode::ON_TARGET_AND_GATED);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor, &cvOnTargetGovernor});

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    15.0f,
    false,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

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

aruwsrc::communication::serial::SentryResponseHandler sentryResponseHandler(*drivers());

extern MultiShotCvCommandMapping leftMousePressedBNotPressed;
ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    drivers()->commandScheduler,
    drivers()->visionCoprocessor,
    clientDisplay,
    &hopperCover,
    frictionWheels,
    agitator,
    turret,
    imuCalibrateCommand,
    &leftMousePressedBNotPressed,
    &cvOnTargetGovernor,
    &beybladeCommand,
    &chassisAutorotateCommand,
    &chassisImuDriveCommand,
    sentryResponseHandler);

aruwsrc::control::buzzer::BuzzerSubsystem buzzer(drivers());

/* define command mappings --------------------------------------------------*/

// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&rotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&beybladeCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&turretCVCommand, &chassisDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rPressed(
    drivers(),
    RemoteMapState({Remote::Key::R}),
    true,
    &cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));

MultiShotCvCommandMapping leftMousePressedBNotPressed(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimiting,
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::B}),
    &manualFireRateReselectionManager,
    cvOnTargetGovernor,
    &rotateAgitator);

HoldRepeatCommandMapping leftMousePressedBPressed(
    drivers(),
    {&rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::B}),
    false);
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));
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
// The user can press q or e to manually rotate the chassis left or right.
// The user can press q and e simultaneously to enable wiggle driving. Wiggling is cancelled
// automatically once a different drive mode is chosen.
PressCommandMapping qEPressed(
    drivers(),
    {&wiggleCommand},
    RemoteMapState({Remote::Key::Q, Remote::Key::E}));
PressCommandMapping qNotEPressed(
    drivers(),
    {&chassisImuDriveCommand},
    RemoteMapState({Remote::Key::Q}, {Remote::Key::E}));
PressCommandMapping eNotQPressed(
    drivers(),
    {&chassisImuDriveCommand},
    RemoteMapState({Remote::Key::E}, {Remote::Key::Q}));
PressCommandMapping xPressed(
    drivers(),
    {&chassisAutorotateCommand},
    RemoteMapState({Remote::Key::X}));

CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>
    vPressed(
        drivers(),
        RemoteMapState({Remote::Key::V}),
        MultiShotCvCommandMapping::SINGLE,
        &leftMousePressedBNotPressed,
        &MultiShotCvCommandMapping::setShooterState);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&sentryRequestSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&hopperCover);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&buzzer);
    drivers->commandScheduler.registerSubsystem(&transformSubsystem);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    sentryRequestSubsystem.initialize();
    turret.initialize();
    chassis.initialize();
    odometrySubsystem.initialize();
    agitator.initialize();
    frictionWheels.initialize();
    hopperCover.initialize();
    clientDisplay.initialize();
    buzzer.initialize();
    transformSubsystem.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(Drivers *drivers)
{
    // drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachTransformer(&transformAdapter);

    drivers->refSerial.attachRobotToRobotMessageHandler(
        aruwsrc::communication::serial::SENTRY_RESPONSE_MESSAGE_ID,
        &sentryResponseHandler);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rPressed);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressedBNotPressed);
    drivers->commandMapper.addMap(&leftMousePressedBPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&zPressed);
    drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&bCtrlPressed);
    drivers->commandMapper.addMap(&qEPressed);
    drivers->commandMapper.addMap(&qNotEPressed);
    drivers->commandMapper.addMap(&eNotQPressed);
    drivers->commandMapper.addMap(&xPressed);
    // drivers->commandMapper.addMap(&cPressed);
    // drivers->commandMapper.addMap(&gPressedCtrlNotPressed);
    // drivers->commandMapper.addMap(&gCtrlPressed);
    drivers->commandMapper.addMap(&vPressed);
}
}  // namespace standard_control

namespace aruwsrc::standard
{
void initSubsystemCommands(aruwsrc::standard::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &standard_control::remoteSafeDisconnectFunction);
    standard_control::initializeSubsystems();
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultStandardCommands(drivers);
    standard_control::startStandardCommands(drivers);
    standard_control::registerStandardIoMappings(drivers);
}
}  // namespace aruwsrc::standard

#ifndef PLATFORM_HOSTED
imu::ImuCalibrateCommand *getImuCalibrateCommand()
{
    return &standard_control::imuCalibrateCommand;
}
#endif

#endif
