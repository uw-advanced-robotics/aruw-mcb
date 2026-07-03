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

#if defined(TARGET_DRONE)
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/control/command_composition_helper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/repeat_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_helpers.hpp"

#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/buzzer/note_sequences.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/imu_not_calibrated_governor.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/drone/drone_ballistics_solver.hpp"
#include "aruwsrc/robot/drone/drone_drivers.hpp"
#include "aruwsrc/robot/drone/drone_imu_calibrate_command.hpp"
#include "aruwsrc/robot/drone/drone_transform_adapter.hpp"
#include "aruwsrc/robot/drone/drone_transformer.hpp"
#include "aruwsrc/robot/drone/drone_transformer_subsystem.hpp"
#include "aruwsrc/robot/drone/drone_turret_subsystem.hpp"
#include "aruwsrc/robot/drone/drone_turret_vector_command.hpp"

using namespace aruwsrc::drone;
using namespace aruwsrc::control;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::buzzer;
using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace tap::algorithms;

using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::turret::cv;
using namespace aruwsrc::control::turret::algorithms;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace drone_control
{
using Compose = CommandCompositionHelper;

/* define subsystems --------------------------------------------------------*/
BuzzerSubsystem buzzer(drivers());

tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_PITCH_MOTOR,
    true,
    "Pitch Turret",
    true,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_GM6020,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::encoder::CanEncoder yawEncoder(
    drivers(),
    YAW_ENCODER_ID,
    CAN_BUS_YAW_ENCODER,
    false,
    YAW_ENCODER_TO_TURRET_RATIO,
    YAW_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_YAW_MOTOR,
    false,
    "Yaw Turret",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M2006,
    0,
    &yawEncoder);

aruwsrc::control::turret::TurretMotor pitchTurretMotor(&pitchMotor, PITCH_MOTOR_CONFIG);
aruwsrc::control::turret::TurretMotor yawTurretMotor(&yawMotor, YAW_MOTOR_CONFIG);

aruwsrc::drone::DroneTurretSubsystem turret(
    drivers(),
    pitchTurretMotor,
    yawTurretMotor,
    &drivers()->turretImu);

DroneTransformer transformer(drivers(), turret, turret.getIMU());
DroneTransformerSubsystem transformSubsystem(*drivers(), transformer);
DroneTransformAdapter transformAdapter(transformer);

// transforms
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

tap::motor::DjiMotor leftFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::RIGHT_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    true,
    "Left flywheel");
tap::motor::DjiMotor rightFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::LEFT_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    false,
    "Right flywheel");
std::array<tap::motor::MotorInterface *, 2> wheels = {&leftFrictionWheel, &rightFrictionWheel};

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE,
    2>
    frictionWheels(
        drivers(),
        wheels,
        aruwsrc::control::launcher::WHEEL_CONFIG,
        aruwsrc::control::launcher::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
        aruwsrc::control::launcher::LAUNCHER_SPEED_CORRECTION_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::ChassisFrameTurretController<algorithms::Axis::YAW> chassisFrameYawFallbackController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

algorithms::ChassisFrameTurretController<algorithms::Axis::PITCH>
    chassisFramePitchFallbackController(turret.pitchMotor, chassis_rel::PITCH_PID_CONFIG);

TurretGravitationalForceOffset turretGravityCompensation(TURRET_GRAVITY_CONFIG);
TurretSpringForceOffset turretSpringCompensation(
    TURRET_SPRING_CONFIG,
    pitchMotor.isMotorInverted());

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidCv(
    world_rel_turret_imu::YAW_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidCv(world_rel_turret_imu::YAW_VEL_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidCv(
    world_rel_turret_imu::PITCH_POS_PID_AUTO_AIM_CONFIG);

WorldFrameTurretImuCascadePidTurretController<Axis::YAW> worldFrameYawTurretImuControllerCv(
    transformer.getWorldToTurret(),
    drivers()->turretImu,
    turret.yawMotor,
    worldFrameYawTurretImuPosPidCv,
    worldFrameYawTurretImuVelPidCv);

WorldFrameTurretImuCascadePidTurretController<Axis::PITCH> worldFramePitchTurretImuControllerCv(
    transformer.getWorldToTurret(),
    drivers()->turretImu,
    turret.pitchMotor,
    worldFramePitchTurretImuPosPidCv,
    worldFramePitchTurretImuVelPid,
    {&turretGravityCompensation, &turretSpringCompensation});

DroneBallisticsSolver ballisticsSolver(
    drivers()->visionCoprocessor,
    transformAdapter,
    turret,
    frictionWheels,
    aruwsrc::control::launcher::LAUNCHER_SPEED,
    0);

TurretCVCommand turretCVCommand(
    &drivers()->visionCoprocessor,
    &drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuControllerCv,
    &worldFramePitchTurretImuControllerCv,
    &ballisticsSolver,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

tap::algorithms::SmoothPid imuCalibrateYawPid(chassis_rel::YAW_PID_CONFIG);

tap::algorithms::SmoothPid imuCalibratePitchPid(chassis_rel::PITCH_PID_CONFIG);

NoteSequenceCommand imuNotCalibratedCommand(buzzer, BUMBLEBEE_NOTES, BUMBLEBEE_NOTE_LENGTH_MS);

NoteSequenceCommand imuCalibrateSuccessBuzzCommand(
    buzzer,
    IMU_CALIBRATE_SUCCESS_NOTES,
    IMU_CALIBRATE_SUCCESS_NOTE_LENGTH_MS);

NoteSequenceCommand imuCalibrateFailBuzzCommand(
    buzzer,
    IMU_CALIBRATE_FAIL_NOTES,
    IMU_CALIBRATE_FAIL_NOTE_LENGTH_MS);

ImuNotCalibratedGovernor imuNotCalibratedGovernor(drivers(), drivers()->mpu6500);

GovernorLimitedCommand<1> imuNotCalibratedCommandLimited(
    {&buzzer},
    imuNotCalibratedCommand,
    {&imuNotCalibratedGovernor});

DroneTurretVectorCommand turretUserVectorCommand(
    drivers()->controlOperatorInterface,
    turret,
    drivers()->turretImu,
    worldFrameYawTurretImuPosPid,
    worldFrameYawTurretImuVelPid,
    worldFramePitchTurretImuPosPid,
    worldFramePitchTurretImuVelPid,
    chassisFrameYawFallbackController,
    chassisFramePitchFallbackController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

DroneImuCalibrateCommand droneImuCalibrateCommand(
    *drivers(),
    turret,
    drivers()->turretImu,
    imuCalibrateYawPid,
    imuCalibratePitchPid,
    &imuCalibrateSuccessBuzzCommand,
    &imuCalibrateFailBuzzCommand);

// base rotate/unjam commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

ManualFireRateReselectionManager manualFireRateReselectionManager;
FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&frictionWheelsOnGovernor, &fireRateLimitGovernor});

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);
GovernorLimitedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor});

RepeatCommand rotateAndUnjamAgitatorRepeat(&rotateAndUnjamAgitatorWithHeatLimiting);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    30.0f,
    false,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

// Remote related mappings
Trigger leftSwitchMiddle =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID)
        .onTrue(&spinFrictionWheels)
        .onFalse(&stopFrictionWheels);

Trigger leftSwitchUp =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)
        .whileTrue(Compose::parallel<2>({&spinFrictionWheels, &rotateAndUnjamAgitatorRepeat}));

Trigger thumbwheelUp =
    TriggerHelpers::channelGreaterThan(drivers(), Remote::Channel::WHEEL, 0.95f, false)
        .onTrue(&droneImuCalibrateCommand);

Trigger rightMousePressed =
    TriggerHelpers::rightMouseButton(drivers()).whileTrue(&turretCVCommand);

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    buzzer.initialize();
    turret.initialize();
    agitator.initialize();
    frictionWheels.initialize();
    transformSubsystem.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&buzzer);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&transformSubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands(Drivers *)
{
    // buzzer.setDefaultCommand(&imuNotCalibratedCommandLimited);
    turret.setDefaultCommand(&turretUserVectorCommand);
    frictionWheels.setDefaultCommand(&stopFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&droneImuCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerDroneIoMappings(Drivers *) {}
}  // namespace drone_control

namespace aruwsrc::drone
{
void initSubsystemCommands(aruwsrc::drone::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &drone_control::remoteSafeDisconnectFunction);
    drone_control::initializeSubsystems();
    drone_control::registerDroneSubsystems(drivers);
    drone_control::setDefaultDroneCommands(drivers);
    drone_control::startDroneCommands(drivers);
    drone_control::registerDroneIoMappings(drivers);
    drivers->visionCoprocessor.attachTransformer(&drone_control::transformAdapter);
}
}  // namespace aruwsrc::drone

#endif
