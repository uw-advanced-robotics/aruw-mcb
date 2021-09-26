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

#include "turret_world_relative_turret_imu_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace tap::sensors;
using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
/**
 * Transforms the specified `angleToTransform`, a yaw angle from the chassis frame to the world
 * frame.
 *
 * @note It is expected that the user wraps the value returned to be between [0, 360)
 *      (or whatever range they require).
 *
 * @param[in] turretChassisFrameCurrAngle The current chassis relative (gimbal encoder) yaw angle.
 * @param[in] turretWorldFrameCurrAngle The current world relative (turret IMU) yaw angle, captured
 *      at the same time as `turretChassisFrameCurrAngle`.
 * @param[in] angleToTransform The angle to transform and return.
 */
static inline float transformChassisFrameYawToWorldFrame(
    const float turretChassisFrameCurrAngle,
    const float turretWorldFrameCurrAngle,
    const float angleToTransform)
{
    return turretWorldFrameCurrAngle + (angleToTransform - turretChassisFrameCurrAngle);
}

/**
 * Transforms the specified `angleToTransform`, a yaw angle, from the world frame to the chassis
 * frame.
 *
 * @note It is expected that the user wraps the value returned to be between [0, 360)
 *      (or whatever range they require).
 *
 * @param[in] turretChassisFrameCurrAngle The current chassis relative (gimbal encoder) yaw angle.
 * @param[in] turretWorldFrameCurrAngle The current world relative (turret IMU) yaw angle, captured
 *      at the same time as `turretChassisFrameCurrAngle`.
 * @param[in] angleToTransform The angle to transform and return.
 */
static inline float transformWorldFrameYawToChassisFrame(
    const float turretChassisFrameCurrAngle,
    const float turretWorldFrameCurrAngle,
    const float angleToTransform)
{
    return turretChassisFrameCurrAngle + (angleToTransform - turretWorldFrameCurrAngle);
}

/**
 * Transforms the specified `angleToTransform`, a pitch angle, from the chassis frame to the world
 * frame.
 *
 * @param[in] turretBaseFramePitchAngle The pitch angle of the turret base relative to the world.
 * @param[in] angleToTransform The angle to transform and return.
 */
static inline float transformChassisFramePitchToWorldFrame(
    const float turretBaseFramePitchAngle,
    const float angleToTransform)
{
    return angleToTransform + turretBaseFramePitchAngle;
}

/**
 * Transforms the specified `angleToTransform`, a pitch angle, from the world frame to the chassis
 * frame.
 *
 * @param[in] turretBaseFramePitchAngle The pitch angle of the turret base relative to the world.
 * @param[in] angleToTransform The angle to transform and return.
 */
static inline float transformWorldFramePitchToChassisFrame(
    const float turretBaseFramePitchAngle,
    const float pitchAngle)
{
    return pitchAngle - turretBaseFramePitchAngle;
}

/**
 * @return the pitch angle relative to the world, based on `turretBaseFramePitchAngle` and the
 *      current chassis frame pitch angle (`TurretSubsystem::getCurrentPitchValue`).
 */
static inline float getPitchWorldRelativeAngle(
    const float turretBaseFramePitchAngle,
    const TurretSubsystem *turret)
{
    return transformChassisFramePitchToWorldFrame(
        turretBaseFramePitchAngle,
        turret->getCurrentPitchValue().getValue());
}

/**
 * @return the pitch velocity relative to the world, based on `turretBaseFramePitchVelocity` and the
 *      current chassis frame pitch velocity (`TurretSubsystem::getPitchVelocity`).
 */
static inline float getPitchWorldRelativeVelocity(
    const float turretBaseFramePitchVelocity,
    const TurretSubsystem *turret)
{
    return transformChassisFramePitchToWorldFrame(
        turretBaseFramePitchVelocity,
        turret->getPitchVelocity());
}

TurretWorldRelativeTurretImuCommand::TurretWorldRelativeTurretImuCommand(
    tap::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    const chassis::ChassisSubsystem *chassis)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      chassisSubsystem(chassis),
      worldFrameYawSetpoint(TurretSubsystem::YAW_START_ANGLE, 0.0f, 360.0f),
      worldFramePitchSetpoint(TurretSubsystem::PITCH_START_ANGLE, 0.0f, 360.0f),
      yawPid(
          YAW_P,
          YAW_I,
          YAW_D,
          YAW_MAX_ERROR_SUM,
          YAW_MAX_OUTPUT,
          YAW_Q_DERIVATIVE_KALMAN,
          YAW_R_DERIVATIVE_KALMAN,
          YAW_Q_PROPORTIONAL_KALMAN,
          YAW_R_PROPORTIONAL_KALMAN),
      pitchPid(
          PITCH_P,
          PITCH_I,
          PITCH_D,
          PITCH_MAX_ERROR_SUM,
          PITCH_MAX_OUTPUT,
          PITCH_Q_DERIVATIVE_KALMAN,
          PITCH_R_DERIVATIVE_KALMAN,
          PITCH_Q_PROPORTIONAL_KALMAN,
          PITCH_R_PROPORTIONAL_KALMAN)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretWorldRelativeTurretImuCommand::isReady() { return !isFinished(); }

bool TurretWorldRelativeTurretImuCommand::isFinished() const
{
    return !turretSubsystem->isOnline() || !drivers->imuRxHandler.isConnected();
}

void TurretWorldRelativeTurretImuCommand::initialize()
{
    // Reset PID controllers
    yawPid.reset();
    pitchPid.reset();

    // Capture initial target angle in chassis frame and transform to world frame.
    worldFrameYawSetpoint.setValue(transformChassisFrameYawToWorldFrame(
        turretSubsystem->getCurrentYawValue().getValue(),
        drivers->imuRxHandler.getYaw(),
        turretSubsystem->getYawSetpoint()));

    worldFramePitchSetpoint.setValue(transformChassisFramePitchToWorldFrame(
        drivers->imuRxHandler.getPitch(),
        turretSubsystem->getPitchSetpoint()));

    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretWorldRelativeTurretImuCommand::end(bool)
{
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

void TurretWorldRelativeTurretImuCommand::execute()
{
    // Update light to indicate IMU message received and turret controller running.
    imuMessageReceivedLEDBlinkCounter = (imuMessageReceivedLEDBlinkCounter + 1) % 100;
    drivers->leds.set(tap::gpio::Leds::Green, imuMessageReceivedLEDBlinkCounter > 50);

    turretSubsystem->updateCurrentTurretAngles();

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    runYawPositionController(dt);
    runPitchPositionController(dt);
}

void TurretWorldRelativeTurretImuCommand::runYawPositionController(uint32_t dt)
{
    worldFrameYawSetpoint.shiftValue(
        USER_YAW_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput());

    const float chassisFrameYaw = turretSubsystem->getCurrentYawValue().getValue();
    const float worldFrameYawAngle = drivers->imuRxHandler.getYaw();
    const float worldFrameYawVelocity = drivers->imuRxHandler.getGz();

    // transform target angle from turret imu relative to chassis relative
    // to keep turret/command setpoints synchronized
    turretSubsystem->setYawSetpoint(transformWorldFrameYawToChassisFrame(
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameYawSetpoint.getValue()));

    if (turretSubsystem->yawLimited())
    {
        // TODO this is broken
        // transform angle that is limited by subsystem to world relative again to run the
        // controller
        worldFrameYawSetpoint.setValue(transformChassisFrameYawToWorldFrame(
            chassisFrameYaw,
            worldFrameYawAngle,
            turretSubsystem->getCurrentYawValue().getValue()));
    }

    // position controller based on imu and yaw gimbal angle,
    // precisly, - (worldFrameYawSetpoint - yawActual), or more obvious, yawActual -
    // worldFrameYawSetpoint
    float positionControllerError = -worldFrameYawSetpoint.difference(worldFrameYawAngle);
    float pidOutput = yawPid.runController(positionControllerError, worldFrameYawVelocity, dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretWorldRelativeTurretImuCommand::runPitchPositionController(uint32_t dt)
{
    worldFramePitchSetpoint.shiftValue(
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput());

    const float turretBasePitchAngle = drivers->imuRxHandler.getPitch();
    const float turretBasePitchVelocity = drivers->imuRxHandler.getGx();

    // Project user desired setpoint that is in world relative to chassis relative
    // to limit the value
    turretSubsystem->setPitchSetpoint(transformWorldFramePitchToChassisFrame(
        turretBasePitchAngle,
        worldFramePitchSetpoint.getValue()));

    // Project angle limited by the TurretSubsystem back to world relative
    // to use the value
    worldFramePitchSetpoint.setValue(transformChassisFramePitchToWorldFrame(
        turretBasePitchAngle,
        turretSubsystem->getPitchSetpoint()));

    const float pitchWorldRelativeAngle =
        getPitchWorldRelativeAngle(turretBasePitchAngle, turretSubsystem);
    const float pitchWorldRelativeVelocity =
        getPitchWorldRelativeVelocity(turretBasePitchVelocity, turretSubsystem);

    // Compute error between pitch target and reference angle
    float positionControllerError = -worldFramePitchSetpoint.difference(pitchWorldRelativeAngle);

    float pidOutput =
        pitchPid.runController(positionControllerError, pitchWorldRelativeVelocity, dt);

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

}  // namespace aruwsrc::control::turret
