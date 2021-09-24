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

#include "turret_world_relative_chassis_imu_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "../algorithms/turret_pid_control_algorithms.hpp"

using namespace tap::sensors;

namespace aruwsrc::control::turret
{
TurretWorldRelativeChassisImuCommand::TurretWorldRelativeChassisImuCommand(
    tap::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    const chassis::ChassisSubsystem *chassisSubsystem)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      chassisSubsystem(chassisSubsystem),
      yawSetpoint(TurretSubsystem::YAW_START_ANGLE, 0.0f, 360.0f),
      currValueImuYawGimbal(0.0f, 0.0f, 360.0f),
      chassisIMUInitialYaw(0.0f),
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

void TurretWorldRelativeChassisImuCommand::initialize()
{
    yawPid.reset();
    pitchPid.reset();

    chassisIMUInitialYaw = drivers->mpu6500.getYaw();

    yawSetpoint.setValue(turretSubsystem->getYawSetpoint());
}

bool TurretWorldRelativeChassisImuCommand::isReady() { return !isFinished(); }

bool TurretWorldRelativeChassisImuCommand::isFinished() const
{
    return !turretSubsystem->isOnline() || !drivers->mpu6500.isRunning();
}

void TurretWorldRelativeChassisImuCommand::execute()
{
    turretSubsystem->updateCurrentTurretAngles();

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    runYawPositionController(dt);

    float pitchUserInput =
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput();
    runSinglePidPitchChassisFrameController(dt, pitchUserInput, pitchPid, turretSubsystem);
}

void TurretWorldRelativeChassisImuCommand::runYawPositionController(uint32_t dt)
{
    yawSetpoint.shiftValue(
        USER_YAW_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput());

    // project target angle in world relative to chassis relative to limit the value
    turretSubsystem->setYawSetpoint(
        projectWorldRelativeYawToChassisFrame(yawSetpoint.getValue(), chassisIMUInitialYaw));

    if (turretSubsystem->yawLimited())
    {
        // project angle that is limited by the subsystem to world relative again to run the
        // controller. Otherwise use yawSetpoint directly.
        yawSetpoint.setValue(projectChassisRelativeYawToWorldRelative(
            turretSubsystem->getYawSetpoint(),
            chassisIMUInitialYaw));
    }

    currValueImuYawGimbal.setValue(projectChassisRelativeYawToWorldRelative(
        turretSubsystem->getCurrentYawValue().getValue(),
        chassisIMUInitialYaw));

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = currValueImuYawGimbal.difference(yawSetpoint);
    float pidOutput;

    pidOutput = yawPid.runController(
        positionControllerError,
        turretSubsystem->getYawVelocity() + drivers->mpu6500.getGz(),
        dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretWorldRelativeChassisImuCommand::end(bool)
{
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

float TurretWorldRelativeChassisImuCommand::projectChassisRelativeYawToWorldRelative(
    float yawAngle,
    float imuInitialAngle)
{
    return yawAngle + drivers->mpu6500.getYaw() - imuInitialAngle;
}

float TurretWorldRelativeChassisImuCommand::projectWorldRelativeYawToChassisFrame(
    float yawAngle,
    float imuInitialAngle)
{
    return yawAngle - drivers->mpu6500.getYaw() + imuInitialAngle;
}

}  // namespace aruwsrc::control::turret
