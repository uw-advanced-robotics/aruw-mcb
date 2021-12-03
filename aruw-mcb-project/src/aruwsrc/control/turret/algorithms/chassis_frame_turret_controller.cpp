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

#include "chassis_frame_turret_controller.hpp"

#include "aruwsrc/drivers.hpp"

#include "turret_gravity_compensation.hpp"

namespace aruwsrc::control::turret
{
void ChassisFrameTurretController::runPitchPidController(
    const uint32_t dt,
    const float desiredSetpoint,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMotorOutputMax,
    tap::algorithms::SmoothPid *pid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchSetpoint(desiredSetpoint);

    // position controller based on turret pitch gimbal
    float positionControllerError =
        turretSubsystem->getCurrentPitchValue().difference(turretSubsystem->getPitchSetpoint());

    float pidOutput =
        pid->runController(positionControllerError, turretSubsystem->getPitchVelocity(), dt);

    pidOutput += computeGravitationalForceOffset(
        turretCGX,
        turretCGZ,
        turretSubsystem->getPitchAngleFromCenter(),
        gravityCompensationMotorOutputMax);

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

void ChassisFrameTurretController::runPitchCascadePidController(
    const uint32_t dt,
    const float desiredSetpoint,
    const float turretCGX,
    const float turretCGZ,
    const float gravityCompensationMotorOutputMax,
    tap::algorithms::SmoothPid *positionPid,
    tap::algorithms::SmoothPid *velocityPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchSetpoint(desiredSetpoint);

    // position controller based on turret pitch gimbal
    float positionControllerError =
        turretSubsystem->getCurrentPitchValue().difference(turretSubsystem->getPitchSetpoint());

    float positionPidOutput = positionPid->runController(
        positionControllerError,
        turretSubsystem->getPitchVelocity(),
        dt);

    float velocityControllerError = positionPidOutput - turretSubsystem->getPitchVelocity();

    float velocityPidOutput = velocityPid->runControllerDerivateError(velocityControllerError, dt);

    velocityPidOutput += computeGravitationalForceOffset(
        turretCGX,
        turretCGZ,
        turretSubsystem->getPitchAngleFromCenter(),
        gravityCompensationMotorOutputMax);

    turretSubsystem->setPitchMotorOutput(velocityPidOutput);
}

void ChassisFrameTurretController::runYawPidController(
    const uint32_t dt,
    const float desiredSetpoint,
    tap::algorithms::SmoothPid *pid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    // limit the yaw min and max angles
    turretSubsystem->setYawSetpoint(desiredSetpoint);

    // position controller based on turret yaw gimbal
    float positionControllerError =
        turretSubsystem->getCurrentYawValue().difference(turretSubsystem->getYawSetpoint());

    float pidOutput =
        pid->runController(positionControllerError, turretSubsystem->getYawVelocity(), dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void ChassisFrameTurretController::runYawCascadePidController(
    const uint32_t dt,
    const float desiredSetpoint,
    tap::algorithms::SmoothPid *positionPid,
    tap::algorithms::SmoothPid *velocityPid,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
{
    // limit the yaw min and max angles
    turretSubsystem->setYawSetpoint(desiredSetpoint);

    // position controller based on turret yaw gimbal
    float positionControllerError =
        turretSubsystem->getCurrentYawValue().difference(turretSubsystem->getYawSetpoint());

    float positionPidOutput =
        positionPid->runController(positionControllerError, turretSubsystem->getYawVelocity(), dt);

    float velocityControllerError = positionPidOutput - turretSubsystem->getYawVelocity();

    float velocityPidOutput = velocityPid->runControllerDerivateError(velocityControllerError, dt);

    turretSubsystem->setYawMotorOutput(velocityPidOutput);
}
}  // namespace aruwsrc::control::turret
