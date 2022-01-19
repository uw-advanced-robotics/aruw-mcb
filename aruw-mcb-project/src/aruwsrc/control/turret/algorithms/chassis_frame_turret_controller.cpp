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

#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

#include "turret_gravity_compensation.hpp"

using namespace tap::control::turret;

namespace aruwsrc::control::turret
{
ChassisFrameYawTurretController::ChassisFrameYawTurretController(
    TurretSubsystem *turretSubsystem,
    float kp,
    float ki,
    float kd,
    float maxICumulative,
    float maxOutput,
    float tQDerivativeKalman,
    float tRDerivativeKalman,
    float tQProportionalKalman,
    float tRProportionalKalman,
    float errDeadzone)
    : TurretYawControllerInterface(turretSubsystem),
      pid(kp,
          ki,
          kd,
          maxICumulative,
          maxOutput,
          tQDerivativeKalman,
          tRDerivativeKalman,
          tQProportionalKalman,
          tRProportionalKalman,
          errDeadzone)
{
}

void ChassisFrameYawTurretController::initialize() { pid.reset(); }

void ChassisFrameYawTurretController::runController(const uint32_t dt, const float desiredSetpoint)
{
    // limit the yaw min and max angles
    turretSubsystem->setYawSetpoint(desiredSetpoint);

    // position controller based on turret yaw gimbal
    float positionControllerError =
        turretSubsystem->getCurrentYawValue().difference(turretSubsystem->getYawSetpoint());

    float pidOutput =
        pid.runController(positionControllerError, turretSubsystem->getYawVelocity(), dt);

    turretSubsystem->setYawMotorOutput(pidOutput);
}

float ChassisFrameYawTurretController::getSetpoint() const
{
    return turretSubsystem->getYawSetpoint();
}

bool ChassisFrameYawTurretController::isFinished() const { return !turretSubsystem->isOnline(); }

ChassisFramePitchTurretController::ChassisFramePitchTurretController(
    TurretSubsystem *turretSubsystem,
    float kp,
    float ki,
    float kd,
    float maxICumulative,
    float maxOutput,
    float tQDerivativeKalman,
    float tRDerivativeKalman,
    float tQProportionalKalman,
    float tRProportionalKalman,
    float errDeadzone)
    : TurretPitchControllerInterface(turretSubsystem),
      pid(kp,
          ki,
          kd,
          maxICumulative,
          maxOutput,
          tQDerivativeKalman,
          tRDerivativeKalman,
          tQProportionalKalman,
          tRProportionalKalman,
          errDeadzone)
{
}

void ChassisFramePitchTurretController::initialize() { pid.reset(); }

void ChassisFramePitchTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchSetpoint(desiredSetpoint);

    // position controller based on turret pitch gimbal
    float positionControllerError =
        turretSubsystem->getCurrentPitchValue().difference(turretSubsystem->getPitchSetpoint());

    float pidOutput =
        pid.runController(positionControllerError, turretSubsystem->getPitchVelocity(), dt);

    pidOutput += computeGravitationalForceOffset(
        TurretSubsystem::TURRET_CG_X,
        TurretSubsystem::TURRET_CG_Z,
        turretSubsystem->getPitchAngleFromCenter(),
        TurretSubsystem::GRAVITY_COMPENSATION_SCALAR);

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

float ChassisFramePitchTurretController::getSetpoint() const
{
    return turretSubsystem->getPitchSetpoint();
}

bool ChassisFramePitchTurretController::isFinished() const { return !turretSubsystem->isOnline(); }

}  // namespace aruwsrc::control::turret
