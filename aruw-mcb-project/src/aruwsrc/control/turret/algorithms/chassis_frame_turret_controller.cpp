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

#include "tap/drivers.hpp"

#include "../constants/turret_constants.hpp"
#include "../turret_subsystem.hpp"

#include "turret_gravity_compensation.hpp"

using namespace tap::control::turret;

namespace aruwsrc::control::turret::algorithms
{
ChassisFrameYawTurretController::ChassisFrameYawTurretController(
    TurretMotor &yawMotor,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretYawControllerInterface(yawMotor),
      pid(pidConfig)
{
}

void ChassisFrameYawTurretController::initialize()
{
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();
        turretMotor.attachTurretController(this);
    }
}

void ChassisFrameYawTurretController::runController(const uint32_t dt, const float desiredSetpoint)
{
    // limit the yaw min and max angles
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);

    // position controller based on turret yaw gimbal
    float positionControllerError = turretMotor.getValidChassisMeasurementError();

    float pidOutput =
        pid.runController(positionControllerError, turretMotor.getChassisFrameVelocity(), dt);

    turretMotor.setMotorOutput(pidOutput);
}

void ChassisFrameYawTurretController::setSetpoint(float desiredSetpoint)
{
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);
}

float ChassisFrameYawTurretController::getSetpoint() const
{
    return turretMotor.getChassisFrameSetpoint();
}

float ChassisFrameYawTurretController::getMeasurement() const
{
    return turretMotor.getChassisFrameUnwrappedMeasuredAngle();
}

bool ChassisFrameYawTurretController::isOnline() const { return turretMotor.isOnline(); }

ChassisFramePitchTurretController::ChassisFramePitchTurretController(
    TurretMotor &pitchMotorp,
    const tap::algorithms::SmoothPidConfig &pidConfig,
    const float turret_cg_x,
    const float turret_cg_z,
    const float gravity_compensation_scalar)
    : TurretPitchControllerInterface(pitchMotorp),
      pid(pidConfig),
      turret_cg_x(turret_cg_x),
      turret_cg_z(turret_cg_z),
      gravity_compensation_scalar(gravity_compensation_scalar)
{
}

void ChassisFramePitchTurretController::initialize()
{
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();
        turretMotor.attachTurretController(this);
    }
}

void ChassisFramePitchTurretController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    // limit the yaw min and max angles
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);

    // position controller based on turret pitch gimbal
    float positionControllerError = turretMotor.getValidChassisMeasurementError();

    float pidOutput =
        pid.runController(positionControllerError, turretMotor.getChassisFrameVelocity(), dt);

    pidOutput += computeGravitationalForceOffset(
        turret_cg_x,
        turret_cg_z,
        -turretMotor.getAngleFromCenter(),
        gravity_compensation_scalar);

    turretMotor.setMotorOutput(pidOutput);
}

void ChassisFramePitchTurretController::setSetpoint(float desiredSetpoint)
{
    turretMotor.setChassisFrameSetpoint(desiredSetpoint);
}

float ChassisFramePitchTurretController::getSetpoint() const
{
    return turretMotor.getChassisFrameSetpoint();
}

float ChassisFramePitchTurretController::getMeasurement() const
{
    return turretMotor.getChassisFrameUnwrappedMeasuredAngle();
}

bool ChassisFramePitchTurretController::isOnline() const { return turretMotor.isOnline(); }

}  // namespace aruwsrc::control::turret::algorithms
