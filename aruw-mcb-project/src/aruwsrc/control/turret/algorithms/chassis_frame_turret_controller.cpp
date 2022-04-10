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

#include "../constants/turret_constants.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

#include "turret_gravity_compensation.hpp"

using namespace tap::control::turret;

namespace aruwsrc::control::turret::algorithms
{
ChassisFrameYawTurretController::ChassisFrameYawTurretController(
    TurretSubsystem *turretSubsystem,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretYawControllerInterface(turretSubsystem),
      pid(pidConfig)
{
}

void ChassisFrameYawTurretController::initialize()
{
    if (turretSubsystem->getPrevRanYawTurretController() != this)
    {
        pid.reset();
        turretSubsystem->setPrevRanYawTurretController(this);
    }
}

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

bool ChassisFrameYawTurretController::isOnline() const { return turretSubsystem->isOnline(); }

ChassisFramePitchTurretController::ChassisFramePitchTurretController(
    TurretSubsystem *turretSubsystem,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretPitchControllerInterface(turretSubsystem),
      pid(pidConfig)
{
}

void ChassisFramePitchTurretController::initialize()
{
    if (turretSubsystem->getPrevRanPitchTurretController() != this)
    {
        pid.reset();
        turretSubsystem->setPrevRanPitchTurretController(this);
    }
}

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
        TURRET_CG_X,
        TURRET_CG_Z,
        -turretSubsystem->getPitchAngleFromCenter(),
        GRAVITY_COMPENSATION_SCALAR);

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

float ChassisFramePitchTurretController::getSetpoint() const
{
    return turretSubsystem->getPitchSetpoint();
}

bool ChassisFramePitchTurretController::isOnline() const { return turretSubsystem->isOnline(); }

}  // namespace aruwsrc::control::turret::algorithms
