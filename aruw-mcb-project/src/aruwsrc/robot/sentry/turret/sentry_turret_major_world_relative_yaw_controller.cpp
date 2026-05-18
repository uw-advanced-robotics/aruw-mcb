/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "sentry_turret_major_world_relative_yaw_controller.hpp"

using namespace tap::algorithms;
using namespace aruwsrc::control::chassis;

namespace aruwsrc::sentry::turret
{
TurretMajorWorldFrameController::TurretMajorWorldFrameController(
    const transforms::Transform& worldToMajor,
    const HolonomicChassisSubsystem& chassis,
    aruwsrc::control::turret::TurretMotor& yawMotor,
    tap::communication::sensors::imu::AbstractIMU& turretMajorIMU,
    const SentryTurretMinorSubsystem& turretWidow,
    SmoothPid& positionPid,
    SmoothPid& velocityPid,
    float maxVelErrorInput)
    : TurretAxisControllerInterface<control::turret::algorithms::Axis::YAW>(yawMotor),
      worldToMajor(worldToMajor),
      chassis(chassis),
      yawMotor(yawMotor),
      turretMajorIMU(turretMajorIMU),
      turretWidow(turretWidow),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(0, 0.0, M_TWOPI),
      maxVelErrorInput(maxVelErrorInput)
{
    assert(maxVelErrorInput >= 0);
}

void TurretMajorWorldFrameController::initialize()
{
    if (yawMotor.getTurretController() != this)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint = yawMotor.getChassisFrameSetpoint() -
                             yawMotor.getChassisFrameMeasuredAngle() + worldToMajor.getYaw();

        yawMotor.attachTurretController(this);
    }
}

/// @todo implement separate controller with limiting or refactor elsewhere
///       rationale: it is not at all intuitive or expected for angle limiting to occur here; makes
///       code difficult to trace, follow, and maintain
void TurretMajorWorldFrameController::runController(
    const float dt,
    const WrappedFloat desiredSetpoint)
{
    worldFrameSetpoint = desiredSetpoint;

    float vel = turretMajorIMU.getGz();

    const float positionControllerError =
        turretMotor.getValidMinError(worldFrameSetpoint, Angle(worldToMajor.getYaw()));

    positionPidOutput = positionPid.runController(positionControllerError, vel, dt);

    const float velocityControllerError =
        limitVal(positionPidOutput - vel, -maxVelErrorInput, maxVelErrorInput);

    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(velocityControllerError, dt);

    turretMotor.setMotorOutput(velocityPidOutput);
}

// @todo what's the point of this; overridden by runController anyways?
void TurretMajorWorldFrameController::setSetpoint(WrappedFloat desiredSetpoint)
{
    worldFrameSetpoint = desiredSetpoint;
}

WrappedFloat TurretMajorWorldFrameController::getSetpoint() const { return worldFrameSetpoint; }

WrappedFloat TurretMajorWorldFrameController::getMeasurement() const
{
    return yawMotor.getChassisFrameMeasuredAngle() + worldToMajor.getYaw();
}

bool TurretMajorWorldFrameController::isOnline() const
{
    return turretMotor.isOnline() &&
           turretMajorIMU.getImuState() !=
               tap::communication::sensors::imu::AbstractIMU::ImuState::IMU_NOT_CONNECTED;
}

}  // namespace aruwsrc::sentry::turret
