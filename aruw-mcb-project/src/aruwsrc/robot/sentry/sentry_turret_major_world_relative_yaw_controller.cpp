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
// using namespace tap::algorithms::transforms;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control::sentry;

namespace aruwsrc::control::turret::algorithms
{
TurretMajorWorldFrameController::TurretMajorWorldFrameController(
    const transforms::Transform& worldToChassis,
    const HolonomicChassisSubsystem& chassis,
    TurretMotor& yawMotor,
    const SentryTurretMinorSubsystem& turretLeft,
    const SentryTurretMinorSubsystem& turretRight,
    SmoothPid& positionPid,
    SmoothPid& velocityPid,
    float maxVelErrorInput,
    float minorMajorTorqueRatio)
    : TurretYawControllerInterface(yawMotor),
      worldToChassis(worldToChassis),
      chassis(chassis),
      yawMotor(yawMotor),
      turretLeft(turretLeft),
      turretRight(turretRight),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(0, 0.0, M_TWOPI),
      maxVelErrorInput(maxVelErrorInput),
      minorMajorTorqueRatio(minorMajorTorqueRatio)
{
    assert(maxVelErrorInput >= 0);
}

void TurretMajorWorldFrameController::initialize()
{
    if (yawMotor.getTurretController() != this)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint.setWrappedValue(
            yawMotor.getChassisFrameSetpoint() + worldToChassis.getYaw());

        yawMotor.attachTurretController(this);
    }
}

// @todo implement separate controller with limiting or refactor elsewhere
//       rationale: it is not at all intuitive or expected for angle limiting to occur here; makes
//       code difficult to trace, follow, and maintain
void TurretMajorWorldFrameController::runController(const uint32_t dt, const float desiredSetpoint)
{
    WrappedFloat localAngle = yawMotor.getChassisFrameMeasuredAngle();

    const float localVelocity = yawMotor.getChassisFrameVelocity();

    const float chassisVelocity = *chassis.getActualVelocityChassisRelative()[2];

    worldFrameSetpoint.setWrappedValue(desiredSetpoint);

    const float positionControllerError = turretMotor.getValidMinError(
        worldFrameSetpoint.getWrappedValue() - worldToChassis.getYaw(),
        localAngle.getWrappedValue());

    // positionPidOutput = positionPid.runControllerDerivateError(positionControllerError, dt);

    const float velocityControllerError = limitVal(
        positionPidOutput - localVelocity - chassisVelocity,
        -maxVelErrorInput,
        maxVelErrorInput);

    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(velocityControllerError, dt);

    torqueCompensation =
        turretLeft.yawMotor.getMotorOutput() + turretRight.yawMotor.getMotorOutput();
    if (abs(torqueCompensation) < 3000)  // @todo make a config
    {
        torqueCompensation = 0;
    }
    // @note: in case things look weird, try adding the chassis' rotational velocity to
    // setMotorOutput
    turretMotor.setMotorOutput(velocityPidOutput + minorMajorTorqueRatio * torqueCompensation);
    // turretMotor.setMotorOutput(velocityPidOutput);  // @todo: final maxOutput for this controller
}

// @todo what's the point of this; overridden by runController anyways?
void TurretMajorWorldFrameController::setSetpoint(float desiredSetpoint)
{
    worldFrameSetpoint.setWrappedValue(desiredSetpoint);
}

float TurretMajorWorldFrameController::getSetpoint() const
{
    return worldFrameSetpoint.getWrappedValue();
}

float TurretMajorWorldFrameController::getMeasurement() const
{
    return yawMotor.getChassisFrameMeasuredAngle().getWrappedValue() + worldToChassis.getYaw();
}

bool TurretMajorWorldFrameController::isOnline() const { return turretMotor.isOnline(); }

}  // namespace aruwsrc::control::turret::algorithms
