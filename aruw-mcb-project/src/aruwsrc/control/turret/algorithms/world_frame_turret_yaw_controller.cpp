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

#include "world_frame_turret_yaw_controller.hpp"

#include "tap/algorithms/contiguous_float.hpp"

#include "../constants/turret_constants.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret::algorithms
{

/**
 * Runs a world frame cascade (position -> velocity) PID controller.
 *
 * @param[in] worldFrameAngleSetpoint World frame angle setpoint, not required to be normalized, in
 * radians.
 * @param[in] worldFrameAngleMeasurement World frame angle measurement, not required to be
 * normalized, in radians.
 * @param[in] worldFrameVelocityMeasured World frame angular velocity measurement, in
 * radians/second.
 * @param[in] dt Time change since this function was last called, in ms.
 * @param[in] turretMotor TurretMotor associated with the angles being measured.
 * @param[out] positionPid Position PID controller.
 * @param[out] velocityPid Velocity PID controller.
 * @return desired PID output from running the position -> velocity cascade controller
 */

WorldFrameTurretYawCascadePIDController::WorldFrameTurretYawCascadePIDController(
        const tap::algorithms::transforms::Transform<aruwsrc::sentry::WorldFrame, aruwsrc::sentry::ChassisFrame>& worldToBaseTransform,
        const aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
        TurretMotor &yawMotor,
        const aruwsrc::control::turret::SentryTurretMinorSubsystem& girlboss,
        const aruwsrc::control::turret::SentryTurretMinorSubsystem& malewife,
        tap::algorithms::SmoothPid &positionPid,
        tap::algorithms::SmoothPid &velocityPid,
        float maxVelErrorInput,
        float minorMajorTorqueRatio)
    : TurretYawControllerInterface(yawMotor),
      worldToBaseTransform(worldToBaseTransform),
      chassis(chassis),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(0, 0.0, M_TWOPI),
      yawMotor(yawMotor),
      girlboss(girlboss),
      malewife(malewife),
      maxVelErrorInput(maxVelErrorInput),
      minorMajorTorqueRatio(minorMajorTorqueRatio)
{
    assert(maxVelErrorInput >= 0);
}

void WorldFrameTurretYawCascadePIDController::initialize()
{
    if (yawMotor.getTurretController() != this)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint.setValue(yawMotor.getChassisFrameSetpoint() + worldToBaseTransform.getYaw());

        yawMotor.attachTurretController(this);
    }
}

// @todo implement separate controller with limiting or refactor elsewhere
//       rationale: it is not at all intuitive or expected for angle limiting to occur here; makes code difficult to trace, follow, and maintain
void WorldFrameTurretYawCascadePIDController::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    ContiguousFloat localAngle = yawMotor.getChassisFrameMeasuredAngle();

    const float localVelocity = yawMotor.getChassisFrameVelocity();

    const float chassisVelocity = *chassis.getActualVelocityChassisRelative()[2];

    worldFrameSetpoint.setValue(desiredSetpoint);

    const float positionControllerError = turretMotor.getValidMinError(
        worldFrameSetpoint.getValue() - worldToBaseTransform.getYaw(),
        localAngle.getValue());

    positionPidOutput =
        positionPid.runControllerDerivateError(positionControllerError, dt);

    const float velocityControllerError = limitVal(positionPidOutput - localVelocity - chassisVelocity, -maxVelErrorInput, maxVelErrorInput);

    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(velocityControllerError, dt);

    torqueCompensation = girlboss.yawMotor.getMotorOutput() + malewife.yawMotor.getMotorOutput();
    if (abs(torqueCompensation) < 2000) { torqueCompensation = 0; }

    turretMotor.setMotorOutput(velocityPidOutput + minorMajorTorqueRatio * torqueCompensation + chassis.getActualVelocityChassisRelative()[3][0]);
}

// @todo what's the point of this; overridden by runController anyways?
void WorldFrameTurretYawCascadePIDController::setSetpoint(float desiredSetpoint)
{
    worldFrameSetpoint.setValue(desiredSetpoint);
}

float WorldFrameTurretYawCascadePIDController::getSetpoint() const
{
    return worldFrameSetpoint.getValue();
}

float WorldFrameTurretYawCascadePIDController::getMeasurement() const
{
    return yawMotor.getChassisFrameMeasuredAngle().getValue() + worldToBaseTransform.getYaw();
}

// @todo ask benjamin about this (benjamin go look at your tracking sheet)
bool WorldFrameTurretYawCascadePIDController::isOnline() const
{
    return turretMotor.isOnline();
}

}  // namespace aruwsrc::control::turret::algorithms
