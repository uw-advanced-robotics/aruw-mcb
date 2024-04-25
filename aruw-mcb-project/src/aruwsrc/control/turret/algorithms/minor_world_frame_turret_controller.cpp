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

#include "minor_world_frame_turret_controller.hpp"

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

WorldFrameTurretYawCascadePIDControllerMinor::WorldFrameTurretYawCascadePIDControllerMinor(
    const tap::algorithms::transforms::Transform &worldToBaseTransform,
    TurretMotor &yawMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid,
    const aruwsrc::can::TurretMCBCanComm &turretMCB)
    : TurretYawControllerInterface(yawMotor),
      worldToBaseTransform(worldToBaseTransform),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(0),
      yawMotor(yawMotor),
      turretMCB(turretMCB)
{
}

void WorldFrameTurretYawCascadePIDControllerMinor::initialize()
{
    if (yawMotor.getTurretController() != this)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint = yawMotor.getChassisFrameSetpoint() + worldToBaseTransform.getYaw();

        yawMotor.attachTurretController(this);
    }
}

float localSetpoint = 0;

// @todo implement separate controller with limiting or refactor elsewhere
//       rationale: it is not at all intuitive or expected for angle limiting to occur here; makes
//       code difficult to trace, follow, and maintain
void WorldFrameTurretYawCascadePIDControllerMinor::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    // float localAngle = turretMCB.getYawUnwrapped() - worldToBaseTransform.getYaw();

    localSetpoint = desiredSetpoint + yawMotor.getChassisFrameUnwrappedMeasuredAngle() -
                    turretMCB.getYawUnwrapped();

    yawMotor.setChassisFrameSetpoint(localSetpoint);

    localSetpoint = yawMotor.getChassisFrameSetpoint();

    worldFrameSetpoint = localSetpoint - yawMotor.getChassisFrameUnwrappedMeasuredAngle() +
                         turretMCB.getYawUnwrapped();

    const float positionControllerError =
        turretMotor.getValidMinError(worldFrameSetpoint, turretMCB.getYawUnwrapped());

    positionPidOutput =
        positionPid.runController(positionControllerError, turretMCB.getYawVelocity(), dt);

    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(positionPidOutput - turretMCB.getYawVelocity(), dt);

    yawMotor.setMotorOutput(velocityPidOutput);
}

// @todo what's the point of this; overridden by runController anyways?
void WorldFrameTurretYawCascadePIDControllerMinor::setSetpoint(float desiredSetpoint)
{
    float localSetpoint =
        turretMotor.getSetpointWithinTurretRange(desiredSetpoint - worldToBaseTransform.getYaw());

    worldFrameSetpoint = localSetpoint + worldToBaseTransform.getYaw();
}

float WorldFrameTurretYawCascadePIDControllerMinor::getSetpoint() const
{
    return worldFrameSetpoint;
}

float WorldFrameTurretYawCascadePIDControllerMinor::getMeasurement() const
{
    return yawMotor.getChassisFrameMeasuredAngle().getWrappedValue() +
           worldToBaseTransform.getYaw();
}

// @todo ask benjamin about this (benjamin go look at your tracking sheet)
bool WorldFrameTurretYawCascadePIDControllerMinor::isOnline() const { return yawMotor.isOnline(); }

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

WorldFrameTurretPitchCascadePIDControllerMinor::WorldFrameTurretPitchCascadePIDControllerMinor(
    const tap::algorithms::transforms::Transform &worldToBaseTransform,
    TurretMotor &pitchMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid,
    const aruwsrc::can::TurretMCBCanComm &turretMCB)
    : TurretPitchControllerInterface(pitchMotor),
      worldToBaseTransform(worldToBaseTransform),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(0),
      pitchMotor(pitchMotor),
      turretMCB(turretMCB)
{
}

void WorldFrameTurretPitchCascadePIDControllerMinor::initialize()
{
    if (pitchMotor.getTurretController() != this)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint = pitchMotor.getChassisFrameSetpoint() + worldToBaseTransform.getPitch();

        pitchMotor.attachTurretController(this);
    }
}

float velocityPidOutput = 0.0f;

// @todo implement separate controller with limiting or refactor elsewhere
//       rationale: it is not at all intuitive or expected for angle limiting to occur here; makes
//       code difficult to trace, follow, and maintain
void WorldFrameTurretPitchCascadePIDControllerMinor::runController(
    const uint32_t dt,
    const float desiredSetpoint)
{
    float localAngle = turretMCB.getPitch() - worldToBaseTransform.getPitch();

    const float worldVelocity = turretMCB.getPitchVelocity();

    float localSetpoint =
        turretMotor.getSetpointWithinTurretRange(desiredSetpoint - worldToBaseTransform.getPitch());

    pitchMotor.setChassisFrameSetpoint(localSetpoint);

    localSetpoint = pitchMotor.getChassisFrameSetpoint();

    worldFrameSetpoint = localSetpoint + worldToBaseTransform.getPitch();

    const float positionControllerError = turretMotor.getValidMinError(localSetpoint, localAngle);

    positionPidOutput =
        positionPid.runController(positionControllerError, turretMCB.getPitchVelocity(), dt);

    float velocityPidOutput =
        velocityPid.runControllerDerivateError(positionPidOutput - worldVelocity, dt);

    // velocityPidOutput +=
    //     computeGravitationalForceOffset(CG_X, CG_Z, turretMCB.getPitch(), G_COMPENSATION_SCALAR);

    pitchMotor.setMotorOutput(velocityPidOutput);
}

// @todo what's the point of this; overridden by runController anyways?
void WorldFrameTurretPitchCascadePIDControllerMinor::setSetpoint(float desiredSetpoint)
{
    worldFrameSetpoint = desiredSetpoint;
}

float WorldFrameTurretPitchCascadePIDControllerMinor::getSetpoint() const
{
    return worldFrameSetpoint;
}

float WorldFrameTurretPitchCascadePIDControllerMinor::getMeasurement() const
{
    return pitchMotor.getChassisFrameUnwrappedMeasuredAngle() + worldToBaseTransform.getPitch();
}

// @todo ask benjamin about this (benjamin go look at your tracking sheet)
bool WorldFrameTurretPitchCascadePIDControllerMinor::isOnline() const
{
    return pitchMotor.isOnline();
}

}  // namespace aruwsrc::control::turret::algorithms
