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
#ifndef WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_IMPL_HPP_
#define WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_IMPL_HPP_

#include "../turret_subsystem.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

#include "turret_gravity_compensation.hpp"
#include "world_frame_turret_imu_turret_controller.hpp"
#include "world_frame_turret_utils.hpp"

namespace aruwsrc::control::turret::algorithms
{
/**
 * Initializes a world frame cascade PID turret controller
 *
 * @param[in] controllerToInitialize The TurretControllerInterface in question being initialized.
 * @param[in] worldFrameMeasurement The measured world frame angle, in radians, not expected to be
 * normalized.
 * @param[out] turretMotor The turret motor that will be controlled by the passed in
 * controllerToInitialize.
 * @param[out] positionPid Position PID controller.
 * @param[out] velocityPid Velocity PID controller.
 * @param[out] worldFrameSetpoint World frame angle setpoint that will be set to the current
 * turretMotor's setpoint.
 */
static inline void initializeWorldFrameTurretImuController(
    const TurretControllerInterface *controllerToInitialize,
    const WrappedFloat worldFrameMeasurement,
    TurretMotor &turretMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid,
    WrappedFloat &worldFrameSetpoint)
{
    if (turretMotor.getTurretController() != controllerToInitialize)
    {
        positionPid.reset();
        velocityPid.reset();

        worldFrameSetpoint = transformChassisFrameToWorldFrame(
            turretMotor.getChassisFrameMeasuredAngle(),
            worldFrameMeasurement,
            turretMotor.getChassisFrameSetpoint());

        turretMotor.attachTurretController(controllerToInitialize);
    }
}

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
static inline float runWorldFrameTurretImuController(
    const WrappedFloat worldFrameAngleError,
    const WrappedFloat chassisFrameAngleMeasurement,
    const float worldFrameVelocityMeasured,
    const uint32_t dt,
    const TurretMotor &turretMotor,
    tap::algorithms::SmoothPid &positionPid,
    tap::algorithms::SmoothPid &velocityPid)
{
    const float positionControllerError = turretMotor.getValidMinError(
        chassisFrameAngleMeasurement + worldFrameAngleError,
        chassisFrameAngleMeasurement);
    const float positionPidOutput =
        positionPid.runController(positionControllerError, worldFrameVelocityMeasured, dt);

    const float velocityControllerError = positionPidOutput - worldFrameVelocityMeasured;
    const float velocityPidOutput =
        velocityPid.runControllerDerivateError(velocityControllerError, dt);

    return velocityPidOutput;
}

template <Axis AXIS>
WorldFrameTurretImuCascadePidTurretController<AXIS>::WorldFrameTurretImuCascadePidTurretController(
    const transforms::Transform &worldToTurret,
    const tap::communication::sensors::imu::AbstractIMU &turretImu,
    TurretMotor &turretMotor,
    SmoothPid &positionPid,
    SmoothPid &velocityPid,
    const std::vector<TurretCompensatorInterface *> compensators)
    : TurretAxisControllerInterface<AXIS>(turretMotor, compensators),
      worldToTurret(worldToTurret),
      turretImu(turretImu),
      positionPid(positionPid),
      velocityPid(velocityPid),
      worldFrameSetpoint(Angle(0))
{
}

template <Axis AXIS>
void WorldFrameTurretImuCascadePidTurretController<AXIS>::initialize()
{
    initializeWorldFrameTurretImuController(
        this,
        Angle(AXIS == Axis::PITCH ? worldToTurret.getPitch() : worldToTurret.getYaw()),
        this->turretMotor,
        positionPid,
        velocityPid,
        worldFrameSetpoint);
}

template <Axis AXIS>
void WorldFrameTurretImuCascadePidTurretController<AXIS>::runController(
    const float dt,
    const WrappedFloat desiredSetpoint)
{
    const WrappedFloat chassisFrame = this->turretMotor.getChassisFrameMeasuredAngle();

    WrappedFloat worldFrameAngle(Angle(0));
    float worldFrameVelocity;
    if constexpr (AXIS == Axis::PITCH)
    {
        worldFrameAngle = Angle(worldToTurret.getPitch());
        worldFrameVelocity = worldToTurret.getPitchVelocity();
    }
    else
    {
        worldFrameAngle = Angle(worldToTurret.getYaw());
        worldFrameVelocity = worldToTurret.getYawVelocity();
    }

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrame,
        worldFrameAngle,
        worldFrameSetpoint,
        this->turretMotor);

    float pidOutput = runWorldFrameTurretImuController(
        worldFrameSetpoint - worldFrameAngle,
        chassisFrame,
        worldFrameVelocity,
        dt,
        this->turretMotor,
        positionPid,
        velocityPid);

    if constexpr (AXIS == Axis::PITCH)
    {
        pidOutput +=
            this->calculateCompensationEffort(TurretCompensatorInterface::TurretCompensatorState{
                .pitchWorldFrame = worldFrameAngle.getWrappedValue(),
                .pitchChassisFrame = chassisFrame.getWrappedValue(),
                .yaw = 0.0f});
    }
    else
    {
        pidOutput +=
            this->calculateCompensationEffort(TurretCompensatorInterface::TurretCompensatorState{
                .pitchWorldFrame = 0.0f,
                .pitchChassisFrame = 0.0f,
                .yaw = chassisFrame.getWrappedValue()});
    }
    this->turretMotor.setMotorOutput(pidOutput);
}

template <Axis AXIS>
void WorldFrameTurretImuCascadePidTurretController<AXIS>::setSetpoint(WrappedFloat desiredSetpoint)
{
    WrappedFloat worldFrameAngle = Angle(0);
    if constexpr (AXIS == Axis::PITCH)
    {
        worldFrameAngle = Angle(worldToTurret.getPitch());
    }
    else
    {
        worldFrameAngle = Angle(worldToTurret.getYaw());
    }

    const WrappedFloat chassisFrameAngle = this->turretMotor.getChassisFrameMeasuredAngle();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameAngle,
        worldFrameAngle,
        worldFrameSetpoint,
        this->turretMotor);
}

template <Axis AXIS>
WrappedFloat WorldFrameTurretImuCascadePidTurretController<AXIS>::getMeasurement() const
{
    if constexpr (AXIS == Axis::PITCH)
    {
        return Angle(worldToTurret.getPitch());
    }
    else
    {
        return Angle(worldToTurret.getYaw());
    }
}

template <Axis AXIS>
bool WorldFrameTurretImuCascadePidTurretController<AXIS>::isOnline() const
{
    return this->turretMotor.isOnline() && turretImu.isOnline();
}

template <Axis AXIS>
WrappedFloat WorldFrameTurretImuCascadePidTurretController<
    AXIS>::convertControllerAngleToChassisFrame(WrappedFloat controllerFrameAngle) const
{
    if constexpr (AXIS == Axis::PITCH)
    {
        const WrappedFloat worldFramePitchAngle = Angle(worldToTurret.getPitch());

        return transformWorldFrameValueToChassisFrame(
            this->turretMotor.getChassisFrameMeasuredAngle(),
            worldFramePitchAngle,
            controllerFrameAngle);
    }
    else
    {
        const WrappedFloat worldFrameYawAngle = Angle(worldToTurret.getYaw());

        return transformWorldFrameValueToChassisFrame(
            this->turretMotor.getChassisFrameMeasuredAngle(),
            worldFrameYawAngle,
            controllerFrameAngle);
    }
}

template <Axis AXIS>
WrappedFloat WorldFrameTurretImuCascadePidTurretController<
    AXIS>::convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const
{
    if constexpr (AXIS == Axis::PITCH)
    {
        const WrappedFloat worldFramePitchAngle = Angle(worldToTurret.getPitch());

        return transformChassisFrameToWorldFrame(
            this->turretMotor.getChassisFrameMeasuredAngle(),
            worldFramePitchAngle,
            chassisFrameAngle);
    }
    else
    {
        const WrappedFloat worldFrameYawAngle = Angle(worldToTurret.getYaw());

        return transformChassisFrameToWorldFrame(
            this->turretMotor.getChassisFrameMeasuredAngle(),
            worldFrameYawAngle,
            chassisFrameAngle);
    }
}
}  // namespace aruwsrc::control::turret::algorithms

#endif  // WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_IMPL_HPP_
