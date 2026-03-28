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
#ifndef WORLD_FRAME_STOS_TURRET_CONTROLLER_IMPL_HPP_
#define WORLD_FRAME_STOS_TURRET_CONTROLLER_IMPL_HPP_

#include "../turret_subsystem.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

#include "turret_gravity_compensation.hpp"
#include "world_frame_stos_turret_controller.hpp"
#include "world_frame_turret_imu_turret_controller_impl.hpp"

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
static inline void initializeWorldFrameSTOSTurretController(
    const TurretControllerInterface *controllerToInitialize,
    const WrappedFloat worldFrameMeasurement,
    TurretMotor &turretMotor,
    WrappedFloat &worldFrameSetpoint)
{
    if (turretMotor.getTurretController() != controllerToInitialize)
    {
        worldFrameSetpoint = transformChassisFrameToWorldFrame(
            turretMotor.getChassisFrameMeasuredAngle(),
            worldFrameMeasurement,
            turretMotor.getChassisFrameSetpoint());

        turretMotor.attachTurretController(controllerToInitialize);
    }
}

template <Axis AXIS>
WorldFrameTurretImuSTOSTurretController<AXIS>::WorldFrameTurretImuSTOSTurretController(
    const transforms::Transform &worldToTurret,
    const aruwsrc::communication::can::TurretMCBCanComm &turretMCBCanComm,
    TurretMotor &turretMotor,
    OptimalSTOSController::STOSConstants constants,
    float lqrT,
    float lqrTd,
    const std::vector<TurretCompensatorInterface *> compensators)
    : TurretAxisControllerInterface<AXIS>(turretMotor, compensators),
      worldToTurret(worldToTurret),
      turretMCBCanComm(turretMCBCanComm),
      stosController(constants),
      lqrT(lqrT),
      lqrTd(lqrTd),
      worldFrameSetpoint(Angle(0))
{
}

template <Axis AXIS>
void WorldFrameTurretImuSTOSTurretController<AXIS>::initialize()
{
    const WrappedFloat worldFrameMeasurement =
        Angle(AXIS == Axis::PITCH ? worldToTurret.getPitch() : worldToTurret.getYaw());
    initializeWorldFrameSTOSTurretController(
        this,
        worldFrameMeasurement,
        this->turretMotor,
        worldFrameSetpoint);

    setpointFilter.initialize(worldFrameSetpoint);
}

template <Axis AXIS>
void WorldFrameTurretImuSTOSTurretController<AXIS>::runController(
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

    float pidOutput = 0;

    setpointFilter.update(worldFrameSetpoint, dt / 1000.0f);

    float velError = setpointFilter.getEstimatedVelocity() - worldFrameVelocity;
    float posError = this->turretMotor.getValidMinError(
        chassisFrame + (worldFrameSetpoint - worldFrameAngle),
        chassisFrame);

    // float vel = worldFrameVelocity;
    DEBUGV = velError;
    DEBUGP = posError;
    float torqueFF = (0.0443 * this->turretMotor.getChassisFrameVelocity());

    const float LINEAR_ZONE = DEBUG1;

    if (std::abs(posError) < LINEAR_ZONE)
    {
        float friction = signbit(posError) ? -0.01 : 0.01;
        pidOutput = friction + torqueFF;
        pidOutput += (posError * lqrT) + (velError * lqrTd);
    }
    else
    {
        pidOutput = stosController.getOptimalTorque(posError, -velError);
    }
    /// convert to torque @TODO: Make passed in or smth
    pidOutput *= (16384.0f / 1.3f);
    DEBUG3 = pidOutput / (16384.0f / 1.3f);

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

    DEBUG2 = worldFrameAngle.minDifference(worldFrameSetpoint);
    std::clamp(pidOutput, -16384.0f, 16384.0f);
    this->turretMotor.setMotorOutput(pidOutput);
}

template <Axis AXIS>
void WorldFrameTurretImuSTOSTurretController<AXIS>::setSetpoint(WrappedFloat desiredSetpoint)
{
    const WrappedFloat chassisFrameYaw = this->turretMotor.getChassisFrameMeasuredAngle();
    const WrappedFloat worldFrameYawAngle = Angle(worldToTurret.getYaw());

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameYaw,
        worldFrameYawAngle,
        worldFrameSetpoint,
        this->turretMotor);
}

template <Axis AXIS>
WrappedFloat WorldFrameTurretImuSTOSTurretController<AXIS>::getMeasurement() const
{
    return Angle(worldToTurret.getYaw());
}

template <Axis AXIS>
bool WorldFrameTurretImuSTOSTurretController<AXIS>::isOnline() const
{
    return this->turretMotor.isOnline() && turretMCBCanComm.isConnected();
}

template <Axis AXIS>
WrappedFloat WorldFrameTurretImuSTOSTurretController<AXIS>::convertControllerAngleToChassisFrame(
    WrappedFloat controllerFrameAngle) const
{
    const WrappedFloat worldFrameYawAngle = Angle(worldToTurret.getYaw());

    return transformWorldFrameValueToChassisFrame(
        this->turretMotor.getChassisFrameMeasuredAngle(),
        worldFrameYawAngle,
        controllerFrameAngle);
}

template <Axis AXIS>
WrappedFloat WorldFrameTurretImuSTOSTurretController<AXIS>::convertChassisAngleToControllerFrame(
    WrappedFloat chassisFrameAngle) const
{
    const WrappedFloat worldFrameYawAngle = Angle(worldToTurret.getYaw());

    return transformChassisFrameToWorldFrame(
        this->turretMotor.getChassisFrameMeasuredAngle(),
        worldFrameYawAngle,
        chassisFrameAngle);
}
}  // namespace aruwsrc::control::turret::algorithms

#endif  // WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_IMPL_HPP_
