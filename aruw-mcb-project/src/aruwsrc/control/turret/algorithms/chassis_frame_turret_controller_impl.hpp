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
#ifndef CHASSIS_FRAME_TURRET_CONTROLLER_IMPL_HPP_
#define CHASSIS_FRAME_TURRET_CONTROLLER_IMPL_HPP_

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "../constants/turret_constants.hpp"
#include "../turret_subsystem.hpp"

#include "chassis_frame_turret_controller.hpp"
#include "turret_gravity_compensation.hpp"

using namespace tap::control::turret;
using tap::algorithms::WrappedFloat;

namespace aruwsrc::control::turret::algorithms
{
template <Axis AXIS>
ChassisFrameTurretController<AXIS>::ChassisFrameTurretController(
    TurretMotor &Motor,
    const tap::algorithms::SmoothPidConfig &pidConfig,
    const std::vector<TurretCompensatorInterface *> compensators)
    : TurretAxisControllerInterface<AXIS>(Motor, compensators),
      pid(pidConfig)
{
}
template <Axis AXIS>
void ChassisFrameTurretController<AXIS>::initialize()
{
    if (this->turretMotor.getTurretController() != this)
    {
        pid.reset();
        this->turretMotor.attachTurretController(this);
    }
}
template <Axis AXIS>
void ChassisFrameTurretController<AXIS>::runController(
    const uint32_t dt,
    const WrappedFloat desiredSetpoint)
{
    // limit the yaw min and max angles
    this->turretMotor.setChassisFrameSetpoint(desiredSetpoint);

    // position controller based on turret yaw gimbal
    float positionControllerError = this->turretMotor.getValidChassisMeasurementError();

    float pidOutput =
        pid.runController(positionControllerError, this->turretMotor.getChassisFrameVelocity(), dt);
    if constexpr (AXIS == Axis::PITCH)
    {
        pidOutput +=
            this->calculateCompensationEffort(TurretCompensatorInterface::TurretCompensatorState{
                .pitch = this->turretMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
                .yaw = 0.0f});
    }
    else
    {
        pidOutput +=
            this->calculateCompensationEffort(TurretCompensatorInterface::TurretCompensatorState{
                .pitch = 0.0f,
                .yaw = this->turretMotor.getChassisFrameMeasuredAngle().getWrappedValue()});
    }

    this->turretMotor.setMotorOutput(pidOutput);
}
template <Axis AXIS>
void ChassisFrameTurretController<AXIS>::setSetpoint(WrappedFloat desiredSetpoint)
{
    this->turretMotor.setChassisFrameSetpoint(desiredSetpoint);
}
template <Axis AXIS>
WrappedFloat ChassisFrameTurretController<AXIS>::getSetpoint() const
{
    return this->turretMotor.getChassisFrameSetpoint();
}
template <Axis AXIS>
WrappedFloat ChassisFrameTurretController<AXIS>::getMeasurement() const
{
    return this->turretMotor.getChassisFrameMeasuredAngle();
}
template <Axis AXIS>
bool ChassisFrameTurretController<AXIS>::isOnline() const
{
    return this->turretMotor.isOnline();
}

}  // namespace aruwsrc::control::turret::algorithms
#endif  // CHASSIS_FRAME_TURRET_CONTROLLER_IMPL_HPP_