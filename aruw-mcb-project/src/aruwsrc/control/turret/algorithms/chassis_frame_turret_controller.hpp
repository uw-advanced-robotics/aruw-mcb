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

#ifndef CHASSIS_FRAME_TURRET_CONTROLLER_HPP_
#define CHASSIS_FRAME_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/smooth_pid.hpp"

#include "turret_controller_interface.hpp"

namespace aruwsrc::control::turret
{
class TurretMotor;
}

namespace aruwsrc::control::turret::algorithms
{
/**
 * Controller that runs a single position PID controller in the chassis frame to control the turret.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
template <tap::algorithms::transforms::Axis AXIS>
class ChassisFrameTurretController : public TurretAxisControllerInterface<AXIS>
{
public:
    /**
     * @param[in] Motor A `yawMotor` object accessible for children objects to use.
     * @param[in] pidConfig PID configuration struct for the controller.
     */
    ChassisFrameTurretController(
        TurretMotor &Motor,
        const tap::algorithms::SmoothPidConfig &pidConfig,
        const std::vector<TurretCompensatorInterface *> compensators = {});

    void initialize();

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The yaw desired setpoint in the chassis frame.
     */
    void runController(const float dt, const tap::algorithms::WrappedFloat desiredSetpoint);

    void setSetpoint(tap::algorithms::WrappedFloat desiredSetpoint);

    /// @return The chassis frame yaw turret measurement, refer to top level documentation for more
    /// details.
    tap::algorithms::WrappedFloat getMeasurement() const;

    /**
     * @return The yaw setpoint, in the chassis frame.
     */
    tap::algorithms::WrappedFloat getSetpoint() const;

    bool isOnline() const;

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline tap::algorithms::WrappedFloat convertControllerAngleToChassisFrame(
        tap::algorithms::WrappedFloat controllerFrameAngle) const
    {
        return controllerFrameAngle;
    }

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline tap::algorithms::WrappedFloat convertChassisAngleToControllerFrame(tap::algorithms::WrappedFloat chassisFrameAngle) const
    {
        return chassisFrameAngle;
    }

private:
    tap::algorithms::SmoothPid pid;
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  // CHASSIS_FRAME_TURRET_CONTROLLER_HPP_
#include "chassis_frame_turret_controller_impl.hpp"