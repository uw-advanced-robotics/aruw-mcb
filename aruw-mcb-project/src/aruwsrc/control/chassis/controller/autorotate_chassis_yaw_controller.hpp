/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef AUTOROTATE_CHASSIS_YAW_CONTROLLER_HPP_
#define AUTOROTATE_CHASSIS_YAW_CONTROLLER_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/wrapped_float.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_yaw_controller_interface.hpp"

namespace aruwsrc::control::chassis::controller
{
class AutorotateChassisYawController : public ChassisYawControllerInterface
{
public:
    /**
     * @param rotationalSymmetry Angle in radians of the chassis' rotational symmetry. Default is
     * 2pi which means no symmetry.
     * @param chassisYawTarget Chassis-turret angle that the controller is aiming for.
     */
    AutorotateChassisYawController(
        const aruwsrc::control::turret::TurretMotor& yawMotor,
        tap::algorithms::SmoothPidConfig pidConfig,
        float followLpAlpha,
        float rotationalSymmetry = M_TWOPI,
        float chassisYawTarget = 0)
        : yawMotor(yawMotor),
          pid(pidConfig),
          followLpAlpha(followLpAlpha),
          rotationalSymmetry(rotationalSymmetry),
          chassisYawTarget(chassisYawTarget),
          setpoint(tap::algorithms::Angle(0))
    {
    }

    float runYawController(const float maxSpeed) override
    {
        setpoint = setpoint.minInterpolate(yawMotor.getChassisFrameSetpoint(), followLpAlpha);
        return pid.runController(
            tap::algorithms::WrappedFloat(setpoint.getWrappedValue(), 0, rotationalSymmetry)
                .minDifference(chassisYawTarget),
            yawMotor.getChassisFrameVelocity(),
            0.002f);
    }

private:
    const aruwsrc::control::turret::TurretMotor& yawMotor;
    tap::algorithms::SmoothPid pid;
    float rotationalSymmetry, chassisYawTarget, followLpAlpha;

    tap::algorithms::WrappedFloat setpoint;
};  // class ChassisYawControllerInterface

}  // namespace aruwsrc::control::chassis::controller

#endif  // AUTOROTATE_CHASSIS_YAW_CONTROLLER_HPP_
