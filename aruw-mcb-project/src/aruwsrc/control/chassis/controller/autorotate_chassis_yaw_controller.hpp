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
#include "tap/algorithms/transforms/transform.hpp"

#include "chassis_yaw_controller_interface.hpp"

namespace aruwsrc::control::chassis::controller
{
class AutorotateChassisYawController : public ChassisYawControllerInterface
{
public:
    /**
     * @param rotationalSymmetry
     */
    AutorotateChassisYawController(
        const tap::algorithms::transforms::Transform& chassisToTurret,
        tap::algorithms::SmoothPidConfig pidConfig,
        float rotationalSymmetry = M_TWOPI,
        float chassisYawTarget = 0)
        : chassisToTurret(chassisToTurret),
          pid(pidConfig),
          rotationalSymmetry(rotationalSymmetry),
          chassisYawTarget(chassisYawTarget)
    {
    }

    float runYawController() override
    {
        return pid.runController(
            chassisToTurret.getYaw() - chassisYawTarget,
            chassisToTurret.getYawVelocity(),
            0.002f);
    }

private:
    const tap::algorithms::transforms::Transform& chassisToTurret;
    tap::algorithms::SmoothPid pid;
    float rotationalSymmetry, chassisYawTarget;
};  // class ChassisYawControllerInterface

}  // namespace aruwsrc::control::chassis::controller

#endif  // AUTOROTATE_CHASSIS_YAW_CONTROLLER_HPP_
