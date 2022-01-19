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

#ifndef WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_
#define WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/smooth_pid.hpp"

#include "../turret_subsystem.hpp"

#include "turret_controller_interface.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class TurretSubsystem;

class WorldFrameYawChassisImuTurretController : public TurretYawControllerInterface
{
public:
    WorldFrameYawChassisImuTurretController(
        aruwsrc::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        float kp,
        float ki,
        float kd,
        float maxICumulative,
        float maxOutput,
        float tQDerivativeKalman,
        float tRDerivativeKalman,
        float tQProportionalKalman,
        float tRProportionalKalman,
        float errDeadzone = 0.0f);

    void initialize() override;

    void runController(const uint32_t dt, const float desiredSetpoint) override;

    float getSetpoint() const override;

    bool isFinished() const override;

private:
    aruwsrc::Drivers *drivers;

    tap::algorithms::SmoothPid pid;

    tap::algorithms::ContiguousFloat worldFrameSetpoint;

    float chassisFrameInitImuYawAngle = 0.0f;
};

}  // namespace aruwsrc::control::turret

#endif  // WORLD_FRAME_CHASSIS_IMU_TURRET_CONTROLLER_HPP_
