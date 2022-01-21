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
class TurretSubsystem;
}

namespace aruwsrc::control::turret::algorithms
{
/**
 *
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class ChassisFrameYawTurretController final : public TurretYawControllerInterface
{
public:
    ChassisFrameYawTurretController(
        TurretSubsystem *turretSubsystem,
        const tap::algorithms::SmoothPidConfig &pidConfig);

    void initialize() override;

    void runController(const uint32_t dt, const float desiredSetpoint) override;

    float getSetpoint() const override;

    bool isOnline() const override;

private:
    tap::algorithms::SmoothPid pid;
};

/**
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class ChassisFramePitchTurretController final : public TurretPitchControllerInterface
{
public:
    ChassisFramePitchTurretController(
        TurretSubsystem *turretSubsystem,
        const tap::algorithms::SmoothPidConfig &pidConfig);

    void initialize() override;

    void runController(const uint32_t dt, const float desiredSetpoint) override;

    float getSetpoint() const override;

    bool isOnline() const override;

private:
    tap::algorithms::SmoothPid pid;
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  // CHASSIS_FRAME_TURRET_CONTROLLER_HPP_
