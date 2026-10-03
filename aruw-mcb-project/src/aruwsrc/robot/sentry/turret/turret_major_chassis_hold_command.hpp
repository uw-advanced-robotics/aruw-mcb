/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_MAJOR_CHASSIS_HOLD_COMMAND_HPP_
#define TURRET_MAJOR_CHASSIS_HOLD_COMMAND_HPP_

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/turret/algorithms/turret_controller_interface.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"

namespace aruwsrc::sentry::turret
{
/**
 * Holds turret major at a fixed chassis-relative yaw. Used when a sensor mounted on turret major
 * must stay rigid with respect to the chassis (e.g. Odin characterization against chassis mocap).
 */
class TurretMajorChassisHoldCommand : public tap::control::Command
{
public:
    /**
     * @param[in] chassisFrameYawController a controller whose setpoint is in the chassis frame
     * @param[in] chassisYaw the chassis-frame yaw (rad) to hold
     */
    TurretMajorChassisHoldCommand(
        aruwsrc::control::turret::YawTurretSubsystem& turretMajor,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::YAW>& chassisFrameYawController,
        float chassisYaw);

    void initialize() override;
    void execute() override;
    bool isFinished() const override { return !yawController.isOnline(); }
    void end(bool) override;
    const char* getName() const override { return "turret major chassis hold"; }

private:
    aruwsrc::control::turret::YawTurretSubsystem& turretMajor;
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        tap::algorithms::transforms::Axis::YAW>& yawController;
    const tap::algorithms::Angle setpoint;
    uint32_t prevTime = 0;
};
}  // namespace aruwsrc::sentry::turret

#endif  // TURRET_MAJOR_CHASSIS_HOLD_COMMAND_HPP_
