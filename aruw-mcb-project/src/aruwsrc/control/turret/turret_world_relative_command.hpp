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

#ifndef TURRET_WORLD_RELATIVE_COMMAND_HPP_
#define TURRET_WORLD_RELATIVE_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"
#include "turret_world_relative_chassis_imu_command.hpp"
#include "turret_world_relative_turret_imu_command.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::chassis
{
class ChassisSubsystem;
}

namespace aruwsrc::control::turret
{
class TurretSubsystem;

/**
 * Turret control, with the yaw gimbal using the world relative frame, such that the
 * desired turret angle is independent of the direction that the chassis is facing
 * or rotating. Assumes the board running this subsystem is a RoboMaster type A
 * board with an Mpu6500 and that this board is mounted statically on the chassis.
 *
 * @note The turret mounted IMU is assumed to interface with the ImuRxListener.
 */
class TurretWorldRelativeCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     * The `ChassisSubsystem` is only used for for odometry information.
     */
    TurretWorldRelativeCommand(
        tap::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        const chassis::ChassisSubsystem *chassisSubsystem);

    bool isReady() override;

    bool isFinished() const override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    const char *getName() const override { return "turret WR command"; }

private:
    TurretWorldRelativeChassisImuCommand turretWRChassisImuCommand;
    TurretWorldRelativeTurretImuCommand turretWRTurretImuCommand;
};  // class TurretWorldRelativeCommand

}  // namespace aruwsrc::control::turret

#endif  // TURRET_WORLD_RELATIVE_COMMAND_HPP_
