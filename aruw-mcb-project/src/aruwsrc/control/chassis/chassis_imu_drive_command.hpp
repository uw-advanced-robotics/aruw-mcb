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

#ifndef CHASSIS_IMU_DRIVE_COMMAND_HPP_
#define CHASSIS_IMU_DRIVE_COMMAND_HPP_

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/control/command.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::chassis
{
class ChassisSubsystem;

/**
 * A command that applies classic chassis-relative mecanum drive, using the chassis
 * mounted IMU such that the chassis drives straight. Similar to the normal chassis
 * drive command but the user commands some direction relative to the chassis yaw that
 * they want to drive.
 */
class ChassisImuDriveCommand : public tap::control::Command
{
public:
    static constexpr float USER_INPUT_TO_ANGLE_DELTA_SCALAR = 1.0f;
    static constexpr float ROTATION_PID_KP = 160.0f;
    static constexpr float MAX_ROTATION_ERR = 30.0f;

    ChassisImuDriveCommand(tap::Drivers* drivers, ChassisSubsystem* chassis);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis imu drive"; }

private:
    tap::Drivers* drivers;
    ChassisSubsystem* chassis;
    tap::algorithms::ContiguousFloat rotationSetpoint;
    bool imuSetpointInitialized = false;
};  // class ChassisImuDriveCommand

}  // namespace aruwsrc::chassis

#endif  // CHASSIS_IMU_DRIVE_COMMAND_HPP_
