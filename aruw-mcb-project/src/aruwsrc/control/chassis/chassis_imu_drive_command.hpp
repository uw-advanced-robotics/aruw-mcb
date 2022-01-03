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

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::chassis
{
class ChassisSubsystem;

/**
 * A command that allows the user to control the translation and rotation of the chassis.
 * User translational input is relative to the chassis in a manner similar to
 * `chassis_rel_drive.hpp`. The user specifies some rotation via the control operator interface (see
 * the function `getChassisRInput`). The chassis mounted IMU is used as feedback for a position
 * controller so that the user specified chassis rotation is absolute. This means the chassis will
 * attempt to maintain a particular world relative chassis rotation angle.
 *
 * @note It is assumed that the onboard `mpu6500` is attached to the chassis.
 */
class ChassisImuDriveCommand : public tap::control::Command
{
public:
    static constexpr float USER_INPUT_TO_ANGLE_DELTA_SCALAR = 1.0f;
    static constexpr float MAX_ROTATION_ERR = 30.0f;

    ChassisImuDriveCommand(aruwsrc::Drivers* drivers, ChassisSubsystem* chassis);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis imu drive"; }

private:
    aruwsrc::Drivers* drivers;
    ChassisSubsystem* chassis;
    tap::algorithms::ContiguousFloat rotationSetpoint;
    bool imuSetpointInitialized = false;
};  // class ChassisImuDriveCommand

}  // namespace aruwsrc::chassis

#endif  // CHASSIS_IMU_DRIVE_COMMAND_HPP_
