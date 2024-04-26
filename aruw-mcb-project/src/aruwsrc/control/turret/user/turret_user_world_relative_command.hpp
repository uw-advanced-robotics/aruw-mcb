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

#ifndef TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_
#define TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "../algorithms/turret_controller_interface.hpp"

#include "turret_user_control_command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control
{
class ControlOperatorInterface;
}

namespace aruwsrc::control::turret
{
class TurretSubsystem;
}

namespace aruwsrc::control::turret::user
{
/**
 * Turret control, with the yaw and pitch gimbals using the world relative frame,
 * such that the desired turret angle is independent of the direction that the chassis
 * is facing or rotating. Assumes the board running this subsystem is a RoboMaster type A
 * board with an Mpu6500 and that this board is mounted statically on the chassis.
 * Also assumes that there is an IMU mounted on the turret that interfaces
 * with the `TurretMCBCanComm`. If there is no such IMU, the chassis IMU will be used
 * to run the turret controller and the pitch axis will run a controller in the chassis
 * frame.
 *
 * Takes in user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret.
 */
class TurretUserWorldRelativeCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     *
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the turret to control.
     * @param[in] chassisImuYawController World frame turret controller that uses the chassis IMU.
     * @param[in] chassisImuPitchController Turret controller that is used when the turret IMU is in  // @todo what the hell is this docstring
     * use.
     * @param[in] turretImuYawController World frame turret controller that uses the turret IMU.
     * @param[in] turretImuPitchController Turret controller that is used when the turret IMU is in
     * use. Doesn't strictly have to be world relative.
     */
    TurretUserWorldRelativeCommand(
        tap::Drivers *drivers,
        ControlOperatorInterface &controlOperatorInterface,
        TurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *chassisImuYawController,
        algorithms::TurretPitchControllerInterface *chassisImuPitchController,
        algorithms::TurretYawControllerInterface *turretImuYawController,
        algorithms::TurretPitchControllerInterface *turretImuPitchController,
        float userYawInputScalar,
        float userPitchInputScalar,
        uint8_t turretID = 0);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupted) override;

    const char *getName() const override { return "turret WR"; }

private:
    TurretUserControlCommand turretWRChassisImuCommand;
    TurretUserControlCommand turretWRTurretImuCommand;
};  // class TurretUserWorldRelativeCommand

}  // namespace aruwsrc::control::turret::user

#endif  // TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_
