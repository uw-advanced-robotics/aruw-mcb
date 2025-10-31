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

#ifndef STICK_TORQUE_COMMAND_HPP_
#define STICK_TORQUE_COMMAND_HPP_

#include "tap/control/command.hpp"

// #include "../algorithms/turret_controller_interface.hpp"
// #include "../turret_subsystem.hpp"
#include "tap/drivers.hpp"
#include "MotorSubsystem.hpp"

namespace aruwsrc::motor_tester
{
/**
 * Command that takes user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret using some passed in yaw and pitch controller upon construction.
 */
class StickTorqueCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the sentry turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] userYawInputScalar Value to scale the user input from `ControlOperatorInterface`
     * by. Basically mouse sensitivity.
     * @param[in] userPitchInputScalar See userYawInputScalar.
     */
    StickTorqueCommand(
        tap::Drivers *drivers,
        MotorSubsystem *motorSubsystem,
        tap::communication::serial::Remote::Channel channel,
        float scale);

    bool isReady() override;

    const char *getName() const override { return "stick torque command"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

private:
    tap::Drivers *drivers;
    MotorSubsystem *motorSubsystem;
    tap::communication::serial::Remote::Channel channel;

    uint32_t prevTime;

    // algorithms::TurretYawControllerInterface *yawController;
    // algorithms::TurretPitchControllerInterface *pitchController;

    const float scale;

    // const uint8_t turretID;
};
}  // namespace aruwsrc::control::turret::user

#endif  // STICK_TORQUE_COMMAND_HPP_
