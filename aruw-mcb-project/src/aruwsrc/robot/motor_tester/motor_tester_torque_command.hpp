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
#ifndef MOTOR_TESTER_TORQUE_COMMAND_HPP_
#define MOTOR_TESTER_TORQUE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "motor_tester_subsystem.hpp"

namespace aruwsrc::motor_tester
{
    using namespace aruwsrc::motor_tester;
    using namespace tap::communication::serial;

class StickTorqueCommand : public tap::control::Command
{
public: 
    StickTorqueCommand(
        tap::Drivers *drivers,
        MotorSubsystem *motorSubsystem,
        Remote::Channel channel,
        int32_t sensitivity)
        : drivers(drivers),
        motorSubsystem(motorSubsystem),
        channel(channel),
        sensitivity(sensitivity)
        {}

    void initialize() override;

    void execute() override{
        motorSubsystem->setDesiredOutput(drivers->remote.getChannel(channel) * sensitivity);
    }

    void end(bool interrupted) override {
        motorSubsystem->setDesiredOutput(0);
    };

    bool isFinished() const override {
        return motorSubsystem->isMotorOnline();
    };

    bool isReady() override {
        return !isFinished();
    }

    const char* getName() const override { return "COMMAND_NAME"; }

private:
    tap::Drivers *drivers;
    MotorSubsystem *motorSubsystem;
    Remote::Channel channel;
    int32_t sensitivity;
};  // class CLASS_NAME

}  // NAMESPACE
#endif  // MOTOR_TESTER_TORQUE_COMMAND_HPP_
