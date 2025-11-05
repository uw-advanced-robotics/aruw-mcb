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
#ifndef MOTOR_TESTER_PID_COMMAND_HPP_
#define MOTOR_TESTER_PID_COMMAND_HPP_

#include <tap/algorithms/smooth_pid.hpp>
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "motor_tester_subsystem.hpp"
#include "motor_tester_constants.hpp"

namespace aruwsrc::motor_tester
{
    using namespace aruwsrc::motor_tester;
    using namespace tap::communication::serial;

class MotorTesterPid : public tap::control::Command
{
public: 
    MotorTesterPid(tap::Drivers *drivers,
        MotorSubsystem *motorSubsystem,
        Remote::Channel channel,
        int32_t sensitivity,
        const tap::algorithms::SmoothPidConfig &posPidConfig,
        float reqPosition)
        : drivers(drivers),
        motorSubsystem(motorSubsystem),
        channel(channel),
        sensitivity(sensitivity),
        posPid(posPidConfig),
        reqPosition(reqPosition)
        {
            addSubsystemRequirement(motorSubsystem);
        };

    void initialize() override {
        counter = 10;
        error = 0;
    };

    void execute() override {
        counter += 1;
        reqPosition = drivers->remote.getChannel(channel) * sensitivity;
        error = reqPosition - motorSubsystem->getMotorPosition().getUnwrappedValue();
        posPid.runController(
            (reqPosition - motorSubsystem->getMotorPosition().getUnwrappedValue()),
            -motorSubsystem->getMotorVelocity(),
            0.002f
        );
        motorSubsystem->setDesiredOutput(posPid.getOutput());
        
    };
    
    bool isReady() override {return !isFinished();};



    void end(bool interrupted) override {
        motorSubsystem->setDesiredOutput(0);
    };

    bool isFinished() const override {
        return !motorSubsystem->isMotorOnline();
    };

    const char* getName() const override { return "COMMAND_NAME"; }
private:
    tap::Drivers *drivers;
    MotorSubsystem *motorSubsystem;
    Remote::Channel channel;
    int32_t sensitivity;
    tap::algorithms::SmoothPid posPid;
    float reqPosition;
    u_int64_t counter = 1;
    float error = 0;
    float p = 0;
};  // class CLASS_NAME

}  // NAMESPACE
#endif  // MOTOR_TESTER_PID_COMMAND_HPP_
