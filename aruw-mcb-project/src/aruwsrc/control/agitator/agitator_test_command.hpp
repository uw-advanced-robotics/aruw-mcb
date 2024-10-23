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

#ifndef AGITATOR_TEST_COMMAND_HPP_
#define AGITATOR_TEST_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"
#include "tap/algorithms/math_user_utils.hpp"


namespace aruwsrc
{
namespace agitator
{
class AgitatorTestCommand : public tap::control::Command
{
public:
    AgitatorTestCommand(tap::control::setpoint::SetpointSubsystem* subsystem) : subsystem(subsystem)
    {
        this->addSubsystemRequirement(subsystem);
    }

    bool isReady() override { return true; };

    void initialize() override { subsystem->setSetpoint(subsystem->getCurrentValue() + M_PI / 2); };

    void execute() override{};

    void end(bool) override{};

    bool isFinished() const override
    {
        return tap::algorithms::compareFloatClose(
            this->subsystem->getSetpoint(),
            this->subsystem->getCurrentValue(),
            M_PI / 16);
    };

    const char* getName() const override { return "agitator test command"; }

private:
    tap::control::setpoint::SetpointSubsystem* subsystem;
};  // class AgitatorTestCommand

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_TEST_COMMAND_HPP_
