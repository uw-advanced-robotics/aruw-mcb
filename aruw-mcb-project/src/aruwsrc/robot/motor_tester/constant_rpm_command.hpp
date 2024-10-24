/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CONSTANT_RPM_COMMAND_HPP_
#define CONSTANT_RPM_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "motor_subsystem.hpp"

namespace aruwsrc::motor_tester
{
class ConstantRpmCommand : public tap::control::Command
{
public:
    explicit ConstantRpmCommand(MotorSubsystem* subsystem, float rpm, float pulleyRatio = 1.0f);

    void initialize() override {}

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "constant rpm"; }

private:
    MotorSubsystem* motorSubsystem;
    float rpm, pulleyRatio;
};  // class ConstantRpmCommand

#endif  // CONSTANT_RPM_COMMAND_HPP_

}  // namespace aruwsrc::motor_tester
