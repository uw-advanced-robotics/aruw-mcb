/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef WRIST_CONTROLLER_COMMAND_HPP_
#define WRIST_CONTROLLER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/engineer-joint/joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

namespace aruwsrc::engineer::wrist
{
using namespace aruwsrc::control::engineer;
/**
 * A command that moves the engineer wrist based on the control operator interface.
 */
class WristControllerCommand : public tap::control::Command
{
public:
    WristControllerCommand(
        aruwsrc::control::JointSubsystem &roll,
        WristSubsystem &wrist,
        EngineerControlOperatorInterface *operatorInterface,
        float rollScalingFactor,
        float pitchScalingFactor,
        float yawScalingFactor);

    void initialize() override;

    /**
     * Updates the PID controller and applies the output to the joint subsystem.
     */
    void execute() override;

    void end(bool) override{};

    const char *getName() const override { return "Engineer Wrist Controller Command"; };

    virtual bool isFinished() const override { return false; }

private:
    aruwsrc::control::JointSubsystem &roll;
    WristSubsystem &wrist;

    EngineerControlOperatorInterface *operatorInterface;
    const float rollScalingFactor;
    const float pitchScalingFactor;
    const float yawScalingFactor;
};

}  // namespace aruwsrc::engineer::wrist

#endif  // WRIST_CONTROLLER_COMMAND_HPP_