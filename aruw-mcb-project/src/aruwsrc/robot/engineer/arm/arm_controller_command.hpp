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

#ifndef ARM_CONTROLLER_COMMAND_HPP_
#define ARM_CONTROLLER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/robot/engineer/arm/arm_extension_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/arm_lift_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/wrist_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"

namespace aruwsrc::control::engineer
{
using namespace aruwsrc::engineer;
/**
 * A command that moves the engineer arm based on the control operator interface.
 */
class ArmControllerCommand : public tap::control::Command
{
public:
    ArmControllerCommand(
        ArmLiftSubsystem &lift,
        ArmExtensionSubsystem &extension,
        JointSubsystem &wristRoll,
        WristSubsystem &wrist,
        EngineerControlOperatorInterface *operatorInterface,
        float liftScalingFactor,
        float extensionScalingFactor,
        float wristRollScalingFactor,
        float wristPitchScalingFactor,
        float wristYawScalingFactor);

    void initialize() override;

    /**
     * Updates the PID controller and applies the output to the joint subsystem.
     */
    void execute() override;

    void end(bool) override{};

    const char *getName() const override { return "Engineer Arm Controller Command"; };

    virtual bool isFinished() const override { return false; }

private:
    ArmLiftSubsystem &lift;
    ArmExtensionSubsystem &extension;
    JointSubsystem &wristRoll;
    WristSubsystem &wrist;

    EngineerControlOperatorInterface *operatorInterface;
    const float liftScalingFactor;
    const float extensionScalingFactor;
    const float wristRollScalingFactor;
    const float wristPitchScalingFactor;
    const float wristYawScalingFactor;
};

}  // namespace aruwsrc::control::engineer

#endif  // ARM_CONTROLLER_COMMAND_HPP_