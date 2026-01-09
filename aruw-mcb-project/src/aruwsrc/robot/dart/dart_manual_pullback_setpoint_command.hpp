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
#ifndef DART_MANUAL_PULLBACK_SETPOINT_COMMAND_HPP_
#define DART_MANUAL_PULLBACK_SETPOINT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"
#include "aruwsrc/robot/dart/dart_control_operator_interface.hpp"

namespace aruwsrc::robot::dart

{
class DartManualPullbackSetpointCommand : public tap::control::Command
{
public:
    DartManualPullbackSetpointCommand(
        aruwsrc::control::joint::homing::TriggerHomedJointSubsystem& dartSystem,
        float moveSpeed,
        aruwsrc::control::dart::DartControlOperatorInterface* controlOperatorInterface);
    void initialize() override;
    void execute() override;
    void end(bool) override {}

    bool isFinished() const override;

    const char* getName() const override { return "DART MANUAL PULLBACK SETPOINT"; }

private:
    aruwsrc::control::joint::homing::TriggerHomedJointSubsystem& dartSystem;
    float moveSpeed;
    aruwsrc::control::dart::DartControlOperatorInterface* controlOperatorInterface;  // NOLINT
};
}  // namespace aruwsrc::robot::dart

#endif  // DART_MANUAL_PULLBACK_SETPOINT_COMMAND