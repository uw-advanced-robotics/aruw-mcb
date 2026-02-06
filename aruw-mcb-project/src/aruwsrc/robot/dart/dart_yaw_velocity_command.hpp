/*
 * Copyright (c) 2025-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef DART_YAW_VELOCITY_COMMAND_HPP_
#define DART_YAW_VELOCITY_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"

using namespace aruwsrc::control::joint::homing;
using tap::communication::serial::Remote;

namespace aruwsrc::robot::dart
{
class DartYawVelocityCommand : public tap::control::Command
{
public:
    DartYawVelocityCommand(
        tap::Drivers* drivers,
        TriggerHomedJointSubsystem& subsystem,
        Remote::Channel channel);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "COMMAND_NAME"; }

private:
    tap::Drivers* drivers;
    TriggerHomedJointSubsystem& subsystem;
    Remote::Channel channel;
};  // class CLASS_NAME

}  // namespace aruwsrc::robot::dart
#endif  // DART_YAW_PID_COMMAND_HPP_
