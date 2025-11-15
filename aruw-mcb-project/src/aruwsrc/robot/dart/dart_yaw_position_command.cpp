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

#include "dart_yaw_position_command.hpp"

#include "tap/control/command.hpp"
#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"
#include "tap/drivers.hpp"
#include "aruwsrc/robot/dart/dart_constants.hpp"

using namespace aruwsrc::control::joint::homing;
using tap::communication::serial::Remote;
namespace aruwsrc::robot::dart
{
    DartYawPositionCommand::DartYawPositionCommand(
        tap::Drivers *drivers, 
        TriggerHomedJointSubsystem *subsystem,
        float setpointDegrees
    ) : drivers(drivers), subsystem(subsystem), setpointDegrees(setpointDegrees){
        addSubsystemRequirement(subsystem);
    }

    void DartYawPositionCommand::initialize(){
        float setpointRotations = (DART_LAUNCHER_YAW_RADIAL_LENGTH * tan(setpointDegrees*PI/180))/0.002;
        subsystem->setSetpoint(setpointRotations);
    }

    void DartYawPositionCommand::execute(){}

    void DartYawPositionCommand::end(bool interrupted){}

    bool DartYawPositionCommand::isFinished() const {
        return true;
    }

}