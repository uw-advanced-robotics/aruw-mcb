/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALANCING_CHASSIS_MANUAL_DRIVE_COMMAND_HPP_
#define BALANCING_CHASSIS_MANUAL_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "balancing_chassis_subsystem.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc
{
namespace chassis
{

class BalancingChassisManualDriveCommand : public tap::control::Command
{
public:
    BalancingChassisManualDriveCommand(
        // tap::Drivers* drivers,
        BalancingChassisSubsystem* chassis,
        aruwsrc::control::ControlOperatorInterface& operatorInterface);

    const char* getName() const override { return "Balancing Chassis Manual Drive Command"; }
    
    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    BalancingChassisSubsystem* chassis;
    control::ControlOperatorInterface& operatorInterface;

};  // class BalancingManualDriveCommand

}   // namespace chassis

}   // namespace aruwsrc


#endif  // BALANCING_CHASSIS_MANUAL_DRIVE_HPP_