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

#include "balancing_chassis_manual_drive_command.hpp"

namespace aruwsrc
{
namespace chassis
{

    BalancingChassisManualDriveCommand::BalancingChassisManualDriveCommand(
        // tap::Drivers* drivers,
        BalancingChassisSubsystem* chassis,
        control::ControlOperatorInterface& operatorInterface
    ) : chassis(chassis),
        operatorInterface(operatorInterface)
    {
        assert(chassis != nullptr);
        addSubsystemRequirement(chassis);
    }

    void BalancingChassisManualDriveCommand::initialize()
    {
        chassis->setDesiredOutput(0, 0);
    }

    void BalancingChassisManualDriveCommand::execute()
    {
        chassis->setDesiredOutput(operatorInterface.getChassisXInput(), operatorInterface.getChassisRInput());
        chassis->setDesiredHeight(operatorInterface.getChassisYInput());
    }

    void BalancingChassisManualDriveCommand::end(bool interrupted)
    {
        chassis->setDesiredOutput(0, 0);
    }

    bool isFinished()
    {
        return false;
    }
}
}