/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef FIXED_PATH_AUTO_NAC_COMMAND_HPP_
#define FIXED_PATH_AUTO_NAC_COMMAND_HPP_

#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/control/chassis/auto_nav_command.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "tap/algorithms/transforms/position.hpp"

namespace aruwsrc::control::chassis
{
class FixedPathAutoNavCommand : public AutoNavCommand
{
public:
    FixedPathAutoNavCommand(    
        const tap::Drivers& drivers,
        chassis::HolonomicChassisSubsystem& chassis,
        aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
        const Position* pathPoints,
        float desiredSpeed,
        bool autoNavOnlyInGame = false,
        bool beybladeEnabled = true);

    bool isFinished() const override;

    const char* getName() const override { return "fixed path autonav command"; }

private:
    aruwsrc::control::chassis::ChassisAutoNavController& autoNavController;
    aruwsrc::algorithms::AutoNavPath path;

};  // class FixedPathAutoNavCommand

}  // namespace aruwsrc::control::chassis

#endif  // FIXED_PATH_AUTO_NAC_COMMAND_HPP_