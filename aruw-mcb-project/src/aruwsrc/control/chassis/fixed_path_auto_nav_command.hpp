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

#ifndef FIXED_PATH_AUTO_NAV_COMMAND_HPP_
#define FIXED_PATH_AUTO_NAV_COMMAND_HPP_

#include <iterator>

#include "tap/algorithms/transforms/position.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/control/chassis/auto_nav_command.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::control::chassis
{
class FixedPathAutoNavCommand : public AutoNavCommand
{
public:
    template <typename Container>
    FixedPathAutoNavCommand(
        const tap::Drivers& drivers,
        chassis::HolonomicChassisSubsystem& chassis,
        aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
        const Container& pathPoints,
        float desiredSpeed,
        bool autoNavOnlyInGame = false,
        bool beybladeEnabled = true)
        : AutoNavCommand(
              drivers,
              chassis,
              autoNavController,
              autoNavOnlyInGame,
              beybladeEnabled,
              true),
          autoNavController(autoNavController),
          path(pathPoints),
          desiredSpeed(desiredSpeed)
    {
    }

    bool isFinished() const override;

    void initialize() override;

    const char* getName() const override { return "fixed path autonav command"; }

private:
    aruwsrc::control::chassis::ChassisAutoNavController& autoNavController;
    aruwsrc::algorithms::AutoNavPath path;
    float desiredSpeed;
};  // class FixedPathAutoNavCommand

}  // namespace aruwsrc::control::chassis

#endif  // FIXED_PATH_AUTO_NAC_COMMAND_HPP_