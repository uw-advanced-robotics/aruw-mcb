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

#ifndef RMUL_STATE_MACHINE_HPP_
#define RMUL_STATE_MACHINE_HPP_

#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"

/**
 * Field:
 * Not to scale or accurate
 * +------------------------------------------+
 * |                          | Blue Loading  |
 * |                          |_______________|
 * |                                          |
 * |                    ______________________|
 * |                    |                     |
 * |                    |  Elevated platform  |
 * |          --------------------------      |
 * |                                   |      |
 * |                                   |______|
 * |                                          |
 * |              ------------                |
 * |             |            |               |
 * |-------      |   Center   |        -------|
 * |             |            |               |
 * |              ------------                |
 * |                                          |
 * |______                                    |
 * |      |                                   |
 * |      |                                   |
 * |      --------------------------          |
 * |Elevated platform        |                |
 * |_________________________|                |
 * |                                          |
 * |___________                               |
 * | Red      |                               |
 * | Loading  |                               |
 * +------------------------------------------+
 *
 */

namespace aruwsrc::algorithms::state_machine
{
using namespace tap::communication::serial;
using namespace aruwsrc::chassis;
using namespace aruwsrc::algorithms;
class RMULStateMachine
{
    RMULStateMachine(RefSerial& refSerial, ChassisAutoNavController& autoNavController)
        : refSerial(refSerial),
          autoNavController(autoNavController)
    {
    }

private:
    RefSerial& refSerial;
    ChassisAutoNavController& autoNavController;

    AutoNavPath path;
    bool isHealing = false;

    // Threshold at which the robot goes to heal due to low health
    static constexpr int HEALING_THRESHOLD = 200;

    // Threshold at which the robot goes back to fight having healed
    static constexpr int ATTACKING_THRESHOLD = 550;
};
}  // namespace aruwsrc::algorithms::state_machine

#endif  // RMUL_STATE_MACHINE_HPP_
