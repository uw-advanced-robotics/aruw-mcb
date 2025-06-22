/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CHASSIS_CONTROLLER_INTERFACE_HPP_
#define CHASSIS_CONTROLLER_INTERFACE_HPP_

#include "aruwsrc/robot/balstd/balstd_control_operator_interface.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_chassis_output.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_chassis_state.hpp"

namespace aruwsrc::balstd::chassis::controllers
{
class BalstdChassisControllerInterface
{
public:
    BalstdChassisControllerInterface(const BalstdControlOperatorInterface& controlOperatorInterface)
        : controlOperatorInterface(controlOperatorInterface)
    {
    }
    virtual ~BalstdChassisControllerInterface() = default;

    virtual void initialize(const BalstdChassisState&) {}

    virtual BalstdChassisOutput runController(const BalstdChassisState& state, float dt) = 0;

protected:
    const BalstdControlOperatorInterface& controlOperatorInterface;
};
}  // namespace aruwsrc::balstd::chassis::controllers

#endif  // CHASSIS_CONTROLLER_INTERFACE_HPP_