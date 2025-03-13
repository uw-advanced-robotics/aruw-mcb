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

#ifndef BALSTD_CONTROL_OPERATOR_INTERFACE_HPP_
#define BALSTD_CONTROL_OPERATOR_INTERFACE_HPP_

#include "tap/drivers.hpp"

#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::control::balstd
{
class BalstdControlOperatorInterface : public ControlOperatorInterface
{
public:
    BalstdControlOperatorInterface(tap::Drivers *drivers) : ControlOperatorInterface(drivers) {}

    // ====================
    // testing input modes
    // ====================
    /**
     * @return The value used for testing leg VMC movement forward/backward
     */
    mockable float getLegXForce() const;

    /**
     * @return The value used for testing leg VMC up/down movement
     */
    mockable float getLegYForce() const;

    /**
     * @return The value used for testing leg wheel torque
     */
    mockable float getWheelTorque() const;

private:
    static constexpr float LEG_FORCE_SCALAR = 10.0f;
    static constexpr float WHEEL_TORQUE_SCALAR = 10.0f;
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_CONTROL_OPERATOR_INTERFACE_HPP__
