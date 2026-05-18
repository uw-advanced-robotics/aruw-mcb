/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DRONE_CONTROL_OPERATOR_INTERFACE_HPP_
#define DRONE_CONTROL_OPERATOR_INTERFACE_HPP_

#include "tap/drivers.hpp"

#include "aruwsrc/control/control_operator_interface.hpp"

namespace aruwsrc::drone
{
class DroneControlOperatorInterface : public control::ControlOperatorInterface
{
public:
    explicit DroneControlOperatorInterface(tap::Drivers *drivers)
        : ControlOperatorInterface(drivers)
    {
    }

    float getTurretPitchInput(uint8_t turretID) override;
};
}  // namespace aruwsrc::drone

#endif  // DRONE_CONTROL_OPERATOR_INTERFACE_HPP_
