/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DRONE_SUBSYSTEM_HPP_
#define DRONE_SUBSYSTEM_HPP_

#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/control/subsystem.hpp"
#include "aruwsrc/control/drone/drone_uart_parser.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace drone
{

class DroneSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
    DroneSubsystem(Drivers* drivers);

    void refresh() override;

    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override;

private:

    DroneUartParser parser;
    Telemetry currentTelemetryData;

};

}  // namespace drone

}  // namespace aruwsrc

#endif