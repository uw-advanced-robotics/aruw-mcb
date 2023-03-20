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

#include "physical_can_bus.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::can
{

PhysicalCanBus::PhysicalCanBus(tap::Drivers* drivers) : drivers(drivers) {}

void PhysicalCanBus::initialize() { drivers->can.initialize(); }

bool PhysicalCanBus::isMessageAvailable(tap::can::CanBus canbus)
{
    return drivers->can.isMessageAvailable(canbus);
}

bool PhysicalCanBus::getMessage(tap::can::CanBus canbus, modm::can::Message* message)
{
    drivers->can.getMessage(canbus, message);
}

bool PhysicalCanBus::isReadyToSend(tap::can::CanBus canbus) { drivers->can.isReadyToSend(canbus); }

bool PhysicalCanBus::sendMessage(tap::can::CanBus canbus, const modm::can::Message& message)
{
    drivers->can.sendMessage(canbus, message);
}

}  // namespace aruwsrc::can