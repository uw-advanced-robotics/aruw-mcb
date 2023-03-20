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

#ifndef PHYSICAL_CAN_BUS_HPP_
#define PHYSICAL_CAN_BUS_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "can_bus.hpp" 

namespace aruwsrc::can
{
class PhysicalCanBus : public CanBus
{
public:
    PhysicalCanBus(tap::Drivers* drivers);

    void initialize() override;

    bool isMessageAvailable(tap::can::CanBus canbus) override;

    bool getMessage(tap::can::CanBus canbus, modm::can::Message* message) override;

    bool isReadyToSend(tap::can::CanBus canbus) override;

    bool sendMessage(tap::can::CanBus canbus, const modm::can::Message& message) override;

private:
    tap::Drivers* drivers;
};

}  // namespace aruwsrc::can

#endif