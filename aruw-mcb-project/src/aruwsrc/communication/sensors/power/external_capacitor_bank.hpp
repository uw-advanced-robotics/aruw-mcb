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

#ifndef EXTERNAL_CAPACITOR_BANK_HPP_
#define EXTERNAL_CAPACITOR_BANK_HPP_

#include "tap/drivers.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/control/chassis/power_limiter.hpp"

#include "modm/architecture/interface/can_message.hpp"

/**
 * Can communication protocol:
 */
namespace aruwsrc::communication::sensors::power
{

const uint16_t CAP_BANK_CAN_ID = 0x1EC;

enum MessageType {
    START = 0x01,
    STOP = 0x02,
    STATUS = 0x04,
    SET_CHARGE_SPEED = 0x08
};

enum Status
{
    UNKNOWN = -1,
    RESET,
    CHARGE,
    DISCHARGE,
    FAULT
};

class ExternalCapacitorBank
    : public tap::can::CanRxListener
{

public:
    ExternalCapacitorBank(tap::Drivers* drivers, tap::can::CanBus canBus, tap::control::chassis::PowerLimiter& powerLimiter, const float capacitance);

    int getAvailableEnergy() const { return availableEnergy; };

    void processMessage(const modm::can::Message& message) override;

    void start() const;
    void stop() const;
    void setPowerLimit(float watts); 

private:
    tap::control::chassis::PowerLimiter& powerLimiter;
    const float capacitance;

    float availableEnergy;
    float current;
    float voltage;
    Status status = Status::UNKNOWN;
};
}  // namespace aruwsrc::communication::sensors::power

#endif  // EXTERNAL_CAPACITOR_BANK_HPP_
