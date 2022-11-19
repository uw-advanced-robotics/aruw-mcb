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

#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/power/external_power_source_interface.hpp"

#include "aruwsrc/drivers.hpp"

/**
 * Can communication protocol:
 */
namespace aruwsrc::communication::sensors::power
{

const uint16_t CAP_BANK_CAN_ID = 0x1EC;

class ExternalCapacitorBank
    : public tap::communication::sensors::power::ExternalPowerSourceInterface,
      public tap::can::CanRxListener
{
    enum Status
    {
        UNKNOWN = -1,
        RESET,
        CHARGE,
        DISCHARGE,
        FAULT
    };

public:
    ExternalCapacitorBank(Drivers* drivers, tap::can::CanBus canBus, const float capacitance);

    int getAvailablePower() { return availablePower; };

    int consumeAvailablePower(int consumed) override;

    void processMessage(const modm::can::Message& message) override;

    void requestP() const { this->requestPIValue(true); };
    void requestI() const { this->requestPIValue(false); };
    void setP(float p) const { this->setPIValue(p, true); };
    void setI(float i) const { this->setPIValue(i, false); };
    void start() const;
    void stop() const;

private:
    void requestPIValue(bool isP) const;
    void setPIValue(float value, bool isP) const;

    const float capacitance;
    float availablePower;
    float current;
    float voltage;
    Status status = Status::UNKNOWN;

    float p, i;
};
}  // namespace aruwsrc::communication::sensors::power

#endif  // EXTERNAL_CAPACITOR_BANK_HPP_
