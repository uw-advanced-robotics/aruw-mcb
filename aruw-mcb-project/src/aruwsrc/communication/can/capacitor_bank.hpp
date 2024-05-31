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

#ifndef CAPACITOR_BANK_HPP_
#define CAPACITOR_BANK_HPP_

#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"

namespace aruwsrc::communication::can::capbank
{
const uint16_t CAP_BANK_CAN_ID = 0x1EC;

enum MessageType
{
    START = 0x01,
    STOP = 0x02,
    STATUS = 0x04,
    SET_CHARGE_SPEED = 0x08,
};

enum State
{
    UNKNOWN = -1,
    RESET = 0,
    SAFE = 1,
    CHARGE = 2,
    CHARGE_DISCHARGE = 3,
    DISCHARGE = 4,
    BATTERY_OFF = 5,
    DISABLED = 6,
};

enum SprintMode
{
    REGULAR = 0,
    SPRINT = 1
};

class CapacitorBank : public tap::can::CanRxListener
{
public:
    CapacitorBank(tap::Drivers* drivers, tap::can::CanBus canBus, const float capacitance);

    void processMessage(const modm::can::Message& message) override;

    void initialize();

    void start() const;
    void stop() const;
    void setPowerLimit(uint16_t watts);

    int getAvailableEnergy() const { return this->availableEnergy; };
    float getCurrent() const { return this->current; };
    float getVoltage() const { return this->voltage; };
    int getPowerLimit() const { return this->powerLimit; };
    State getState() const { return this->state; };

    bool isEnabled() const {
        return 
            this->getState() == communication::can::capbank::State::SAFE ||
            this->getState() == communication::can::capbank::State::CHARGE ||
            this->getState() == communication::can::capbank::State::CHARGE_DISCHARGE ||
            this->getState() == communication::can::capbank::State::DISCHARGE ||
            this->getState() == communication::can::capbank::State::BATTERY_OFF;
    }

    bool isDisabled() const {
        return 
            this->getState() == communication::can::capbank::State::RESET ||
            this->getState() == communication::can::capbank::State::DISABLED;
    }

    void setSprinting(SprintMode sprint) { this->sprint = sprint; };
    SprintMode getSprinting() const { return this->sprint; };

private:
    const float capacitance;

    uint16_t powerLimit = 0;

    float availableEnergy = 0;
    float current = 0;
    float voltage = 0;
    State state = State::UNKNOWN;

    bool started = false;  // Set to true once any message from the cap bank is received

    SprintMode sprint = SprintMode::REGULAR;
};
}  // namespace aruwsrc::communication::can::capbank

#endif  // CAPACITOR_BANK_HPP_