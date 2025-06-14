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

#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::can::capbank
{
static constexpr float CAPACITOR_BANK_OUTPUT_VOLTAGE = 24.0f;
static constexpr float CAPACITOR_BANK_EFFICIENCY = 0.9f;
static constexpr float CAPACITOR_BANK_MIN_VOLTAGE = 8.0f;

static constexpr uint16_t CAP_BANK_CAN_ID = 0x1EC;

enum MessageType
{
    START = 0x01,
    STOP = 0x02,
    STATUS = 0x04,
    SET_CHARGE_SPEED = 0x08,
    PING = 0x10,
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
    NO_SPRINT = 0,
    HALF_SPRINT = 1,
    SPRINT = 2
};

static constexpr modm::Pair<float, float> CAP_VOLTAGE_TO_MAX_OUT_CURRENT_LUT[] = {
    {7.0, 2.5},
    {9.0, 4.0},
    {11.0, 6.0},
    {14.0, 7.0},
    {17.0, 10.0},
    {20.0, 12.0},
    {23.0, 12.0},
    {26.0, 12.0},
    {29.0, 12.0}};

static modm::interpolation::Linear<modm::Pair<float, float>> CAP_VOLTAGE_TO_MAX_OUT_CURRENT(
    CAP_VOLTAGE_TO_MAX_OUT_CURRENT_LUT,
    MODM_ARRAY_SIZE(CAP_VOLTAGE_TO_MAX_OUT_CURRENT_LUT));

class CapacitorBank : public tap::can::CanRxListener
{
public:
    CapacitorBank(tap::Drivers* drivers, tap::can::CanBus canBus, const float capacitance);

    void processMessage(const modm::can::Message& message) override;

    mockable void initialize();

    mockable void start() const;
    mockable void stop() const;
    mockable void ping() const;
    mockable void setPowerLimit(uint16_t watts);

public:
    int getAvailableEnergy() const { return this->availableEnergy; };
    float getCurrent() const { return this->current; };
    float getVoltage() const { return this->voltage; };
    int getPowerLimit() const { return this->powerLimit; };
    State getState() const { return this->state; };

    bool isEnabled() const
    {
        return this->getState() == State::SAFE || this->getState() == State::CHARGE ||
               this->getState() == State::CHARGE_DISCHARGE ||
               this->getState() == State::DISCHARGE || this->getState() == State::BATTERY_OFF;
    }

    bool isDisabled() const
    {
        return this->getState() == State::RESET || this->getState() == State::DISABLED;
    }

    bool isOnline() const
    {
        return !(this->getState() == State::UNKNOWN || this->heartbeat.isExpired());
    }

    void setSprinting(SprintMode sprint) { this->sprint = sprint; };
    bool isSprinting() const { return this->sprint != SprintMode::NO_SPRINT; };

    float getMaximumOutputCurrent() const;

#ifndef ENV_UNIT_TESTS
private:
#endif
    const float capacitance;

    uint16_t powerLimit = 0;

    float availableEnergy = 0;
    float current = 0;
    float voltage = 0;
    State state = State::UNKNOWN;

    SprintMode sprint = SprintMode::NO_SPRINT;

    tap::arch::MilliTimeout heartbeat;
};
}  // namespace aruwsrc::can::capbank

#endif  // CAPACITOR_BANK_HPP_
