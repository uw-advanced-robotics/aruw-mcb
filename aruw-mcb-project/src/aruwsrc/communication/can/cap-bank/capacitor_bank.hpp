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

namespace aruwsrc::communication::can::cap_bank
{
static constexpr float CAPACITOR_BANK_OUTPUT_VOLTAGE = 24.0f;
static constexpr float CAPACITOR_BANK_EFFICIENCY = 0.9f;
/// Cap-bank firmware refuses to discharge below this voltage (CAP_DISCHARGE_STOP_V in main.rs).
/// Used by availableEnergy = 0.5·C·(V² - V_min²) so MCB UI never reports unusable energy.
static constexpr float CAPACITOR_BANK_MIN_VOLTAGE = 10.0f;

static constexpr uint16_t CAP_BANK_CAN_ID = 0x1EC;

/**
 * Referee power limit packed in CAP_COMMAND byte 6: watts = raw * scale.
 * MUST match CAP_POWER_WATT_SCALE in the cap bank firmware (can_messages.rs).
 */
static constexpr uint16_t CAP_POWER_WATT_SCALE = 4;

/**
 * Cap bank protocol v2 message tags (CAP_BANK_CAN_ID, 8-byte classic frames).
 */
enum MessageType
{
    STATUS = 0x05,      // CAP -> MCB: state + telemetry (see processMessage)
    CAP_COMMAND = 0x28,  // MCB -> CAP: mode + referee ref_limit only
};

/**
 * Mode commanded to the cap bank in CAP_COMMAND byte 1. Wire values 0..3 must match the
 * firmware's CapCommandMode.
 */
enum CapCommandMode
{
    OFF = 0,
    IDLE = 1,
    CHARGE = 2,
    DISCHARGE = 3,
};

/**
 * Cap bank state reported in STATUS byte 1. Wire values 0..4 must match the firmware's CanState.
 * UNKNOWN is an MCB-only sentinel meaning "no STATUS received yet"; it never appears on the bus.
 */
enum State
{
    UNKNOWN = -1,
    RESET = 0,
    SAFE = 1,
    REGULATING = 2,
    BATTERY_OFF = 3,
    FAILURE = 4,
};

enum SprintMode
{
    NO_SPRINT = 0,
    SPRINT = 1,
};

class CapacitorBank : public tap::can::CanRxListener
{
public:
    CapacitorBank(tap::Drivers* drivers, tap::can::CanBus canBus, const float capacitance);

    void processMessage(const modm::can::Message& message) override;

    mockable void initialize();

    /**
     * Sends CAP_COMMAND (0x28): byte 1 = mode, byte 6 = referee ref_limit (watts/4).
     * Bytes 2-5 and 7 are zero. Cap bank reads battery I/V from CAN 0x1C5 directly.
     */
    mockable void sendCapCommand(CapCommandMode mode) const;

public:
    int getAvailableEnergy() const { return this->availableEnergy; };
    float getCurrent() const { return this->current; };
    float getVoltage() const { return this->voltage; };
    /** Usable state-of-charge, 0..100 %, as reported by the cap bank. */
    uint8_t getEnergyPercent() const { return this->energyPercent; };
    /** Instantaneous supply-power headroom the cap reports, in watts. */
    int getAvailableSupplyPower() const { return this->availableSupplyPower; };
    State getState() const { return this->state; };

    bool isEnabled() const
    {
        return this->getState() == State::SAFE || this->getState() == State::REGULATING;
    }

    bool isDisabled() const { return this->getState() == State::RESET; }

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
    /** Packs a watt value into byte 6 encoding (watts / CAP_POWER_WATT_SCALE). */
    static uint8_t packWatts(uint16_t watts);

    const float capacitance;

    int availableSupplyPower = 0;  // watts, from STATUS byte 7
    uint8_t energyPercent = 0;     // 0..100 %, from STATUS byte 6

    float availableEnergy = 0;
    float current = 0;
    float voltage = 0;
    State state = State::UNKNOWN;

    SprintMode sprint = SprintMode::NO_SPRINT;

    tap::arch::MilliTimeout heartbeat;
};
}  // namespace aruwsrc::communication::can::cap_bank

#endif  // CAPACITOR_BANK_HPP_
