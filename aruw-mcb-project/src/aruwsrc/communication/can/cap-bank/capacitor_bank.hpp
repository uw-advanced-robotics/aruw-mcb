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
#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/communication/sensors/voltage/voltage_sensor_interface.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::communication::can::cap_bank
{
static constexpr float CAPACITOR_BANK_OUTPUT_VOLTAGE = 24.0f;
static constexpr float CAPACITOR_BANK_EFFICIENCY = 0.9f;
static constexpr float CAPACITOR_BANK_MIN_VOLTAGE = 8.0f;

static constexpr uint16_t CAP_BANK_CAN_ID = 0x1EC;

/**
 * Power fields in the cascade protocol are packed into a single u8: watts = raw * scale.
 * MUST match CASCADE_POWER_WATT_SCALE in the cap bank firmware (can_messages.rs).
 */
static constexpr uint16_t CASCADE_POWER_WATT_SCALE = 4;

/**
 * Cascade protocol v2 message tags (CAP_BANK_CAN_ID, 8-byte classic frames). Only these two are
 * used; legacy tags (0x01..0x20) are gone.
 */
enum MessageType
{
    STATUS = 0x05,           // CAP -> MCB: state + telemetry (see processMessage)
    CASCADE_COMMAND = 0x28,  // MCB -> CAP: mode + bus current/voltage + power limits
};

/**
 * Mode commanded to the cap bank in CASCADE_COMMAND byte 1. Wire values 0..3 must match the
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

    /**
     * Sends a CASCADE_COMMAND (0x28) for the given mode. Bus current/voltage are read from the
     * chassis sensors (see setChassisSensors); the referee power limit is read from RefSerial.
     */
    mockable void sendCascadeCommand(CapCommandMode mode) const;

    /**
     * Provide the MCB's chassis current/voltage sensor; these readings are relayed to the cap bank
     * (as I_bus / V_bus) in every CASCADE_COMMAND so the cap can size its own power. Pass nullptr to
     * relay zero.
     */
    void setChassisSensors(
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        tap::communication::sensors::voltage::VoltageSensorInterface* voltageSensor)
    {
        this->chassisCurrentSensor = currentSensor;
        this->chassisVoltageSensor = voltageSensor;
    }

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
    /** Packs a watt value into the u8 cascade encoding (watts / CASCADE_POWER_WATT_SCALE). */
    static uint8_t packWatts(uint16_t watts);

    const float capacitance;

    int availableSupplyPower = 0;  // watts, from STATUS byte 7
    uint8_t energyPercent = 0;     // 0..100 %, from STATUS byte 6

    float availableEnergy = 0;
    float current = 0;
    float voltage = 0;
    State state = State::UNKNOWN;

    SprintMode sprint = SprintMode::NO_SPRINT;

    tap::communication::sensors::current::CurrentSensorInterface* chassisCurrentSensor = nullptr;
    tap::communication::sensors::voltage::VoltageSensorInterface* chassisVoltageSensor = nullptr;

    tap::arch::MilliTimeout heartbeat;
};
}  // namespace aruwsrc::communication::can::cap_bank

#endif  // CAPACITOR_BANK_HPP_
