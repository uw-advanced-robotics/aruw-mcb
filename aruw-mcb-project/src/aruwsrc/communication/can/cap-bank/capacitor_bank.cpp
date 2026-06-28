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

#include "capacitor_bank.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::communication::can::cap_bank
{
CapacitorBank::CapacitorBank(
    tap::Drivers* drivers,
    tap::can::CanBus canBus,
    const float capacitance,
    const int maxAvailablePower)
    : tap::can::CanRxListener(drivers, CAP_BANK_CAN_ID, canBus),
      capacitance(capacitance),
      maxAvailablePower(maxAvailablePower)
{
}

void CapacitorBank::processMessage(const modm::can::Message& message)
{
    switch (static_cast<MessageType>(message.data[0]))
    {
        case MessageType::STATUS:  // STATUS v2 (0x05)
            // data[1] = State in bits 0-6, latched error flag in bit 7; data[2:3] = cap current
            // (i16 LE, mA), data[4:5] = cap voltage (u16 LE, mV), data[6] = energy % (0..100),
            // data[7] = available supply power (raw, watts = raw * CAP_POWER_WATT_SCALE).
            this->state = static_cast<State>(message.data[1] & 0x7F);
            this->errorFlag = (message.data[1] & 0x80) != 0;
            this->current =
                *reinterpret_cast<int16_t*>(const_cast<uint8_t*>(&message.data[2])) / 1000.0;
            this->voltage =
                *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[4])) / 1000.0;
            this->energyPercent = message.data[6] > 100 ? 100 : message.data[6];
            this->availableSupplyPower = message.data[7] * CAP_POWER_WATT_SCALE;
            this->availableEnergy = tap::algorithms::limitVal(
                1.0 / 2.0 * this->capacitance *
                    (powf(this->voltage, 2) - powf(CAPACITOR_BANK_MIN_VOLTAGE, 2)),
                0.0,
                2000.0);

            this->heartbeat.restart(80);
            break;
        default:
            // Ignore unknown / legacy message tags.
            break;
    }
}

void CapacitorBank::initialize()
{
    this->attachSelfToRxHandler();
    this->heartbeat.restart(0);
}

uint8_t CapacitorBank::packWatts(uint16_t watts)
{
    uint16_t raw = watts / CAP_POWER_WATT_SCALE;
    return static_cast<uint8_t>(raw > 255 ? 255 : raw);
}

void CapacitorBank::sendCapCommand(CapCommandMode mode) const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = static_cast<uint8_t>(MessageType::CAP_COMMAND);
    message.data[1] = static_cast<uint8_t>(mode);
    // Bytes 2-5: reserved (battery I/V from CAN 0x1C5 on the cap bank).
    message.data[2] = 0;
    message.data[3] = 0;
    message.data[4] = 0;
    message.data[5] = 0;

    uint16_t refWatts = 0;
    if (this->drivers->refSerial.getRefSerialReceivingData())
    {
        refWatts = this->drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    }
    message.data[6] = packWatts(refWatts);
    message.data[7] = 0;

    this->drivers->can.sendMessage(this->canBus, message);
}

float CapacitorBank::getMaximumOutputCurrent() const
{
    // Single source of truth: cap-bank firmware's live "available supply power" report
    // (STATUS_V2 byte 7, watts). Convert to amps at nominal chassis voltage for the
    // legacy amps-domain callers (CapBankPowerLimiter, holonomic_chassis_subsystem).
    // The static voltage→current LUT this used to consult is gone — the firmware knows
    // its own SOC, fault state, and regen ceiling, so it computes a better answer.
    // Read through the capped getter so maxAvailablePower limits the power limiter too.
    return static_cast<float>(this->getAvailableSupplyPower()) / CAPACITOR_BANK_OUTPUT_VOLTAGE;
}

}  // namespace aruwsrc::communication::can::cap_bank
