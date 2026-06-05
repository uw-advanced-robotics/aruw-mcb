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
    const float capacitance)
    : tap::can::CanRxListener(drivers, CAP_BANK_CAN_ID, canBus),
      capacitance(capacitance)
{
}

void CapacitorBank::processMessage(const modm::can::Message& message)
{
    switch (static_cast<MessageType>(message.data[0]))
    {
        case MessageType::STATUS:  // STATUS v2 (0x05)
            // data[1] = State, data[2:3] = cap current (i16 LE, mA), data[4:5] = cap voltage
            // (u16 LE, mV), data[6] = energy % (0..100), data[7] = available supply power (raw,
            // watts = raw * CASCADE_POWER_WATT_SCALE).
            this->state = static_cast<State>(message.data[1]);
            this->current =
                *reinterpret_cast<int16_t*>(const_cast<uint8_t*>(&message.data[2])) / 1000.0;
            this->voltage =
                *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[4])) / 1000.0;
            this->energyPercent = message.data[6] > 100 ? 100 : message.data[6];
            this->availableSupplyPower = message.data[7] * CASCADE_POWER_WATT_SCALE;
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
    uint16_t raw = watts / CASCADE_POWER_WATT_SCALE;
    return static_cast<uint8_t>(raw > 255 ? 255 : raw);
}

void CapacitorBank::sendCascadeCommand(CapCommandMode mode) const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::CASCADE_COMMAND;
    message.data[1] = static_cast<uint8_t>(mode);

    // I_bus / V_bus: the MCB's measured chassis bus current and voltage, relayed so the cap can
    // size its own charge/discharge power. Sign of I_bus follows the sensor (see interface notes).
    int16_t iBus = this->chassisCurrentSensor != nullptr
                       ? static_cast<int16_t>(this->chassisCurrentSensor->getCurrentMa())
                       : 0;
    uint16_t vBus = this->chassisVoltageSensor != nullptr
                        ? static_cast<uint16_t>(this->chassisVoltageSensor->getVoltageMv())
                        : 0;
    message.data[2] = static_cast<uint8_t>(iBus);
    message.data[3] = static_cast<uint8_t>(iBus >> 8);
    message.data[4] = static_cast<uint8_t>(vBus);
    message.data[5] = static_cast<uint8_t>(vBus >> 8);

    uint16_t refWatts = 0;
    if (this->drivers->refSerial.getRefSerialReceivingData())
    {
        refWatts = this->drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    }
    const uint8_t refPacked = packWatts(refWatts);
    message.data[6] = refPacked;  // ref_limit (charge ceiling / discharge regen ceiling)
    // P_target (byte 7) is derived by the cap firmware from its own current plus the relayed
    // I_bus/V_bus, so the MCB does not run the outer loop; mirror ref_limit as a safe placeholder.
    message.data[7] = refPacked;

    this->drivers->can.sendMessage(this->canBus, message);
}

const float HALF_SPRINT_POWER_BOOST = 0.5f;
float CapacitorBank::getMaximumOutputCurrent() const
{
    if (this->sprint == SprintMode::HALF_SPRINT)
    {
        return drivers->refSerial.getRobotData().chassis.powerConsumptionLimit /
               CAPACITOR_BANK_OUTPUT_VOLTAGE * (1.0f + HALF_SPRINT_POWER_BOOST);
    }

    float capacitorVoltage = this->getVoltage();
    float maxOutput = CAP_VOLTAGE_TO_MAX_OUT_CURRENT.interpolate(capacitorVoltage);

    return maxOutput;
}

}  // namespace aruwsrc::communication::can::cap_bank
