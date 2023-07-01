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

#include "external_capacitor_bank.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::communication::sensors::power
{
ExternalCapacitorBank::ExternalCapacitorBank(
    tap::Drivers* drivers,
    tap::can::CanBus canBus,
    const float capacitance)
    : tap::can::CanRxListener(drivers, CAP_BANK_CAN_ID, canBus),
      capacitance(capacitance){};

void ExternalCapacitorBank::processMessage(const modm::can::Message& message)
{
    switch (static_cast<MessageType>(message.data[7]))
    {
        case MessageType::STATUS:  // Update message
            this->status = static_cast<Status>(message.data[5]);
            this->powerLimit = message.data[4];
            this->voltage =
                *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[2])) / 1000.0;
            this->current =
                *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[0])) / 1000.0;
            this->availableEnergy = tap::algorithms::limitVal(
                1.0 / 2.0 * this->capacitance * (powf(this->voltage, 2) - powf(9, 2)),
                0.0,
                2000.0);
            break;
        default:
            // Ignore unknown message IDs
            break;
    }

    if (drivers->refSerial.getRefSerialReceivingData())
    {
        uint16_t powerLimit = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
        if (powerLimit != this->powerLimit)
        {
            this->setPowerLimit(powerLimit);
        }

        this->setBatteryVoltage(drivers->refSerial.getRobotData().chassis.volt);

        {
            if (!this->started || this->status == Status::RESET)
            {
                this->start();
                this->started = true;
            }
        }
    }
}

void ExternalCapacitorBank::initialize()
{
    this->attachSelfToRxHandler();
    this->started = false;
}

void ExternalCapacitorBank::start() const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = MessageType::START;
    this->drivers->can.sendMessage(this->canBus, message);
}

void ExternalCapacitorBank::stop() const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = MessageType::STOP;
    this->drivers->can.sendMessage(this->canBus, message);
}

void ExternalCapacitorBank::discharge() const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = MessageType::DISCHARGE;
    this->drivers->can.sendMessage(this->canBus, message);
}

void ExternalCapacitorBank::setPowerLimit(uint16_t watts)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = MessageType::SET_CHARGE_SPEED;
    message.data[0] = watts;
    message.data[1] = watts >> 8;  // Should always be zero or we are drawing 250+ watts.
    this->drivers->can.sendMessage(this->canBus, message);
}

void ExternalCapacitorBank::setBatteryVoltage(uint16_t milliVolts)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = MessageType::SET_BATTERY_VOLTAGE;
    message.data[0] = milliVolts;
    message.data[1] = milliVolts >> 8;
    this->drivers->can.sendMessage(this->canBus, message);
}

void ExternalCapacitorBank::setSprintModifier(float newSprintModifier)
{
    sprintModifier = tap::algorithms::limitVal(newSprintModifier, 0.0f, 1.0f);
}
}  // namespace aruwsrc::communication::sensors::power
