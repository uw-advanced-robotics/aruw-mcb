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

namespace aruwsrc::communication::can::capbank
{
CapacitorBank::CapacitorBank(
    tap::Drivers* drivers,
    tap::can::CanBus canBus,
    const float capacitance)
    : tap::can::CanRxListener(drivers, CAP_BANK_CAN_ID, canBus),
      capacitance(capacitance){};

void CapacitorBank::processMessage(const modm::can::Message& message)
{
    switch (static_cast<MessageType>(message.data[0]))
    {
        case MessageType::STATUS:  // Update message
            this->state = static_cast<State>(message.data[1]);
            this->current =
                *reinterpret_cast<int16_t*>(const_cast<uint8_t*>(&message.data[2])) / 1000.0;
            this->voltage =
                *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[4])) / 1000.0;
            this->powerLimit = message.data[6];
            this->availableEnergy = tap::algorithms::limitVal(
                1.0 / 2.0 * this->capacitance * (powf(this->voltage, 2) - powf(8, 2)),
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

        {
            if (!this->started && this->state == State::RESET)
            {
                this->start();
                this->started = true;
            }
        }
    }
}

void CapacitorBank::initialize()
{
    this->attachSelfToRxHandler();
    this->started = false;
}

void CapacitorBank::start() const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::START;
    this->drivers->can.sendMessage(this->canBus, message);
}

void CapacitorBank::stop() const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STOP;
    this->drivers->can.sendMessage(this->canBus, message);
}

void CapacitorBank::setPowerLimit(uint16_t watts)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::SET_CHARGE_SPEED;
    message.data[2] = watts;
    message.data[3] = watts >> 8;  // Should always be zero or we are drawing 250+ watts.
    this->drivers->can.sendMessage(this->canBus, message);
}

void CapacitorBank::setSprintModifier(float newSprintModifier)
{
    sprintModifier = tap::algorithms::limitVal(newSprintModifier, 0.0f, 1.0f);
}
}  // namespace aruwsrc::communication::can::capbank