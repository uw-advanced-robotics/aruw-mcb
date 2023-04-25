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

namespace aruwsrc::communication::sensors::power
{
ExternalCapacitorBank::ExternalCapacitorBank(
    tap::Drivers* drivers,
    tap::can::CanBus canBus,
    tap::control::chassis::PowerLimiter& powerLimiter,
    const float capacitance)
    : tap::can::CanRxListener(drivers, CAP_BANK_CAN_ID, canBus),
      capacitance(capacitance),
      powerLimiter(powerLimiter){};

void ExternalCapacitorBank::processMessage(const modm::can::Message& message)
{
    switch (static_cast<MessageType>(message.data[7]))
    {
        case MessageType::STATUS:  // Update message
            this->status = static_cast<Status>(message.data[4]);
            this->voltage = *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[2]));
            this->current = *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[0]));
            this->availableEnergy = 1.0 / 2.0 * this->capacitance * powf(this->voltage, 2);

            this->powerLimiter.setExternalEnergyBuffer(this->availableEnergy);
            break;
        default:
            // Ignore unknown message IDs
            break;
    }
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

void ExternalCapacitorBank::setPowerLimit(float watts)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = MessageType::SET_CHARGE_SPEED;

    this->drivers->can.sendMessage(this->canBus, message);
}
}  // namespace aruwsrc::communication::sensors::power
