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
    Drivers* drivers,
    tap::can::CanBus canBus,
    const float capacitance)
    : tap::can::CanRxListener(drivers, CAP_BANK_CAN_ID, canBus),
      capacitance(capacitance){};

void ExternalCapacitorBank::processMessage(const modm::can::Message& message)
{
    switch (message.data[7])
    {
        case 0x04:  // Update message
            this->status = static_cast<Status>(message.data[4]);
            this->voltage = *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[2]));
            this->current = *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&message.data[0]));
            break;
        case 0x20:
            if (message.data[4])
            {
                this->p = *reinterpret_cast<float*>(const_cast<uint8_t*>(&message.data[0]));
            }
            else
            {
                this->i = *reinterpret_cast<float*>(const_cast<uint8_t*>(&message.data[0]));
            }
            break;
    }

    this->availablePower = 1 / 2 * this->capacitance * this->voltage * this->voltage;
}

// TODO: Make this an unit16_t in taproot
int ExternalCapacitorBank::consumeAvailablePower(int power)
{
    if (power > this->availablePower)
    {
        power -= this->availablePower;
        this->availablePower = 0;
        return power;
    }

    this->availablePower -= power;
    return 0;
}

void ExternalCapacitorBank::start() const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = 0x01;
    this->drivers->can.sendMessage(this->canBus, message);
}

void ExternalCapacitorBank::stop() const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = 0x02;
    this->drivers->can.sendMessage(this->canBus, message);
}

void ExternalCapacitorBank::requestPIValue(bool isP) const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = 0x10;
    message.data[0] = isP ? 1 : 0;
    this->drivers->can.sendMessage(this->canBus, message);
}

void ExternalCapacitorBank::setPIValue(float value, bool isP) const
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[7] = 0x10;
    message.data[4] = isP ? 1 : 0;
    float* data = reinterpret_cast<float*>(const_cast<uint8_t*>(&message.data[0]));
    *data = value;
    this->drivers->can.sendMessage(this->canBus, message);
}
}  // namespace aruwsrc::communication::sensors::power