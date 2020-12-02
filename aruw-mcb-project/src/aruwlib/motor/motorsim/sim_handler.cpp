/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any latersd version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED

#include "sim_handler.hpp"
#include "motor_sim.hpp"
#include "can_serializer.hpp"

namespace aruwlib
{
namespace motorsim
{
SimHandler::SimHandler()
{
    nextSendIndex = 0;
}

void SimHandler::registerSim(MotorSim::MotorType type, uint8_t port)
{
    if (port < capacity && sims[port] == nullptr)
    {
        sims[port] = &MotorSim(type);
    }
}

void SimHandler::registerSim(MotorSim::MotorType type, float loading, uint8_t port)
{
    if (port < capacity && sims[port] == nullptr)
    {
        sims[port] = &MotorSim(type, loading);
    }
}

bool SimHandler::getMessage(aruwlib::can::CanBus bus, const modm::can::Message& message)
{
    modm::can::Message m = message;
    std::array<int16_t, 4> newInputs = CanSerializer::parseMessage(&m);

    int startingIndex;
    switch(bus)
    {
        case aruwlib::can::CanBus::CAN_BUS1:
        if (m.getIdentifier() == 0x1FF)
        {
            startingIndex = 0;
        }
        else if (m.getIdentifier() == 0x200)
        {
            startingIndex = 4;
        }
        break;

        case aruwlib::can::CanBus::CAN_BUS2:
        if (m.getIdentifier() == 0x1FF)
        {
            startingIndex = 8;
        }
        else if (m.getIdentifier() == 0x200)
        {
            startingIndex = 12;
        }
        break;

        default:
        // throw an exception here
        startingIndex = capacity;
    }

    for (int i = 0; i < 4; i++)
    {
        if (sims[startingIndex + i] != nullptr)
        {
            sims[startingIndex + i]->setInput(newInputs[i]);
        }
    }

    return true;
}

bool SimHandler::sendMessage(modm::can::Message* message)
{
    // TODO: Is this proper C++ syntax?
    message = CanSerializer::serializeFeedback(
        sims[nextSendIndex]->getEnc(),
        sims[nextSendIndex]->getRPM(),
        sims[nextSendIndex]->getInput());

    nextSendIndex++;
    if (nextSendIndex >= capacity)
    {
        nextSendIndex = 0;
    }
    
    return true;
}

void SimHandler::updateSims()
{
    for (int i = 0; i < capacity; i++)
    {
        if (sims[i] != nullptr)
        {
            sims[i]->update();
        }
    }
}

}
}
#endif